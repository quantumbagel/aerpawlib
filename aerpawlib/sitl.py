"""
SITL (Software In The Loop) management for aerpawlib.

This module provides utilities for starting and managing ArduPilot SITL
simulation instances for testing and development.

Supports:
- ArduPilot SITL with various vehicle types (copter, plane, rover)
- Multiple instances for multi-vehicle simulation
- QGroundControl connectivity
- Custom starting locations
- Simulation speedup

Usage:
    from aerpawlib.sitl import SITLManager

    # Quick start with default copter
    with SITLManager() as sitl:
        drone = Drone(sitl.connection_string)
        ...

    # With custom arguments (passed directly to sim_vehicle.py)
    with SITLManager("-v copter --console --map --speedup 2") as sitl:
        drone = Drone(sitl.connection_string)
        ...

    # Manual management
    sitl = SITLManager("-v rover -L AERPAW")
    sitl.start()
    # ... do stuff ...
    sitl.stop()
"""
from __future__ import annotations

import os
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import time
from typing import Optional, List


class SITLManager:
    """
    Manages ArduPilot SITL (Software In The Loop) simulation instances.

    All configuration is done via a single args string that is passed
    directly to sim_vehicle.py. This gives full control over all SITL options.

    Example:
        # Default copter
        sitl = SITLManager()
        sitl.start()

        # Custom configuration
        sitl = SITLManager("-v copter --console --map --speedup 2")
        sitl.start()

        # Rover with custom location
        sitl = SITLManager("-v rover -L AERPAW --console")
        sitl.start()

        # With context manager
        with SITLManager("-v copter --console") as sitl:
            drone = Drone(sitl.connection_string)
            ...

    Common sim_vehicle.py arguments:
        -v, --vehicle     Vehicle type: copter, plane, rover, sub
        -f, --frame       Frame type: quad, hexa, +, x, etc.
        -I, --instance    Instance number for multi-vehicle (0, 1, 2, ...)
        --speedup         Simulation speedup (1.0 = realtime)
        -L, --location    Named location or lat,lon,alt,heading
        --console         Open MAVProxy console window
        --map             Open MAVProxy map display
        -w                Wipe EEPROM on start
        --no-mavproxy     Run without MAVProxy
        --out             Add output (e.g., --out udp:127.0.0.1:14550)
    """

    # Port configuration
    BASE_MAVLINK_PORT = 5760
    BASE_UDP_PORT = 14550
    INSTANCE_PORT_OFFSET = 10

    def __init__(self, args: str = "-v copter", ardupilot_path: Optional[str] = None):
        """
        Initialize SITL manager.

        Args:
            args: Arguments to pass to sim_vehicle.py (default: "-v copter")
                  Example: "-v copter --console --map --speedup 2"
            ardupilot_path: Path to ArduPilot source (auto-detected if not provided)
        """
        self.args = args
        self.ardupilot_path = ardupilot_path or self._find_ardupilot_path()

        # Parse instance from args (for port calculation)
        self.instance = self._parse_instance_from_args()

        self._process: Optional[subprocess.Popen] = None
        self._started = False
        self._venv_path: Optional[str] = None

    def _parse_instance_from_args(self) -> int:
        """Extract instance number from args string."""
        try:
            args_list = shlex.split(self.args)
            for i, arg in enumerate(args_list):
                if arg in ["-I", "--instance"] and i + 1 < len(args_list):
                    return int(args_list[i + 1])
        except (ValueError, IndexError):
            pass
        return 0

    def _find_ardupilot_path(self) -> Optional[str]:
        """Try to find ArduPilot installation."""
        # Check environment variable first
        if os.environ.get("ARDUPILOT_HOME"):
            path = os.environ["ARDUPILOT_HOME"]
            if os.path.exists(os.path.join(path, "Tools", "autotest")):
                return path

        # Check project-local /ardupilot directory
        this_file = os.path.abspath(__file__)
        aerpawlib_dir = os.path.dirname(this_file)
        project_root = os.path.dirname(aerpawlib_dir)
        project_ardupilot = os.path.join(project_root, "ardupilot")

        if os.path.exists(os.path.join(project_ardupilot, "Tools", "autotest")):
            return project_ardupilot

        # Check common locations
        common_paths = [
            os.path.expanduser("~/ardupilot"),
            os.path.expanduser("~/ArduPilot"),
            "/opt/ardupilot",
            os.path.expanduser("~/src/ardupilot"),
        ]

        for path in common_paths:
            if os.path.exists(os.path.join(path, "Tools", "autotest")):
                return path

        return None

    def _find_ardupilot_venv(self) -> Optional[str]:
        """Find the ArduPilot virtual environment."""
        this_file = os.path.abspath(__file__)
        aerpawlib_dir = os.path.dirname(this_file)
        project_root = os.path.dirname(aerpawlib_dir)
        project_venv = os.path.join(project_root, "ardupilot-venv")

        if os.path.exists(os.path.join(project_venv, "bin", "python")):
            return project_venv

        if self.ardupilot_path:
            parent = os.path.dirname(self.ardupilot_path)
            venv_path = os.path.join(parent, "ardupilot-venv")
            if os.path.exists(os.path.join(venv_path, "bin", "python")):
                return venv_path

        return None

    def _get_venv_python(self) -> str:
        """Get the Python executable from the ArduPilot venv, or system python."""
        if self._venv_path is None:
            self._venv_path = self._find_ardupilot_venv()

        if self._venv_path:
            venv_python = os.path.join(self._venv_path, "bin", "python")
            if os.path.exists(venv_python):
                return venv_python

        return sys.executable

    def _find_sim_vehicle(self) -> Optional[str]:
        """Find sim_vehicle.py script."""
        sim_vehicle = shutil.which("sim_vehicle.py")
        if sim_vehicle:
            return sim_vehicle

        if self.ardupilot_path:
            sim_vehicle = os.path.join(
                self.ardupilot_path, "Tools", "autotest", "sim_vehicle.py"
            )
            if os.path.exists(sim_vehicle):
                return sim_vehicle

        return None

    @property
    def tcp_port(self) -> int:
        """Get TCP MAVLink port for this instance."""
        return self.BASE_MAVLINK_PORT + (self.INSTANCE_PORT_OFFSET * self.instance)

    @property
    def udp_port(self) -> int:
        """Get UDP port for this instance (for MAVSDK connection)."""
        return self.BASE_UDP_PORT + self.instance

    @property
    def connection_string(self) -> str:
        """Get connection string for this SITL instance (for MAVSDK)."""
        return f"udpin://127.0.0.1:{self.udp_port}"

    @property
    def tcp_connection_string(self) -> str:
        """Get TCP connection string (for pymavlink/MAVProxy)."""
        return f"tcp:127.0.0.1:{self.tcp_port}"

    @property
    def is_running(self) -> bool:
        """Check if SITL is running."""
        return self._process is not None and self._process.poll() is None

    def _build_command(self) -> List[str]:
        """Build the sim_vehicle.py command."""
        sim_vehicle = self._find_sim_vehicle()
        if not sim_vehicle:
            raise RuntimeError(
                "ArduPilot SITL not found. Please install ArduPilot using:\n"
                "  ./aerpawlib/install_ardupilot.sh\n"
                "\n"
                "Or set ARDUPILOT_HOME environment variable to your ArduPilot installation.\n"
                "See: https://ardupilot.org/dev/docs/building-setup-linux.html"
            )

        python_exe = self._get_venv_python()
        cmd = [python_exe, sim_vehicle]

        # Parse and add user args
        args_list = shlex.split(self.args)
        cmd.extend(args_list)

        # Add UDP output for MAVSDK/pymavlink connection if not already specified
        if "--out" not in self.args:
            cmd.extend(["--out", f"udpout:127.0.0.1:{self.udp_port}"])

        return cmd

    def start(self, timeout: float = 120.0) -> str:
        """
        Start the SITL instance.

        Args:
            timeout: Maximum time to wait for SITL to start (seconds)

        Returns:
            Connection string for the SITL instance

        Raises:
            RuntimeError: If SITL fails to start
        """
        if self._started:
            print(f"[aerpawlib] SITL instance {self.instance} already running")
            return self.connection_string

        print(f"[aerpawlib] Starting ArduPilot SITL...")
        print(f"[aerpawlib]   Args: {self.args}")
        print(f"[aerpawlib]   Instance: {self.instance}")
        print(f"[aerpawlib]   UDP Port: {self.udp_port}")
        print(f"[aerpawlib]   TCP Port: {self.tcp_port}")

        venv_path = self._find_ardupilot_venv()
        if venv_path:
            print(f"[aerpawlib]   Using venv: {venv_path}")
        else:
            print(f"[aerpawlib]   Using system Python: {sys.executable}")

        cmd = self._build_command()
        print(f"[aerpawlib]   Command: {' '.join(cmd)}")

        env = os.environ.copy()
        if self.ardupilot_path:
            env["ARDUPILOT_HOME"] = self.ardupilot_path

        verbose = os.environ.get("SITL_VERBOSE", "0") == "1"

        self._process = subprocess.Popen(
            cmd,
            cwd=self.ardupilot_path or os.getcwd(),
            env=env,
            stdout=None if verbose else subprocess.DEVNULL,
            stderr=None if verbose else subprocess.DEVNULL,
            preexec_fn=os.setsid if sys.platform != "win32" else None,
        )

        self._wait_for_sitl(timeout)
        self._started = True

        print(f"[aerpawlib] SITL ready!")
        print(f"[aerpawlib]   MAVSDK connection: {self.connection_string}")
        print(f"[aerpawlib]   TCP connection: {self.tcp_connection_string}")
        print(f"[aerpawlib] ------------------------------------------------")
        print(f"[aerpawlib] QGroundControl: Connect via TCP to localhost:{self.tcp_port}")
        print(f"[aerpawlib]   In QGC: Application Settings > Comm Links > Add")
        print(f"[aerpawlib]   Type: TCP, Host: localhost, Port: {self.tcp_port}")
        print(f"[aerpawlib] ------------------------------------------------")

        return self.connection_string

    def _wait_for_sitl(self, timeout: float = 120.0) -> None:
        """Wait for SITL to be ready to accept connections."""
        print("[aerpawlib] Waiting for SITL to initialize...")
        start_time = time.time()

        while time.time() - start_time < timeout:
            if self._process and self._process.poll() is not None:
                raise RuntimeError(
                    f"SITL process terminated unexpectedly with code {self._process.returncode}"
                )

            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(2.0)
                result = sock.connect_ex(("127.0.0.1", self.tcp_port))
                sock.close()

                if result == 0:
                    time.sleep(3)
                    return

            except socket.error:
                pass

            time.sleep(1)

        raise RuntimeError(f"SITL failed to start within {timeout} seconds")

    def stop(self) -> None:
        """Stop the SITL instance."""
        if self._process is None:
            return

        print(f"[aerpawlib] Stopping SITL instance {self.instance}...")

        try:
            if sys.platform != "win32":
                os.killpg(os.getpgid(self._process.pid), signal.SIGTERM)
            else:
                self._process.terminate()

            try:
                self._process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                if sys.platform != "win32":
                    os.killpg(os.getpgid(self._process.pid), signal.SIGKILL)
                else:
                    self._process.kill()
                self._process.wait()

        except (ProcessLookupError, OSError):
            pass

        self._process = None
        self._started = False
        print("[aerpawlib] SITL stopped")

    def __enter__(self) -> "SITLManager":
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        """Context manager exit."""
        self.stop()
        return False


# Convenience function
def start_sitl(args: str = "-v copter") -> SITLManager:
    """
    Start a SITL instance (convenience function).

    Args:
        args: Arguments to pass to sim_vehicle.py

    Returns:
        Started SITLManager instance

    Example:
        sitl = start_sitl("-v copter --console --map")
        drone = Drone(sitl.connection_string)
        # ... do stuff ...
        sitl.stop()
    """
    manager = SITLManager(args)
    manager.start()
    return manager


__all__ = [
    "SITLManager",
    "start_sitl",
]

