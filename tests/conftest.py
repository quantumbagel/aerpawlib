"""
Pytest configuration and fixtures for aerpawlib v1 tests.

SITL is managed by pytest: started before integration tests, stopped after.
Full SITL reset (disarm, clear mission, battery reset) runs between each test.
"""

from __future__ import annotations

import asyncio
import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import AsyncGenerator, Optional

import pytest
import pytest_asyncio

from aerpawlib.log import LogComponent, LogLevel, configure_logging, get_logger

logger = get_logger(LogComponent.SITL)

# Constants
DEFAULT_SITL_PORT = 14550
DEFAULT_SITL_PORT_DRONE = 14550
DEFAULT_SITL_PORT_ROVER = 14560
# SITL instance IDs - each instance uses different internal TCP ports (5760 + instance*10)
# This allows running drone and rover concurrently without port conflicts
DEFAULT_SITL_INSTANCE_DRONE = 0  # Uses TCP 5760-5769
DEFAULT_SITL_INSTANCE_ROVER = 1  # Uses TCP 5770-5779
# MAVSDK server ports - each vehicle needs its own gRPC port to avoid conflicts
DEFAULT_MAVSDK_SERVER_PORT_DRONE = 50051
DEFAULT_MAVSDK_SERVER_PORT_ROVER = 50052
SITL_STARTUP_TIMEOUT = 90
SITL_GPS_TIMEOUT = 120
LAKE_WHEELER_LAT = 35.727436
LAKE_WHEELER_LON = -78.696587


def _find_sim_vehicle() -> Optional[Path]:
    """Locate sim_vehicle.py from ARDUPILOT_HOME or common paths."""
    project_root = Path(__file__).resolve().parent.parent
    # Check for both 'ardupilot' and versioned directories like 'ardupilot-4.6.3'
    ardupilot_dirs = list(project_root.glob("ardupilot*"))

    candidates = [
        os.environ.get("ARDUPILOT_HOME"),
    ] + [str(d) for d in ardupilot_dirs if d.is_dir()]

    logger.debug(f"Searching for sim_vehicle.py in candidates: {candidates}")
    for base in candidates:
        if base is None:
            continue
        base = Path(base)
        script = base / "Tools" / "autotest" / "sim_vehicle.py"
        if script.exists():
            logger.info(f"Found sim_vehicle.py at {script}")
            return script
    return None


def _port_available(host: str, port: int) -> bool:
    """Check if a port is accepting connections."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(0.5)
            s.connect((host, port))
        return True
    except (socket.error, OSError):
        return False


def pytest_addoption(parser: pytest.Parser) -> None:
    """Add custom command-line options."""
    parser.addoption(
        "--sitl-port",
        action="store",
        default=str(DEFAULT_SITL_PORT),
        help=f"UDP port for SITL (legacy, applies to drone; default: {DEFAULT_SITL_PORT})",
    )
    parser.addoption(
        "--sitl-port-drone",
        action="store",
        default=None,
        help=f"UDP port for ArduCopter SITL (default: {DEFAULT_SITL_PORT_DRONE})",
    )
    parser.addoption(
        "--sitl-port-rover",
        action="store",
        default=None,
        help=f"UDP port for Rover SITL (default: {DEFAULT_SITL_PORT_ROVER})",
    )
    parser.addoption(
        "--no-sitl",
        action="store_true",
        help="Skip SITL-managed integration tests (SITL must be running externally)",
    )
    parser.addoption(
        "--no-sitl-manage",
        action="store_false",
        dest="sitl_manage",
        default=True,
        help="Pytest does not start/stop SITL; use external SITL (default: pytest manages SITL)",
    )


def pytest_configure(config: pytest.Config) -> None:
    """Pytest configuration."""
    configure_logging(level=LogLevel.DEBUG, root_name="aerpawlib")


def pytest_collection_modifyitems(config: pytest.Config, items: list) -> None:
    """Auto-apply markers based on test path."""
    for item in items:
        path_str = str(item.path)
        if "/unit/" in path_str:
            item.add_marker(pytest.mark.unit)
        elif "/integration/" in path_str:
            item.add_marker(pytest.mark.integration)


# ---------------------------------------------------------------------------
# Unit test fixtures (no SITL)
# ---------------------------------------------------------------------------


@pytest.fixture
def origin_coordinate():
    """Coordinate at AERPAW Lake Wheeler."""
    from aerpawlib.v1.util import Coordinate
    return Coordinate(LAKE_WHEELER_LAT, LAKE_WHEELER_LON, 0)


@pytest.fixture
def nearby_coordinate():
    """Coordinate ~100m north of origin."""
    from aerpawlib.v1.util import Coordinate
    return Coordinate(LAKE_WHEELER_LAT + 0.0009, LAKE_WHEELER_LON, 0)


@pytest.fixture
def zero_vector():
    """Zero VectorNED."""
    from aerpawlib.v1.util import VectorNED
    return VectorNED(0, 0, 0)


# ---------------------------------------------------------------------------
# SITL management
# ---------------------------------------------------------------------------


class SITLManager:
    """
    Manages ArduPilot SITL process for integration tests.
    Starts SITL with MAVProxy, provides connection string, performs full reset between tests.
    """

    def __init__(
        self,
        port: int,
        vehicle_type: str = "ArduCopter",
        manage: bool = True,
        instance_id: int = 0,
    ):
        self.port = port
        self.vehicle_type = vehicle_type
        self.manage = manage
        self.instance_id = instance_id
        self._process: Optional[subprocess.Popen] = None
        self._sim_vehicle_path: Optional[Path] = None

    def start(self) -> str:
        """Start SITL and return connection string."""
        if not self.manage:
            logger.info(f"Using external SITL at udpin://127.0.0.1:{self.port}")
            return f"udpin://127.0.0.1:{self.port}"

        sim_vehicle = _find_sim_vehicle()
        if sim_vehicle is None:
            msg = "sim_vehicle.py not found. Set ARDUPILOT_HOME or ensure ardupilot directory exists."
            logger.error(msg)
            pytest.skip(msg)

        self._sim_vehicle_path = sim_vehicle
        ardupilot_home = sim_vehicle.parent.parent.parent

        env = os.environ.copy()
        env["ARDUPILOT_HOME"] = str(ardupilot_home)
        env.setdefault("SIM_SPEEDUP", "5")
        # Prevent sim_vehicle's run_in_terminal_window.sh from opening a new Terminal window in GUI environments
        env.pop("DISPLAY", None)

        cmd = [
            sys.executable,
            str(sim_vehicle),
            "-v", self.vehicle_type,
            "-I", str(self.instance_id),
            "--out", f"udp:127.0.0.1:{self.port}",
            "-w",
        ]

        logger.info(f"Starting SITL with command: {' '.join(cmd)}")
        logger.info(f"SITL Working Directory: {ardupilot_home}")

        # Capture SITL output to a per-vehicle log file for debugging
        log_suffix = "drone" if self.vehicle_type == "ArduCopter" else "rover"
        sitl_log_path = Path(f"logs/sitl_{log_suffix}_output.log")
        sitl_log_path.parent.mkdir(exist_ok=True)
        self._sitl_log = open(sitl_log_path, "w")
        logger.info(f"SITL output log: {sitl_log_path}")

        self._process = subprocess.Popen(
            cmd,
            cwd=str(ardupilot_home),
            env=env,
            stdout=self._sitl_log,
            stderr=subprocess.STDOUT,
        )

        logger.debug(f"SITL process PID: {self._process.pid}")
        logger.info(f"Waiting for MAVLink port {self.port} (timeout {SITL_STARTUP_TIMEOUT}s)...")

        # Wait for MAVLink port
        start = time.monotonic()
        while time.monotonic() - start < SITL_STARTUP_TIMEOUT:
            if _port_available("127.0.0.1", self.port):
                logger.info(f"SITL Ready on udpin://127.0.0.1:{self.port}")
                return f"udpin://127.0.0.1:{self.port}"

            # Check if process died
            if self._process.poll() is not None:
                self._sitl_log.close()
                with open(sitl_log_path, "r") as f:
                    output = f.read()
                sitl_process_log = f"/tmp/{self.vehicle_type}.log"
                msg = (
                    f"SITL process exited prematurely with code {self._process.returncode}. "
                    f"sim_vehicle output:\n{output}\n"
                    f"Also check {sitl_process_log} for SITL binary output."
                )
                logger.error(msg)
                pytest.fail(msg)

            time.sleep(1)

        self.stop()
        sitl_process_log = f"/tmp/{self.vehicle_type}.log"
        msg = (
            f"SITL failed to start within {SITL_STARTUP_TIMEOUT}s. "
            f"Check {sitl_log_path} (sim_vehicle output) and {sitl_process_log} (SITL process)."
        )
        logger.error(msg)
        pytest.fail(msg)

    def stop(self) -> None:
        """Stop SITL process."""
        if self._process is not None:
            logger.info("Stopping SITL...")
            try:
                self._process.terminate()
                self._process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                logger.warning("SITL didn't terminate, killing...")
                self._process.kill()
            self._process = None
            if hasattr(self, '_sitl_log'):
                self._sitl_log.close()
            logger.info("SITL Stopped")

    def connection_string(self) -> str:
        """Return MAVLink connection string."""
        return f"udpin://127.0.0.1:{self.port}"


def _get_sitl_port_drone(config) -> int:
    """Resolve drone SITL port: --sitl-port-drone or --sitl-port or default."""
    port_opt = config.getoption("--sitl-port-drone", default=None)
    if port_opt is not None:
        return int(port_opt)
    return int(config.getoption("--sitl-port", default=DEFAULT_SITL_PORT_DRONE))


def _get_sitl_port_rover(config) -> int:
    """Resolve rover SITL port: --sitl-port-rover or default."""
    port_opt = config.getoption("--sitl-port-rover", default=None)
    if port_opt is not None:
        return int(port_opt)
    return DEFAULT_SITL_PORT_ROVER


# Session-scoped SITL: started once per vehicle type, shared across integration tests
@pytest.fixture(scope="session")
def sitl_manager_drone(request: pytest.FixtureRequest) -> SITLManager:
    """Session-scoped ArduCopter SITL manager for drone tests."""
    port = _get_sitl_port_drone(request.config)
    no_sitl = request.config.getoption("--no-sitl", default=False)
    manage = request.config.getoption("sitl_manage", default=True) and not no_sitl

    manager = SITLManager(
        port=port,
        vehicle_type="ArduCopter",
        manage=manage,
        instance_id=DEFAULT_SITL_INSTANCE_DRONE,
    )
    if manage:
        manager.start()
        request.addfinalizer(manager.stop)
    return manager


@pytest.fixture(scope="session")
def sitl_manager_rover(request: pytest.FixtureRequest) -> SITLManager:
    """Session-scoped Rover SITL manager for rover tests."""
    port = _get_sitl_port_rover(request.config)
    no_sitl = request.config.getoption("--no-sitl", default=False)
    manage = request.config.getoption("sitl_manage", default=True) and not no_sitl

    manager = SITLManager(
        port=port,
        vehicle_type="Rover",
        manage=manage,
        instance_id=DEFAULT_SITL_INSTANCE_ROVER,
    )
    if manage:
        manager.start()
        request.addfinalizer(manager.stop)
    return manager


@pytest.fixture
def sitl_connection_string_drone(sitl_manager_drone: SITLManager) -> str:
    """Connection string for drone SITL."""
    return sitl_manager_drone.connection_string()


@pytest.fixture
def sitl_connection_string_rover(sitl_manager_rover: SITLManager) -> str:
    """Connection string for rover SITL."""
    return sitl_manager_rover.connection_string()


# ---------------------------------------------------------------------------
# Full SITL reset (between each integration test)
# ---------------------------------------------------------------------------


async def _full_sitl_reset(vehicle) -> None:
    """Disarm, clear mission, battery reset. Full clean state between tests."""
    from mavsdk.mavlink_direct import MavlinkMessage

    system = getattr(vehicle, "_system", None)
    if not system:
        return

    async def _reset():
        try:
            await system.mission.clear_mission()
        except Exception:
            pass
        try:
            await system.geofence.clear_geofence()
        except Exception:
            pass
        try:
            await system.action.return_to_launch()
            await asyncio.sleep(2)
        except Exception:
            pass
        try:
            # long BATTERY_RESET 1 100
            fields = {
                "target_system": 1, "target_component": 1,
                "command": 42651, "confirmation": 0,
                "param1": 1.0, "param2": 100.0,
                "param3": 0.0, "param4": 0.0,
                "param5": 0.0, "param6": 0.0, "param7": 0.0,
            }
            msg = MavlinkMessage(
                system_id=1, component_id=1,
                target_system_id=1, target_component_id=1,
                message_name="COMMAND_LONG",
                fields_json=json.dumps(fields),
            )
            await system.mavlink_direct.send_message(msg)
        except Exception:
            pass
        try:
            await system.action.disarm()
        except Exception:
            pass

    try:
        await vehicle._run_on_mavsdk_loop(_reset())
        await asyncio.sleep(2)
    except Exception:
        pass


async def _connect_and_wait_gps(
    vehicle_class,
    connection_string: str,
    mavsdk_server_port: int = 50051,
    timeout: int = SITL_GPS_TIMEOUT,
):
    """Connect vehicle and wait for 3D GPS fix.

    Args:
        vehicle_class: The vehicle class to instantiate (Drone or Rover).
        connection_string: MAVLink connection string.
        mavsdk_server_port: Port for the embedded mavsdk_server gRPC interface.
            Each vehicle instance should use a unique port to avoid conflicts.
        timeout: Timeout for GPS fix in seconds.
    """
    from aerpawlib.v1.exceptions import ConnectionTimeoutError

    try:
        vehicle = await asyncio.to_thread(
            vehicle_class, connection_string, mavsdk_server_port=mavsdk_server_port
        )
    except ConnectionTimeoutError:
        pytest.fail(f"Connection timeout to {connection_string}")
    except Exception as e:
        pytest.fail(f"Vehicle init failed: {type(e).__name__}: {e}")

    start = time.monotonic()
    while time.monotonic() - start < timeout:
        fix = vehicle.gps.fix_type
        if fix >= 3:
            return vehicle
        await asyncio.sleep(1)

    vehicle.close()
    pytest.fail(f"No 3D GPS fix within {timeout}s")


# ---------------------------------------------------------------------------
# Integration test fixtures (connected vehicles with full reset between tests)
# ---------------------------------------------------------------------------


@pytest_asyncio.fixture
async def connected_drone(sitl_connection_string_drone: str) -> AsyncGenerator:
    """Drone connected to SITL. Full reset before each test."""
    from aerpawlib.v1.vehicle import Drone

    drone = await _connect_and_wait_gps(
        Drone,
        sitl_connection_string_drone,
        mavsdk_server_port=DEFAULT_MAVSDK_SERVER_PORT_DRONE,
    )
    yield drone
    try:
        await _full_sitl_reset(drone)
    finally:
        drone.close()


@pytest_asyncio.fixture
async def connected_rover(sitl_connection_string_rover: str) -> AsyncGenerator:
    """Rover connected to SITL. Full reset before each test."""
    from aerpawlib.v1.vehicle import Rover

    rover = await _connect_and_wait_gps(
        Rover,
        sitl_connection_string_rover,
        mavsdk_server_port=DEFAULT_MAVSDK_SERVER_PORT_ROVER,
    )
    yield rover
    try:
        await _full_sitl_reset(rover)
    finally:
        rover.close()
