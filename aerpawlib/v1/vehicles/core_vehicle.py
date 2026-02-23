"""
Core logic surrounding the various `Vehicle`s available to aerpawlib user
scripts.

This is the MAVSDK-based implementation of the v1 API, maintaining full
backward compatibility with the original DroneKit-based interface.

@author: Julian Reder (quantumbagel)
"""

import asyncio
import errno
import re
import socket
from urllib.parse import urlparse

from grpc.aio import AioRpcError

from aerpawlib.log import get_logger, LogComponent
import math
import time
import threading
from typing import Callable, List, Optional, Tuple

from mavsdk import System
from mavsdk.action import ActionError


from aerpawlib.v1 import util
from aerpawlib.v1.aerpaw import AERPAW_Platform
from aerpawlib.v1.constants import (
    ARMING_SEQUENCE_DELAY_S,
    POLLING_DELAY_S,
    INTERNAL_UPDATE_DELAY_S,
    CONNECTION_TIMEOUT_S,
    ARMABLE_TIMEOUT_S,
    ARMABLE_STATUS_LOG_INTERVAL_S,
    POSITION_READY_TIMEOUT_S,
    DEFAULT_POSITION_TOLERANCE_M,
    DEFAULT_GOTO_TIMEOUT_S,
    VERBOSE_LOG_FILE_PREFIX,
    VERBOSE_LOG_DELAY_S,
)
from aerpawlib.v1.exceptions import (
    ConnectionTimeoutError,
    ArmError,
    DisarmError,
    NotArmableError,
    NotImplementedForVehicleError,
    AerpawConnectionError,
    PortInUseError,
)
from aerpawlib.v1.helpers import (
    wait_for_condition,
    ThreadSafeValue,
)

# Configure module logger
logger = get_logger(LogComponent.VEHICLE)


def _parse_udp_connection_port(connection_string: str) -> Optional[Tuple[str, int]]:
    """
    Parse host and port from a UDP connection string for port-in-use checks.

    Supports MAVSDK/aerpawlib UDP formats:
    - udp://:port, udp://host:port
    - udpin://:port, udpin://host:port, udpin://[ipv6]:port
    - udpout:// returns None (client mode; no local bind)

    Returns:
        (host, port) for server/listen modes, None for client mode or non-UDP.
    """
    parsed = urlparse(connection_string.strip())
    scheme = parsed.scheme.lower()
    netloc = parsed.netloc

    # udpout is client mode - we connect to remote, no local bind to check
    if scheme == "udpout":
        return None

    if scheme not in ("udp", "udpin"):
        return None

    if not netloc:
        return None

    # Parse host and port from netloc
    # IPv6: [::1]:14540 or [fe80::1]:14540
    ipv6_match = re.match(r"\[([^\]]+)\]:(\d+)$", netloc)
    if ipv6_match:
        host, port_str = ipv6_match.group(1), ipv6_match.group(2)
    else:
        # IPv4 or hostname: 127.0.0.1:14550 or :14540
        parts = netloc.rsplit(":", 1)
        if len(parts) != 2:
            return None
        host, port_str = parts

    try:
        port = int(port_str)
    except ValueError:
        return None

    if not (0 < port <= 65535):
        return None

    host = host.strip() if host else "0.0.0.0"
    return (host, port)


def is_udp_port_in_use(host: str, port: int) -> bool:
    """
    Check if a local UDP port is in use by trying to bind to it.

    Args:
        host: The local IP address to bind to (e.g., '127.0.0.1', '0.0.0.0', or '::1').
        port: The port number to check.

    Returns:
        True if the port is in use, False otherwise.
    """
    family = socket.AF_INET6 if ":" in host else socket.AF_INET
    with socket.socket(family, socket.SOCK_DGRAM) as s:
        try:
            s.bind((host, port))
            return False
        except OSError as e:
            if e.errno == errno.EADDRINUSE:
                return True
            logger.warning("Port check failed: %s", e)
            return True


class _BatteryCompat:
    """
    Compatibility wrapper to match dronekit.Battery interface.

    Attributes:
        voltage: Battery voltage in volts
        current: Battery current draw in amps
        level: Battery level as percentage (0-100)
    """

    __slots__ = ("voltage", "current", "level")

    def __init__(self):
        self.voltage: float = 0.0
        self.current: float = 0.0
        self.level: int = 0

    def __str__(self) -> str:
        return f"Battery:voltage={self.voltage},current={self.current},level={self.level}"

    def __repr__(self) -> str:
        return f"_BatteryCompat(voltage={self.voltage}, current={self.current}, level={self.level})"


class _GPSInfoCompat:
    """
    Compatibility wrapper to match dronekit.GPSInfo interface.

    Attributes:
        fix_type: GPS fix type (0-1: no fix, 2: 2d fix, 3: 3d fix)
        satellites_visible: Number of visible satellites
    """

    __slots__ = ("fix_type", "satellites_visible")

    def __init__(self):
        self.fix_type: int = 0
        self.satellites_visible: int = 0

    def __str__(self) -> str:
        return f"GPSInfo:fix={self.fix_type},num_sat={self.satellites_visible}"

    def __repr__(self) -> str:
        return f"_GPSInfoCompat(fix_type={self.fix_type}, satellites_visible={self.satellites_visible})"


class _AttitudeCompat:
    """
    Compatibility wrapper to match dronekit.Attitude interface.

    All angles in radians.

    Attributes:
        pitch: Pitch angle in radians
        roll: Roll angle in radians
        yaw: Yaw angle in radians
    """

    __slots__ = ("pitch", "roll", "yaw")

    def __init__(self):
        self.pitch: float = 0.0
        self.roll: float = 0.0
        self.yaw: float = 0.0

    def __str__(self) -> str:
        return f"Attitude:pitch={self.pitch},yaw={self.yaw},roll={self.roll}"

    def __repr__(self) -> str:
        return f"_AttitudeCompat(pitch={self.pitch}, roll={self.roll}, yaw={self.yaw})"


class _VersionCompat:
    """
    Compatibility wrapper to match dronekit.Version interface.

    Attributes:
        major: Major version number
        minor: Minor version number
        patch: Patch version number
        release: Release type (if available)
    """

    __slots__ = ("major", "minor", "patch", "release")

    def __init__(self):
        self.major: Optional[int] = None
        self.minor: Optional[int] = None
        self.patch: Optional[int] = None
        self.release: Optional[str] = None

    def __str__(self) -> str:
        return f"{self.major}.{self.minor}.{self.patch}"

    def __repr__(self) -> str:
        return f"_VersionCompat(major={self.major}, minor={self.minor}, patch={self.patch})"


class DummyVehicle:
    """
    A placeholder vehicle class for scripts that do not require physical vehicle interaction.

    This class provides the same interface as `Vehicle` but with empty implementations.
    """

    def __init__(self):
        pass

    def close(self):
        pass

    def _initialize_prearm(self, should_postarm_init):
        pass

    async def _initialize_postarm(self):
        pass


class Vehicle:
    """
    Overarching "generic vehicle" type.

    Implements common functionality for all vehicle types (drone, rover, etc.),
    excluding specific movement commands. This class maintains an internal
    MAVSDK session while providing a DroneKit-compatible API.

    Safety tenets:
    - Never auto-arms by default; waits for external actor (safety pilot/GCS).
    - Detects armed state and transitions to GUIDED mode.
    - Captures home location upon entering GUIDED mode.
    - Tracks connection via heartbeat monitoring.
    - Supports configurable auto-RTL or landing upon script termination.

    Attributes:
        _system (System): The MAVSDK system instance.
        _has_heartbeat (bool): Whether a heartbeat has been received.
        _home_location (Coordinate): The captured home position.
        _armed_state (ThreadSafeValue): Current arm status.
        _mode (ThreadSafeValue): Current flight mode name.
    """

    _system: Optional[System]
    _has_heartbeat: bool

    # function used by "verb" functions to check and see if the vehicle can be
    # commanded to move. should be set to a new closure by verb functions to
    # redefine functionality
    _ready_to_move: Callable[["Vehicle"], bool] = lambda _: True

    # Controls whether the vehicle can be aborted during movement
    _abortable: bool = False
    _aborted: bool = False

    _home_location: Optional[util.Coordinate] = None

    # _current_heading is used to blend heading and velocity control commands
    _current_heading: Optional[float] = None

    _last_nav_controller_output = None
    _last_mission_item_int = None

    # Verbose logging configuration
    _verbose_logging: bool = False
    _verbose_logging_file_prefix: str = VERBOSE_LOG_FILE_PREFIX
    _verbose_logging_file_writer = None
    _verbose_logging_last_log_time: float = 0
    _verbose_logging_delay: float = VERBOSE_LOG_DELAY_S
    _verbose_log_lock: threading.Lock

    # Safety initialization state
    _initialization_complete: bool = False
    _skip_init: bool = False  # Set via CLI --skip-init flag
    _skip_rtl: bool = False  # Set via CLI --skip-rtl flag

    # Connection/heartbeat tracking
    _last_heartbeat_time: float = 0.0

    def __init__(self, connection_string: str, mavsdk_server_port: int = 50051):
        """
        Initialize the vehicle and connect to the autopilot.

        Args:
            connection_string (str): MAVLink connection string (e.g., 'udp://:14540').
            mavsdk_server_port (int): Port for the embedded mavsdk_server gRPC interface.
                Each Vehicle instance should use a unique port to avoid conflicts.
                Defaults to 50051.

        Raises:
            ConnectionTimeoutError: If connection cannot be established within timeout.
        """
        self._connection_string = connection_string
        self._mavsdk_server_port = mavsdk_server_port
        self._system = None
        self._has_heartbeat = False
        self._connection_error: Optional[BaseException] = None
        self._closed = False
        self._verbose_log_lock = threading.Lock()
        self._should_postarm_init = True
        self._mission_start_time: Optional[float] = None

        # Safety initialization state
        self._initialization_complete = False
        self._skip_init = False
        self._skip_rtl = False
        self._was_already_armed_on_connect = False
        self._last_heartbeat_time = 0.0

        # Safety checker setup
        self._armed_state = ThreadSafeValue(False)
        self._is_armable_state = ThreadSafeValue(False)
        self._health_val = ThreadSafeValue(None)
        self._last_arm_time = ThreadSafeValue(0.0)
        self._position_lat = ThreadSafeValue(0.0)
        self._position_lon = ThreadSafeValue(0.0)
        self._position_alt = ThreadSafeValue(0.0)
        self._position_abs_alt = ThreadSafeValue(0.0)
        self._heading_deg = ThreadSafeValue(0.0)
        self._velocity_ned = ThreadSafeValue([0.0, 0.0, 0.0])
        self._home_position = ThreadSafeValue(None)
        self._home_abs_alt = ThreadSafeValue(0.0)

        # Compatibility objects (ThreadSafeValue for atomic swap from telemetry thread)
        self._battery_val = ThreadSafeValue(_BatteryCompat())
        self._gps_val = ThreadSafeValue(_GPSInfoCompat())
        self._attitude_val = ThreadSafeValue(_AttitudeCompat())
        self._autopilot_info = _VersionCompat()
        self._mode = ThreadSafeValue("UNKNOWN")

        # Flag set once the first armed-state telemetry message arrives
        self._armed_telemetry_received = ThreadSafeValue(False)

        # Track active futures for cancellation in close()
        self._pending_mavsdk_futures = set()

        # Telemetry tasks
        self._telemetry_tasks: List[asyncio.Task] = []
        self._running = ThreadSafeValue(True)

        # Event loop for MAVSDK operations (runs in background thread)
        self._mavsdk_loop: Optional[asyncio.AbstractEventLoop] = None

        # Connect synchronously (blocking)
        self._connect_sync()

    def _connect_sync(self):
        """
        Establish connection and start telemetry in background threads.

        This is a synchronous wrapper around asynchronous connection logic.
        """
        # Fail fast if UDP port from connection string is already in use (avoids hanging)
        parsed = _parse_udp_connection_port(self._connection_string)
        if parsed is not None:
            host, port = parsed
            if is_udp_port_in_use(host, port):
                raise PortInUseError(
                    port,
                    f"UDP port {port} is already in use. "
                    "Stop the other process or use a different connection string.",
                )

        loop = asyncio.new_event_loop()
        self._mavsdk_loop = loop  # Store reference for thread-safe calls

        def _run_connection():
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(self._connect_async())
                # Keep the loop running for telemetry
                loop.run_forever()
            except BaseException as e:
                self._connection_error = e
            finally:
                # Ensure all tasks are cancelled and loop is closed
                pending = asyncio.all_tasks(loop)
                for task in pending:
                    task.cancel()
                if pending:
                    loop.run_until_complete(
                        asyncio.gather(*pending, return_exceptions=True)
                    )
                loop.close()

        # Start connection in background thread
        self._mavsdk_thread = threading.Thread(target=_run_connection, daemon=True)
        self._mavsdk_thread.start()

        # Wait for connection with timeout
        start = time.time()
        while not self._has_heartbeat:
            if self._connection_error is not None:
                err = self._connection_error
                self._connection_error = None
                if isinstance(err, AerpawConnectionError):
                    raise err
                raise AerpawConnectionError(
                    f"Connection failed: {err}", original_error=err
                )
            if time.time() - start > CONNECTION_TIMEOUT_S:
                raise ConnectionTimeoutError(CONNECTION_TIMEOUT_S)
            time.sleep(POLLING_DELAY_S)

        # Start internal update loop
        update_thread = threading.Thread(
            target=self._internal_update_loop, daemon=True
        )
        update_thread.start()

    async def _run_on_mavsdk_loop(self, coro):
        """
        Run a coroutine on the MAVSDK event loop.

        Args:
            coro: The coroutine to execute.

        Returns:
            The result of the coroutine.

        Raises:
            RuntimeError: If the MAVSDK loop is not initialized or not running.
        """
        if self._mavsdk_loop is None:
            raise RuntimeError("MAVSDK loop not initialized")
        if not self._mavsdk_loop.is_running():
            # If the loop isn't running yet, we might be in the middle of connecting
            # or it has crashed.
            logger.warning("MAVSDK loop is not yet running, waiting...")
            start_time = time.time()
            while not self._mavsdk_loop.is_running() and time.time() - start_time < 5.0:
                await asyncio.sleep(POLLING_DELAY_S)
            if not self._mavsdk_loop.is_running():
                raise RuntimeError("MAVSDK loop is not running")

        future = asyncio.run_coroutine_threadsafe(coro, self._mavsdk_loop)
        self._pending_mavsdk_futures.add(future)
        try:
            return await asyncio.wait_for(asyncio.wrap_future(future), timeout=30.0)
        except asyncio.TimeoutError:
            future.cancel()
            raise RuntimeError(
                "MAVSDK operation timed out after 30s — "
                "the MAVSDK event loop may have crashed"
            )
        except (AioRpcError, Exception) as e:
            if isinstance(e, AioRpcError):
                raise AerpawConnectionError(f"MAVSDK gRPC error: {e}")
            raise
        finally:
            self._pending_mavsdk_futures.discard(future)

    async def _connect_async(self):
        """
        Asynchronously connect to the MAVSDK system and start telemetry tasks.
        """
        self._system = System(port=self._mavsdk_server_port)
        await asyncio.wait_for(
            self._system.connect(system_address=self._connection_string),
            timeout=CONNECTION_TIMEOUT_S,
        )
        # Wait for connection state with timeout
        start = time.time()
        async for state in self._system.core.connection_state():
            if state.is_connected:
                self._has_heartbeat = True
                break
            if time.time() - start > CONNECTION_TIMEOUT_S:
                raise ConnectionTimeoutError(
                    CONNECTION_TIMEOUT_S,
                    message="Connection established but no heartbeat received within timeout",
                )

        # Start telemetry subscriptions
        await self._start_telemetry()

        # Fetch vehicle info
        await self._fetch_vehicle_info()

    async def _resilient_telemetry_task(self, name, coro_factory):
        """Wrap a telemetry subscription in retry logic."""
        retry_count = 0
        max_retries = 10
        while self._running.get() and retry_count < max_retries:
            try:
                await coro_factory()
            except asyncio.CancelledError:
                return
            except Exception as e:
                retry_count += 1
                logger.warning(
                    f"Telemetry stream '{name}' failed (attempt {retry_count}): {e}"
                )
                if retry_count < max_retries:
                    await asyncio.sleep(min(2 ** retry_count, 30))
                else:
                    logger.error(
                        f"Telemetry stream '{name}' failed after {max_retries} retries"
                    )

    async def _start_telemetry(self):
        """
        Spawn background tasks to subscribe to various telemetry streams.
        """

        async def _position_update():
            async for position in self._system.telemetry.position():
                self._position_lat.set(position.latitude_deg)
                self._position_lon.set(position.longitude_deg)
                self._position_alt.set(position.relative_altitude_m)
                self._position_abs_alt.set(position.absolute_altitude_m)

        async def _attitude_update():
            async for attitude in self._system.telemetry.attitude_euler():
                new_att = _AttitudeCompat()
                new_att.roll = math.radians(attitude.roll_deg)
                new_att.pitch = math.radians(attitude.pitch_deg)
                new_att.yaw = math.radians(attitude.yaw_deg)
                self._attitude_val.set(new_att)
                self._heading_deg.set(attitude.yaw_deg % 360)

        async def _velocity_update():
            async for velocity in self._system.telemetry.velocity_ned():
                self._velocity_ned.set(
                    [velocity.north_m_s, velocity.east_m_s, velocity.down_m_s]
                )

        async def _gps_update():
            async for gps_info in self._system.telemetry.gps_info():
                new_gps = _GPSInfoCompat()
                new_gps.satellites_visible = gps_info.num_satellites
                new_gps.fix_type = gps_info.fix_type.value
                self._gps_val.set(new_gps)

        async def _battery_update():
            async for battery in self._system.telemetry.battery():
                new_bat = _BatteryCompat()
                new_bat.voltage = battery.voltage_v
                new_bat.level = int(battery.remaining_percent)
                self._battery_val.set(new_bat)

        async def _flight_mode_update():
            async for mode in self._system.telemetry.flight_mode():
                self._mode.set(mode.name)

        async def _armed_update():
            async for armed in self._system.telemetry.armed():
                old_armed = self._armed_state.get()
                self._armed_state.set(armed)
                self._armed_telemetry_received.set(True)
                if armed and not old_armed:
                    self._last_arm_time.set(time.time())

        async def _health_update():
            async for health in self._system.telemetry.health():
                self._health_val.set(health) # Used to provide information when arming fails
                self._is_armable_state.set(
                    health.is_global_position_ok
                    and health.is_home_position_ok
                    and health.is_armable
                )

        async def _home_update():
            async for home in self._system.telemetry.home():
                self._home_position.set(util.Coordinate(
                    home.latitude_deg,
                    home.longitude_deg,
                    home.relative_altitude_m,
                ))
                self._home_abs_alt.set(home.absolute_altitude_m)

        # Start all telemetry tasks
        telemetry_defs = [
            ("position", lambda: _position_update()),
            ("attitude", lambda: _attitude_update()),
            ("velocity", lambda: _velocity_update()),
            ("gps", lambda: _gps_update()),
            ("battery", lambda: _battery_update()),
            ("flight_mode", lambda: _flight_mode_update()),
            ("armed", lambda: _armed_update()),
            ("health", lambda: _health_update()),
            ("home", lambda: _home_update()),
        ]

        for name, factory in telemetry_defs:
            task = asyncio.create_task(
                self._resilient_telemetry_task(name, factory)
            )
            self._telemetry_tasks.append(task)

    async def _fetch_vehicle_info(self):
        """
        Fetch static vehicle information like firmware version once.
        """
        try:
            version = await self._system.info.get_version()
            self._autopilot_info.major = version.flight_sw_major
            self._autopilot_info.minor = version.flight_sw_minor
            self._autopilot_info.patch = version.flight_sw_patch
        except Exception as e:
            logger.debug("Could not fetch vehicle version info: %s", e)

    # Properties - maintaining original API
    @property
    def connected(self) -> bool:
        """
        True if receiving heartbeats, False otherwise
        """
        return self._has_heartbeat

    @property
    def position(self) -> util.Coordinate:
        """
        Get the current position of the Vehicle as a `util.Coordinate`
        """
        return util.Coordinate(
            self._position_lat.get(),
            self._position_lon.get(),
            self._position_alt.get(),
        )

    @property
    def home_amsl(self) -> float:
        """
        Get the absolute altitude (AMSL) of the home position in meters.

        Returns:
            float: Altitude Above Mean Sea Level.
        """
        return self._home_abs_alt.get()

    @property
    def battery(self) -> _BatteryCompat:
        """
        Get the status of the battery. Returns object with `voltage`, `current`, and `level`.
        """
        return self._battery_val.get()

    @property
    def gps(self) -> _GPSInfoCompat:
        """
        Get the current GPS status.
        Exposes the `fix_type` (0-1: no fix, 2: 2d fix, 3: 3d fix),
        and number of `satellites_visible`.
        """
        return self._gps_val.get()

    @property
    def armed(self) -> bool:
        return self._armed_state.get()

    @property
    def home_coords(self) -> Optional[util.Coordinate]:
        """
        Get the home location from MAVLink telemetry.
        Returns the autopilot's home position, or falls back to _home_location if not available.
        """
        home = self._home_position.get()
        if home is not None:
            return home
        return self._home_location

    @property
    def heading(self) -> float:
        return self._heading_deg.get()

    @property
    def velocity(self) -> util.VectorNED:
        return util.VectorNED(*self._velocity_ned.get())

    @property
    def autopilot_info(self) -> _VersionCompat:
        return self._autopilot_info

    @property
    def attitude(self) -> _AttitudeCompat:
        """
        Attitude of the vehicle, all values in radians.
        - pitch/roll are horizon-relative
        - yaw is world relative (north=0)
        """
        return self._attitude_val.get()

    def debug_dump(self) -> str:
        """
        Generate a CSV-formatted string of current vehicle state.

        Returns:
            str: Comma-separated values of all tracked vehicle properties.
        """
        nav_controller_output = (
            None,
            None,
            None,
            None,
            None,
            None,
            None,
            None,
        )
        if self._last_nav_controller_output is not None:
            nav_controller_output = self._last_nav_controller_output

        mission_item_output = (None, None, None, None)
        if self._last_mission_item_int is not None:
            mission_item_output = self._last_mission_item_int

        props = [
            time.time_ns(),
            self.armed,
            self.attitude,
            self.autopilot_info,
            self.battery,
            self.gps,
            self.heading,
            self.home_coords,
            self.position,
            self.velocity,
            self._mode.get(),
            nav_controller_output,
            mission_item_output,
        ]

        return ",".join(map(str, props))

    # Internal logic
    def _internal_update_loop(self):
        """
        Background loop for periodic internal state maintenance and logging.
        """
        while self._running.get():
            self._internal_update()
            time.sleep(INTERNAL_UPDATE_DELAY_S)

    def _internal_update(self):
        """
        Perform a single iteration of internal updates.

        Handles verbose logging if enabled.
        """
        # Called regularly at some given frequency by an internal update loop
        if self._verbose_logging and (
            self._verbose_logging_last_log_time + self._verbose_logging_delay
            < time.time()
        ):
            with self._verbose_log_lock:
                if self._verbose_logging_file_writer is None:
                    self._verbose_logging_file_writer = open(
                        f"{self._verbose_logging_file_prefix}_{time.time_ns()}.csv",
                        "w",
                    )
                    # Write header row (F4)
                    self._verbose_logging_file_writer.write(
                        "timestamp_ns,armed,attitude,autopilot_info,battery,gps,"
                        "heading,home_coords,position,velocity,mode,nav_output,"
                        "mission_item\n"
                    )
                log_output = self.debug_dump()
                self._verbose_logging_file_writer.write(f"{log_output}\n")
                self._verbose_logging_last_log_time = time.time()

    # Special things
    def done_moving(self) -> bool:
        """
        See if the vehicle is ready to move (i.e. if the last movement command
        has been completed). Also makes sure that the vehicle is connected and
        that we haven't aborted.
        """
        if not self.connected or self._aborted:
            return False

        if hasattr(self._ready_to_move, "__func__"):
            return self._ready_to_move.__func__(self)
        return self._ready_to_move(self)

    async def await_ready_to_move(self) -> None:
        """
        Block and wait until the vehicle is ready for the next command.

        Ensures the vehicle is armed and the previous movement has finished.
        """
        if not self.armed:
            await self._initialize_postarm()

        await wait_for_condition(
            self.done_moving,
            timeout=DEFAULT_GOTO_TIMEOUT_S,
            poll_interval=POLLING_DELAY_S,
            timeout_message=f"Vehicle did not report done_moving within {DEFAULT_GOTO_TIMEOUT_S}s",
        )

    def _abort(self):
        """
        Trigger an abort of the current operation if it is marked as abortable.
        """
        if self._abortable:
            # log_to_oeo is blocking, run in thread (E6)
            threading.Thread(
                target=AERPAW_Platform.log_to_oeo,
                args=("[aerpawlib] Aborted.",),
                daemon=True,
            ).start()
            self._abortable = False
            self._aborted = True

    # Verbs
    def close(self) -> None:
        """
        Clean up the `Vehicle` object/any state
        """
        if self._closed:
            return
        self._closed = True
        logger.debug("Closing vehicle connection...")
        self._running.set(False)

        # Cancel pending MAVSDK operations
        for future in list(self._pending_mavsdk_futures):
            try:
                future.cancel()
            except Exception as e:
                logger.debug("Error cancelling MAVSDK future: %s", e)

        # Cancel telemetry tasks on their own event loop (thread-safe)
        if self._mavsdk_loop is not None and self._mavsdk_loop.is_running():
            for task in self._telemetry_tasks:
                try:
                    self._mavsdk_loop.call_soon_threadsafe(task.cancel)
                except RuntimeError:
                    pass

        # Stop MAVSDK loop (only if it's still running)
        if self._mavsdk_loop is not None and self._mavsdk_loop.is_running():
            try:
                self._mavsdk_loop.call_soon_threadsafe(self._mavsdk_loop.stop)
            except RuntimeError:
                pass

        # Close verbose log writer under the same lock the update loop uses
        with self._verbose_log_lock:
            if self._verbose_logging_file_writer is not None:
                self._verbose_logging_file_writer.close()
                self._verbose_logging_file_writer = None

        if hasattr(self, "_mavsdk_thread") and self._mavsdk_thread.is_alive():
            self._mavsdk_thread.join(timeout=5.0)
            if self._mavsdk_thread.is_alive():
                logger.warning(
                    "MAVSDK thread did not exit within 5s; process will exit"
                )

        # Clear system reference to help garbage collection release the gRPC server
        self._system = None
        self._mavsdk_loop = None

        logger.info("Vehicle connection closed")

    async def set_armed(self, value: bool) -> None:
        """
        Arm or disarm this vehicle, and wait for it to be armed (if possible).

        Args:
            value: True to arm, False to disarm

        Raises:
            NotArmableError: If attempting to arm when vehicle is not ready
            ArmError: If arming fails
            DisarmError: If disarming fails
        """
        logger.debug(f"set_armed({value}) called")
        if not self._is_armable_state.get() and value:
            health_summary = self._get_health_status_summary()
            logger.error(f"Cannot arm: vehicle not in armable state. Status: {health_summary}")
            raise NotArmableError(f"Vehicle not armable. Status: {health_summary}")

        try:
            if value:
                logger.debug("Sending arm command...")
                await self._run_on_mavsdk_loop(self._system.action.arm())
            else:
                logger.debug("Sending disarm command...")
                await self._run_on_mavsdk_loop(self._system.action.disarm())

            # Wait for arm state to match
            await wait_for_condition(
                lambda: self._armed_state.get() == value,
                timeout=ARMABLE_TIMEOUT_S,
                poll_interval=POLLING_DELAY_S,
                timeout_message=f"Arm/disarm did not complete within {ARMABLE_TIMEOUT_S}s",
            )
            logger.debug(
                f"Vehicle {'armed' if value else 'disarmed'} successfully"
            )
        except ActionError as e:
            logger.error(f"Arm/disarm failed: {e}")
            if value:
                raise ArmError(str(e), original_error=e)
            else:
                raise DisarmError(str(e), original_error=e)

    def _initialize_prearm(self, should_postarm_init: bool) -> None:
        """
        Wait for pre-arm conditions (GPS fix, etc.) to be satisfied.

        Args:
            should_postarm_init (bool): Whether to perform post-arm initialization later.
        """
        logger.debug(
            f"_initialize_prearm(should_postarm_init={should_postarm_init}) called"
        )
        start = time.time()
        last_log = 0.0
        while not self._is_armable_state.get():
            if time.time() - start > ARMABLE_TIMEOUT_S:
                logger.warning(
                    f"Timeout waiting for armable state ({ARMABLE_TIMEOUT_S}s). "
                    f"Final status: {self._get_health_status_summary()}"
                )
                break
            # Log status at configured interval
            if time.time() - last_log > ARMABLE_STATUS_LOG_INTERVAL_S:
                logger.debug(
                    f"Waiting for armable state... Status: {self._get_health_status_summary()}"
                )
                last_log = time.time()
            time.sleep(POLLING_DELAY_S)

        if self._is_armable_state.get():
            logger.debug("Vehicle is armable")
        else:
            logger.warning(
                f"Vehicle may not be fully ready to arm. Status: {self._get_health_status_summary()}"
            )

        self._should_postarm_init = should_postarm_init

    async def _initialize_postarm(self) -> None:
        """
        Generic pre-mission manipulation of the vehicle into a state that is
        acceptable. MUST be called before anything else.

        In AERPAW environment: waits for safety pilot to arm
        In standalone/SITL: auto-arms the vehicle
        """
        if not self._should_postarm_init:
            logger.debug("Skipping postarm init (disabled)")
            return

        logger.debug("_initialize_postarm() called")

        # Check if we're in AERPAW environment
        is_aerpaw = AERPAW_Platform._is_aerpaw_environment()

        if is_aerpaw:
            # In AERPAW environment, wait for safety pilot to arm
            AERPAW_Platform.log_to_oeo(
                "[aerpawlib] Guided command attempted. Waiting for safety pilot to arm"
            )
            logger.info("Waiting for safety pilot to arm vehicle...")

            await wait_for_condition(
                lambda: self._is_armable_state.get(),
                poll_interval=POLLING_DELAY_S,
            )
            await wait_for_condition(
                lambda: self.armed, poll_interval=POLLING_DELAY_S
            )
        else:
            # In standalone/SITL, auto-arm the vehicle
            logger.info("Standalone mode: auto-arming vehicle...")

            # Wait for armable state with timeout
            try:
                await wait_for_condition(
                    lambda: self._is_armable_state.get(),
                    timeout=CONNECTION_TIMEOUT_S,
                    poll_interval=POLLING_DELAY_S,
                    timeout_message=f"Vehicle not armable after {CONNECTION_TIMEOUT_S}s - check GPS and pre-flight conditions",
                )
            except TimeoutError as e:
                health_summary = self._get_health_status_summary()
                msg = f"{str(e)}. Status: {health_summary}"
                logger.error(msg)
                raise NotArmableError(msg)

            # Wait for GPS 3D fix explicitly. MAVSDK's is_global_position_ok can report
            # true before the autopilot has valid position for GUIDED mode (e.g. when
            # SITL is still starting up). Without this, takeoff fails with
            # "Mode change to GUIDED failed: requires position".
            logger.debug("Waiting for GPS 3D fix (position ready for GUIDED)...")
            try:
                await wait_for_condition(
                    lambda: self.gps.fix_type >= 3,
                    timeout=POSITION_READY_TIMEOUT_S,
                    poll_interval=POLLING_DELAY_S,
                    timeout_message=f"No GPS 3D fix after {POSITION_READY_TIMEOUT_S}s - ensure SITL/hardware is fully started",
                )
            except TimeoutError as e:
                health_summary = self._get_health_status_summary()
                msg = f"{str(e)}. Status: {health_summary}"
                logger.error(msg)
                raise NotArmableError(msg)

            logger.debug("Vehicle is armable, sending arm command...")

            # Arm the vehicle
            await self.set_armed(True)
            logger.info("Vehicle armed successfully")

        await asyncio.sleep(ARMING_SEQUENCE_DELAY_S)

        self._abortable = True

        # Wait for home position to be populated from telemetry (up to 5s)
        logger.debug("Waiting for auto-set home position...")
        await wait_for_condition(
            lambda: self._home_position.get() is not None,
            timeout=5.0,
            poll_interval=POLLING_DELAY_S,
            timeout_message="Home position not available within 5s",
        )

        self._home_location = self.home_coords
        logger.debug(f"Home location set to: {self._home_location}")

    async def goto_coordinates(
        self,
        coordinates: util.Coordinate,
        tolerance: float = DEFAULT_POSITION_TOLERANCE_M,
        target_heading: Optional[float] = None,
    ) -> None:
        """
        Make the vehicle go to provided coordinates.

        Args:
            coordinates: Target position
            tolerance: Distance in meters to consider destination reached
            target_heading: Optional heading to maintain during movement

        Raises:
            NotImplementedForVehicleError: Generic vehicles cannot navigate
        """
        raise NotImplementedForVehicleError(
            "goto_coordinates", "generic Vehicle"
        )

    async def set_velocity(
        self,
        velocity_vector: util.VectorNED,
        global_relative: bool = True,
        duration: Optional[float] = None,
    ) -> None:
        """
        Set a drone's velocity that it will use for `duration` seconds.

        Args:
            velocity_vector: Velocity in NED frame
            global_relative: If True, vector is in global frame; if False, in body frame
            duration: How long to maintain velocity (None = until changed)

        Raises:
            NotImplementedForVehicleError: Generic vehicles cannot set velocity
        """
        raise NotImplementedForVehicleError("set_velocity", "generic Vehicle")

    async def set_groundspeed(self, velocity: float) -> None:
        """
        Set a vehicle's cruise velocity as used by the autopilot.

        Args:
            velocity: Groundspeed in m/s

        Raises:
            ValueError: If velocity is out of acceptable range
        """
        # Note: speed bounds (min_speed / max_speed) are enforced by the
        # SafetyCheckerServer via validate_change_speed_command. No redundant
        # constant-based check here.
        logger.debug(f"set_groundspeed({velocity}) called")
        try:
            await self._run_on_mavsdk_loop(
                self._system.action.set_maximum_speed(velocity)
            )
            logger.debug(f"Maximum speed set to {velocity} m/s")
        except ActionError:
            logger.debug("set_maximum_speed not supported by autopilot")
            pass  # Not all autopilots support this

    async def _stop(self) -> None:
        """
        Stop any background movement tasks.
        """
        self._ready_to_move = lambda _: True

    def _get_health_status_summary(self) -> str:
        """
        Get a human-readable summary of the current vehicle health status.
        """
        health = self._health_val.get()
        if health is None:
            return "UNKNOWN (no telemetry)"

        summary = (
            f"Global: {'OK' if health.is_global_position_ok else 'FAIL'}, "
            f"Home: {'OK' if health.is_home_position_ok else 'FAIL'}, "
            f"Local: {'OK' if health.is_local_position_ok else 'FAIL'}, "
            f"Armable: {'OK' if health.is_armable else 'FAIL'}, "
            f"Gyro: {'OK' if health.is_gyrometer_calibration_ok else 'FAIL'}, "
            f"Accel: {'OK' if health.is_accelerometer_calibration_ok else 'FAIL'}, "
            f"Mag: {'OK' if health.is_magnetometer_calibration_ok else 'FAIL'}, "
            f"Fix: {self.gps.fix_type} ({self.gps.satellites_visible} sats)"
        )
        return summary

