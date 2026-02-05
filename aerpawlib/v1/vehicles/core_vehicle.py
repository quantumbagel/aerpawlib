"""
Core logic surrounding the various `Vehicle`s available to aerpawlib user
scripts.

This is the MAVSDK-based implementation of the v1 API, maintaining full
backward compatibility with the original DroneKit-based interface.

@author: Julian Reder (quantumbagel)
"""

import asyncio
import logging
import math
import time
import threading
from typing import Callable, Optional, List

from mavsdk import System
from mavsdk.action import ActionError


from aerpawlib.v1 import util
from aerpawlib.v1.aerpaw import AERPAW_Platform
from aerpawlib.v1.constants import (
    POLLING_DELAY_S,
    ARMING_SEQUENCE_DELAY_S,
    INTERNAL_UPDATE_DELAY_S,
    CONNECTION_TIMEOUT_S,
    ARMABLE_TIMEOUT_S,
    ARMABLE_STATUS_LOG_INTERVAL_S,
    DEFAULT_POSITION_TOLERANCE_M,
    VERBOSE_LOG_FILE_PREFIX,
    VERBOSE_LOG_DELAY_S,
)
from aerpawlib.v1.exceptions import (
    ConnectionTimeoutError,
    ArmError,
    DisarmError,
    NotArmableError,
    NotImplementedForVehicleError,
)
from aerpawlib.v1.helpers import (
    wait_for_condition,
    validate_speed,
    ThreadSafeValue,
)

# Configure module logger
logger = logging.getLogger(__name__)


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

    Safety Initialization Patterns:
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

    # Safety initialization state
    _initialization_complete: bool = False
    _skip_init: bool = False  # Set via CLI --skip-init flag
    _skip_rtl: bool = False  # Set via CLI --skip-rtl flag

    # Connection/heartbeat tracking
    _last_heartbeat_time: float = 0.0

    def __init__(self, connection_string: str):
        """
        Initialize the vehicle and connect to the autopilot.

        Args:
            connection_string (str): MAVLink connection string (e.g., 'udp://:14540').

        Raises:
            ConnectionTimeoutError: If connection cannot be established within timeout.
        """
        self._connection_string = connection_string
        self._system = None
        self._has_heartbeat = False
        self._should_postarm_init = True
        self._mission_start_time: Optional[float] = None

        # Safety initialization state
        self._initialization_complete = False
        self._skip_init = False
        self._skip_rtl = False
        self._was_already_armed_on_connect = False
        self._last_heartbeat_time = 0.0

        # Internal state tracking (use ThreadSafeValue for thread-safe access)
        self._armed_state = ThreadSafeValue(False)
        self._is_armable_state = ThreadSafeValue(False)
        self._position_lat = ThreadSafeValue(0.0)
        self._position_lon = ThreadSafeValue(0.0)
        self._position_alt = ThreadSafeValue(0.0)
        self._position_abs_alt = ThreadSafeValue(0.0)
        self._heading_deg = ThreadSafeValue(0.0)
        self._velocity_ned = ThreadSafeValue([0.0, 0.0, 0.0])
        self._home_position: Optional[util.Coordinate] = None
        self._home_abs_alt = ThreadSafeValue(0.0)

        # Compatibility objects
        self._battery = _BatteryCompat()
        self._gps = _GPSInfoCompat()
        self._attitude = _AttitudeCompat()
        self._autopilot_info = _VersionCompat()
        self._mode = ThreadSafeValue("UNKNOWN")

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
        loop = asyncio.new_event_loop()
        self._mavsdk_loop = loop  # Store reference for thread-safe calls

        def _run_connection():
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(self._connect_async())
                # Keep the loop running for telemetry
                loop.run_forever()
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
        t = threading.Thread(target=_run_connection, daemon=True)
        t.start()

        # Wait for connection with timeout
        start = time.time()
        while not self._has_heartbeat:
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
            RuntimeError: If the MAVSDK loop is not initialized.
        """
        if self._mavsdk_loop is None:
            raise RuntimeError("MAVSDK loop not initialized")

        future = asyncio.run_coroutine_threadsafe(coro, self._mavsdk_loop)
        # Wait for result without blocking the current event loop
        while not future.done():
            await asyncio.sleep(POLLING_DELAY_S)
        return future.result()

    async def _connect_async(self):
        """
        Asynchronously connect to the MAVSDK system and start telemetry tasks.
        """
        self._system = System()
        await self._system.connect(system_address=self._connection_string)

        # Wait for connection
        async for state in self._system.core.connection_state():
            if state.is_connected:
                self._has_heartbeat = True
                break

        # Start telemetry subscriptions
        await self._start_telemetry()

        # Fetch vehicle info
        await self._fetch_vehicle_info()

    async def _start_telemetry(self):
        """
        Spawn background tasks to subscribe to various telemetry streams.
        """

        async def _position_update():
            async for position in self._system.telemetry.position():
                # print(position)
                self._position_lat.set(position.latitude_deg)
                self._position_lon.set(position.longitude_deg)
                self._position_alt.set(position.relative_altitude_m)
                self._position_abs_alt.set(position.absolute_altitude_m)

        async def _attitude_update():
            async for attitude in self._system.telemetry.attitude_euler():
                # print(attitude)
                self._attitude.roll = math.radians(attitude.roll_deg)
                self._attitude.pitch = math.radians(attitude.pitch_deg)
                self._attitude.yaw = math.radians(attitude.yaw_deg)
                self._heading_deg.set(attitude.yaw_deg % 360)

        async def _velocity_update():
            async for velocity in self._system.telemetry.velocity_ned():
                # print(velocity)
                self._velocity_ned.set(
                    [velocity.north_m_s, velocity.east_m_s, velocity.down_m_s]
                )

        async def _gps_update():
            async for gps_info in self._system.telemetry.gps_info():
                # print(gps_info)
                self._gps.satellites_visible = gps_info.num_satellites
                self._gps.fix_type = gps_info.fix_type.value

        async def _battery_update():
            async for battery in self._system.telemetry.battery():
                # print(battery)
                self._battery.voltage = battery.voltage_v
                self._battery.level = int(battery.remaining_percent)

        async def _flight_mode_update():
            async for mode in self._system.telemetry.flight_mode():
                # print(mode)
                self._mode.set(mode.name)

        async def _armed_update():
            async for armed in self._system.telemetry.armed():
                # print(armed)
                self._armed_state.set(armed)

        async def _health_update():
            async for health in self._system.telemetry.health():
                # print(health)
                self._is_armable_state.set(
                    health.is_global_position_ok
                    and health.is_home_position_ok
                    and health.is_armable
                )

        async def _home_update():
            async for home in self._system.telemetry.home():
                # print(home)
                self._home_position = util.Coordinate(
                    home.latitude_deg,
                    home.longitude_deg,
                    home.relative_altitude_m,
                )
                self._home_abs_alt.set(home.absolute_altitude_m)

        # Start all telemetry tasks
        telemetry_coros = [
            _position_update(),
            _attitude_update(),
            _velocity_update(),
            _gps_update(),
            _battery_update(),
            _flight_mode_update(),
            _armed_update(),
            _health_update(),
            _home_update(),
        ]

        for coro in telemetry_coros:
            task = asyncio.create_task(coro)
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
        except Exception:
            pass

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
        return self._battery

    @property
    def gps(self) -> _GPSInfoCompat:
        """
        Get the current GPS status.
        Exposes the `fix_type` (0-1: no fix, 2: 2d fix, 3: 3d fix),
        and number of `satellites_visible`.
        """
        return self._gps

    @property
    def armed(self) -> bool:
        return self._armed_state.get()

    @property
    def home_coords(self) -> Optional[util.Coordinate]:
        """
        Get the home location from MAVLink telemetry.
        Returns the autopilot's home position, or falls back to _home_location if not available.
        """
        if self._home_position is not None:
            return self._home_position
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
        return self._attitude

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
            if self._verbose_logging_file_writer is None:
                self._verbose_logging_file_writer = open(
                    f"{self._verbose_logging_file_prefix}_{time.time_ns()}.csv",
                    "w",
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
            self.done_moving, poll_interval=POLLING_DELAY_S
        )

    def _abort(self):
        """
        Trigger an abort of the current operation if it is marked as abortable.
        """
        if self._abortable:
            AERPAW_Platform.log_to_oeo("[aerpawlib] Aborted.")
            self._abortable = False
            self._aborted = True

    # Verbs
    def close(self) -> None:
        """
        Clean up the `Vehicle` object/any state
        """
        self._running.set(False)
        for task in self._telemetry_tasks:
            task.cancel()

        if self._mavsdk_loop and self._mavsdk_loop.is_running():
            self._mavsdk_loop.call_soon_threadsafe(self._mavsdk_loop.stop, *())

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
            logger.error("Cannot arm: vehicle not in armable state")
            raise NotArmableError()

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
                poll_interval=POLLING_DELAY_S,
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
                    f"Timeout waiting for armable state ({ARMABLE_TIMEOUT_S}s)"
                )
                break
            # Log status at configured interval
            if time.time() - last_log > ARMABLE_STATUS_LOG_INTERVAL_S:
                logger.debug(
                    f"Waiting for armable state... "
                    f"(GPS fix={self._gps.fix_type}, sats={self._gps.satellites_visible})"
                )
                last_log = time.time()
            time.sleep(POLLING_DELAY_S)

        if self._is_armable_state.get():
            logger.debug("Vehicle is armable")
        else:
            logger.warning("Vehicle may not be fully ready to arm")

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
                logger.error(str(e))
                raise NotArmableError(str(e))

            logger.debug("Vehicle is armable, sending arm command...")

            # Arm the vehicle
            await self.set_armed(True)
            logger.info("Vehicle armed successfully")

        await asyncio.sleep(ARMING_SEQUENCE_DELAY_S)

        self._abortable = True
        self._home_location = self.position
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
        validate_speed(velocity, "velocity")
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
