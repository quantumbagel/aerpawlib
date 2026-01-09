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
from typing import Callable, Optional, List, TYPE_CHECKING

try:
    from mavsdk import System
    from mavsdk.offboard import PositionNedYaw, VelocityNedYaw, OffboardError
    from mavsdk.action import ActionError
    from mavsdk.telemetry import FlightMode as MavFlightMode
    MAVSDK_AVAILABLE = True
except ImportError:
    MAVSDK_AVAILABLE = False
    System = None
    PositionNedYaw = None
    VelocityNedYaw = None
    OffboardError = Exception
    ActionError = Exception
    MavFlightMode = None

from . import util
from .aerpaw import AERPAW_Platform
from .constants import (
    POLLING_DELAY_S,
    ARMING_SEQUENCE_DELAY_S,
    INTERNAL_UPDATE_DELAY_S,
    CONNECTION_TIMEOUT_S,
    ARMABLE_TIMEOUT_S,
    ARMABLE_STATUS_LOG_INTERVAL_S,
    DEFAULT_POSITION_TOLERANCE_M,
    DEFAULT_ROVER_POSITION_TOLERANCE_M,
    DEFAULT_TAKEOFF_ALTITUDE_TOLERANCE,
    POST_TAKEOFF_STABILIZATION_S,
    HEADING_TOLERANCE_DEG,
    VELOCITY_UPDATE_DELAY_S,
    VERBOSE_LOG_FILE_PREFIX,
    VERBOSE_LOG_DELAY_S,
)
from .exceptions import (
    MAVSDKNotInstalledError,
    ConnectionTimeoutError,
    ArmError,
    DisarmError,
    TakeoffError,
    LandingError,
    NavigationError,
    VelocityError,
    RTLError,
    NotArmableError,
    NotImplementedForVehicleError,
)
from .helpers import (
    wait_for_condition,
    validate_tolerance,
    validate_altitude,
    validate_speed,
    normalize_heading,
    heading_difference,
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
    __slots__ = ('voltage', 'current', 'level')

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
    __slots__ = ('fix_type', 'satellites_visible')

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
    __slots__ = ('pitch', 'roll', 'yaw')

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
    __slots__ = ('major', 'minor', 'patch', 'release')

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
    Vehicle class for things that don't need vehicles.
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
    Overarching "generic vehicle" type. Implements all functionality, excluding
    movement commands (which are *always* vehicle specific).

    This implementation uses MAVSDK internally while maintaining the exact same
    API as the original DroneKit-based implementation.
    """
    _system: Optional[System]
    _has_heartbeat: bool

    # function used by "verb" functions to check and see if the vehicle can be
    # commanded to move. should be set to a new closure by verb functions to
    # redefine functionality
    _ready_to_move: Callable[['Vehicle'], bool] = lambda _: True

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

    def __init__(self, connection_string: str):
        if not MAVSDK_AVAILABLE:
            raise MAVSDKNotInstalledError()

        self._connection_string = connection_string
        self._system = None
        self._has_heartbeat = False
        self._should_postarm_init = True
        self._mission_start_time: Optional[float] = None

        # Internal state tracking (use ThreadSafeValue for thread-safe access)
        self._armed_state = ThreadSafeValue(False)
        self._is_armable_state = ThreadSafeValue(False)
        self._position_lat = ThreadSafeValue(0.0)
        self._position_lon = ThreadSafeValue(0.0)
        self._position_alt = ThreadSafeValue(0.0)
        self._heading_deg = ThreadSafeValue(0.0)
        self._velocity_ned = ThreadSafeValue([0.0, 0.0, 0.0])
        self._home_position: Optional[util.Coordinate] = None

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
        """Synchronous connection for compatibility with original API."""
        loop = asyncio.new_event_loop()
        self._mavsdk_loop = loop  # Store reference for thread-safe calls

        def _run_connection():
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._connect_async())
            # Keep the loop running for telemetry
            loop.run_forever()

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
        update_thread = threading.Thread(target=self._internal_update_loop, daemon=True)
        update_thread.start()

    async def _run_on_mavsdk_loop(self, coro):
        """
        Run a coroutine on the MAVSDK event loop from any other loop.
        This is necessary because gRPC futures are bound to the loop they were created on.
        """
        if self._mavsdk_loop is None:
            raise RuntimeError("MAVSDK loop not initialized")

        future = asyncio.run_coroutine_threadsafe(coro, self._mavsdk_loop)
        # Wait for result without blocking the current event loop
        while not future.done():
            await asyncio.sleep(POLLING_DELAY_S)
        return future.result()

    async def _connect_async(self):
        """Async connection and telemetry setup."""
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
        """Start background telemetry update tasks."""

        async def _position_update():
            async for position in self._system.telemetry.position():
                self._position_lat.set(position.latitude_deg)
                self._position_lon.set(position.longitude_deg)
                self._position_alt.set(position.relative_altitude_m)

        async def _attitude_update():
            async for attitude in self._system.telemetry.attitude_euler():
                self._attitude.roll = math.radians(attitude.roll_deg)
                self._attitude.pitch = math.radians(attitude.pitch_deg)
                self._attitude.yaw = math.radians(attitude.yaw_deg)
                self._heading_deg.set(attitude.yaw_deg % 360)

        async def _velocity_update():
            async for velocity in self._system.telemetry.velocity_ned():
                self._velocity_ned.set([
                    velocity.north_m_s,
                    velocity.east_m_s,
                    velocity.down_m_s
                ])

        async def _gps_update():
            async for gps_info in self._system.telemetry.gps_info():
                self._gps.satellites_visible = gps_info.num_satellites
                self._gps.fix_type = gps_info.fix_type.value

        async def _battery_update():
            async for battery in self._system.telemetry.battery():
                self._battery.voltage = battery.voltage_v
                self._battery.level = int(battery.remaining_percent)

        async def _flight_mode_update():
            async for mode in self._system.telemetry.flight_mode():
                self._mode.set(mode.name)

        async def _armed_update():
            async for armed in self._system.telemetry.armed():
                self._armed_state.set(armed)

        async def _health_update():
            async for health in self._system.telemetry.health():
                self._is_armable_state.set(
                    health.is_global_position_ok and
                    health.is_home_position_ok and
                    health.is_armable
                )

        async def _home_update():
            async for home in self._system.telemetry.home():
                self._home_position = util.Coordinate(
                    home.latitude_deg,
                    home.longitude_deg,
                    home.relative_altitude_m
                )

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
        """Fetch static vehicle information."""
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
            self._position_alt.get()
        )

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
        Dump various properties collected by this vehicle for logging/debug purposes.
        """
        nav_controller_output = (None, None, None, None, None, None, None, None)
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
        while self._running.get():
            self._internal_update()
            time.sleep(INTERNAL_UPDATE_DELAY_S)

    def _internal_update(self):
        # Called regularly at some given frequency by an internal update loop
        if self._verbose_logging and (self._verbose_logging_last_log_time + self._verbose_logging_delay < time.time()):
            if self._verbose_logging_file_writer is None:
                self._verbose_logging_file_writer = open(
                    f"{self._verbose_logging_file_prefix}_{time.time_ns()}.csv", 'w'
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
        Helper function that blocks execution and waits for the vehicle to
        finish the current action/movement.
        """
        if not self.armed:
            await self._initialize_postarm()

        await wait_for_condition(
            self.done_moving,
            poll_interval=POLLING_DELAY_S
        )

    def _abort(self):
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
                poll_interval=POLLING_DELAY_S
            )
            logger.debug(f"Vehicle {'armed' if value else 'disarmed'} successfully")
        except ActionError as e:
            logger.error(f"Arm/disarm failed: {e}")
            if value:
                raise ArmError(str(e), original_error=e)
            else:
                raise DisarmError(str(e), original_error=e)

    def _initialize_prearm(self, should_postarm_init: bool) -> None:
        logger.debug(f"_initialize_prearm(should_postarm_init={should_postarm_init}) called")
        start = time.time()
        last_log = 0.0
        while not self._is_armable_state.get():
            if time.time() - start > ARMABLE_TIMEOUT_S:
                logger.warning(f"Timeout waiting for armable state ({ARMABLE_TIMEOUT_S}s)")
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
            AERPAW_Platform.log_to_oeo("[aerpawlib] Guided command attempted. Waiting for safety pilot to arm")
            logger.info("Waiting for safety pilot to arm vehicle...")

            await wait_for_condition(
                lambda: self._is_armable_state.get(),
                poll_interval=POLLING_DELAY_S
            )
            await wait_for_condition(
                lambda: self.armed,
                poll_interval=POLLING_DELAY_S
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
                    timeout_message=f"Vehicle not armable after {CONNECTION_TIMEOUT_S}s - check GPS and pre-flight conditions"
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
        target_heading: Optional[float] = None
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
        raise NotImplementedForVehicleError("goto_coordinates", "generic Vehicle")

    async def set_velocity(
        self,
        velocity_vector: util.VectorNED,
        global_relative: bool = True,
        duration: Optional[float] = None
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
            await self._run_on_mavsdk_loop(self._system.action.set_maximum_speed(velocity))
            logger.debug(f"Maximum speed set to {velocity} m/s")
        except ActionError:
            logger.debug("set_maximum_speed not supported by autopilot")
            pass  # Not all autopilots support this

    async def _stop(self) -> None:
        """
        Internal utility to stop any movement being run in the background.
        """
        self._ready_to_move = lambda _: True


# noinspection PyUnusedLocal
class Drone(Vehicle):
    """
    Drone vehicle type. Implements all functionality that AERPAW's drones
    expose to user scripts, which includes basic movement control (going to
    coords, turning, landing).
    """

    _velocity_loop_active: bool = False

    async def set_heading(
        self,
        heading: Optional[float],
        blocking: bool = True,
        lock_in: bool = True
    ) -> None:
        """
        Set the heading of the vehicle (in absolute deg).

        To turn a relative # of degrees, you can do something like
        `drone.set_heading(drone.heading + x)`

        Pass in `None` to make the drone return to ardupilot's default heading
        behavior (usually facing the direction of movement)

        Args:
            heading: Target heading in degrees (0-360), or None to clear locked heading
            blocking: If True, wait for heading to be reached
            lock_in: If True, maintain this heading during subsequent movements
        """
        logger.debug(f"set_heading(heading={heading}, blocking={blocking}, lock_in={lock_in}) called")
        if blocking:
            await self.await_ready_to_move()

        if heading is None:
            logger.debug("Clearing locked heading, returning to default behavior")
            self._current_heading = None
            return

        heading = normalize_heading(heading)
        if lock_in:
            self._current_heading = heading
        if not blocking:
            return

        logger.debug(f"Turning to heading {heading}° (current: {self.heading}°)")
        # Use offboard mode to control yaw
        try:
            await self._run_on_mavsdk_loop(self._system.offboard.set_position_ned(
                PositionNedYaw(0, 0, -self._position_alt.get(), heading)
            ))
            try:
                await self._run_on_mavsdk_loop(self._system.offboard.start())
            except OffboardError:
                pass  # Already in offboard mode

            def _pointed_at_heading(self) -> bool:
                return heading_difference(heading, self.heading) <= HEADING_TOLERANCE_DEG

            self._ready_to_move = _pointed_at_heading

            await wait_for_condition(
                lambda: _pointed_at_heading(self),
                poll_interval=POLLING_DELAY_S
            )

            await self._run_on_mavsdk_loop(self._system.offboard.stop())
        except (OffboardError, ActionError):
            # Fallback - just update internal heading
            pass

    async def takeoff(
        self,
        target_alt: float,
        min_alt_tolerance: float = DEFAULT_TAKEOFF_ALTITUDE_TOLERANCE
    ) -> None:
        """
        Make the drone take off to a specific altitude, and blocks until the
        drone has reached that altitude.

        Args:
            target_alt: Target altitude in meters AGL
            min_alt_tolerance: Fraction of target altitude to consider takeoff complete (0.0-1.0)

        Raises:
            ValueError: If target_alt is out of acceptable range
            TakeoffError: If takeoff command fails
        """
        validate_altitude(target_alt, "target_alt")

        logger.debug(f"takeoff(target_alt={target_alt}, min_alt_tolerance={min_alt_tolerance}) called")
        await self.await_ready_to_move()

        # wait_for_throttle is ignored in MAVSDK implementation
        # as we don't have direct RC channel access

        if self._mission_start_time is None:
            self._mission_start_time = time.time()

        try:
            logger.debug(f"Setting takeoff altitude to {target_alt}m")
            await self._run_on_mavsdk_loop(self._system.action.set_takeoff_altitude(target_alt))
            logger.debug("Sending takeoff command...")
            await self._run_on_mavsdk_loop(self._system.action.takeoff())

            def taken_off(self) -> bool:
                return self.position.alt >= target_alt * min_alt_tolerance

            self._ready_to_move = taken_off

            logger.debug(f"Waiting to reach altitude {target_alt * min_alt_tolerance}m...")
            await wait_for_condition(
                lambda: taken_off(self),
                poll_interval=POLLING_DELAY_S
            )

            logger.debug(f"Reached target altitude, current alt: {self.position.alt}m")
            # Post-takeoff stabilization delay
            await asyncio.sleep(POST_TAKEOFF_STABILIZATION_S)
            logger.debug("Takeoff complete")
        except ActionError as e:
            logger.error(f"Takeoff failed: {e}")
            raise TakeoffError(str(e), original_error=e)

    async def land(self) -> None:
        """
        Land the drone at its current position and block while waiting for it
        to be disarmed.

        Raises:
            LandingError: If landing command fails
        """
        logger.debug("land() called")
        await self.await_ready_to_move()

        self._abortable = False

        try:
            logger.debug("Sending land command...")
            await self._run_on_mavsdk_loop(self._system.action.land())

            self._ready_to_move = lambda _: False

            logger.debug("Waiting for vehicle to disarm...")
            await wait_for_condition(
                lambda: not self.armed,
                poll_interval=POLLING_DELAY_S
            )
            logger.debug("Landing complete, vehicle disarmed")
        except ActionError as e:
            logger.error(f"Land failed: {e}")
            raise LandingError(str(e), original_error=e)

    async def return_to_launch(self) -> None:
        """
        Command the drone to return to launch (home) position using the
        autopilot's built-in RTL mode. This will fly back to the MAVLink
        home position and land automatically.

        Raises:
            RTLError: If return to launch command fails
        """
        logger.debug("return_to_launch() called")
        await self.await_ready_to_move()

        self._abortable = False

        try:
            logger.debug("Sending RTL command...")
            await self._run_on_mavsdk_loop(self._system.action.return_to_launch())

            self._ready_to_move = lambda _: False

            logger.debug("Waiting for vehicle to complete RTL and disarm...")
            await wait_for_condition(
                lambda: not self.armed,
                poll_interval=POLLING_DELAY_S
            )
            logger.debug("RTL complete, vehicle disarmed")
        except ActionError as e:
            logger.error(f"RTL failed: {e}")
            raise RTLError(str(e), original_error=e)

    async def goto_coordinates(
        self,
        coordinates: util.Coordinate,
        tolerance: float = DEFAULT_POSITION_TOLERANCE_M,
        target_heading: Optional[float] = None
    ) -> None:
        """
        Make the vehicle go to provided coordinates.

        Args:
            coordinates: Target position
            tolerance: Distance in meters to consider destination reached
            target_heading: Optional heading to maintain during movement

        Raises:
            ValueError: If tolerance is out of acceptable range
            NavigationError: If navigation command fails
        """
        validate_tolerance(tolerance, "tolerance")

        logger.debug(f"goto_coordinates(lat={coordinates.lat}, lon={coordinates.lon}, alt={coordinates.alt}, tolerance={tolerance}, heading={target_heading}) called")
        if target_heading is not None:
            await self.set_heading(target_heading)

        await self.await_ready_to_move()
        await self._stop()

        self._ready_to_move = lambda self: False

        if self._current_heading is not None:
            heading = self._current_heading
        else:
            heading = self.position.bearing(coordinates)
        await self.set_heading(heading, lock_in=False, blocking=False)

        try:
            # Use goto_location action
            target_alt = coordinates.alt + (self._home_location.alt if self._home_location else 0)
            logger.debug(f"Navigating to: lat={coordinates.lat}, lon={coordinates.lon}, alt={target_alt}, heading={heading}")
            await self._run_on_mavsdk_loop(self._system.action.goto_location(
                coordinates.lat,
                coordinates.lon,
                target_alt,
                heading if not math.isnan(heading) else 0
            ))

            def at_coords(self) -> bool:
                return coordinates.distance(self.position) <= tolerance

            self._ready_to_move = at_coords

            logger.debug(f"Waiting to reach destination (tolerance={tolerance}m)...")
            await wait_for_condition(
                lambda: at_coords(self),
                poll_interval=POLLING_DELAY_S
            )
            logger.debug(f"Arrived at destination, distance: {coordinates.distance(self.position)}m")
        except ActionError as e:
            logger.error(f"Goto failed: {e}")
            raise NavigationError(str(e), original_error=e)

    async def set_velocity(
        self,
        velocity_vector: util.VectorNED,
        global_relative: bool = True,
        duration: Optional[float] = None
    ) -> None:
        """
        Set a drone's velocity that it will use for `duration` seconds.

        Args:
            velocity_vector: Velocity in NED frame (m/s)
            global_relative: If True, vector is in global frame; if False, in body frame
            duration: How long to maintain velocity (None = until changed)

        Raises:
            VelocityError: If velocity command fails
        """
        logger.debug(f"set_velocity(N={velocity_vector.north}, E={velocity_vector.east}, D={velocity_vector.down}, global_relative={global_relative}, duration={duration}) called")
        await self.await_ready_to_move()

        self._velocity_loop_active = False
        await asyncio.sleep(POLLING_DELAY_S)

        if not global_relative:
            velocity_vector = velocity_vector.rotate_by_angle(-self.heading)
            logger.debug(f"Rotated velocity to global frame: N={velocity_vector.north}, E={velocity_vector.east}, D={velocity_vector.down}")

        yaw = self._current_heading if self._current_heading is not None else self.heading

        try:
            logger.debug(f"Setting velocity NED: N={velocity_vector.north}, E={velocity_vector.east}, D={velocity_vector.down}, yaw={yaw}")
            await self._run_on_mavsdk_loop(self._system.offboard.set_velocity_ned(
                VelocityNedYaw(
                    velocity_vector.north,
                    velocity_vector.east,
                    velocity_vector.down,
                    yaw
                )
            ))

            try:
                await self._run_on_mavsdk_loop(self._system.offboard.start())
            except OffboardError:
                pass  # Already in offboard mode

            self._ready_to_move = lambda _: True
            target_end = time.time() + duration if duration is not None else None

            async def _velocity_helper():
                while self._velocity_loop_active:
                    if target_end is not None and time.time() > target_end:
                        self._velocity_loop_active = False
                        await self._run_on_mavsdk_loop(self._system.offboard.stop())
                    await asyncio.sleep(VELOCITY_UPDATE_DELAY_S)

            self._velocity_loop_active = True
            asyncio.ensure_future(_velocity_helper())

        except (OffboardError, ActionError) as e:
            raise VelocityError(str(e), original_error=e)

    async def _stop(self) -> None:
        await super()._stop()
        if self.armed:
            await self.set_velocity(util.VectorNED(0, 0, 0))
        self._velocity_loop_active = False


class Rover(Vehicle):
    """
    Rover vehicle type. Implements all functionality that AERPAW's rovers
    expose to user scripts, which includes basic movement control (going to
    coords).

    `target_heading` is ignored for rovers, as they can't strafe.
    """

    async def goto_coordinates(
        self,
        coordinates: util.Coordinate,
        tolerance: float = DEFAULT_ROVER_POSITION_TOLERANCE_M,
        target_heading: Optional[float] = None
    ) -> None:
        """
        Make the vehicle go to provided coordinates.

        Args:
            coordinates: Target position
            tolerance: Distance in meters to consider destination reached
            target_heading: Ignored for rovers (they can't strafe)

        Raises:
            ValueError: If tolerance is out of acceptable range
            NavigationError: If navigation command fails
        """
        validate_tolerance(tolerance, "tolerance")

        await self.await_ready_to_move()
        self._ready_to_move = lambda self: False

        if self._mission_start_time is None:
            self._mission_start_time = time.time()

        try:
            await self._run_on_mavsdk_loop(self._system.action.goto_location(
                coordinates.lat,
                coordinates.lon,
                0,  # Rovers ignore altitude
                0   # Heading
            ))

            def at_coords(self) -> bool:
                return coordinates.ground_distance(self.position) <= tolerance

            self._ready_to_move = at_coords

            await wait_for_condition(
                lambda: at_coords(self),
                poll_interval=POLLING_DELAY_S
            )
        except ActionError as e:
            raise NavigationError(str(e), original_error=e)

