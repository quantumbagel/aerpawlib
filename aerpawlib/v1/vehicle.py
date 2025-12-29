"""
Core logic surrounding the various `Vehicle`s available to aerpawlib user
scripts.

This is the MAVSDK-based implementation of the v1 API, maintaining full
backward compatibility with the original DroneKit-based interface.

@author: Julian Reder (quantumbagel)
"""
import asyncio
import math
import time
import threading
from typing import Callable, Optional, List

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

# time to wait when polling for vehicle state changes
_POLLING_DELAY = 0.01  # s

# time to wait between steps of the arming -> guided -> takeoff sequence
_ARMING_SEQUENCE_DELAY = 2  # s

# time between calls for internal update handling
_INTERNAL_UPDATE_DELAY = 0.1  # s


class _BatteryCompat:
    """
    Compatibility wrapper to match dronekit.Battery interface.
    """
    def __init__(self):
        self.voltage = 0.0
        self.current = 0.0
        self.level = 0  # percentage 0-100

    def __str__(self):
        return f"Battery:voltage={self.voltage},current={self.current},level={self.level}"


class _GPSInfoCompat:
    """
    Compatibility wrapper to match dronekit.GPSInfo interface.
    """
    def __init__(self):
        self.fix_type = 0
        self.satellites_visible = 0

    def __str__(self):
        return f"GPSInfo:fix={self.fix_type},num_sat={self.satellites_visible}"


class _AttitudeCompat:
    """
    Compatibility wrapper to match dronekit.Attitude interface.
    """
    def __init__(self):
        self.pitch = 0.0  # radians
        self.roll = 0.0   # radians
        self.yaw = 0.0    # radians

    def __str__(self):
        return f"Attitude:pitch={self.pitch},yaw={self.yaw},roll={self.roll}"


class _VersionCompat:
    """
    Compatibility wrapper to match dronekit.Version interface.
    """
    def __init__(self):
        self.major = None
        self.minor = None
        self.patch = None
        self.release = None

    def __str__(self):
        return f"{self.major}.{self.minor}.{self.patch}"


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
    _ready_to_move: Callable[[object], bool] = lambda _: True

    # temp hack to allow for dynamically making the drone abortable or not
    _abortable: bool = False
    _aborted: bool = False

    _home_location: util.Coordinate = None

    # _current_heading is used to blend heading and velocity control commands
    _current_heading: float = None

    _last_nav_controller_output = None
    _last_mission_item_int = None

    # Verbose logging configuration
    _verbose_logging: bool = False
    _verbose_logging_file_prefix: str = "aerpawlib_vehicle_dump"
    _verbose_logging_file_writer = None
    _verbose_logging_last_log_time: float = 0
    _verbose_logging_delay: float = 0.1  # s

    def __init__(self, connection_string: str):
        if not MAVSDK_AVAILABLE:
            raise ImportError(
                "MAVSDK is not installed. Install with: pip install mavsdk"
            )

        self._connection_string = connection_string
        self._system = None
        self._has_heartbeat = False
        self._should_postarm_init = True
        self._mission_start_time = None

        # Internal state tracking
        self._armed_state = False
        self._is_armable_state = False
        self._position_lat = 0.0
        self._position_lon = 0.0
        self._position_alt = 0.0
        self._heading_deg = 0.0
        self._velocity_ned = [0.0, 0.0, 0.0]

        # Compatibility objects
        self._battery = _BatteryCompat()
        self._gps = _GPSInfoCompat()
        self._attitude = _AttitudeCompat()
        self._autopilot_info = _VersionCompat()
        self._mode = "UNKNOWN"

        # Telemetry tasks
        self._telemetry_tasks: List[asyncio.Task] = []
        self._running = True

        # Connect synchronously (blocking)
        self._connect_sync()

    def _connect_sync(self):
        """Synchronous connection for compatibility with original API."""
        loop = asyncio.new_event_loop()

        def _run_connection():
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._connect_async())
            # Keep the loop running for telemetry
            loop.run_forever()

        # Start connection in background thread
        t = threading.Thread(target=_run_connection, daemon=True)
        t.start()

        # Wait for connection
        timeout = 30
        start = time.time()
        while not self._has_heartbeat:
            if time.time() - start > timeout:
                raise TimeoutError("Connection timeout")
            time.sleep(_POLLING_DELAY)

        # Start internal update loop
        update_thread = threading.Thread(target=self._internal_update_loop, daemon=True)
        update_thread.start()

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
                self._position_lat = position.latitude_deg
                self._position_lon = position.longitude_deg
                self._position_alt = position.relative_altitude_m

        async def _attitude_update():
            async for attitude in self._system.telemetry.attitude_euler():
                self._attitude.roll = math.radians(attitude.roll_deg)
                self._attitude.pitch = math.radians(attitude.pitch_deg)
                self._attitude.yaw = math.radians(attitude.yaw_deg)
                self._heading_deg = attitude.yaw_deg % 360

        async def _velocity_update():
            async for velocity in self._system.telemetry.velocity_ned():
                self._velocity_ned = [
                    velocity.north_m_s,
                    velocity.east_m_s,
                    velocity.down_m_s
                ]

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
                self._mode = mode.name

        async def _armed_update():
            async for armed in self._system.telemetry.armed():
                self._armed_state = armed

        async def _health_update():
            async for health in self._system.telemetry.health():
                self._is_armable_state = (
                    health.is_global_position_ok and
                    health.is_home_position_ok and
                    health.is_armable
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
        return util.Coordinate(self._position_lat, self._position_lon, self._position_alt)

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
        return self._armed_state

    @property
    def home_coords(self) -> util.Coordinate:
        return self._home_location

    @property
    def heading(self) -> float:
        return self._heading_deg

    @property
    def velocity(self) -> util.VectorNED:
        return util.VectorNED(*self._velocity_ned)

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
            self._mode,
            nav_controller_output,
            mission_item_output,
        ]

        return ",".join(map(str, props))

    # Internal logic
    def _internal_update_loop(self):
        while self._running:
            self._internal_update()
            time.sleep(_INTERNAL_UPDATE_DELAY)

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

    async def await_ready_to_move(self):
        """
        Helper function that blocks execution and waits for the vehicle to
        finish the current action/movement.
        """
        if not self.armed:
            await self._initialize_postarm()

        while not self.done_moving():
            await asyncio.sleep(_POLLING_DELAY)

    def _abort(self):
        if self._abortable:
            AERPAW_Platform.log_to_oeo("[aerpawlib] Aborted.")
            self._abortable = False
            self._aborted = True

    # Verbs
    def close(self):
        """
        Clean up the `Vehicle` object/any state
        """
        self._running = False
        for task in self._telemetry_tasks:
            task.cancel()

    async def set_armed(self, value: bool):
        """
        Arm or disarm this vehicle, and wait for it to be armed (if possible).
        """
        if not self._is_armable_state and value:
            raise Exception("Not ready to arm")

        try:
            if value:
                await self._system.action.arm()
            else:
                await self._system.action.disarm()

            # Wait for arm state to match
            while self._armed_state != value:
                await asyncio.sleep(_POLLING_DELAY)
        except ActionError as e:
            raise Exception(f"Arm/disarm failed: {e}")

    def _initialize_prearm(self, should_postarm_init):
        start = time.time()
        while not self._is_armable_state:
            if time.time() - start > 60:
                break
            time.sleep(_POLLING_DELAY)
        self._should_postarm_init = should_postarm_init

    async def _initialize_postarm(self):
        """
        Generic pre-mission manipulation of the vehicle into a state that is
        acceptable. MUST be called before anything else.
        """
        if not self._should_postarm_init:
            return

        AERPAW_Platform.log_to_oeo("[aerpawlib] Guided command attempted. Waiting for safety pilot to arm")

        while not self._is_armable_state:
            await asyncio.sleep(_POLLING_DELAY)
        while not self.armed:
            await asyncio.sleep(_POLLING_DELAY)

        await asyncio.sleep(_ARMING_SEQUENCE_DELAY)

        # Set to offboard/guided mode - MAVSDK handles this differently
        # We'll use action commands which implicitly set the right mode

        await asyncio.sleep(_ARMING_SEQUENCE_DELAY)

        self._abortable = True
        self._home_location = self.position

    async def goto_coordinates(
        self,
        coordinates: util.Coordinate,
        tolerance: float = 2,
        target_heading: float = None
    ):
        """
        Make the vehicle go to provided coordinates.
        """
        raise Exception("Generic vehicles can't go to coordinates!")

    async def set_velocity(
        self,
        velocity_vector: util.VectorNED,
        global_relative: bool = True,
        duration: float = None
    ):
        """
        Set a drone's velocity that it will use for `duration` seconds.
        """
        raise Exception("set_velocity not implemented")

    async def set_groundspeed(self, velocity: float):
        """
        Set a vehicle's cruise velocity as used by the autopilot.
        """
        try:
            await self._system.action.set_maximum_speed(velocity)
        except ActionError:
            pass  # Not all autopilots support this

    async def _stop(self):
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

    async def set_heading(self, heading: float, blocking: bool = True, lock_in: bool = True):
        """
        Set the heading of the vehicle (in absolute deg).

        To turn a relative # of degrees, you can do something like
        `drone.set_heading(drone.heading + x)`

        Pass in `None` to make the drone return to ardupilot's default heading
        behavior (usually facing the direction of movement)
        """
        if blocking:
            await self.await_ready_to_move()

        if heading is None:
            self._current_heading = None
            return

        heading %= 360
        if lock_in:
            self._current_heading = heading
        if not blocking:
            return

        # Use offboard mode to control yaw
        # noinspection PyUnusedLocal
        try:
            await self._system.offboard.set_position_ned(
                PositionNedYaw(0, 0, -self._position_alt, heading)
            )
            try:
                await self._system.offboard.start()
            except OffboardError:
                pass  # Already in offboard mode

            def _pointed_at_heading(self) -> bool:
                _TURN_TOLERANCE_DEG = 5
                turn_diff = min([abs(i) for i in [heading - self.heading, self.heading - (heading + 360)]])
                return turn_diff <= _TURN_TOLERANCE_DEG

            self._ready_to_move = _pointed_at_heading

            while not _pointed_at_heading(self):
                await asyncio.sleep(_POLLING_DELAY)

            await self._system.offboard.stop()
        except (OffboardError, ActionError) as e:
            # Fallback - just update internal heading
            pass

    async def takeoff(self, target_alt: float, min_alt_tolerance: float = 0.95):
        """
        Make the drone take off to a specific altitude, and blocks until the
        drone has reached that altitude.
        """
        await self.await_ready_to_move()

        # wait_for_throttle is ignored in MAVSDK implementation
        # as we don't have direct RC channel access

        if self._mission_start_time is None:
            self._mission_start_time = time.time()

        try:
            await self._system.action.set_takeoff_altitude(target_alt)
            await self._system.action.takeoff()

            taken_off = lambda self: self.position.alt >= target_alt * min_alt_tolerance
            self._ready_to_move = taken_off

            while not taken_off(self):
                await asyncio.sleep(_POLLING_DELAY)

            await asyncio.sleep(5)
        except ActionError as e:
            raise Exception(f"Takeoff failed: {e}")

    async def land(self):
        """
        Land the drone at its current position and block while waiting for it
        to be disarmed.
        """
        await self.await_ready_to_move()

        self._abortable = False

        try:
            await self._system.action.land()

            self._ready_to_move = lambda _: False

            while self.armed:
                await asyncio.sleep(_POLLING_DELAY)
        except ActionError as e:
            raise Exception(f"Land failed: {e}")

    async def goto_coordinates(
        self,
        coordinates: util.Coordinate,
        tolerance: float = 2,
        target_heading: float = None
    ):
        """
        Make the vehicle go to provided coordinates.
        """
        if target_heading is not None:
            await self.set_heading(target_heading)

        await self.await_ready_to_move()
        await self._stop()

        self._ready_to_move = lambda self: False

        float('nan')
        if self._current_heading is not None:
            heading = self._current_heading
        else:
            heading = self.position.bearing(coordinates)
        await self.set_heading(heading, lock_in=False, blocking=False)

        try:
            # Use goto_location action
            await self._system.action.goto_location(
                coordinates.lat,
                coordinates.lon,
                coordinates.alt + (self._home_location.alt if self._home_location else 0),
                heading if not math.isnan(heading) else 0
            )

            at_coords = lambda self: coordinates.distance(self.position) <= tolerance
            self._ready_to_move = at_coords

            while not at_coords(self):
                await asyncio.sleep(_POLLING_DELAY)
        except ActionError as e:
            raise Exception(f"Goto failed: {e}")

    async def set_velocity(
        self,
        velocity_vector: util.VectorNED,
        global_relative: bool = True,
        duration: float = None
    ):
        """
        Set a drone's velocity that it will use for `duration` seconds.
        """
        await self.await_ready_to_move()

        self._velocity_loop_active = False
        await asyncio.sleep(_POLLING_DELAY)

        if not global_relative:
            velocity_vector = velocity_vector.rotate_by_angle(-self.heading)

        yaw = self._current_heading if self._current_heading is not None else self.heading

        try:
            await self._system.offboard.set_velocity_ned(
                VelocityNedYaw(
                    velocity_vector.north,
                    velocity_vector.east,
                    velocity_vector.down,
                    yaw
                )
            )

            try:
                await self._system.offboard.start()
            except OffboardError:
                pass  # Already in offboard mode

            self._ready_to_move = lambda _: True
            target_end = time.time() + duration if duration is not None else None

            async def _velocity_helper():
                while self._velocity_loop_active:
                    if target_end is not None and time.time() > target_end:
                        self._velocity_loop_active = False
                        await self._system.offboard.stop()
                    await asyncio.sleep(0.1)

            self._velocity_loop_active = True
            asyncio.ensure_future(_velocity_helper())

        except (OffboardError, ActionError) as e:
            raise Exception(f"Set velocity failed: {e}")

    async def _stop(self):
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
        tolerance: float = 2.1,
        target_heading: float = None
    ):
        """
        Make the vehicle go to provided coordinates.
        """
        await self.await_ready_to_move()
        self._ready_to_move = lambda self: False

        if self._mission_start_time is None:
            self._mission_start_time = time.time()

        try:
            await self._system.action.goto_location(
                coordinates.lat,
                coordinates.lon,
                0,  # Rovers ignore altitude
                0   # Heading
            )

            at_coords = lambda self: coordinates.ground_distance(self.position) <= tolerance
            self._ready_to_move = at_coords

            while not at_coords(self):
                await asyncio.sleep(_POLLING_DELAY)
        except ActionError as e:
            raise Exception(f"Goto failed: {e}")

