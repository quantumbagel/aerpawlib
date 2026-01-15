"""Vehicle classes for aerpawlib v2 API using MAVSDK."""

from __future__ import annotations

import asyncio
import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Callable, Dict, List, Optional, Union, TYPE_CHECKING

# Fail fast on MAVSDK - no try/except dance
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw, OffboardError
from mavsdk.action import ActionError
from mavsdk.telemetry import (
    FlightMode as MavFlightMode,
    LandedState as MavLandedState,
)

from .types import (
    Coordinate,
    VectorNED,
    Attitude,
    DroneState,
    FlightMode,
    LandedState,
    Waypoint,
)
from .exceptions import (
    ConnectionError as AerpawConnectionError,
    ConnectionTimeoutError,
    CommandError,
    ArmError,
    DisarmError,
    TakeoffError,
    TakeoffTimeoutError,
    LandingError,
    LandingTimeoutError,
    NavigationError,
    GotoTimeoutError,
    ModeChangeError,
    OffboardError as AerpawOffboardError,
    TimeoutError as AerpawTimeoutError,
    AbortError,
    NotArmableError,
    CommandCancelledError,
    PreflightCheckError,
    ParameterValidationError,
)
from .safety import (
    SafetyLimits,
    SafetyMonitor,
    PreflightCheckResult,
    validate_coordinate,
    validate_altitude,
    validate_speed,
    validate_timeout,
    validate_tolerance,
    clamp_speed,
    run_preflight_checks,
    validate_waypoint_with_checker,
    validate_speed_with_checker,
    validate_takeoff_with_checker,
    ConnectionHandler,
    DisconnectReason,
)
from .aerpaw import OEOClient
from .logging import get_logger, LogComponent

if TYPE_CHECKING:
    from .safety import SafetyCheckerClient

_POLLING_DELAY = 0.01
_HEARTBEAT_TIMEOUT = 5.0

logger = get_logger(LogComponent.VEHICLE)


class CommandStatus(Enum):
    """Status of a command being executed."""

    PENDING = auto()
    RUNNING = auto()
    COMPLETED = auto()
    FAILED = auto()
    CANCELLED = auto()
    TIMED_OUT = auto()


@dataclass
class CommandResult:
    """Result of a completed command."""

    status: CommandStatus
    command: str
    start_time: float
    end_time: float
    error: Optional[Exception] = None
    details: Dict[str, Any] = field(default_factory=dict)

    @property
    def duration(self) -> float:
        return self.end_time - self.start_time

    @property
    def succeeded(self) -> bool:
        return self.status == CommandStatus.COMPLETED

    @property
    def was_cancelled(self) -> bool:
        return self.status == CommandStatus.CANCELLED


class CommandHandle:
    """Handle for tracking and controlling command execution."""

    def __init__(
        self,
        command: str,
        completion_condition: Callable[[], bool],
        cancel_action: Optional[Callable[[], Any]] = None,
        timeout: Optional[float] = None,
        abort_checker: Optional[Callable[[], bool]] = None,
        progress_getter: Optional[Callable[[], Dict[str, Any]]] = None,
    ):
        self._command = command
        self._completion_condition = completion_condition
        self._cancel_action = cancel_action
        self._timeout = timeout
        self._abort_checker = abort_checker
        self._progress_getter = progress_getter
        self._status = CommandStatus.PENDING
        self._start_time = time.time()
        self._end_time: Optional[float] = None
        self._error: Optional[Exception] = None
        self._cancelled = False
        self._task: Optional[asyncio.Task] = None
        self._completion_event = asyncio.Event()

    @property
    def command(self) -> str:
        return self._command

    @property
    def status(self) -> CommandStatus:
        return self._status

    @property
    def is_pending(self) -> bool:
        return self._status == CommandStatus.PENDING

    @property
    def is_running(self) -> bool:
        return self._status == CommandStatus.RUNNING

    @property
    def is_complete(self) -> bool:
        return self._status in (
            CommandStatus.COMPLETED,
            CommandStatus.FAILED,
            CommandStatus.CANCELLED,
            CommandStatus.TIMED_OUT,
        )

    @property
    def succeeded(self) -> bool:
        return self._status == CommandStatus.COMPLETED

    @property
    def was_cancelled(self) -> bool:
        return self._status == CommandStatus.CANCELLED

    @property
    def timed_out(self) -> bool:
        return self._status == CommandStatus.TIMED_OUT

    @property
    def error(self) -> Optional[Exception]:
        return self._error

    @property
    def elapsed_time(self) -> float:
        return (self._end_time or time.time()) - self._start_time

    @property
    def time_remaining(self) -> Optional[float]:
        return (
            max(0, self._timeout - self.elapsed_time)
            if self._timeout
            else None
        )

    @property
    def progress(self) -> Dict[str, Any]:
        return self._progress_getter() if self._progress_getter else {}

    def result(self) -> CommandResult:
        if not self.is_complete:
            raise RuntimeError(
                "Command is still running. Use await handle.wait() first."
            )
        return CommandResult(
            self._status,
            self._command,
            self._start_time,
            self._end_time or time.time(),
            self._error,
            self.progress,
        )

    def _start(self) -> None:
        self._status = CommandStatus.RUNNING
        self._task = asyncio.create_task(self._monitor())

    async def _monitor(self) -> None:
        try:
            while not self._cancelled:
                if self._completion_condition():
                    self._status, self._end_time = (
                        CommandStatus.COMPLETED,
                        time.time(),
                    )
                    self._completion_event.set()
                    return
                if self._abort_checker and self._abort_checker():
                    self._status, self._error = (
                        CommandStatus.CANCELLED,
                        AbortError(message="Vehicle abort triggered"),
                    )
                    self._end_time = time.time()
                    self._completion_event.set()
                    return
                if self._timeout and self.elapsed_time > self._timeout:
                    self._status = CommandStatus.TIMED_OUT
                    self._error = AerpawTimeoutError(
                        message=f"{self._command} timed out after {self._timeout}s",
                        timeout=self._timeout,
                    )
                    self._end_time = time.time()
                    self._completion_event.set()
                    return
                await asyncio.sleep(_POLLING_DELAY)
            self._status, self._end_time = CommandStatus.CANCELLED, time.time()
            self._completion_event.set()
        except asyncio.CancelledError:
            self._status, self._end_time = CommandStatus.CANCELLED, time.time()
            self._completion_event.set()
            raise
        except Exception as e:
            self._status, self._error, self._end_time = (
                CommandStatus.FAILED,
                e,
                time.time(),
            )
            self._completion_event.set()

    async def wait(self, timeout: Optional[float] = None) -> "CommandHandle":
        if self.is_complete:
            return self._raise_if_error()
        wait_timeout = timeout or self._timeout
        try:
            if wait_timeout:
                await asyncio.wait_for(
                    self._completion_event.wait(), timeout=wait_timeout
                )
            else:
                await self._completion_event.wait()
        except asyncio.TimeoutError:
            self._status = CommandStatus.TIMED_OUT
            self._error = AerpawTimeoutError(
                message=f"Wait for {self._command} timed out",
                timeout=wait_timeout,
            )
            self._end_time = time.time()
        return self._raise_if_error()

    def _raise_if_error(self) -> "CommandHandle":
        if self._status == CommandStatus.CANCELLED:
            raise CommandCancelledError(
                message=f"{self._command} was cancelled", command=self._command
            )
        if self._status == CommandStatus.TIMED_OUT and self._error:
            raise self._error
        if self._status == CommandStatus.FAILED and self._error:
            raise self._error
        return self

    async def cancel(self, execute_cancel_action: bool = True) -> bool:
        if self.is_complete:
            return False
        self._cancelled = True
        if execute_cancel_action and self._cancel_action:
            try:
                result = self._cancel_action()
                if asyncio.iscoroutine(result):
                    await result
            except Exception as e:
                logger.warning(f"Cancel action failed: {e}")
        if self._task and not self._task.done():
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        self._status, self._end_time = CommandStatus.CANCELLED, time.time()
        self._completion_event.set()
        return True

    def __await__(self):
        return self.wait().__await__()

    def __repr__(self) -> str:
        return f"CommandHandle({self._command}, status={self._status.name}, elapsed={self.elapsed_time:.2f}s)"


# Flight mode conversion maps
_FLIGHT_MODE_MAP = {
    MavFlightMode.MANUAL: FlightMode.MANUAL,
    MavFlightMode.STABILIZED: FlightMode.STABILIZED,
    MavFlightMode.ALTCTL: FlightMode.ALTITUDE,
    MavFlightMode.POSCTL: FlightMode.POSITION,
    MavFlightMode.OFFBOARD: FlightMode.OFFBOARD,
    MavFlightMode.HOLD: FlightMode.HOLD,
    MavFlightMode.MISSION: FlightMode.MISSION,
    MavFlightMode.RETURN_TO_LAUNCH: FlightMode.RETURN_TO_LAUNCH,
    MavFlightMode.LAND: FlightMode.LAND,
    MavFlightMode.TAKEOFF: FlightMode.TAKEOFF,
    MavFlightMode.FOLLOW_ME: FlightMode.FOLLOW_ME,
}

_LANDED_STATE_MAP = {
    MavLandedState.ON_GROUND: LandedState.ON_GROUND,
    MavLandedState.IN_AIR: LandedState.IN_AIR,
    MavLandedState.TAKING_OFF: LandedState.TAKING_OFF,
    MavLandedState.LANDING: LandedState.LANDING,
}


@dataclass
class StateContainer:
    """Container for drone telemetry state."""

    heading: float = 0.0
    velocity: VectorNED = field(default_factory=VectorNED)
    attitude: Attitude = field(default_factory=Attitude)
    altitude: float = 0.0
    relative_altitude: float = 0.0
    latitude: float = 0.0
    longitude: float = 0.0
    groundspeed: float = 0.0
    airspeed: float = 0.0
    climb_rate: float = 0.0
    flight_mode: FlightMode = FlightMode.UNKNOWN
    landed_state: LandedState = LandedState.UNKNOWN

    @property
    def position(self) -> Coordinate:
        return Coordinate(
            self.latitude, self.longitude, self.relative_altitude
        )

    @property
    def speed(self) -> float:
        return self.velocity.magnitude()

    @property
    def horizontal_speed(self) -> float:
        return self.velocity.magnitude(ignore_vertical=True)

    @property
    def is_in_air(self) -> bool:
        return self.landed_state == LandedState.IN_AIR

    def snapshot(self) -> DroneState:
        return DroneState(
            heading=self.heading,
            velocity=self.velocity,
            attitude=self.attitude,
            altitude=self.altitude,
            relative_altitude=self.relative_altitude,
            latitude=self.latitude,
            longitude=self.longitude,
            groundspeed=self.groundspeed,
            airspeed=self.airspeed,
            climb_rate=self.climb_rate,
            flight_mode=self.flight_mode,
            landed_state=self.landed_state,
        )


@dataclass
class GPSContainer:
    """Container for GPS information."""

    satellites: int = 0
    fix_type: int = 0

    @property
    def quality(self) -> str:
        return {
            0: "No GPS",
            1: "No Fix",
            2: "2D Fix",
            3: "3D Fix",
            4: "DGPS",
            5: "RTK Float",
            6: "RTK Fixed",
        }.get(self.fix_type, "Unknown")

    @property
    def has_fix(self) -> bool:
        return self.fix_type >= 2


@dataclass
class BatteryContainer:
    """Container for battery information."""

    id: int = 0
    voltage: float = 0.0
    current: float = 0.0
    charge: float = 0.0
    temperature: Optional[float] = None
    consumption: float = 0.0

    @property
    def percentage(self) -> float:
        return self.charge * 100

    @property
    def is_low(self) -> bool:
        return self.charge < 0.20

    @property
    def is_critical(self) -> bool:
        return self.charge < 0.10


@dataclass
class InfoContainer:
    """Container for static vehicle information."""

    hardware_uuid: str = ""
    legacy_uuid: str = ""
    vendor_id: int = 0
    vendor_name: str = ""
    product_id: int = 0
    product_name: str = ""
    version: str = ""
    time_boot_ms: int = 0
    flight_id: str = ""
    arm_time: Optional[float] = None
    takeoff_time: Optional[float] = None

    @property
    def time_since_arm_ms(self) -> int:
        return (
            int((time.time() - self.arm_time) * 1000) if self.arm_time else 0
        )

    @property
    def time_since_takeoff_ms(self) -> int:
        return (
            int((time.time() - self.takeoff_time) * 1000)
            if self.takeoff_time
            else 0
        )


class VehicleEvent(Enum):
    """Vehicle events for internal state tracking."""

    CONNECT = "on_connect"
    DISCONNECT = "on_disconnect"
    CONNECTION_SEVERED = (
        "on_connection_severed"  # MAVLink filter or hardware failure
    )
    ARM = "on_arm"
    DISARM = "on_disarm"
    LOW_BATTERY = "on_low_battery"
    CRITICAL_BATTERY = "on_critical_battery"
    MODE_CHANGE = "on_mode_change"
    SAFETY_VIOLATION = "on_safety_violation"  # Illegal command blocked


class Vehicle:
    """
    Base vehicle class providing common functionality.

    Connection Handling:
        In AERPAW infrastructure, there are two types of connection failures:

        1. Hardware failure (USB disconnect, autopilot failure)
           - Not recoverable
           - Requires safety pilot intervention

        2. MAVLink filter disconnect
           - Happens when illegal command is blocked
           - Puts vehicle in HOLD mode
           - Not recoverable (experiment code deemed unsafe)

        Both failures send notifications to OEO for operator awareness.
    """

    def __init__(
        self,
        connection: str = "udp://:14540",
        oeo_client: Optional[OEOClient] = None,
    ):
        self._connection_string = connection
        self._system: Optional[System] = None
        self._connected = False
        self._armed = False
        self._is_armable = False
        self.state = StateContainer()
        self.gps = GPSContainer()
        self.battery = BatteryContainer()
        self.info = InfoContainer()
        self._home: Optional[Coordinate] = None
        self._throttle: float = 0.0
        self._telemetry_tasks: List[asyncio.Task] = []
        self._running = True
        self._current_heading: Optional[float] = None
        self._velocity_loop_active = False
        self._abortable = False
        self._aborted = False
        self._last_heartbeat: float = 0.0
        self._max_reconnect_attempts: int = 3
        self._low_battery_triggered = False
        self._critical_battery_triggered = False
        self._heartbeat_monitor_task: Optional[asyncio.Task] = None
        self._recording = False
        self._flight_log: List[Dict[str, Any]] = []
        self._recording_interval: float = 0.1

        # OEO client for operator notifications
        self._oeo = oeo_client

        # Connection handler for managing disconnections
        self._connection_handler: Optional[ConnectionHandler] = None


    async def connect(
        self,
        timeout: float = 30.0,
        retry_count: int = 3,
        retry_delay: float = 2.0,
    ) -> bool:
        """
        Connect to the vehicle.

        Args:
            timeout: Connection timeout in seconds
            retry_count: Number of initial connection attempts
            retry_delay: Delay between retry attempts

        Returns:
            True if connected successfully

        Raises:
            ConnectionTimeoutError: If connection times out
            ConnectionError: If connection fails

        Note:
            Reconnection is not supported per AERPAW policy.
            Connection failures (hardware or MAVLink filter) result in
            experiment abort with operator notification.
        """
        self._max_reconnect_attempts = retry_count
        last_error: Optional[Exception] = None

        # Initialize connection handler
        self._connection_handler = ConnectionHandler(
            self,
            self._oeo,
            heartbeat_timeout=_HEARTBEAT_TIMEOUT,
        )

        for attempt in range(1, retry_count + 1):
            try:
                logger.info(
                    f"Connecting to {self._connection_string} (attempt {attempt}/{retry_count})..."
                )
                self._system = System()
                await self._system.connect(
                    system_address=self._connection_string
                )

                start_time = time.time()
                async for conn_state in self._system.core.connection_state():
                    if conn_state.is_connected:
                        self._connected = True
                        self._last_heartbeat = time.time()
                        self._connection_handler.mark_connected()
                        logger.info("Connected to vehicle")
                        break
                    if time.time() - start_time > timeout:
                        raise ConnectionTimeoutError(
                            timeout=timeout,
                            address=self._connection_string,
                            attempt=attempt,
                            max_attempts=retry_count,
                        )
                    await asyncio.sleep(_POLLING_DELAY)

                if not self._connected:
                    continue

                await self._start_telemetry()
                self._start_heartbeat_monitor()
                self._connection_handler.start_monitoring()
                await self._fetch_vehicle_info()
                await self._trigger_callbacks(VehicleEvent.CONNECT)
                return True

            except ConnectionTimeoutError as e:
                last_error = e
                logger.warning(
                    f"Connection attempt {attempt} timed out, retrying in {retry_delay}s..."
                )
            except Exception as e:
                last_error = AerpawConnectionError(
                    message=f"Connection failed: {e}",
                    address=self._connection_string,
                    attempt=attempt,
                    max_attempts=retry_count,
                )
                logger.warning(f"Connection attempt {attempt} failed: {e}")

            if attempt < retry_count:
                await asyncio.sleep(retry_delay)

        raise last_error or AerpawConnectionError(
            message="All connection attempts failed",
            address=self._connection_string,
            attempt=retry_count,
            max_attempts=retry_count,
        )

    def _start_heartbeat_monitor(self) -> None:
        """
        Start the heartbeat monitor for detecting connection loss.

        In AERPAW infrastructure, connection loss typically means:
        - Hardware failure: requires safety pilot intervention
        - MAVLink filter disconnect: experiment violated safety rules

        Reconnection is not supported.
        """
        if self._heartbeat_monitor_task is not None:
            return

        async def _monitor():
            while self._running and self._connected:
                await asyncio.sleep(1.0)
                if not self.connection_healthy:
                    logger.warning(
                        f"Heartbeat lost (last: {self.seconds_since_heartbeat:.1f}s ago)"
                    )
                    await self._trigger_callbacks(VehicleEvent.DISCONNECT)

                    # Notify OEO of connection loss
                    if self._oeo:
                        await self._oeo.notify_connection_failure(
                            reason=f"Lost heartbeat - no telemetry received for {self.seconds_since_heartbeat:.1f}s",
                            is_mavlink_filter=False,
                            details={
                                "last_heartbeat_seconds_ago": self.seconds_since_heartbeat,
                                "was_armed": self._armed,
                                "was_in_air": self.state.is_in_air,
                            },
                        )

                    # Per AERPAW policy: do not reconnect
                    self._connected = False
                    await self._trigger_callbacks(
                        VehicleEvent.CONNECTION_SEVERED
                    )
                    logger.critical(
                        "Connection lost. Per safety policy, not attempting reconnection. "
                        "Experiment should abort. Safety pilots should take manual control."
                    )

        self._heartbeat_monitor_task = asyncio.create_task(_monitor())

    async def _start_telemetry(self):
        """Start all telemetry subscriptions using a generic pattern."""

        # Define telemetry handlers as (stream_getter, handler) pairs
        telemetry_handlers = [
            (self._system.telemetry.position, self._handle_position),
            (self._system.telemetry.attitude_euler, self._handle_attitude),
            (self._system.telemetry.velocity_ned, self._handle_velocity),
            (self._system.telemetry.gps_info, self._handle_gps),
            (self._system.telemetry.battery, self._handle_battery),
            (self._system.telemetry.flight_mode, self._handle_flight_mode),
            (self._system.telemetry.landed_state, self._handle_landed_state),
            (self._system.telemetry.armed, self._handle_armed),
            (self._system.telemetry.health, self._handle_health),
            (self._system.telemetry.home, self._handle_home),
        ]

        async def create_subscription(stream_getter, handler):
            """Generic telemetry subscription coroutine."""
            async for data in stream_getter():
                self._last_heartbeat = time.time()
                await handler(data)

        for stream_getter, handler in telemetry_handlers:
            task = asyncio.create_task(
                create_subscription(stream_getter, handler)
            )
            self._telemetry_tasks.append(task)

    async def _handle_position(self, p):
        self.state.latitude, self.state.longitude = (
            p.latitude_deg,
            p.longitude_deg,
        )
        self.state.altitude, self.state.relative_altitude = (
            p.absolute_altitude_m,
            p.relative_altitude_m,
        )

    async def _handle_attitude(self, a):
        self.state.attitude = Attitude(
            roll=math.radians(a.roll_deg),
            pitch=math.radians(a.pitch_deg),
            yaw=math.radians(a.yaw_deg),
        )
        self.state.heading = a.yaw_deg % 360

    async def _handle_velocity(self, v):
        self.state.velocity = VectorNED(v.north_m_s, v.east_m_s, v.down_m_s)
        self.state.groundspeed, self.state.climb_rate = (
            math.hypot(v.north_m_s, v.east_m_s),
            -v.down_m_s,
        )

    async def _handle_gps(self, g):
        self.gps.satellites, self.gps.fix_type = (
            g.num_satellites,
            g.fix_type.value,
        )

    async def _handle_battery(self, b):
        self.battery.voltage, self.battery.charge = (
            b.voltage_v,
            b.remaining_percent / 100.0,
        )
        if self.battery.is_critical and not self._critical_battery_triggered:
            self._critical_battery_triggered = True
            logger.warning(f"Critical battery: {self.battery.percentage:.1f}%")
            await self._trigger_callbacks(VehicleEvent.CRITICAL_BATTERY)
        elif self.battery.is_low and not self._low_battery_triggered:
            self._low_battery_triggered = True
            logger.warning(f"Low battery: {self.battery.percentage:.1f}%")
            await self._trigger_callbacks(VehicleEvent.LOW_BATTERY)

    async def _handle_flight_mode(self, mode):
        old = self.state.flight_mode
        self.state.flight_mode = _FLIGHT_MODE_MAP.get(mode, FlightMode.UNKNOWN)
        if old != self.state.flight_mode:
            await self._trigger_callbacks(
                VehicleEvent.MODE_CHANGE, old, self.state.flight_mode
            )

    async def _handle_landed_state(self, s):
        self.state.landed_state = _LANDED_STATE_MAP.get(s, LandedState.UNKNOWN)

    async def _handle_armed(self, armed):
        was_armed = self._armed
        self._armed = armed
        if armed and not was_armed:
            self.info.arm_time = time.time()
            await self._trigger_callbacks(VehicleEvent.ARM)
        elif not armed and was_armed:
            self.info.arm_time = None
            await self._trigger_callbacks(VehicleEvent.DISARM)

    async def _handle_health(self, h):
        self._is_armable = (
            h.is_global_position_ok and h.is_home_position_ok and h.is_armable
        )

    async def _handle_home(self, h):
        self._home = Coordinate(
            h.latitude_deg, h.longitude_deg, h.relative_altitude_m
        )

    async def _fetch_vehicle_info(self):
        try:
            info = await self._system.info.get_identification()
            self.info.hardware_uuid, self.info.legacy_uuid = (
                info.hardware_uid,
                getattr(info, "legacy_uid", ""),
            )
        except Exception as e:
            logger.warning(f"Failed to fetch vehicle identification: {e}")
        try:
            v = await self._system.info.get_version()
            self.info.version = (
                f"{v.flight_sw_major}.{v.flight_sw_minor}.{v.flight_sw_patch}"
            )
            self.info.vendor_name, self.info.product_name = getattr(
                v, "vendor_name", ""
            ), getattr(v, "product_name", "")
        except Exception as e:
            logger.warning(f"Failed to fetch vehicle version: {e}")

    async def disconnect(self):
        """Disconnect from the vehicle and clean up resources."""
        self._running = False
        if self._recording:
            self.stop_recording()
        if self._heartbeat_monitor_task:
            self._heartbeat_monitor_task.cancel()
            try:
                await self._heartbeat_monitor_task
            except asyncio.CancelledError:
                pass
            self._heartbeat_monitor_task = None

        # Stop connection handler
        if self._connection_handler:
            self._connection_handler.stop_monitoring()
            self._connection_handler.mark_disconnected(
                DisconnectReason.USER_DISCONNECT
            )

        for task in self._telemetry_tasks:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        self._telemetry_tasks.clear()
        self._connected = False
        logger.info("Disconnected from vehicle")

    async def hold(self):
        """Hold current position."""
        try:
            await self._system.action.hold()
        except ActionError:
            pass

    async def rtl(
        self, wait: bool = True, timeout: float = 300.0
    ) -> Optional[CommandHandle]:
        """
        Return to launch.
        To be implemented by subclasses.
        """
        raise NotImplementedError("RTL not supported for this vehicle type")

    def start_recording(self, interval: float = 0.1) -> None:
        if self._recording:
            return
        self._recording, self._recording_interval = True, interval
        self._flight_log.clear()

        async def _record_loop():
            while self._recording and self._connected:
                self._flight_log.append(self._capture_telemetry_snapshot())
                await asyncio.sleep(self._recording_interval)

        asyncio.create_task(_record_loop())

    def stop_recording(self) -> int:
        if not self._recording:
            return len(self._flight_log)
        self._recording = False
        return len(self._flight_log)

    def _capture_telemetry_snapshot(self) -> Dict[str, Any]:
        return {
            "timestamp": time.time(),
            "latitude": self.state.latitude,
            "longitude": self.state.longitude,
            "altitude": self.state.relative_altitude,
            "heading": self.state.heading,
            "groundspeed": self.state.groundspeed,
            "velocity_north": self.state.velocity.north,
            "velocity_east": self.state.velocity.east,
            "velocity_down": self.state.velocity.down,
            "roll": self.state.attitude.roll,
            "pitch": self.state.attitude.pitch,
            "yaw": self.state.attitude.yaw,
            "battery_voltage": self.battery.voltage,
            "battery_percent": self.battery.charge * 100,
            "gps_satellites": self.gps.satellites,
            "gps_fix_type": self.gps.fix_type,
            "armed": self._armed,
            "flight_mode": self.state.flight_mode.name,
        }

    def get_flight_log(self) -> List[Dict[str, Any]]:
        return self._flight_log.copy()

    def save_flight_log(self, path: str, format: str = "json") -> None:
        import json

        if format == "json":
            with open(path, "w") as f:
                json.dump(
                    {
                        "metadata": {
                            "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                            "data_points": len(self._flight_log),
                            "interval": self._recording_interval,
                        },
                        "telemetry": self._flight_log,
                    },
                    f,
                    indent=2,
                )
        elif format == "csv":
            if not self._flight_log:
                return
            import csv

            with open(path, "w", newline="") as f:
                writer = csv.DictWriter(
                    f, fieldnames=self._flight_log[0].keys()
                )
                writer.writeheader()
                writer.writerows(self._flight_log)
        else:
            raise ValueError(f"Unsupported format: {format}")

    def clear_flight_log(self) -> None:
        self._flight_log.clear()

    async def _trigger_callbacks(
        self, event: VehicleEvent, *args, **kwargs
    ) -> None:
        """Internal event handler for logging vehicle state changes."""
        # Log the event for debugging purposes
        logger.debug(f"Vehicle event: {event.value} args={args} kwargs={kwargs}")

    async def wait_for_gps_fix(
        self, min_satellites: int = 6, timeout: float = 60.0
    ) -> bool:
        await self._await_condition(
            lambda: self.gps.has_fix and self.gps.satellites >= min_satellites,
            timeout=timeout,
            message=f"GPS fix timeout after {timeout}s",
        )
        return True

    async def wait_for_armable(self, timeout: float = 60.0) -> bool:
        await self._await_condition(
            lambda: self._is_armable,
            timeout=timeout,
            message=f"Armable timeout after {timeout}s",
        )
        return True

    def reset_abort(self) -> None:
        self._aborted = False

    async def set_home(self, position: Optional[Coordinate] = None) -> bool:
        position = position or self.position
        try:
            await self._system.action.set_current_home_position(
                position.latitude, position.longitude, position.altitude
            )
            self._home = position
            return True
        except ActionError as e:
            raise CommandError(
                message=f"Failed to set home position: {e}",
                command="set_home",
                reason=str(e),
            )

    async def get_param(self, name: str) -> float:
        try:
            return (await self._system.param.get_param_float(name)).value
        except Exception as e:
            raise CommandError(
                message=f"Failed to get parameter '{name}': {e}",
                command="get_param",
                reason=str(e),
            )

    async def set_param(self, name: str, value: float) -> bool:
        try:
            await self._system.param.set_param_float(name, value)
            return True
        except Exception as e:
            raise CommandError(
                message=f"Failed to set parameter '{name}': {e}",
                command="set_param",
                reason=str(e),
            )

    async def get_param_int(self, name: str) -> int:
        try:
            return (await self._system.param.get_param_int(name)).value
        except Exception as e:
            raise CommandError(
                message=f"Failed to get parameter '{name}': {e}",
                command="get_param",
                reason=str(e),
            )

    async def set_param_int(self, name: str, value: int) -> bool:
        try:
            await self._system.param.set_param_int(name, value)
            return True
        except Exception as e:
            raise CommandError(
                message=f"Failed to set parameter '{name}': {e}",
                command="set_param",
                reason=str(e),
            )

    async def get_all_params(self) -> Dict[str, Any]:
        try:
            params = {}
            async for p in self._system.param.get_all_params():
                if hasattr(p, "float_param"):
                    params[p.float_param.name] = p.float_param.value
                elif hasattr(p, "int_param"):
                    params[p.int_param.name] = p.int_param.value
            return params
        except Exception as e:
            raise CommandError(
                message=f"Failed to get all parameters: {e}",
                command="get_all_params",
                reason=str(e),
            )

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def connection_healthy(self) -> bool:
        return (
            self._connected
            and (time.time() - self._last_heartbeat) < _HEARTBEAT_TIMEOUT
        )

    @property
    def seconds_since_heartbeat(self) -> float:
        return (
            float("inf")
            if self._last_heartbeat == 0
            else time.time() - self._last_heartbeat
        )

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def is_armable(self) -> bool:
        return self._is_armable

    @property
    def home(self) -> Optional[Coordinate]:
        return self._home

    @property
    def throttle(self) -> float:
        return self._throttle

    @property
    def position(self) -> Coordinate:
        return self.state.position

    @property
    def heading(self) -> float:
        return self.state.heading

    @property
    def velocity(self) -> VectorNED:
        return self.state.velocity

    @property
    def altitude(self) -> float:
        return self.state.relative_altitude

    @property
    def is_in_air(self) -> bool:
        return self.state.is_in_air

    async def arm(self, force: bool = False) -> bool:
        if self._armed:
            return True
        if not force and not self._is_armable:
            raise NotArmableError(
                message="Vehicle is not armable. Check GPS and other pre-flight conditions."
            )
        try:
            await self._system.action.arm()
            start = time.time()
            while not self._armed and time.time() - start < 10:
                await asyncio.sleep(_POLLING_DELAY)
            return self._armed
        except ActionError as e:
            raise ArmError(message=f"Failed to arm: {e}", reason=str(e))

    async def disarm(self, force: bool = False) -> bool:
        if not self._armed:
            return True
        try:
            if force:
                await self._system.action.kill()
            else:
                await self._system.action.disarm()
            start = time.time()
            while self._armed and time.time() - start < 10:
                await asyncio.sleep(_POLLING_DELAY)
            return not self._armed
        except ActionError as e:
            raise DisarmError(message=f"Failed to disarm: {e}", reason=str(e))

    async def wait(self, seconds: float):
        await asyncio.sleep(seconds)

    async def _await_condition(
        self,
        condition: Callable[[], bool],
        timeout: Optional[float] = None,
        message: str = "Condition not met",
    ):
        start = time.time()
        while not condition():
            if timeout and time.time() - start > timeout:
                raise AerpawTimeoutError(message=message, timeout=timeout)
            if self._aborted:
                raise AbortError(message="Operation aborted")
            await asyncio.sleep(_POLLING_DELAY)

    async def handle_safety_violation(
        self,
        violation_type: str,
        command: str,
        message: str,
    ) -> None:
        """
        Handle a safety violation (e.g., from MAVLink filter).

        This is called when a command is blocked by the safety infrastructure.
        In AERPAW, illegal commands result in:
        1. Connection being severed by MAVLink filter
        2. Drone being put into HOLD mode
        3. OEO notification to operators
        4. Experiment abort (handled by safety pilots via RC)

        Args:
            violation_type: Type of violation (e.g., "geofence", "speed_limit")
            command: The command that was blocked
            message: Detailed error message
        """
        logger.critical(f"SAFETY VIOLATION - {violation_type}: {message}")

        # Trigger safety violation callback
        await self._trigger_callbacks(
            VehicleEvent.SAFETY_VIOLATION, violation_type, command, message
        )

        # Notify OEO
        if self._oeo:
            await self._oeo.notify_illegal_command_blocked(
                command=command,
                violation_type=violation_type,
                message=message,
            )

        # Set aborted flag to stop any running operations
        self._aborted = True

    async def handle_mavlink_filter_disconnect(
        self, violation_message: str
    ) -> None:
        """
        Handle a disconnection caused by the MAVLink filter.

        This is called when the MAVLink filter severs the connection due to
        an illegal command. The vehicle should already be in HOLD mode.

        Per AERPAW policy:
        - Connection is NOT recoverable
        - OEO is notified
        - Safety pilots take manual control via RC

        Args:
            violation_message: Description of the safety violation
        """
        logger.critical(
            f"MAVLink filter severed connection: {violation_message}"
        )

        # Update connection state
        self._connected = False
        self._aborted = True

        if self._connection_handler:
            await self._connection_handler.handle_mavlink_filter_disconnect(
                violation_message
            )

        # Trigger callbacks
        await self._trigger_callbacks(
            VehicleEvent.CONNECTION_SEVERED, violation_message
        )


class Drone(Vehicle):
    """
    Drone (multicopter) vehicle class with flight control methods.

    This class integrates with AERPAW's safety infrastructure:

    Safety Layers:
        1. Client-side validation (SafetyLimits) - catches obvious errors locally
        2. SafetyCheckerClient - validates against geofences before sending commands
        3. MAVLink filter (server-side) - blocks illegal commands at the stream level

    Battery Monitoring:
        - min_takeoff_battery_percent: Required before takeoff (preflight check)
        - min_battery_percent: Warning threshold during flight
        - critical_battery_percent: Auto-RTL threshold
        - Voltage thresholds can also be configured

    Connection Handling:
        - Connection drops are NOT recoverable (hardware or MAVLink filter failure)
        - OEO is notified of all connection failures
        - Safety pilots maintain RC override for manual intervention

    Example:
        class MyMission(BasicRunner):
            @entrypoint
            async def run(self, drone: Drone):
                await drone.takeoff(altitude=10)
                await drone.goto(latitude=35.727, longitude=-78.696)
                await drone.land()
    """

    def __init__(
        self,
        connection: str = "udp://:14540",
        safety_limits: Optional[SafetyLimits] = None,
        safety_checker: Optional["SafetyCheckerClient"] = None,
        oeo_client: Optional[OEOClient] = None,
    ):
        """
        Initialize the Drone.

        Args:
            connection: MAVLink connection string
            safety_limits: Safety configuration (defaults to SafetyLimits())
            safety_checker: Optional SafetyCheckerClient for geofence validation
            oeo_client: Optional OEO client for operator notifications
        """
        super().__init__(connection, oeo_client)
        self._waypoints: List[Waypoint] = []
        self.safety_limits = safety_limits or SafetyLimits()
        self._safety_monitor: Optional[SafetyMonitor] = None
        self._safety_checker = safety_checker
        errors = self.safety_limits.validate()
        if errors:
            raise ValueError(f"Invalid safety limits: {', '.join(errors)}")

    async def connect(
        self,
        timeout: float = 30.0,
        retry_count: int = 3,
        retry_delay: float = 2.0,
    ) -> bool:
        """
        Connect to the drone.

        Note: Reconnection is not supported per AERPAW policy.
        Connection failures require operator intervention.
        """
        result = await super().connect(timeout, retry_count, retry_delay)
        if result and self.safety_limits:
            self._safety_monitor = SafetyMonitor(
                self, self.safety_limits, self._oeo
            )
            self._safety_monitor.start()
        return result

    async def disconnect(self):
        if self._safety_monitor:
            self._safety_monitor.stop()
            self._safety_monitor = None
        await super().disconnect()

    async def preflight_check(self) -> PreflightCheckResult:
        return await run_preflight_checks(self, self.safety_limits)

    async def arm(
        self, force: bool = False, skip_preflight: bool = False
    ) -> bool:
        """
        Arm the drone.

        Args:
            force: Skip armability check and force arm
            skip_preflight: Skip preflight safety checks

        Returns:
            True if armed successfully

        Raises:
            PreflightCheckError: If preflight checks fail
            NotArmableError: If vehicle is not armable

        Note:
            Preflight checks include battery level vs takeoff threshold.
            In AERPAW, operators also check parameters via OEO before experiments.
        """
        if self._safety_monitor and not self._safety_monitor.ready:
            logger.info(
                "Waiting for safety monitor to be ready before arming..."
            )
            await self._safety_monitor.wait_until_ready()

        if self.safety_limits.enable_preflight_checks and not skip_preflight:
            result = await self.preflight_check()
            if not result:
                # Notify OEO of preflight failure
                if self._oeo:
                    await self._oeo.notify_preflight_failed(
                        result.failed_checks
                    )
                if not force:
                    raise PreflightCheckError(result=result)
        return await super().arm(force=force)

    async def takeoff(
        self, altitude: float = 5.0, wait: bool = True, timeout: float = 60.0
    ) -> Optional[CommandHandle]:
        if not self._armed:
            await self.arm()
        self._abortable, self.info.takeoff_time = True, time.time()

        if self._safety_checker:
            await validate_takeoff_with_checker(
                self._safety_checker,
                altitude,
                self.state.latitude,
                self.state.longitude,
                raise_on_fail=True,
            )

        try:
            await self._system.action.set_takeoff_altitude(altitude)
            await self._system.action.takeoff()
        except ActionError as e:
            raise TakeoffError(
                message=f"Takeoff failed: {e}",
                target_altitude=altitude,
                reason=str(e),
            )

        handle = CommandHandle(
            command="takeoff",
            completion_condition=lambda: self.state.relative_altitude
            >= altitude * 0.95,
            cancel_action=self.hold,
            timeout=timeout,
            abort_checker=lambda: self._aborted,
            progress_getter=lambda: {
                "current_altitude": self.state.relative_altitude,
                "target_altitude": altitude,
                "altitude_remaining": max(
                    0, altitude - self.state.relative_altitude
                ),
            },
        )
        handle._start()

        if wait:
            try:
                await handle.wait()
                await asyncio.sleep(2)
            except AerpawTimeoutError:
                raise TakeoffTimeoutError(
                    target_altitude=altitude,
                    current_altitude=self.state.relative_altitude,
                    timeout=timeout,
                )
            return None
        return handle

    async def land(
        self, wait: bool = True, timeout: float = 120.0
    ) -> Optional[CommandHandle]:
        self._abortable, self._velocity_loop_active = False, False
        try:
            await self._system.action.land()
        except ActionError as e:
            raise LandingError(
                message=f"Landing failed: {e}",
                current_altitude=self.state.relative_altitude,
                reason=str(e),
            )

        handle = CommandHandle(
            command="land",
            completion_condition=lambda: not self._armed,
            cancel_action=None,
            timeout=timeout,
            abort_checker=None,
            progress_getter=lambda: {
                "current_altitude": self.state.relative_altitude,
                "landed_state": self.state.landed_state.name,
                "armed": self._armed,
            },
        )
        handle._start()

        if wait:
            try:
                await handle.wait()
            except AerpawTimeoutError:
                raise LandingTimeoutError(
                    current_altitude=self.state.relative_altitude,
                    timeout=timeout,
                )
            return None
        return handle

    async def rtl(
        self, wait: bool = True, timeout: float = 300.0
    ) -> Optional[CommandHandle]:
        self._abortable, self._velocity_loop_active = False, False
        try:
            await self._system.action.return_to_launch()
        except ActionError as e:
            raise NavigationError(message=f"RTL failed: {e}", reason=str(e))

        def progress():
            r = {
                "current_altitude": self.state.relative_altitude,
                "landed_state": self.state.landed_state.name,
                "armed": self._armed,
            }
            if self._home:
                r["distance_to_home"] = self.position.distance_to(self._home)
            return r

        handle = CommandHandle(
            command="rtl",
            completion_condition=lambda: not self._armed,
            cancel_action=None,
            timeout=timeout,
            abort_checker=None,
            progress_getter=progress,
        )
        handle._start()
        if wait:
            await handle.wait()
            return None
        return handle

    async def abort(self, rtl: bool = True):
        self._aborted, self._velocity_loop_active = True, False
        await (self.rtl(wait=False) if rtl else self.hold())

    async def hold(self):
        try:
            await self._system.action.hold()
        except ActionError:
            await self.set_velocity(VectorNED(0, 0, 0))

    async def goto(
        self,
        latitude: Optional[float] = None,
        longitude: Optional[float] = None,
        altitude: Optional[float] = None,
        coordinates: Optional[Coordinate] = None,
        tolerance: float = 2.0,
        speed: Optional[float] = None,
        heading: Optional[float] = None,
        timeout: float = 300.0,
        wait: bool = True,
    ) -> Optional[CommandHandle]:
        target = (
            coordinates
            if coordinates
            else (
                Coordinate(
                    latitude,
                    longitude,
                    altitude or self.state.relative_altitude,
                )
                if latitude and longitude
                else None
            )
        )
        if not target:
            raise ValueError(
                "Must provide either coordinates or latitude/longitude"
            )

        if self.safety_limits.enable_parameter_validation:
            for name, check, val in [
                ("target", validate_coordinate(target, "target"), target),
                (
                    "altitude",
                    validate_altitude(target.altitude, "altitude"),
                    target.altitude,
                ),
                (
                    "tolerance",
                    validate_tolerance(tolerance, "tolerance"),
                    tolerance,
                ),
                ("timeout", validate_timeout(timeout, "timeout"), timeout),
            ]:
                if not check:
                    raise ParameterValidationError(
                        message=check.message, parameter=name, value=val
                    )
            if speed:
                check = validate_speed(speed, self.safety_limits, "speed")
                if not check:
                    speed = (
                        clamp_speed(speed, self.safety_limits)
                        if self.safety_limits.auto_clamp_values
                        else None
                    )
                    if speed is None:
                        raise ParameterValidationError(
                            message=check.message,
                            parameter="speed",
                            value=speed,
                        )

        if self._safety_checker:
            await validate_waypoint_with_checker(
                self._safety_checker, self.position, target, raise_on_fail=True
            )
            if speed:
                await validate_speed_with_checker(
                    self._safety_checker, speed, raise_on_fail=True
                )

        if speed:
            await self._system.action.set_maximum_speed(speed)
        self._current_heading = (
            heading
            or self._current_heading
            or self.position.bearing_to(target)
        )

        try:
            await self._system.action.goto_location(
                target.latitude,
                target.longitude,
                target.altitude + (self._home.altitude if self._home else 0),
                self._current_heading or 0,
            )
        except ActionError as e:
            raise NavigationError(
                message=f"Goto failed: {e}",
                target=target,
                current_position=self.position,
                reason=str(e),
            )

        handle = CommandHandle(
            command="goto",
            completion_condition=lambda: self.position.distance_to(target)
            <= tolerance,
            cancel_action=self.hold,
            timeout=timeout,
            abort_checker=lambda: self._aborted,
            progress_getter=lambda: {
                "distance": self.position.distance_to(target),
                "target": target,
                "tolerance": tolerance,
            },
        )
        handle._start()

        if wait:
            try:
                await handle.wait()
            except AerpawTimeoutError:
                raise GotoTimeoutError(
                    target=target,
                    distance_remaining=self.position.distance_to(target),
                    timeout=timeout,
                )
            return None
        return handle

    async def set_heading(
        self, degrees: float, blocking: bool = True, timeout: float = 30.0
    ) -> Optional[CommandHandle]:
        target_heading = degrees % 360
        self._current_heading = target_heading

        try:
            await self._system.offboard.set_position_ned(
                PositionNedYaw(
                    0, 0, -self.state.relative_altitude, self._current_heading
                )
            )
            await self._system.offboard.start()
        except (OffboardError, ActionError) as e:
            raise ModeChangeError(
                message=f"Heading change failed: {e}",
                target_mode="offboard",
                reason=str(e),
            )

        def diff():
            return abs((self.state.heading - target_heading + 180) % 360 - 180)

        async def stop():
            try:
                await self._system.offboard.stop()
            except Exception:
                pass

        handle = CommandHandle(
            command="set_heading",
            completion_condition=lambda: diff() < 5,
            cancel_action=stop,
            timeout=timeout,
            abort_checker=lambda: self._aborted,
            progress_getter=lambda: {
                "current_heading": self.state.heading,
                "target_heading": target_heading,
                "heading_diff": diff(),
            },
        )
        handle._start()

        if blocking:
            try:
                await handle.wait()
            finally:
                await stop()
            return None
        return handle

    async def point_at(self, target: Optional[Coordinate] = None):
        if target is None:
            if self.state.velocity.magnitude(ignore_vertical=True) > 0.5:
                await self.set_heading(self.state.velocity.heading())
        else:
            await self.set_heading(self.position.bearing_to(target))

    async def move_in_current_direction(
        self, distance: float, speed: float = 5.0
    ):
        await self.move_in_direction(
            distance, self._current_heading or self.state.heading, speed
        )

    async def move_in_direction(
        self, distance: float, degrees: float, speed: float = 5.0
    ):
        rad = math.radians(degrees)
        await self.goto(
            coordinates=self.position
            + VectorNED(distance * math.cos(rad), distance * math.sin(rad), 0),
            speed=speed,
        )

    async def move_towards(
        self, target: Coordinate, distance: float, speed: float = 5.0
    ):
        await self.move_in_direction(
            distance, self.position.bearing_to(target), speed
        )

    async def set_velocity(
        self,
        velocity: VectorNED,
        heading: Optional[float] = None,
        duration: Optional[float] = None,
        wait: bool = True,
    ) -> Optional[CommandHandle]:
        if heading:
            self._current_heading = heading
        try:
            await self._system.offboard.set_velocity_ned(
                VelocityNedYaw(
                    velocity.north,
                    velocity.east,
                    velocity.down,
                    self._current_heading or self.state.heading,
                )
            )
            try:
                await self._system.offboard.start()
            except OffboardError:
                pass
            self._velocity_loop_active = True
        except (OffboardError, ActionError) as e:
            raise AerpawOffboardError(
                message=f"Set velocity failed: {e}", reason=str(e)
            )

        if duration is None:
            return None

        start = time.time()

        async def stop():
            self._velocity_loop_active = False
            try:
                await self._system.offboard.stop()
            except Exception:
                pass

        handle = CommandHandle(
            command="set_velocity",
            completion_condition=lambda: time.time() - start >= duration,
            cancel_action=stop,
            timeout=duration + 5,
            abort_checker=lambda: self._aborted,
            progress_getter=lambda: {
                "elapsed": time.time() - start,
                "duration": duration,
                "time_remaining": max(0, duration - (time.time() - start)),
                "velocity": velocity,
            },
        )
        handle._start()

        if wait:
            try:
                await handle.wait()
            finally:
                await stop()
            return None
        return handle

    async def set_groundspeed(self, speed: float):
        await self._system.action.set_maximum_speed(speed)

    async def set_altitude(self, altitude: float, tolerance: float = 0.5):
        await self.goto(
            coordinates=Coordinate(
                self.state.latitude, self.state.longitude, altitude
            ),
            tolerance=tolerance,
        )

    async def orbit(
        self,
        center: Coordinate,
        radius: float,
        speed: float = 5.0,
        clockwise: bool = True,
        revolutions: float = 1.0,
        wait: bool = True,
    ) -> Optional[CommandHandle]:
        orbit_time = (2 * math.pi * radius * revolutions) / speed
        angular_speed = (
            (360 * revolutions) / orbit_time * (1 if clockwise else -1)
        )
        start_angle = math.degrees(
            math.atan2(
                center.vector_to(self.position).east,
                center.vector_to(self.position).north,
            )
        )
        start_time, orbit_active = time.time(), True

        async def stop():
            nonlocal orbit_active
            orbit_active, self._velocity_loop_active = False, False
            try:
                await self._system.offboard.stop()
            except Exception:
                pass

        async def execute():
            while (
                orbit_active
                and time.time() - start_time < orbit_time
                and not self._aborted
            ):
                elapsed = time.time() - start_time
                tangent = math.radians(
                    start_angle
                    + angular_speed * elapsed
                    + (90 if clockwise else -90)
                )
                await self.set_velocity(
                    VectorNED(
                        speed * math.cos(tangent), speed * math.sin(tangent), 0
                    ),
                    heading=self.position.bearing_to(center),
                )
                await asyncio.sleep(_POLLING_DELAY * 10)
            await stop()

        task = asyncio.create_task(execute())

        def progress():
            elapsed, angle = time.time() - start_time, abs(
                angular_speed * (time.time() - start_time)
            )
            return {
                "elapsed": elapsed,
                "orbit_time": orbit_time,
                "time_remaining": max(0, orbit_time - elapsed),
                "angle_completed": angle,
                "total_angle": 360 * revolutions,
                "revolutions_completed": angle / 360,
                "progress_percent": min(
                    100, angle / (360 * revolutions) * 100
                ),
            }

        handle = CommandHandle(
            command="orbit",
            completion_condition=lambda: time.time() - start_time >= orbit_time
            or not orbit_active,
            cancel_action=stop,
            timeout=orbit_time + 10,
            abort_checker=lambda: self._aborted,
            progress_getter=progress,
        )
        handle._start()

        if wait:
            try:
                await handle.wait()
                await task
            except Exception:
                await stop()
                raise
            return None
        return handle

    def add_waypoint(self, waypoint: Union[Coordinate, Waypoint]):
        self._waypoints.append(
            Waypoint(coordinate=waypoint)
            if isinstance(waypoint, Coordinate)
            else waypoint
        )

    def add_waypoints(self, waypoints: List[Union[Coordinate, Waypoint]]):
        for wp in waypoints:
            self.add_waypoint(wp)

    def clear_waypoints(self):
        self._waypoints.clear()

    async def execute_waypoints(self, tolerance: float = 2.0):
        for wp in self._waypoints:
            await self.goto(
                coordinates=wp.coordinate,
                tolerance=wp.acceptance_radius or tolerance,
                speed=wp.speed,
            )
            if wp.hold_time > 0:
                await asyncio.sleep(wp.hold_time)
        self._waypoints.clear()


class Rover(Vehicle):
    """Ground rover vehicle class."""

    def __init__(self, connection: str = "udp://:14540"):
        super().__init__(connection)
        self._waypoints: List[Waypoint] = []

    async def goto(
        self,
        latitude: Optional[float] = None,
        longitude: Optional[float] = None,
        coordinates: Optional[Coordinate] = None,
        tolerance: float = 2.0,
        speed: Optional[float] = None,
    ):
        target = Coordinate(
            coordinates.latitude if coordinates else latitude,
            coordinates.longitude if coordinates else longitude,
            0,
        )
        if speed:
            await self._system.action.set_maximum_speed(speed)
        try:
            await self._system.action.goto_location(
                target.latitude, target.longitude, 0, 0
            )
            await self._await_condition(
                lambda: self.position.ground_distance_to(target) <= tolerance,
                timeout=300,
                message="Goto timeout",
            )
        except ActionError as e:
            raise RuntimeError(f"Goto failed: {e}")

    async def stop(self):
        try:
            await self._system.action.hold()
        except ActionError:
            pass

    async def move_in_direction(
        self, distance: float, degrees: float, speed: float = 2.0
    ):
        rad = math.radians(degrees)
        await self.goto(
            coordinates=self.position
            + VectorNED(distance * math.cos(rad), distance * math.sin(rad), 0),
            speed=speed,
        )

    async def move_towards(
        self, target: Coordinate, distance: float, speed: float = 2.0
    ):
        await self.move_in_direction(
            distance, self.position.bearing_to(target), speed
        )

    def add_waypoint(self, waypoint: Union[Coordinate, Waypoint]):
        self._waypoints.append(
            Waypoint(coordinate=waypoint)
            if isinstance(waypoint, Coordinate)
            else waypoint
        )

    def add_waypoints(self, waypoints: List[Union[Coordinate, Waypoint]]):
        for wp in waypoints:
            self.add_waypoint(wp)

    def clear_waypoints(self):
        self._waypoints.clear()

    async def execute_waypoints(self, tolerance: float = 2.0):
        for wp in self._waypoints:
            await self.goto(
                coordinates=wp.coordinate,
                tolerance=wp.acceptance_radius or tolerance,
                speed=wp.speed,
            )
            if wp.hold_time > 0:
                await asyncio.sleep(wp.hold_time)
        self._waypoints.clear()


__all__ = [
    "Vehicle",
    "Drone",
    "Rover",
    "CommandHandle",
    "CommandStatus",
    "CommandResult",
    "VehicleEvent",
    "StateContainer",
    "GPSContainer",
    "BatteryContainer",
    "InfoContainer",
]
