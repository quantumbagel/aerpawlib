"""
Vehicle classes for aerpawlib v2 API using MAVSDK.

This module provides Drone and Rover classes with a clean, Pythonic interface
for vehicle control.
"""
from __future__ import annotations

import asyncio
import logging
import math
import time
from typing import Any, Callable, Dict, List, Optional

try:
    from mavsdk import System
    from mavsdk.offboard import PositionNedYaw, VelocityNedYaw, OffboardError
    from mavsdk.action import ActionError
    from mavsdk.telemetry import FlightMode as MavFlightMode, LandedState as MavLandedState
    MAVSDK_AVAILABLE = True
except ImportError:
    MAVSDK_AVAILABLE = False
    System = None
    PositionNedYaw = None
    VelocityNedYaw = None
    OffboardError = Exception
    ActionError = Exception
    MavFlightMode = None
    MavLandedState = None

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
    ReconnectionError,
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
)

# Safety features
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
    PreflightCheckError,
    ParameterValidationError,
    validate_waypoint_with_checker,
    validate_speed_with_checker,
    validate_takeoff_with_checker,
)


# Polling delays
_POLLING_DELAY = 0.01  # seconds
_TELEMETRY_UPDATE_RATE = 10  # Hz
_HEARTBEAT_TIMEOUT = 5.0  # seconds without heartbeat before considered disconnected

# Module logger - use modular logging system
from .logging import get_logger, LogComponent
logger = get_logger(LogComponent.VEHICLE)


# ============================================================================
# Command Handle - Track command execution and completion
# ============================================================================

from enum import Enum, auto
from dataclasses import dataclass, field


class CommandStatus(Enum):
    """Status of a command being executed."""
    PENDING = auto()      # Command is queued but not yet started
    RUNNING = auto()      # Command is currently executing
    COMPLETED = auto()    # Command finished successfully
    FAILED = auto()       # Command failed with an error
    CANCELLED = auto()    # Command was cancelled by user
    TIMED_OUT = auto()    # Command timed out before completion


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
        """Duration of the command in seconds."""
        return self.end_time - self.start_time

    @property
    def succeeded(self) -> bool:
        """True if the command completed successfully."""
        return self.status == CommandStatus.COMPLETED

    @property
    def was_cancelled(self) -> bool:
        """True if the command was cancelled."""
        return self.status == CommandStatus.CANCELLED


class CommandHandle:
    """
    Handle for tracking and controlling command execution.

    CommandHandle allows non-blocking command execution with the ability to:
    - Check if the command is still running
    - Wait for completion (with optional timeout)
    - Cancel the command
    - Get the result once complete

    Example:
        # Non-blocking goto
        handle = await drone.goto(latitude=51.5, longitude=-0.1, wait=False)

        # Do other things while moving
        while handle.is_running:
            print(f"Distance remaining: {handle.progress.get('distance', 'unknown')}")
            await asyncio.sleep(1)

        # Check result
        if handle.succeeded:
            print("Arrived at destination!")
        elif handle.was_cancelled:
            print("Goto was cancelled")
        else:
            print(f"Goto failed: {handle.error}")

        # Or just wait for completion
        handle = await drone.goto(latitude=51.5, longitude=-0.1, wait=False)
        await handle.wait()  # Blocks until complete

        # Cancel a command
        handle = await drone.goto(latitude=51.5, longitude=-0.1, wait=False)
        await asyncio.sleep(5)
        await handle.cancel()  # Stop the goto
    """

    def __init__(
        self,
        command: str,
        completion_condition: Callable[[], bool],
        cancel_action: Optional[Callable[[], Any]] = None,
        timeout: Optional[float] = None,
        abort_checker: Optional[Callable[[], bool]] = None,
        progress_getter: Optional[Callable[[], Dict[str, Any]]] = None,
    ):
        """
        Initialize a command handle.

        Args:
            command: Name of the command (e.g., "goto", "takeoff")
            completion_condition: Callable that returns True when command is complete
            cancel_action: Optional async callable to execute when cancelling
            timeout: Optional timeout in seconds
            abort_checker: Optional callable to check if vehicle has been aborted
            progress_getter: Optional callable to get progress information
        """
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
        """Name of the command."""
        return self._command

    @property
    def status(self) -> CommandStatus:
        """Current status of the command."""
        return self._status

    @property
    def is_pending(self) -> bool:
        """True if the command hasn't started yet."""
        return self._status == CommandStatus.PENDING

    @property
    def is_running(self) -> bool:
        """True if the command is currently executing."""
        return self._status == CommandStatus.RUNNING

    @property
    def is_complete(self) -> bool:
        """True if the command has finished (success, failure, or cancelled)."""
        return self._status in (
            CommandStatus.COMPLETED,
            CommandStatus.FAILED,
            CommandStatus.CANCELLED,
            CommandStatus.TIMED_OUT,
        )

    @property
    def succeeded(self) -> bool:
        """True if the command completed successfully."""
        return self._status == CommandStatus.COMPLETED

    @property
    def was_cancelled(self) -> bool:
        """True if the command was cancelled."""
        return self._status == CommandStatus.CANCELLED

    @property
    def timed_out(self) -> bool:
        """True if the command timed out."""
        return self._status == CommandStatus.TIMED_OUT

    @property
    def error(self) -> Optional[Exception]:
        """The exception if the command failed, None otherwise."""
        return self._error

    @property
    def elapsed_time(self) -> float:
        """Time elapsed since command started in seconds."""
        if self._end_time is not None:
            return self._end_time - self._start_time
        return time.time() - self._start_time

    @property
    def time_remaining(self) -> Optional[float]:
        """Estimated time remaining until timeout, or None if no timeout."""
        if self._timeout is None:
            return None
        remaining = self._timeout - self.elapsed_time
        return max(0, remaining)

    @property
    def progress(self) -> Dict[str, Any]:
        """
        Get current progress information.

        Returns a dictionary with command-specific progress data.
        Common fields include:
        - distance: Distance remaining (for goto)
        - altitude_diff: Altitude difference (for takeoff/land)
        - heading_diff: Heading difference (for set_heading)
        """
        if self._progress_getter:
            return self._progress_getter()
        return {}

    def result(self) -> CommandResult:
        """
        Get the command result.

        Returns:
            CommandResult with status, timing, and error information

        Raises:
            RuntimeError: If command is still running
        """
        if not self.is_complete:
            raise RuntimeError("Command is still running. Use await handle.wait() first.")

        return CommandResult(
            status=self._status,
            command=self._command,
            start_time=self._start_time,
            end_time=self._end_time or time.time(),
            error=self._error,
            details=self.progress,
        )

    def _start(self) -> None:
        """Start the command execution monitoring."""
        self._status = CommandStatus.RUNNING
        self._task = asyncio.create_task(self._monitor())

    async def _monitor(self) -> None:
        """Monitor command completion in the background."""
        try:
            while not self._cancelled:
                # Check for completion
                if self._completion_condition():
                    self._status = CommandStatus.COMPLETED
                    self._end_time = time.time()
                    self._completion_event.set()
                    return

                # Check for abort
                if self._abort_checker and self._abort_checker():
                    self._status = CommandStatus.CANCELLED
                    self._error = AbortError("Vehicle abort triggered")
                    self._end_time = time.time()
                    self._completion_event.set()
                    return

                # Check for timeout
                if self._timeout and self.elapsed_time > self._timeout:
                    self._status = CommandStatus.TIMED_OUT
                    self._error = AerpawTimeoutError(
                        f"{self._command} timed out after {self._timeout}s",
                        timeout=self._timeout
                    )
                    self._end_time = time.time()
                    self._completion_event.set()
                    return

                await asyncio.sleep(_POLLING_DELAY)

            # Cancelled flag was set
            self._status = CommandStatus.CANCELLED
            self._end_time = time.time()
            self._completion_event.set()

        except asyncio.CancelledError:
            self._status = CommandStatus.CANCELLED
            self._end_time = time.time()
            self._completion_event.set()
            raise

        except Exception as e:
            self._status = CommandStatus.FAILED
            self._error = e
            self._end_time = time.time()
            self._completion_event.set()

    async def wait(self, timeout: Optional[float] = None) -> "CommandHandle":
        """
        Wait for the command to complete.

        Args:
            timeout: Optional additional timeout for waiting (None = use command timeout)

        Returns:
            Self for chaining

        Raises:
            CommandCancelledError: If the command was cancelled
            TimeoutError: If the command timed out (either from command or wait timeout)
            Exception: Any error that occurred during command execution
        """
        if self.is_complete:
            return self._raise_if_error()

        wait_timeout = timeout or self._timeout

        try:
            if wait_timeout:
                await asyncio.wait_for(
                    self._completion_event.wait(),
                    timeout=wait_timeout
                )
            else:
                await self._completion_event.wait()
        except asyncio.TimeoutError:
            self._status = CommandStatus.TIMED_OUT
            self._error = AerpawTimeoutError(
                f"Wait for {self._command} timed out",
                timeout=wait_timeout
            )
            self._end_time = time.time()

        return self._raise_if_error()

    def _raise_if_error(self) -> "CommandHandle":
        """Raise stored error if command failed."""
        if self._status == CommandStatus.CANCELLED:
            raise CommandCancelledError(
                f"{self._command} was cancelled",
                command=self._command
            )
        if self._status == CommandStatus.TIMED_OUT and self._error:
            raise self._error
        if self._status == CommandStatus.FAILED and self._error:
            raise self._error
        return self

    async def cancel(self, execute_cancel_action: bool = True) -> bool:
        """
        Cancel the command.

        Args:
            execute_cancel_action: If True, execute the cancel action (e.g., hold position)

        Returns:
            True if command was cancelled, False if already complete
        """
        if self.is_complete:
            return False

        self._cancelled = True

        # Execute cancel action (e.g., hold position)
        if execute_cancel_action and self._cancel_action:
            try:
                result = self._cancel_action()
                if asyncio.iscoroutine(result):
                    await result
            except Exception as e:
                logger.warning(f"Cancel action failed: {e}")

        # Cancel the monitoring task
        if self._task and not self._task.done():
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

        self._status = CommandStatus.CANCELLED
        self._end_time = time.time()
        self._completion_event.set()

        logger.debug(f"Command '{self._command}' cancelled")
        return True

    def __await__(self):
        """Allow awaiting the handle directly."""
        return self.wait().__await__()

    def __repr__(self) -> str:
        return f"CommandHandle({self._command}, status={self._status.name}, elapsed={self.elapsed_time:.2f}s)"


def _flight_mode_from_mavsdk(mode) -> FlightMode:
    """Convert MAVSDK flight mode to our enum."""
    if not MAVSDK_AVAILABLE:
        return FlightMode.UNKNOWN
    mode_map = {
        MavFlightMode.MANUAL: FlightMode.MANUAL,
        MavFlightMode.STABILIZED: FlightMode.STABILIZED,
        MavFlightMode.ALTITUDE: FlightMode.ALTITUDE,
        MavFlightMode.POSCTL: FlightMode.POSITION,
        MavFlightMode.OFFBOARD: FlightMode.OFFBOARD,
        MavFlightMode.HOLD: FlightMode.HOLD,
        MavFlightMode.MISSION: FlightMode.MISSION,
        MavFlightMode.RETURN_TO_LAUNCH: FlightMode.RETURN_TO_LAUNCH,
        MavFlightMode.LAND: FlightMode.LAND,
        MavFlightMode.TAKEOFF: FlightMode.TAKEOFF,
        MavFlightMode.FOLLOW_ME: FlightMode.FOLLOW_ME,
    }
    return mode_map.get(mode, FlightMode.UNKNOWN)


def _landed_state_from_mavsdk(state) -> LandedState:
    """Convert MAVSDK landed state to our enum."""
    if not MAVSDK_AVAILABLE:
        return LandedState.UNKNOWN
    state_map = {
        MavLandedState.ON_GROUND: LandedState.ON_GROUND,
        MavLandedState.IN_AIR: LandedState.IN_AIR,
        MavLandedState.TAKING_OFF: LandedState.TAKING_OFF,
        MavLandedState.LANDING: LandedState.LANDING,
    }
    return state_map.get(state, LandedState.UNKNOWN)


class _StateContainer:
    """
    Container for drone state with property access.

    This class is updated by telemetry subscriptions and provides
    a clean interface for state access.
    """

    def __init__(self):
        self._heading: float = 0.0
        self._velocity: VectorNED = VectorNED()
        self._attitude: Attitude = Attitude()
        self._altitude: float = 0.0
        self._relative_altitude: float = 0.0
        self._latitude: float = 0.0
        self._longitude: float = 0.0
        self._groundspeed: float = 0.0
        self._airspeed: float = 0.0
        self._climb_rate: float = 0.0
        self._flight_mode: FlightMode = FlightMode.UNKNOWN
        self._landed_state: LandedState = LandedState.UNKNOWN

    @property
    def heading(self) -> float:
        """Current heading in degrees (0-360, 0=North)."""
        return self._heading

    @property
    def velocity(self) -> VectorNED:
        """Current velocity in NED frame."""
        return self._velocity

    @property
    def attitude(self) -> Attitude:
        """Current attitude (roll, pitch, yaw)."""
        return self._attitude

    @property
    def altitude(self) -> float:
        """Absolute altitude in meters (MSL)."""
        return self._altitude

    @property
    def relative_altitude(self) -> float:
        """Altitude above home/takeoff point in meters."""
        return self._relative_altitude

    @property
    def latitude(self) -> float:
        """Current latitude in degrees."""
        return self._latitude

    @property
    def longitude(self) -> float:
        """Current longitude in degrees."""
        return self._longitude

    @property
    def groundspeed(self) -> float:
        """Ground speed in m/s."""
        return self._groundspeed

    @property
    def airspeed(self) -> float:
        """Air speed in m/s."""
        return self._airspeed

    @property
    def climb_rate(self) -> float:
        """Vertical speed in m/s (positive = ascending)."""
        return self._climb_rate

    @property
    def flight_mode(self) -> FlightMode:
        """Current flight mode."""
        return self._flight_mode

    @property
    def landed_state(self) -> LandedState:
        """Current landed state."""
        return self._landed_state

    @property
    def position(self) -> Coordinate:
        """Current position as a Coordinate."""
        return Coordinate(self._latitude, self._longitude, self._relative_altitude)

    @property
    def speed(self) -> float:
        """3D speed in m/s."""
        return self._velocity.magnitude()

    @property
    def horizontal_speed(self) -> float:
        """Horizontal speed in m/s."""
        return self._velocity.magnitude(ignore_vertical=True)

    @property
    def is_in_air(self) -> bool:
        """True if the vehicle is currently airborne."""
        return self._landed_state == LandedState.IN_AIR

    def snapshot(self) -> DroneState:
        """Create a snapshot of the current state."""
        return DroneState(
            heading=self._heading,
            velocity=self._velocity,
            attitude=self._attitude,
            altitude=self._altitude,
            relative_altitude=self._relative_altitude,
            latitude=self._latitude,
            longitude=self._longitude,
            groundspeed=self._groundspeed,
            airspeed=self._airspeed,
            climb_rate=self._climb_rate,
            flight_mode=self._flight_mode,
            landed_state=self._landed_state,
        )


class _GPSContainer:
    """Container for GPS information."""

    def __init__(self):
        self._satellites: int = 0
        self._fix_type: int = 0

    @property
    def satellites(self) -> int:
        """Number of visible satellites."""
        return self._satellites

    @property
    def fix_type(self) -> int:
        """GPS fix type (0-1: no fix, 2: 2D, 3: 3D, 4+: better)."""
        return self._fix_type

    @property
    def quality(self) -> str:
        """Human-readable GPS quality."""
        if self._fix_type == 0:
            return "No GPS"
        elif self._fix_type == 1:
            return "No Fix"
        elif self._fix_type == 2:
            return "2D Fix"
        elif self._fix_type == 3:
            return "3D Fix"
        elif self._fix_type == 4:
            return "DGPS"
        elif self._fix_type == 5:
            return "RTK Float"
        elif self._fix_type == 6:
            return "RTK Fixed"
        return "Unknown"

    @property
    def has_fix(self) -> bool:
        """True if GPS has at least a 2D fix."""
        return self._fix_type >= 2


class _BatteryContainer:
    """Container for battery information."""

    def __init__(self):
        self._id: int = 0
        self._voltage: float = 0.0
        self._current: float = 0.0
        self._charge: float = 0.0
        self._temperature: Optional[float] = None
        self._consumption: float = 0.0

    @property
    def id(self) -> int:
        """Battery ID."""
        return self._id

    @property
    def voltage(self) -> float:
        """Battery voltage in Volts."""
        return self._voltage

    @property
    def current(self) -> float:
        """Current draw in Amperes."""
        return self._current

    @property
    def charge(self) -> float:
        """Charge level as decimal (0.0 to 1.0)."""
        return self._charge

    @property
    def percentage(self) -> float:
        """Charge level as percentage (0 to 100)."""
        return self._charge * 100

    @property
    def temperature(self) -> Optional[float]:
        """Battery temperature in Celsius, if available."""
        return self._temperature

    @property
    def consumption(self) -> float:
        """Consumed capacity in Ampere-hours."""
        return self._consumption

    @property
    def is_low(self) -> bool:
        """True if battery is below 20%."""
        return self._charge < 0.20

    @property
    def is_critical(self) -> bool:
        """True if battery is below 10%."""
        return self._charge < 0.10


class _InfoContainer:
    """Container for static vehicle information."""

    def __init__(self):
        self._hardware_uuid: str = ""
        self._legacy_uuid: str = ""
        self._vendor_id: int = 0
        self._vendor_name: str = ""
        self._product_id: int = 0
        self._product_name: str = ""
        self._version: str = ""
        self._time_boot_ms: int = 0
        self._flight_id: str = ""
        self._arm_time: Optional[float] = None
        self._takeoff_time: Optional[float] = None

    @property
    def hardware_uuid(self) -> str:
        return self._hardware_uuid

    @property
    def legacy_uuid(self) -> str:
        return self._legacy_uuid

    @property
    def vendor_id(self) -> int:
        return self._vendor_id

    @property
    def vendor_name(self) -> str:
        return self._vendor_name

    @property
    def product_id(self) -> int:
        return self._product_id

    @property
    def product_name(self) -> str:
        return self._product_name

    @property
    def version(self) -> str:
        return self._version

    @property
    def time_since_boot_ms(self) -> int:
        return self._time_boot_ms

    @property
    def flight_id(self) -> str:
        return self._flight_id

    @property
    def time_since_arm_ms(self) -> int:
        if self._arm_time is None:
            return 0
        return int((time.time() - self._arm_time) * 1000)

    @property
    def time_since_takeoff_ms(self) -> int:
        if self._takeoff_time is None:
            return 0
        return int((time.time() - self._takeoff_time) * 1000)


class Vehicle:
    """
    Base vehicle class providing common functionality.

    This class handles MAVSDK connection, telemetry subscriptions,
    and basic vehicle operations. Specific vehicle types (Drone, Rover)
    extend this with movement-specific methods.
    """

    def __init__(self, connection: str = "udp://:14540"):
        """
        Initialize the vehicle connection.

        Args:
            connection: MAVSDK connection string (e.g., "udp://:14540",
                       "serial:///dev/ttyUSB0:57600")
        """
        if not MAVSDK_AVAILABLE:
            raise ImportError(
                "MAVSDK is not installed. Install with: pip install mavsdk"
            )

        self._connection_string = connection
        self._system: Optional[System] = None
        self._connected = False
        self._armed = False
        self._is_armable = False

        # State containers with property access
        self.state = _StateContainer()
        self.gps = _GPSContainer()
        self.battery = _BatteryContainer()
        self.info = _InfoContainer()

        # Home position
        self._home: Optional[Coordinate] = None

        # Internal state
        self._throttle: float = 0.0
        self._telemetry_tasks: List[asyncio.Task] = []
        self._running = True

        # Movement state
        self._current_heading: Optional[float] = None
        self._velocity_loop_active = False
        self._abortable = False
        self._aborted = False

        # Connection health tracking
        self._last_heartbeat: float = 0.0
        self._auto_reconnect: bool = False
        self._reconnect_attempts: int = 0
        self._max_reconnect_attempts: int = 3

        # Event callbacks
        self._callbacks: Dict[str, List[Callable]] = {
            "on_connect": [],
            "on_disconnect": [],
            "on_reconnect": [],
            "on_arm": [],
            "on_disarm": [],
            "on_low_battery": [],
            "on_critical_battery": [],
            "on_mode_change": [],
        }
        self._low_battery_triggered = False
        self._critical_battery_triggered = False

        # Heartbeat monitor task
        self._heartbeat_monitor_task: Optional[asyncio.Task] = None

        # Telemetry recording
        self._recording = False
        self._flight_log: List[Dict[str, Any]] = []
        self._recording_interval: float = 0.1  # seconds

    # Context manager support
    async def __aenter__(self) -> "Vehicle":
        """Async context manager entry."""
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb) -> None:
        """Async context manager exit."""
        await self.disconnect()

    async def connect(
        self,
        timeout: float = 30.0,
        auto_reconnect: bool = False,
        retry_count: int = 3,
        retry_delay: float = 2.0
    ) -> bool:
        """
        Connect to the vehicle with automatic retry support.

        Args:
            timeout: Maximum time to wait for each connection attempt in seconds
            auto_reconnect: If True, automatically attempt to reconnect on connection loss
            retry_count: Number of connection attempts before giving up
            retry_delay: Delay between retry attempts in seconds

        Returns:
            True if connected successfully

        Raises:
            ConnectionTimeoutError: If all connection attempts time out
            AerpawConnectionError: If connection fails for other reasons
        """
        self._auto_reconnect = auto_reconnect
        self._max_reconnect_attempts = retry_count

        last_error: Optional[Exception] = None

        for attempt in range(1, retry_count + 1):
            try:
                logger.info(f"Connecting to {self._connection_string} (attempt {attempt}/{retry_count})...")
                self._system = System()
                await self._system.connect(system_address=self._connection_string)

                # Wait for connection
                start_time = time.time()
                async for conn_state in self._system.core.connection_state():
                    if conn_state.is_connected:
                        self._connected = True
                        self._last_heartbeat = time.time()
                        self._reconnect_attempts = 0
                        logger.info("Connected to vehicle")
                        break
                    if time.time() - start_time > timeout:
                        raise ConnectionTimeoutError(
                            f"Connection timeout after {timeout}s",
                            timeout=timeout,
                            address=self._connection_string,
                            attempt=attempt,
                            max_attempts=retry_count
                        )
                    await asyncio.sleep(_POLLING_DELAY)

                if not self._connected:
                    continue

                # Start telemetry subscriptions
                await self._start_telemetry()

                # Start heartbeat monitor if auto_reconnect is enabled
                if auto_reconnect:
                    self._start_heartbeat_monitor()

                # Get vehicle info
                await self._fetch_vehicle_info()

                # Trigger callbacks
                await self._trigger_callbacks("on_connect")

                return True

            except ConnectionTimeoutError:
                last_error = ConnectionTimeoutError(
                    f"Connection attempt {attempt} timed out",
                    timeout=timeout,
                    address=self._connection_string,
                    attempt=attempt,
                    max_attempts=retry_count
                )
                logger.warning(f"Connection attempt {attempt} timed out, retrying in {retry_delay}s...")
            except Exception as e:
                last_error = AerpawConnectionError(
                    f"Connection failed: {e}",
                    address=self._connection_string,
                    attempt=attempt,
                    max_attempts=retry_count
                )
                logger.warning(f"Connection attempt {attempt} failed: {e}")

            if attempt < retry_count:
                await asyncio.sleep(retry_delay)

        # All attempts failed
        raise last_error or AerpawConnectionError(
            "All connection attempts failed",
            address=self._connection_string,
            attempt=retry_count,
            max_attempts=retry_count
        )

    def _start_heartbeat_monitor(self) -> None:
        """Start the heartbeat monitoring task."""
        if self._heartbeat_monitor_task is not None:
            return

        async def _monitor():
            while self._running and self._connected:
                await asyncio.sleep(1.0)
                if not self.connection_healthy:
                    logger.warning(f"Heartbeat lost (last: {self.seconds_since_heartbeat:.1f}s ago)")
                    await self._trigger_callbacks("on_disconnect")

                    if self._auto_reconnect:
                        await self._attempt_reconnect()
                    else:
                        self._connected = False

        self._heartbeat_monitor_task = asyncio.create_task(_monitor())

    async def _attempt_reconnect(self) -> bool:
        """Attempt to reconnect to the vehicle."""
        self._connected = False
        self._reconnect_attempts += 1

        if self._reconnect_attempts > self._max_reconnect_attempts:
            logger.error(f"Max reconnection attempts ({self._max_reconnect_attempts}) reached")
            raise ReconnectionError(
                f"Failed to reconnect after {self._max_reconnect_attempts} attempts",
                address=self._connection_string,
                attempt=self._reconnect_attempts,
                max_attempts=self._max_reconnect_attempts
            )

        logger.info(f"Attempting reconnection ({self._reconnect_attempts}/{self._max_reconnect_attempts})...")

        # Cancel existing telemetry tasks
        for task in self._telemetry_tasks:
            task.cancel()
        self._telemetry_tasks.clear()

        try:
            # Reconnect
            self._system = System()
            await self._system.connect(system_address=self._connection_string)

            start_time = time.time()
            async for conn_state in self._system.core.connection_state():
                if conn_state.is_connected:
                    self._connected = True
                    self._last_heartbeat = time.time()
                    self._reconnect_attempts = 0
                    logger.info("Reconnected to vehicle")
                    break
                if time.time() - start_time > 10:
                    return False
                await asyncio.sleep(_POLLING_DELAY)

            if self._connected:
                await self._start_telemetry()
                await self._trigger_callbacks("on_reconnect")
                return True

        except Exception as e:
            logger.error(f"Reconnection failed: {e}")

        return False

    async def _start_telemetry(self):
        """Start background telemetry update tasks."""

        async def _position_update():
            async for position in self._system.telemetry.position():
                self._last_heartbeat = time.time()
                self.state._latitude = position.latitude_deg
                self.state._longitude = position.longitude_deg
                self.state._altitude = position.absolute_altitude_m
                self.state._relative_altitude = position.relative_altitude_m

        async def _attitude_update():
            async for attitude in self._system.telemetry.attitude_euler():
                self.state._attitude = Attitude(
                    roll=math.radians(attitude.roll_deg),
                    pitch=math.radians(attitude.pitch_deg),
                    yaw=math.radians(attitude.yaw_deg)
                )
                self.state._heading = attitude.yaw_deg % 360

        async def _velocity_update():
            async for velocity in self._system.telemetry.velocity_ned():
                self.state._velocity = VectorNED(
                    velocity.north_m_s,
                    velocity.east_m_s,
                    velocity.down_m_s
                )
                self.state._groundspeed = math.hypot(velocity.north_m_s, velocity.east_m_s)
                self.state._climb_rate = -velocity.down_m_s

        async def _gps_update():
            async for gps_info in self._system.telemetry.gps_info():
                self.gps._satellites = gps_info.num_satellites
                self.gps._fix_type = gps_info.fix_type.value

        async def _battery_update():
            async for battery in self._system.telemetry.battery():
                self.battery._voltage = battery.voltage_v
                self.battery._charge = battery.remaining_percent / 100.0
                # Check for low/critical battery events
                if self.battery.is_critical and not self._critical_battery_triggered:
                    self._critical_battery_triggered = True
                    logger.warning(f"Critical battery: {self.battery.percentage:.1f}%")
                    await self._trigger_callbacks("on_critical_battery")
                elif self.battery.is_low and not self._low_battery_triggered:
                    self._low_battery_triggered = True
                    logger.warning(f"Low battery: {self.battery.percentage:.1f}%")
                    await self._trigger_callbacks("on_low_battery")

        async def _flight_mode_update():
            async for mode in self._system.telemetry.flight_mode():
                old_mode = self.state._flight_mode
                self.state._flight_mode = _flight_mode_from_mavsdk(mode)
                if old_mode != self.state._flight_mode:
                    logger.debug(f"Flight mode changed: {old_mode} -> {self.state._flight_mode}")
                    await self._trigger_callbacks("on_mode_change", old_mode, self.state._flight_mode)

        async def _landed_state_update():
            async for state in self._system.telemetry.landed_state():
                self.state._landed_state = _landed_state_from_mavsdk(state)

        async def _armed_update():
            async for armed in self._system.telemetry.armed():
                was_armed = self._armed
                self._armed = armed
                if armed and not was_armed:
                    self.info._arm_time = time.time()
                    logger.info("Vehicle armed")
                    await self._trigger_callbacks("on_arm")
                elif not armed and was_armed:
                    self.info._arm_time = None
                    logger.info("Vehicle disarmed")
                    await self._trigger_callbacks("on_disarm")

        async def _health_update():
            async for health in self._system.telemetry.health():
                self._is_armable = (
                    health.is_global_position_ok and
                    health.is_home_position_ok and
                    health.is_armable
                )

        async def _home_update():
            async for home in self._system.telemetry.home():
                self._home = Coordinate(
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
            _landed_state_update(),
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
            info = await self._system.info.get_identification()
            self.info._hardware_uuid = info.hardware_uid
            self.info._legacy_uuid = getattr(info, 'legacy_uid', '')
        except Exception:
            pass

        try:
            version = await self._system.info.get_version()
            self.info._version = f"{version.flight_sw_major}.{version.flight_sw_minor}.{version.flight_sw_patch}"
            self.info._vendor_name = getattr(version, 'vendor_name', '')
            self.info._product_name = getattr(version, 'product_name', '')
        except Exception:
            pass

    async def disconnect(self):
        """Disconnect from the vehicle and clean up resources."""
        self._running = False

        # Stop recording if active
        if self._recording:
            self.stop_recording()

        # Cancel heartbeat monitor
        if self._heartbeat_monitor_task is not None:
            self._heartbeat_monitor_task.cancel()
            try:
                await self._heartbeat_monitor_task
            except asyncio.CancelledError:
                pass
            self._heartbeat_monitor_task = None

        # Cancel telemetry tasks
        for task in self._telemetry_tasks:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

        self._telemetry_tasks.clear()
        self._connected = False
        logger.info("Disconnected from vehicle")

    # Telemetry Recording

    def start_recording(self, interval: float = 0.1) -> None:
        """
        Start recording telemetry data.

        Args:
            interval: Time between recordings in seconds (default 0.1s = 10Hz)

        Example:
            drone.start_recording(interval=0.1)
            # ... fly mission ...
            drone.stop_recording()
            drone.save_flight_log("flight.json")
        """
        if self._recording:
            logger.warning("Recording already in progress")
            return

        self._recording = True
        self._recording_interval = interval
        self._flight_log.clear()
        logger.info(f"Started telemetry recording at {1/interval:.1f}Hz")

        async def _record_loop():
            while self._recording and self._connected:
                self._flight_log.append(self._capture_telemetry_snapshot())
                await asyncio.sleep(self._recording_interval)

        asyncio.create_task(_record_loop())

    def stop_recording(self) -> int:
        """
        Stop recording telemetry data.

        Returns:
            Number of data points recorded
        """
        if not self._recording:
            return len(self._flight_log)

        self._recording = False
        count = len(self._flight_log)
        logger.info(f"Stopped recording. Captured {count} data points")
        return count

    def _capture_telemetry_snapshot(self) -> Dict[str, Any]:
        """Capture a snapshot of current telemetry."""
        return {
            "timestamp": time.time(),
            "latitude": self.state._latitude,
            "longitude": self.state._longitude,
            "altitude": self.state._relative_altitude,
            "heading": self.state._heading,
            "groundspeed": self.state._groundspeed,
            "velocity_north": self.state._velocity.north,
            "velocity_east": self.state._velocity.east,
            "velocity_down": self.state._velocity.down,
            "roll": self.state._attitude.roll,
            "pitch": self.state._attitude.pitch,
            "yaw": self.state._attitude.yaw,
            "battery_voltage": self.battery._voltage,
            "battery_percent": self.battery._charge * 100,
            "gps_satellites": self.gps._satellites,
            "gps_fix_type": self.gps._fix_type,
            "armed": self._armed,
            "flight_mode": self.state._flight_mode.name,
        }

    def get_flight_log(self) -> List[Dict[str, Any]]:
        """
        Get the recorded flight log data.

        Returns:
            List of telemetry snapshots
        """
        return self._flight_log.copy()

    def save_flight_log(self, path: str, format: str = "json") -> None:
        """
        Save the flight log to a file.

        Args:
            path: File path to save to
            format: Output format ("json" or "csv")

        Raises:
            ValueError: If format is not supported
        """
        import json

        if format == "json":
            with open(path, "w") as f:
                json.dump({
                    "metadata": {
                        "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                        "data_points": len(self._flight_log),
                        "interval": self._recording_interval,
                    },
                    "telemetry": self._flight_log
                }, f, indent=2)
        elif format == "csv":
            if not self._flight_log:
                logger.warning("No flight log data to save")
                return

            import csv
            with open(path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=self._flight_log[0].keys())
                writer.writeheader()
                writer.writerows(self._flight_log)
        else:
            raise ValueError(f"Unsupported format: {format}. Use 'json' or 'csv'")

        logger.info(f"Saved flight log to {path} ({len(self._flight_log)} points)")

    def clear_flight_log(self) -> None:
        """Clear the recorded flight log data."""
        self._flight_log.clear()
        logger.debug("Flight log cleared")

    # Event callback registration
    def on(self, event: str, callback: Callable) -> None:
        """
        Register a callback for an event.

        Available events:
            - on_connect: Called when connected to vehicle
            - on_disconnect: Called when disconnected or heartbeat lost
            - on_reconnect: Called when successfully reconnected after connection loss
            - on_arm: Called when vehicle is armed
            - on_disarm: Called when vehicle is disarmed
            - on_low_battery: Called when battery drops below 20%
            - on_critical_battery: Called when battery drops below 10%
            - on_mode_change: Called when flight mode changes (receives old_mode, new_mode)

        Args:
            event: Event name
            callback: Callback function (async or sync)
        """
        if event not in self._callbacks:
            raise ValueError(f"Unknown event: {event}. Available: {list(self._callbacks.keys())}")
        self._callbacks[event].append(callback)

    def off(self, event: str, callback: Callable) -> None:
        """
        Unregister a callback for an event.

        Args:
            event: Event name
            callback: The callback to remove
        """
        if event in self._callbacks and callback in self._callbacks[event]:
            self._callbacks[event].remove(callback)

    async def _trigger_callbacks(self, event: str, *args, **kwargs) -> None:
        """Trigger all callbacks for an event."""
        for callback in self._callbacks.get(event, []):
            try:
                result = callback(*args, **kwargs)
                if asyncio.iscoroutine(result):
                    await result
            except Exception as e:
                logger.error(f"Error in {event} callback: {e}")

    # Utility methods
    async def wait_for_gps_fix(self, min_satellites: int = 6, timeout: float = 60.0) -> bool:
        """
        Wait for GPS to acquire a fix.

        Args:
            min_satellites: Minimum number of satellites required
            timeout: Maximum time to wait in seconds

        Returns:
            True if GPS fix acquired

        Raises:
            TimeoutError: If timeout expires before GPS fix
        """
        logger.info(f"Waiting for GPS fix (min {min_satellites} satellites)...")
        await self._await_condition(
            lambda: self.gps.has_fix and self.gps.satellites >= min_satellites,
            timeout=timeout,
            message=f"GPS fix timeout after {timeout}s"
        )
        logger.info(f"GPS fix acquired: {self.gps.satellites} satellites, {self.gps.quality}")
        return True

    async def wait_for_armable(self, timeout: float = 60.0) -> bool:
        """
        Wait until the vehicle is ready to be armed.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if vehicle is armable

        Raises:
            TimeoutError: If timeout expires
        """
        logger.info("Waiting for vehicle to be armable...")
        await self._await_condition(
            lambda: self._is_armable,
            timeout=timeout,
            message=f"Armable timeout after {timeout}s"
        )
        logger.info("Vehicle is ready to arm")
        return True

    def reset_abort(self) -> None:
        """
        Reset the abort flag to allow new operations.

        Call this after handling an abort to resume normal operations.
        """
        self._aborted = False
        logger.debug("Abort flag reset")

    async def set_home(self, position: Optional[Coordinate] = None) -> bool:
        """
        Set the home position.

        Args:
            position: New home position. If None, uses current position.

        Returns:
            True if home position was set successfully
        """
        if position is None:
            position = self.position

        try:
            await self._system.action.set_current_home_position(
                position.latitude,
                position.longitude,
                position.altitude
            )
            self._home = position
            logger.info(f"Home position set to {position}")
            return True
        except ActionError as e:
            logger.error(f"Failed to set home position: {e}")
            raise CommandError(f"Failed to set home position: {e}", command="set_home", reason=str(e))

    # Parameter interface
    async def get_param(self, name: str) -> float:
        """
        Get a parameter value from the vehicle.

        Args:
            name: Parameter name (e.g., "MPC_XY_CRUISE" for max horizontal speed)

        Returns:
            Parameter value as float

        Raises:
            CommandError: If parameter cannot be read

        Example:
            speed = await drone.get_param("MPC_XY_CRUISE")
            print(f"Max cruise speed: {speed} m/s")
        """
        try:
            param = await self._system.param.get_param_float(name)
            return param.value
        except Exception as e:
            raise CommandError(f"Failed to get parameter '{name}': {e}", command="get_param", reason=str(e))

    async def set_param(self, name: str, value: float) -> bool:
        """
        Set a parameter value on the vehicle.

        Args:
            name: Parameter name
            value: New parameter value

        Returns:
            True if parameter was set successfully

        Raises:
            CommandError: If parameter cannot be set

        Example:
            await drone.set_param("MPC_XY_CRUISE", 10.0)  # Set max speed to 10 m/s
        """
        try:
            await self._system.param.set_param_float(name, value)
            logger.info(f"Set parameter {name} = {value}")
            return True
        except Exception as e:
            raise CommandError(f"Failed to set parameter '{name}': {e}", command="set_param", reason=str(e))

    async def get_param_int(self, name: str) -> int:
        """
        Get an integer parameter value from the vehicle.

        Args:
            name: Parameter name

        Returns:
            Parameter value as integer

        Raises:
            CommandError: If parameter cannot be read
        """
        try:
            param = await self._system.param.get_param_int(name)
            return param.value
        except Exception as e:
            raise CommandError(f"Failed to get parameter '{name}': {e}", command="get_param", reason=str(e))

    async def set_param_int(self, name: str, value: int) -> bool:
        """
        Set an integer parameter value on the vehicle.

        Args:
            name: Parameter name
            value: New parameter value

        Returns:
            True if parameter was set successfully

        Raises:
            CommandError: If parameter cannot be set
        """
        try:
            await self._system.param.set_param_int(name, value)
            logger.info(f"Set parameter {name} = {value}")
            return True
        except Exception as e:
            raise CommandError(f"Failed to set parameter '{name}': {e}", command="set_param", reason=str(e))

    async def get_all_params(self) -> Dict[str, Any]:
        """
        Get all parameters from the vehicle.

        Returns:
            Dictionary of parameter names to values

        Raises:
            CommandError: If parameters cannot be read

        Note:
            This may take several seconds on vehicles with many parameters.
        """
        try:
            params = {}
            async for param in self._system.param.get_all_params():
                if hasattr(param, 'float_param'):
                    params[param.float_param.name] = param.float_param.value
                elif hasattr(param, 'int_param'):
                    params[param.int_param.name] = param.int_param.value
            logger.info(f"Retrieved {len(params)} parameters")
            return params
        except Exception as e:
            raise CommandError(f"Failed to get all parameters: {e}", command="get_all_params", reason=str(e))

    # Properties
    @property
    def connected(self) -> bool:
        """True if connected and receiving heartbeats."""
        return self._connected

    @property
    def connection_healthy(self) -> bool:
        """True if connection is active and heartbeats are being received."""
        if not self._connected:
            return False
        return (time.time() - self._last_heartbeat) < _HEARTBEAT_TIMEOUT

    @property
    def seconds_since_heartbeat(self) -> float:
        """Seconds since last telemetry update was received."""
        if self._last_heartbeat == 0:
            return float('inf')
        return time.time() - self._last_heartbeat

    @property
    def armed(self) -> bool:
        """True if the vehicle is armed."""
        return self._armed

    @property
    def is_armable(self) -> bool:
        """True if the vehicle can be armed."""
        return self._is_armable

    @property
    def home(self) -> Optional[Coordinate]:
        """Home/launch position."""
        return self._home

    @property
    def throttle(self) -> float:
        """Current throttle setting (0-1)."""
        return self._throttle

    @property
    def position(self) -> Coordinate:
        """Current position as a Coordinate."""
        return self.state.position

    @property
    def heading(self) -> float:
        """Current heading in degrees (0-360)."""
        return self.state.heading

    @property
    def velocity(self) -> VectorNED:
        """Current velocity in NED frame."""
        return self.state.velocity

    @property
    def altitude(self) -> float:
        """Current altitude above home in meters."""
        return self.state.relative_altitude

    @property
    def is_in_air(self) -> bool:
        """True if the vehicle is currently in the air."""
        return self.state.is_in_air

    # Basic operations
    async def arm(self, force: bool = False) -> bool:
        """
        Arm the vehicle.

        Args:
            force: If True, attempt to arm even if pre-arm checks fail

        Returns:
            True if armed successfully

        Raises:
            NotArmableError: If vehicle is not ready to arm and force=False
            ArmError: If arming command fails
        """
        logger.debug(f"arm(force={force}) called")
        if self._armed:
            logger.debug("Vehicle already armed")
            return True

        if not force and not self._is_armable:
            logger.error("Vehicle is not armable - check GPS and pre-flight conditions")
            raise NotArmableError(
                "Vehicle is not armable. Check GPS and other pre-flight conditions."
            )

        try:
            logger.debug("Sending arm command...")
            await self._system.action.arm()
            # Wait for arm confirmation
            start = time.time()
            while not self._armed and time.time() - start < 10:
                await asyncio.sleep(_POLLING_DELAY)
            logger.debug(f"Arm result: {'success' if self._armed else 'failed'}")
            return self._armed
        except ActionError as e:
            logger.error(f"Failed to arm: {e}")
            raise ArmError(f"Failed to arm: {e}", reason=str(e))

    async def disarm(self, force: bool = False) -> bool:
        """
        Disarm the vehicle.

        Args:
            force: If True, force disarm even in flight (DANGEROUS!)

        Returns:
            True if disarmed successfully

        Raises:
            DisarmError: If disarm command fails
        """
        logger.debug(f"disarm(force={force}) called")
        if not self._armed:
            logger.debug("Vehicle already disarmed")
            return True

        try:
            if force:
                logger.warning("Force disarm (kill) requested!")
                await self._system.action.kill()
            else:
                await self._system.action.disarm()

            start = time.time()
            while self._armed and time.time() - start < 10:
                await asyncio.sleep(_POLLING_DELAY)
            return not self._armed
        except ActionError as e:
            raise DisarmError(f"Failed to disarm: {e}", reason=str(e))

    async def wait(self, seconds: float):
        """
        Wait for a specified duration.

        Args:
            seconds: Time to wait in seconds
        """
        await asyncio.sleep(seconds)

    async def _await_condition(
        self,
        condition: Callable[[], bool],
        timeout: Optional[float] = None,
        message: str = "Condition not met"
    ):
        """Wait for a condition to be true."""
        start = time.time()
        while not condition():
            if timeout and time.time() - start > timeout:
                raise AerpawTimeoutError(message, timeout=timeout)
            if self._aborted:
                raise AbortError("Operation aborted")
            await asyncio.sleep(_POLLING_DELAY)


class Drone(Vehicle):
    """
    Drone (multicopter) vehicle class.

    Provides a clean, Pythonic API for drone control with intuitive methods
    for navigation, heading control, and mission management.

    Args:
        connection: MAVSDK connection string
        safety_limits: Optional safety limits configuration. If None, uses defaults.
        safety_checker: Optional SafetyCheckerClient for server-side geofence validation.

    Example:
        async def main():
            drone = Drone("udp://:14540")
            await drone.connect()

            await drone.arm()
            await drone.takeoff(altitude=5)

            await drone.goto(latitude=51.5, longitude=-0.1)
            await drone.heading(degrees=90)

            await drone.land()

        # With custom safety limits
        async def safe_main():
            drone = Drone("udp://:14540", safety_limits=SafetyLimits.restrictive())
            await drone.connect()

            # Run pre-flight checks
            result = await drone.preflight_check()
            if not result:
                print(f"Pre-flight failed: {result.failed_checks}")
                return

            await drone.arm()
            await drone.takeoff(altitude=5)
            await drone.land()

        # With external safety checker for geofence validation
        async def geofenced_main():
            async with SafetyCheckerClient("localhost", 14580) as checker:
                drone = Drone("udp://:14540", safety_checker=checker)
                await drone.connect()
                # Commands are now validated against geofences
                await drone.arm()
                await drone.takeoff(altitude=10)
                await drone.goto(latitude=35.7, longitude=-78.6)  # Validated by server
                await drone.land()
    """

    def __init__(
        self,
        connection: str = "udp://:14540",
        safety_limits: Optional[SafetyLimits] = None,
        safety_checker: Optional["SafetyCheckerClient"] = None
    ):
        super().__init__(connection)
        self._waypoints: List[Waypoint] = []

        # Safety configuration
        self.safety_limits = safety_limits or SafetyLimits()
        self._safety_monitor: Optional[SafetyMonitor] = None
        self._safety_checker = safety_checker

        # Validate safety limits
        errors = self.safety_limits.validate()
        if errors:
            raise ValueError(f"Invalid safety limits: {', '.join(errors)}")

    async def connect(
        self,
        timeout: float = 30.0,
        auto_reconnect: bool = False,
        retry_count: int = 3,
        retry_delay: float = 2.0
    ) -> bool:
        """
        Connect to the vehicle with automatic retry support.

        Also starts the safety monitor if safety limits are configured.
        """
        result = await super().connect(timeout, auto_reconnect, retry_count, retry_delay)

        if result and self.safety_limits:
            # Start safety monitor
            self._safety_monitor = SafetyMonitor(self, self.safety_limits)
            self._safety_monitor.start()
            logger.info("Safety monitor started with limits: "
                       f"max_speed={self.safety_limits.max_speed}m/s, "
                       f"battery_failsafe={'enabled' if self.safety_limits.enable_battery_failsafe else 'disabled'}")

        return result

    async def disconnect(self):
        """Disconnect from the vehicle and clean up resources."""
        # Stop safety monitor
        if self._safety_monitor:
            self._safety_monitor.stop()
            self._safety_monitor = None

        await super().disconnect()

    async def preflight_check(self) -> PreflightCheckResult:
        """
        Run pre-flight safety checks before arming.

        Checks GPS quality, battery level, and configuration validity.

        Returns:
            PreflightCheckResult with detailed check results

        Example:
            result = await drone.preflight_check()
            if not result:
                print(f"Failed checks: {result.failed_checks}")
                print(result.summary())
            else:
                await drone.arm()
        """
        return await run_preflight_checks(self, self.safety_limits)

    async def arm(self, force: bool = False, skip_preflight: bool = False) -> bool:
        """
        Arm the vehicle.

        Args:
            force: If True, attempt to arm even if pre-arm checks fail
            skip_preflight: If True, skip pre-flight safety checks

        Returns:
            True if armed successfully

        Raises:
            PreflightCheckError: If pre-flight checks fail and force=False
            NotArmableError: If vehicle is not ready to arm and force=False
            ArmError: If arming command fails
        """
        # Run pre-flight checks unless skipped
        if self.safety_limits.enable_preflight_checks and not skip_preflight:
            result = await self.preflight_check()
            if not result:
                if force:
                    logger.warning(f"Pre-flight checks failed but force=True: {result.failed_checks}")
                else:
                    raise PreflightCheckError(result)

        return await super().arm(force=force)

    async def takeoff(self, altitude: float = 5.0, wait: bool = True, timeout: float = 60.0) -> Optional[CommandHandle]:
        """
        Take off to the specified altitude.

        Args:
            altitude: Target altitude in meters above ground
            wait: If True, wait until altitude is reached. If False, return CommandHandle.
            timeout: Maximum time to reach altitude in seconds

        Returns:
            None if wait=True, CommandHandle if wait=False

        Raises:
            TakeoffError: If takeoff command fails
            TakeoffTimeoutError: If altitude is not reached within timeout (only if wait=True)

        Example:
            # Blocking (default)
            await drone.takeoff(altitude=10)

            # Non-blocking
            handle = await drone.takeoff(altitude=10, wait=False)
            while handle.is_running:
                print(f"Current altitude: {drone.altitude}m")
                await asyncio.sleep(0.5)
        """
        logger.debug(f"takeoff(altitude={altitude}, wait={wait}, timeout={timeout}) called")
        if not self._armed:
            logger.debug("Vehicle not armed, arming first...")
            await self.arm()

        self._abortable = True
        self.info._takeoff_time = time.time()

        # Validate takeoff against safety checker server
        if self._safety_checker is not None:
            logger.debug("Validating takeoff with safety checker...")
            await validate_takeoff_with_checker(
                self._safety_checker,
                altitude,
                self.state.latitude,
                self.state.longitude,
                raise_on_fail=True
            )

        try:
            logger.debug(f"Setting takeoff altitude to {altitude}m")
            await self._system.action.set_takeoff_altitude(altitude)
            logger.debug("Sending takeoff command...")
            await self._system.action.takeoff()
        except ActionError as e:
            logger.error(f"Takeoff failed: {e}")
            raise TakeoffError(
                f"Takeoff failed: {e}",
                target_altitude=altitude,
                reason=str(e)
            )

        # Create completion condition and progress getter
        def completion_condition() -> bool:
            return self.state.relative_altitude >= altitude * 0.95

        def progress_getter() -> Dict[str, Any]:
            return {
                "current_altitude": self.state.relative_altitude,
                "target_altitude": altitude,
                "altitude_remaining": max(0, altitude - self.state.relative_altitude),
            }

        # Create command handle
        handle = CommandHandle(
            command="takeoff",
            completion_condition=completion_condition,
            cancel_action=self.hold,
            timeout=timeout,
            abort_checker=lambda: self._aborted,
            progress_getter=progress_getter,
        )
        handle._start()

        if wait:
            try:
                await handle.wait()
                # Small delay to stabilize
                await asyncio.sleep(2)
            except AerpawTimeoutError:
                raise TakeoffTimeoutError(
                    f"Takeoff timed out waiting for {altitude}m",
                    target_altitude=altitude,
                    current_altitude=self.state.relative_altitude,
                    timeout=timeout
                )
            return None
        else:
            return handle

    async def land(self, wait: bool = True, timeout: float = 120.0) -> Optional[CommandHandle]:
        """
        Land the drone at its current position.

        Args:
            wait: If True, wait until landed and disarmed. If False, return CommandHandle.
            timeout: Maximum time to complete landing in seconds

        Returns:
            None if wait=True, CommandHandle if wait=False

        Raises:
            LandingError: If landing command fails
            LandingTimeoutError: If landing does not complete within timeout (only if wait=True)

        Example:
            # Blocking (default)
            await drone.land()

            # Non-blocking
            handle = await drone.land(wait=False)
            while handle.is_running:
                print(f"Altitude: {drone.altitude}m")
                await asyncio.sleep(0.5)
        """
        logger.debug(f"land(wait={wait}, timeout={timeout}) called")
        self._abortable = False
        self._velocity_loop_active = False

        try:
            logger.debug("Sending land command...")
            await self._system.action.land()
        except ActionError as e:
            logger.error(f"Landing failed: {e}")
            raise LandingError(
                f"Landing failed: {e}",
                current_altitude=self.state.relative_altitude,
                reason=str(e)
            )

        # Create completion condition and progress getter
        def completion_condition() -> bool:
            return not self._armed

        def progress_getter() -> Dict[str, Any]:
            return {
                "current_altitude": self.state.relative_altitude,
                "landed_state": self.state.landed_state.name,
                "armed": self._armed,
            }

        # Create command handle
        handle = CommandHandle(
            command="land",
            completion_condition=completion_condition,
            cancel_action=None,  # Can't cancel landing
            timeout=timeout,
            abort_checker=None,  # Don't check abort during landing
            progress_getter=progress_getter,
        )
        handle._start()

        if wait:
            try:
                await handle.wait()
            except AerpawTimeoutError:
                raise LandingTimeoutError(
                    "Landing timed out",
                    current_altitude=self.state.relative_altitude,
                    timeout=timeout
                )
            return None
        else:
            return handle

    async def rtl(self, wait: bool = True, timeout: float = 300.0) -> Optional[CommandHandle]:
        """
        Return to launch (home) position.

        Args:
            wait: If True, wait until returned and landed. If False, return CommandHandle.
            timeout: Maximum time to complete RTL in seconds

        Returns:
            None if wait=True, CommandHandle if wait=False

        Raises:
            NavigationError: If RTL command fails

        Example:
            # Blocking (default)
            await drone.rtl()

            # Non-blocking
            handle = await drone.rtl(wait=False)
            while handle.is_running:
                print(f"Distance to home: {drone.position.distance_to(drone._home)}m")
                await asyncio.sleep(1)
        """
        self._abortable = False
        self._velocity_loop_active = False

        try:
            await self._system.action.return_to_launch()
        except ActionError as e:
            raise NavigationError(f"RTL failed: {e}", reason=str(e))

        # Create completion condition and progress getter
        def completion_condition() -> bool:
            return not self._armed

        def progress_getter() -> Dict[str, Any]:
            result = {
                "current_altitude": self.state.relative_altitude,
                "landed_state": self.state.landed_state.name,
                "armed": self._armed,
            }
            if self._home:
                result["distance_to_home"] = self.position.distance_to(self._home)
            return result

        # Create command handle
        handle = CommandHandle(
            command="rtl",
            completion_condition=completion_condition,
            cancel_action=None,  # Can't cancel RTL easily
            timeout=timeout,
            abort_checker=None,  # Don't check abort during RTL
            progress_getter=progress_getter,
        )
        handle._start()

        if wait:
            await handle.wait()
            return None
        else:
            return handle

    async def abort(self, rtl: bool = True):
        """
        Abort current operation.

        Args:
            rtl: If True, return to launch. If False, hold position.
        """
        self._aborted = True
        self._velocity_loop_active = False

        if rtl:
            await self.rtl(wait=False)
        else:
            await self.hold()

    async def hold(self):
        """Hold current position."""
        try:
            await self._system.action.hold()
        except ActionError:
            # Try setting velocity to zero as fallback
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
        wait: bool = True
    ) -> Optional[CommandHandle]:
        """
        Go to a specified location.

        Can be called with individual lat/lon/alt parameters or a Coordinate object.

        Args:
            latitude: Target latitude (or use coordinates parameter)
            longitude: Target longitude (or use coordinates parameter)
            altitude: Target altitude (uses current if not specified)
            coordinates: Coordinate object with target location
            tolerance: Acceptance radius in meters
            speed: Ground speed in m/s (uses default if not specified)
            heading: Target heading (uses bearing to target if not specified)
            timeout: Maximum time to reach target in seconds
            wait: If True, block until destination reached. If False, return CommandHandle.

        Returns:
            None if wait=True, CommandHandle if wait=False

        Raises:
            ValueError: If neither coordinates nor lat/lon provided
            ParameterValidationError: If parameters are invalid
            NavigationError: If goto command fails
            GotoTimeoutError: If target not reached within timeout (only if wait=True)

        Example:
            # Blocking (default)
            await drone.goto(latitude=51.5, longitude=-0.1)
            await drone.goto(coordinates=Coordinate(51.5, -0.1, 10, "Waypoint 1"))

            # Non-blocking with handle
            handle = await drone.goto(latitude=51.5, longitude=-0.1, wait=False)
            while handle.is_running:
                print(f"Distance: {handle.progress.get('distance', '?')}m")
                await asyncio.sleep(1)

            # Or await the handle later
            handle = await drone.goto(latitude=51.5, longitude=-0.1, wait=False)
            # ... do other things ...
            await handle  # Wait for completion
        """
        logger.debug(f"goto(lat={latitude}, lon={longitude}, alt={altitude}, coords={coordinates}, tolerance={tolerance}, speed={speed}, heading={heading}, wait={wait}) called")
        # Build target coordinate
        if coordinates is not None:
            target = coordinates
        elif latitude is not None and longitude is not None:
            target = Coordinate(
                latitude,
                longitude,
                altitude if altitude is not None else self.state.relative_altitude
            )
        else:
            raise ValueError("Must provide either coordinates or latitude/longitude")

        # Parameter validation
        if self.safety_limits.enable_parameter_validation:
            # Validate coordinate
            coord_check = validate_coordinate(target, "target")
            if not coord_check:
                raise ParameterValidationError(coord_check.message, "coordinates", target)

            # Validate altitude
            alt_check = validate_altitude(target.altitude, "altitude")
            if not alt_check:
                raise ParameterValidationError(alt_check.message, "altitude", target.altitude)

            # Validate tolerance
            tol_check = validate_tolerance(tolerance, "tolerance")
            if not tol_check:
                raise ParameterValidationError(tol_check.message, "tolerance", tolerance)

            # Validate timeout
            timeout_check = validate_timeout(timeout, "timeout")
            if not timeout_check:
                raise ParameterValidationError(timeout_check.message, "timeout", timeout)

            # Validate and clamp speed
            if speed is not None:
                speed_check = validate_speed(speed, self.safety_limits, "speed")
                if not speed_check:
                    if self.safety_limits.auto_clamp_values:
                        speed = clamp_speed(speed, self.safety_limits)
                    else:
                        raise ParameterValidationError(speed_check.message, "speed", speed)

        # Validate waypoint against safety checker server (geofence)
        if self._safety_checker is not None:
            await validate_waypoint_with_checker(
                self._safety_checker,
                self.position,
                target,
                raise_on_fail=True
            )
            # Also validate speed if provided
            if speed is not None:
                await validate_speed_with_checker(
                    self._safety_checker,
                    speed,
                    raise_on_fail=True
                )

        if speed is not None:
            await self._system.action.set_maximum_speed(speed)

        # Set heading
        if heading is not None:
            self._current_heading = heading
        elif self._current_heading is None:
            # Face direction of travel
            self._current_heading = self.position.bearing_to(target)

        try:
            target_alt = target.altitude + (self._home.altitude if self._home else 0)
            logger.debug(f"Navigating to: lat={target.latitude}, lon={target.longitude}, alt={target_alt}, heading={self._current_heading or 0}")
            await self._system.action.goto_location(
                target.latitude,
                target.longitude,
                target_alt,
                self._current_heading or 0
            )
        except ActionError as e:
            logger.error(f"Goto failed: {e}")
            raise NavigationError(
                f"Goto failed: {e}",
                target=target,
                current_position=self.position,
                reason=str(e)
            )

        # Create completion condition and progress getter
        def completion_condition() -> bool:
            return self.position.distance_to(target) <= tolerance

        def progress_getter() -> Dict[str, Any]:
            return {
                "distance": self.position.distance_to(target),
                "target": target,
                "tolerance": tolerance,
            }

        # Create command handle
        handle = CommandHandle(
            command="goto",
            completion_condition=completion_condition,
            cancel_action=self.hold,
            timeout=timeout,
            abort_checker=lambda: self._aborted,
            progress_getter=progress_getter,
        )
        handle._start()

        if wait:
            try:
                await handle.wait()
            except AerpawTimeoutError:
                raise GotoTimeoutError(
                    f"Goto timed out",
                    target=target,
                    distance_remaining=self.position.distance_to(target),
                    timeout=timeout
                )
            return None
        else:
            return handle

    async def set_heading(self, degrees: float, blocking: bool = True, timeout: float = 30.0) -> Optional[CommandHandle]:
        """
        Set the drone's heading (yaw).

        Args:
            degrees: Target heading in degrees (0 = North, 90 = East)
            blocking: If True, wait until heading is achieved. If False, return CommandHandle.
            timeout: Maximum time to achieve heading in seconds

        Returns:
            None if blocking=True, CommandHandle if blocking=False

        Raises:
            ModeChangeError: If offboard mode fails

        Example:
            # Blocking (default)
            await drone.set_heading(90)  # Face east

            # Non-blocking
            handle = await drone.set_heading(180, blocking=False)
            await handle  # Wait for completion
        """
        target_heading = degrees % 360
        self._current_heading = target_heading

        # Use offboard mode to control yaw
        try:
            await self._system.offboard.set_position_ned(
                PositionNedYaw(0, 0, -self.state.relative_altitude, self._current_heading)
            )
            await self._system.offboard.start()
        except (OffboardError, ActionError) as e:
            raise ModeChangeError(
                f"Heading change failed: {e}",
                target_mode="offboard",
                reason=str(e)
            )

        # Create completion condition and progress getter
        def heading_diff() -> float:
            return abs((self.state.heading - target_heading + 180) % 360 - 180)

        def completion_condition() -> bool:
            return heading_diff() < 5

        async def stop_offboard():
            try:
                await self._system.offboard.stop()
            except Exception:
                pass

        def progress_getter() -> Dict[str, Any]:
            return {
                "current_heading": self.state.heading,
                "target_heading": target_heading,
                "heading_diff": heading_diff(),
            }

        # Create command handle
        handle = CommandHandle(
            command="set_heading",
            completion_condition=completion_condition,
            cancel_action=stop_offboard,
            timeout=timeout,
            abort_checker=lambda: self._aborted,
            progress_getter=progress_getter,
        )
        handle._start()

        if blocking:
            try:
                await handle.wait()
            finally:
                await stop_offboard()
            return None
        else:
            return handle

    async def point_at(self, target: Optional[Coordinate] = None):
        """
        Point the drone at a target coordinate.

        Args:
            target: Coordinate to point at. If None, faces direction of travel.
        """
        if target is None:
            # Face direction of travel
            if self.state.velocity.magnitude(ignore_vertical=True) > 0.5:
                self._current_heading = self.state.velocity.heading()
                await self.set_heading(self._current_heading)
        else:
            bearing = self.position.bearing_to(target)
            await self.set_heading(bearing)

    async def move_in_current_direction(self, distance: float, speed: float = 5.0):
        """
        Move in the current heading direction.

        Args:
            distance: Distance to travel in meters
            speed: Speed in m/s
        """
        current_heading = self._current_heading or self.state.heading
        await self.move_in_direction(distance, current_heading, speed)

    async def move_in_direction(self, distance: float, degrees: float, speed: float = 5.0):
        """
        Move in a specified direction.

        Args:
            distance: Distance to travel in meters
            degrees: Direction in degrees (0 = North, 90 = East)
            speed: Speed in m/s
        """
        # Calculate target position
        rad = math.radians(degrees)
        north = distance * math.cos(rad)
        east = distance * math.sin(rad)

        target = self.position + VectorNED(north, east, 0)
        await self.goto(coordinates=target, speed=speed)

    async def move_towards(self, target: Coordinate, distance: float, speed: float = 5.0):
        """
        Move towards a target by a specified distance.

        Args:
            target: Target coordinate to move towards
            distance: Distance to travel in meters
            speed: Speed in m/s
        """
        bearing = self.position.bearing_to(target)
        await self.move_in_direction(distance, bearing, speed)

    async def set_velocity(
        self,
        velocity: VectorNED,
        heading: Optional[float] = None,
        duration: Optional[float] = None,
        wait: bool = True
    ) -> Optional[CommandHandle]:
        """
        Set the drone's velocity in NED frame.

        Args:
            velocity: Velocity vector in NED frame (m/s)
            heading: Target heading (uses current if not specified)
            duration: How long to maintain velocity (infinite if not specified)
            wait: If True and duration is set, wait until duration completes.
                  If False and duration is set, return CommandHandle.
                  Ignored if duration is None.

        Returns:
            None if wait=True or duration is None, CommandHandle if wait=False and duration is set

        Raises:
            OffboardError: If velocity command fails

        Example:
            # Continuous velocity (no duration)
            await drone.set_velocity(VectorNED(5, 0, 0))  # Move north at 5 m/s
            await asyncio.sleep(5)
            await drone.hold()

            # Timed velocity, blocking
            await drone.set_velocity(VectorNED(5, 0, 0), duration=10)  # Move north for 10s

            # Timed velocity, non-blocking
            handle = await drone.set_velocity(VectorNED(5, 0, 0), duration=10, wait=False)
            while handle.is_running:
                print(f"Time remaining: {handle.time_remaining}s")
                await asyncio.sleep(1)
        """
        logger.debug(f"set_velocity(N={velocity.north}, E={velocity.east}, D={velocity.down}, heading={heading}, duration={duration}, wait={wait}) called")
        if heading is not None:
            self._current_heading = heading

        target_yaw = self._current_heading or self.state.heading

        try:
            logger.debug(f"Setting velocity NED: N={velocity.north}, E={velocity.east}, D={velocity.down}, yaw={target_yaw}")
            await self._system.offboard.set_velocity_ned(
                VelocityNedYaw(
                    velocity.north,
                    velocity.east,
                    velocity.down,
                    target_yaw
                )
            )

            try:
                await self._system.offboard.start()
            except OffboardError:
                pass  # Already in offboard mode

            self._velocity_loop_active = True

        except (OffboardError, ActionError) as e:
            raise AerpawOffboardError(f"Set velocity failed: {e}", reason=str(e))

        # If no duration, this is a continuous velocity command
        if duration is None:
            return None

        # Create completion condition for timed velocity
        start_time = time.time()

        def completion_condition() -> bool:
            return time.time() - start_time >= duration

        async def stop_velocity():
            self._velocity_loop_active = False
            try:
                await self._system.offboard.stop()
            except Exception:
                pass

        def progress_getter() -> Dict[str, Any]:
            elapsed = time.time() - start_time
            return {
                "elapsed": elapsed,
                "duration": duration,
                "time_remaining": max(0, duration - elapsed),
                "velocity": velocity,
            }

        # Create command handle
        handle = CommandHandle(
            command="set_velocity",
            completion_condition=completion_condition,
            cancel_action=stop_velocity,
            timeout=duration + 5,  # Small buffer
            abort_checker=lambda: self._aborted,
            progress_getter=progress_getter,
        )
        handle._start()

        if wait:
            try:
                await handle.wait()
            finally:
                await stop_velocity()
            return None
        else:
            return handle

    async def set_groundspeed(self, speed: float):
        """
        Set the maximum ground speed for goto operations.

        Args:
            speed: Maximum ground speed in m/s
        """
        await self._system.action.set_maximum_speed(speed)

    async def set_altitude(self, altitude: float, tolerance: float = 0.5):
        """
        Change altitude while maintaining current position.

        Args:
            altitude: Target altitude in meters above home
            tolerance: Acceptance tolerance in meters
        """
        target = Coordinate(
            self.state.latitude,
            self.state.longitude,
            altitude
        )
        await self.goto(coordinates=target, tolerance=tolerance)

    async def orbit(
        self,
        center: Coordinate,
        radius: float,
        speed: float = 5.0,
        clockwise: bool = True,
        revolutions: float = 1.0,
        wait: bool = True
    ) -> Optional[CommandHandle]:
        """
        Orbit around a center point.

        Args:
            center: Center point of the orbit
            radius: Radius of the orbit in meters
            speed: Speed in m/s
            clockwise: If True, orbit clockwise (when viewed from above)
            revolutions: Number of revolutions (can be fractional)
            wait: If True, wait until orbit is complete. If False, return CommandHandle.

        Returns:
            None if wait=True, CommandHandle if wait=False

        Example:
            # Blocking orbit
            await drone.orbit(center=target, radius=50, revolutions=2)

            # Non-blocking orbit
            handle = await drone.orbit(center=target, radius=50, revolutions=2, wait=False)
            while handle.is_running:
                print(f"Orbit progress: {handle.progress.get('angle_completed', 0):.1f}°")
                await asyncio.sleep(1)
            await handle.cancel()  # Stop early if needed
        """
        # Calculate orbit parameters
        circumference = 2 * math.pi * radius
        total_distance = circumference * revolutions
        orbit_time = total_distance / speed
        angular_speed = (360 * revolutions) / orbit_time  # degrees per second

        if not clockwise:
            angular_speed = -angular_speed

        # Calculate starting angle from center
        start_vector = center.vector_to(self.position)
        start_angle = math.degrees(math.atan2(start_vector.east, start_vector.north))
        start_time = time.time()

        # Internal state for orbit execution
        orbit_active = True

        async def _stop_orbit():
            nonlocal orbit_active
            orbit_active = False
            self._velocity_loop_active = False
            try:
                await self._system.offboard.stop()
            except (OffboardError, ActionError):
                pass

        async def _execute_orbit():
            nonlocal orbit_active
            while orbit_active and time.time() - start_time < orbit_time:
                if self._aborted:
                    break

                elapsed = time.time() - start_time
                target_angle = start_angle + (angular_speed * elapsed)
                target_angle_rad = math.radians(target_angle)

                # Calculate target position on orbit
                north_offset = radius * math.cos(target_angle_rad)
                east_offset = radius * math.sin(target_angle_rad)
                target_pos = center + VectorNED(north_offset, east_offset, 0)
                Coordinate(target_pos.latitude, target_pos.longitude, self.altitude)

                # Calculate velocity to maintain orbit
                tangent_angle = target_angle + (90 if clockwise else -90)
                tangent_rad = math.radians(tangent_angle)
                vel_north = speed * math.cos(tangent_rad)
                vel_east = speed * math.sin(tangent_rad)

                # Face center or direction of travel
                heading = self.position.bearing_to(center)

                await self.set_velocity(
                    VectorNED(vel_north, vel_east, 0),
                    heading=heading
                )
                await asyncio.sleep(_POLLING_DELAY * 10)

            # Stop velocity control
            await _stop_orbit()

        def completion_condition() -> bool:
            return time.time() - start_time >= orbit_time or not orbit_active

        def progress_getter() -> Dict[str, Any]:
            elapsed = time.time() - start_time
            angle_completed = abs(angular_speed * elapsed)
            total_angle = 360 * revolutions
            return {
                "elapsed": elapsed,
                "orbit_time": orbit_time,
                "time_remaining": max(0, orbit_time - elapsed),
                "angle_completed": angle_completed,
                "total_angle": total_angle,
                "revolutions_completed": angle_completed / 360,
                "target_revolutions": revolutions,
                "progress_percent": min(100, (angle_completed / total_angle) * 100),
            }

        # Start orbit execution in background
        orbit_task = asyncio.create_task(_execute_orbit())

        # Create command handle
        handle = CommandHandle(
            command="orbit",
            completion_condition=completion_condition,
            cancel_action=_stop_orbit,
            timeout=orbit_time + 10,  # Small buffer
            abort_checker=lambda: self._aborted,
            progress_getter=progress_getter,
        )
        handle._start()

        if wait:
            try:
                await handle.wait()
                await orbit_task
            except Exception:
                await _stop_orbit()
                raise
            return None
        else:
            return handle

    # Waypoint management
    def add_waypoint(self, waypoint: Coordinate | Waypoint):
        """
        Add a waypoint to the mission queue.

        Args:
            waypoint: Coordinate or Waypoint to add
        """
        if isinstance(waypoint, Coordinate):
            self._waypoints.append(Waypoint(coordinate=waypoint))
        else:
            self._waypoints.append(waypoint)

    def add_waypoints(self, waypoints: List[Coordinate | Waypoint]):
        """
        Add multiple waypoints to the mission queue.

        Args:
            waypoints: List of Coordinates or Waypoints to add
        """
        for wp in waypoints:
            self.add_waypoint(wp)

    def clear_waypoints(self):
        """Clear all waypoints from the mission queue."""
        self._waypoints.clear()

    async def execute_waypoints(self, tolerance: float = 2.0):
        """
        Execute all queued waypoints in sequence.

        Args:
            tolerance: Acceptance radius in meters
        """
        for waypoint in self._waypoints:
            await self.goto(
                coordinates=waypoint.coordinate,
                tolerance=waypoint.acceptance_radius or tolerance,
                speed=waypoint.speed
            )
            if waypoint.hold_time > 0:
                await asyncio.sleep(waypoint.hold_time)

        self._waypoints.clear()


class Rover(Vehicle):
    """
    Ground rover vehicle class.

    Provides a clean, Pythonic API for rover control with intuitive methods
    for navigation.
    """

    def __init__(self, connection: str = "udp://:14540"):
        super().__init__(connection)
        self._waypoints: List[Waypoint] = []

    async def goto(
        self,
        latitude: Optional[float] = None,
        longitude: Optional[float] = None,
        coordinates: Optional[Coordinate] = None,
        tolerance: float = 2.0,
        speed: Optional[float] = None
    ):
        """
        Go to a specified location.

        Args:
            latitude: Target latitude (or use coordinates parameter)
            longitude: Target longitude (or use coordinates parameter)
            coordinates: Coordinate object with target location
            tolerance: Acceptance radius in meters
            speed: Ground speed in m/s (uses default if not specified)
        """
        if coordinates is not None:
            target = Coordinate(coordinates.latitude, coordinates.longitude, 0)
        elif latitude is not None and longitude is not None:
            target = Coordinate(latitude, longitude, 0)
        else:
            raise ValueError("Must provide either coordinates or latitude/longitude")

        if speed is not None:
            await self._system.action.set_maximum_speed(speed)

        try:
            await self._system.action.goto_location(
                target.latitude,
                target.longitude,
                0,
                0
            )

            await self._await_condition(
                lambda: self.position.ground_distance_to(target) <= tolerance,
                timeout=300,
                message="Goto timeout"
            )
        except ActionError as e:
            raise RuntimeError(f"Goto failed: {e}")

    async def stop(self):
        """Stop the rover."""
        try:
            await self._system.action.hold()
        except ActionError:
            pass

    async def move_in_direction(self, distance: float, degrees: float, speed: float = 2.0):
        """
        Move in a specified direction.

        Args:
            distance: Distance to travel in meters
            degrees: Direction in degrees (0 = North, 90 = East)
            speed: Speed in m/s
        """
        rad = math.radians(degrees)
        north = distance * math.cos(rad)
        east = distance * math.sin(rad)

        target = self.position + VectorNED(north, east, 0)
        await self.goto(coordinates=target, speed=speed)

    async def move_towards(self, target: Coordinate, distance: float, speed: float = 2.0):
        """
        Move towards a target by a specified distance.

        Args:
            target: Target coordinate to move towards
            distance: Distance to travel in meters
            speed: Speed in m/s
        """
        bearing = self.position.bearing_to(target)
        await self.move_in_direction(distance, bearing, speed)

    # Waypoint management
    def add_waypoint(self, waypoint: Coordinate | Waypoint):
        """
        Add a waypoint to the mission queue.

        Args:
            waypoint: Coordinate or Waypoint to add
        """
        if isinstance(waypoint, Coordinate):
            self._waypoints.append(Waypoint(coordinate=waypoint))
        else:
            self._waypoints.append(waypoint)

    def add_waypoints(self, waypoints: List[Coordinate | Waypoint]):
        """
        Add multiple waypoints to the mission queue.

        Args:
            waypoints: List of Coordinates or Waypoints to add
        """
        for wp in waypoints:
            self.add_waypoint(wp)

    def clear_waypoints(self):
        """Clear all waypoints from the mission queue."""
        self._waypoints.clear()

    async def execute_waypoints(self, tolerance: float = 2.0):
        """
        Execute all queued waypoints in sequence.

        Args:
            tolerance: Acceptance radius in meters
        """
        for waypoint in self._waypoints:
            await self.goto(
                coordinates=waypoint.coordinate,
                tolerance=waypoint.acceptance_radius or tolerance,
                speed=waypoint.speed
            )
            if waypoint.hold_time > 0:
                await asyncio.sleep(waypoint.hold_time)

        self._waypoints.clear()


# For backwards compatibility and convenience
__all__ = [
    "Vehicle",
    "Drone",
    "Rover",
]

