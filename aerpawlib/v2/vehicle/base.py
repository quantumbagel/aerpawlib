"""
Base vehicle for aerpawlib v2.

Single async loop, direct MAVSDK calls, native telemetry.
"""

from __future__ import annotations

import asyncio
import math
import time
from typing import Callable, List, Optional

from mavsdk import System
from mavsdk.action import ActionError

from ..constants import (
    CONNECTION_TIMEOUT_S,
    HEARTBEAT_START_DELAY_S,
    MIN_POSITION_TOLERANCE_M,
    MAX_POSITION_TOLERANCE_M,
)
from ..exceptions import (
    AerpawConnectionError,
    ConnectionTimeoutError,
    NotArmableError,
)
from ..log import LogComponent, get_logger
from ..types import Attitude, Battery, Coordinate, GPSInfo, VectorNED
from .state import VehicleState

logger = get_logger(LogComponent.VEHICLE)


def _validate_tolerance(tolerance: float, param_name: str = "tolerance") -> float:
    """Validate tolerance is within bounds."""
    if not (MIN_POSITION_TOLERANCE_M <= tolerance <= MAX_POSITION_TOLERANCE_M):
        raise ValueError(
            f"{param_name} must be between {MIN_POSITION_TOLERANCE_M} and "
            f"{MAX_POSITION_TOLERANCE_M}, got {tolerance}"
        )
    return tolerance


async def _wait_for_condition(
    condition: Callable[[], bool],
    timeout: Optional[float] = None,
    poll_interval: float = 0.05,
    timeout_message: str = "Operation timed out",
) -> bool:
    """Wait for condition. Uses minimal poll_interval to yield to event loop."""
    start = time.monotonic()
    while not condition():
        if timeout is not None and (time.monotonic() - start) > timeout:
            logger.warning(f"_wait_for_condition timeout after {timeout}s: {timeout_message}")
            raise TimeoutError(timeout_message)
        await asyncio.sleep(poll_interval)  # Justified: yield to loop, allow telemetry
    return True


class VehicleTask:
    """
    Handle for non-blocking commands with progress and cancellation.

    Use event-driven completion via position/landed_state subscriptions.
    """

    def __init__(self) -> None:
        self._done = asyncio.Event()
        self._cancelled = False
        self._progress: float = 0.0
        self._error: Optional[Exception] = None
        self._on_cancel: Optional[Callable[[], object]] = None

    @property
    def progress(self) -> float:
        """Progress 0.0 to 1.0."""
        return self._progress

    def is_done(self) -> bool:
        """True if the command has completed (success, error, or cancelled)."""
        return self._done.is_set()

    def set_progress(self, value: float) -> None:
        """Update progress (0.0-1.0). Internal use by command implementation."""
        self._progress = max(0.0, min(1.0, value))

    def set_complete(self) -> None:
        """Mark command as successfully complete. Internal use."""
        self._error = None
        self._done.set()

    def set_error(self, error: Exception) -> None:
        """Mark command as failed with error. Internal use."""
        self._error = error
        self._done.set()

    def set_on_cancel(self, callback: Callable[[], object]) -> None:
        """Set async callback to run when cancel() is called (e.g. RTL to stop goto)."""
        self._on_cancel = callback

    def cancel(self) -> None:
        """Request cancellation. Invokes on_cancel callback if set to stop the vehicle."""
        logger.debug("VehicleTask: cancel requested")
        self._cancelled = True
        if self._on_cancel:
            try:
                loop = asyncio.get_running_loop()
                result = self._on_cancel()
                if asyncio.iscoroutine(result):
                    loop.create_task(result)
            except RuntimeError:
                pass

    def is_cancelled(self) -> bool:
        return self._cancelled

    async def wait_done(self) -> None:
        """Wait until command completes or is cancelled."""
        await self._done.wait()
        if self._error is not None:
            raise self._error


class Vehicle:
    """
    Base vehicle with async connect, single-loop telemetry, no ThreadSafeValue.
    """

    def __init__(
        self,
        system: System,
        connection_string: str,
        mavsdk_server_port: int = 50051,
    ) -> None:
        self._system = system
        self._connection_string = connection_string
        self._mavsdk_server_port = mavsdk_server_port
        self._state = VehicleState()
        self._telemetry_tasks: List[asyncio.Task] = []
        self._command_tasks: List[asyncio.Task] = []
        self._running = True
        self._closed = False
        self._ready_to_move: Callable[["Vehicle"], bool] = lambda _: True
        self._heartbeat_tick_cb: Optional[Callable[[], None]] = None
        self._mission_start_time: Optional[float] = None
        self._should_postarm_init: bool = True

    def set_heartbeat_tick_callback(self, cb: Callable[[], None]) -> None:
        """Set callback invoked when heartbeat/telemetry received."""
        self._heartbeat_tick_cb = cb

    def _heartbeat_tick(self) -> None:
        """Called from telemetry when we receive data (heartbeat indicator)."""
        if self._heartbeat_tick_cb:
            self._heartbeat_tick_cb()

    def heartbeat_tick(self) -> None:
        """Protocol method for ConnectionHandler."""
        self._heartbeat_tick()

    @property
    def connected(self) -> bool:
        return self._running and not self._closed

    @property
    def closed(self) -> bool:
        """True if the vehicle connection has been closed."""
        return self._closed

    @property
    def position(self) -> Coordinate:
        return self._state.position

    @property
    def home_coords(self) -> Optional[Coordinate]:
        return self._state.home_coords

    @property
    def home_amsl(self) -> float:
        return self._state.home_amsl

    @property
    def battery(self) -> Battery:
        return self._state.battery

    @property
    def gps(self) -> GPSInfo:
        return self._state.gps

    @property
    def armed(self) -> bool:
        return self._state.armed

    @property
    def heading(self) -> float:
        return self._state.heading

    @property
    def velocity(self) -> VectorNED:
        return self._state.velocity

    @property
    def attitude(self) -> Attitude:
        return self._state.attitude

    @property
    def mode(self) -> str:
        return self._state.mode

    @classmethod
    async def connect(
        cls,
        connection_string: str,
        mavsdk_server_port: int = 50051,
        timeout: float = CONNECTION_TIMEOUT_S,
    ) -> "Vehicle":
        """
        Connect to vehicle. Returns initialized instance.
        """
        logger.info(
            f"Connecting to vehicle at {connection_string} "
            f"(port={mavsdk_server_port}, timeout={timeout}s)"
        )
        system = System(port=mavsdk_server_port)
        await asyncio.wait_for(
            system.connect(system_address=connection_string),
            timeout=timeout,
        )
        logger.debug("MAVSDK connect() returned, waiting for connection state")
        # Wait for connection state
        start = time.monotonic()
        async for state in system.core.connection_state():
            if state.is_connected:
                break
            if time.monotonic() - start > timeout:
                logger.error(
                    f"Connection timeout: no heartbeat within {timeout}s"
                )
                raise ConnectionTimeoutError(
                    timeout,
                    "Connection established but no heartbeat within timeout",
                )

        # Create instance and start telemetry
        self = cls(system, connection_string, mavsdk_server_port)
        await self._start_telemetry()
        logger.info("Vehicle connected and telemetry started")
        return self

    async def _start_telemetry(self) -> None:
        """Start telemetry subscriptions on same loop."""
        logger.debug("Starting telemetry subscriptions (position, attitude, velocity, gps, battery, mode, armed, health, home)")

        async def _position_update() -> None:
            async for position in self._system.telemetry.position():
                if not self._running:
                    return
                self._state.update_position(
                    position.latitude_deg,
                    position.longitude_deg,
                    position.relative_altitude_m,
                    position.absolute_altitude_m,
                )
                self._heartbeat_tick()

        async def _attitude_update() -> None:
            async for att in self._system.telemetry.attitude_euler():
                if not self._running:
                    return
                self._state.update_attitude(
                    math.radians(att.roll_deg),
                    math.radians(att.pitch_deg),
                    math.radians(att.yaw_deg),
                )

        async def _velocity_update() -> None:
            async for vel in self._system.telemetry.velocity_ned():
                if not self._running:
                    return
                self._state.update_velocity(
                    vel.north_m_s, vel.east_m_s, vel.down_m_s
                )

        async def _gps_update() -> None:
            async for gps in self._system.telemetry.gps_info():
                if not self._running:
                    return
                fix = gps.fix_type.value if hasattr(gps.fix_type, "value") else gps.fix_type
                self._state.update_gps(fix, gps.num_satellites)

        async def _battery_update() -> None:
            async for bat in self._system.telemetry.battery():
                if not self._running:
                    return
                current = getattr(bat, "current_battery_a", 0.0) or 0.0
                self._state.update_battery(
                    bat.voltage_v, current, int(bat.remaining_percent)
                )

        async def _flight_mode_update() -> None:
            async for mode in self._system.telemetry.flight_mode():
                if not self._running:
                    return
                self._state.update_mode(mode.name)

        async def _armed_update() -> None:
            async for armed in self._system.telemetry.armed():
                if not self._running:
                    return
                self._state.update_armed(armed)

        async def _health_update() -> None:
            async for health in self._system.telemetry.health():
                if not self._running:
                    return
                self._state.update_armable(
                    health.is_global_position_ok,
                    health.is_home_position_ok,
                    health.is_armable,
                )

        async def _home_update() -> None:
            async for home in self._system.telemetry.home():
                if not self._running:
                    return
                self._state.update_home(
                    home.latitude_deg,
                    home.longitude_deg,
                    home.relative_altitude_m,
                    home.absolute_altitude_m,
                )

        for coro in [
            _position_update,
            _attitude_update,
            _velocity_update,
            _gps_update,
            _battery_update,
            _flight_mode_update,
            _armed_update,
            _health_update,
            _home_update,
        ]:
            task = asyncio.create_task(coro())
            self._telemetry_tasks.append(task)
        logger.debug(f"Started {len(self._telemetry_tasks)} telemetry tasks")

    def done_moving(self) -> bool:
        """True if ready for next command."""
        return self._ready_to_move(self)

    async def _initialize_postarm(self) -> None:
        """Arm and prepare for mission. Override in Drone for full logic."""
        raise NotImplementedError("Override in subclass")

    async def await_ready_to_move(self) -> None:
        """Wait until vehicle is ready for next command."""
        if self._should_postarm_init and not self.armed:
            logger.debug("await_ready_to_move: vehicle not armed, running postarm init")
            await self._initialize_postarm()
        logger.debug("await_ready_to_move: waiting for done_moving")
        await _wait_for_condition(
            self.done_moving,
            timeout=300.0,
            poll_interval=0.05,
            timeout_message="Vehicle did not report ready within timeout",
        )
        logger.debug("await_ready_to_move: vehicle ready")

    async def set_armed(self, value: bool) -> None:
        """Arm or disarm."""
        from ..constants import CONNECTION_TIMEOUT_S

        logger.info(f"set_armed({value})")
        if value and not self._state.armable:
            logger.warning(f"Arm rejected: {self._get_health_summary()}")
            raise NotArmableError(
                f"Vehicle not armable: {self._get_health_summary()}"
            )
        try:
            if value:
                await self._system.action.arm()
            else:
                await self._system.action.disarm()
            await _wait_for_condition(
                lambda: self._state.armed == value,
                timeout=60.0,
                poll_interval=0.05,
                timeout_message="Arm/disarm did not complete within 60s",
            )
        except ActionError as e:
            from ..exceptions import ArmError, DisarmError

            if value:
                raise ArmError(str(e), original_error=e)
            raise DisarmError(str(e), original_error=e)

    def _get_health_summary(self) -> str:
        """Human-readable health status."""
        return (
            f"GPS fix: {self._state.gps.fix_type}, "
            f"sats: {self._state.gps.satellites_visible}, "
            f"armable: {self._state.armable}"
        )

    async def set_groundspeed(self, velocity: float) -> None:
        """Set cruise speed (m/s)."""
        try:
            await self._system.action.set_maximum_speed(velocity)
        except ActionError:
            logger.debug("set_maximum_speed not supported")

    async def _stop(self) -> None:
        """Stop background movement."""
        self._ready_to_move = lambda _: True

    def close(self) -> None:
        """Clean up. Cancels all telemetry and command tasks."""
        if self._closed:
            logger.debug("close() called but already closed")
            return
        logger.info("Closing vehicle connection")
        self._closed = True
        self._running = False
        for task in getattr(self, "_telemetry_tasks", []):
            task.cancel()
        for task in getattr(self, "_command_tasks", []):
            task.cancel()
        self._telemetry_tasks.clear()
        if hasattr(self, "_command_tasks"):
            self._command_tasks.clear()
        self._system = None
        logger.info("Vehicle connection closed")

    async def goto_coordinates(
        self,
        coordinates: Coordinate,
        tolerance: float = 2.0,
        target_heading: Optional[float] = None,
    ) -> None:
        """Override in subclass."""
        raise NotImplementedError("Generic Vehicle cannot navigate")


class DummyVehicle(Vehicle):
    """No-op vehicle for testing without hardware."""

    def __init__(self) -> None:
        # Skip parent init - no system
        self._state = VehicleState()
        self._state._position_lat = 35.727436
        self._state._position_lon = -78.696587
        self._state._position_alt = 0.0
        self._state._gps = GPSInfo(3, 10)
        self._state._battery = Battery(12.6, 0.0, 100)
        self._state._home = Coordinate(35.727436, -78.696587, 0)
        self._state._armable = True
        self._system = None
        self._connection_string = ""
        self._mavsdk_server_port = 50051
        self._telemetry_tasks = []
        self._command_tasks = []
        self._running = True
        self._closed = False
        self._ready_to_move = lambda _: True
        self._heartbeat_tick_cb = None
        self._mission_start_time = None

    @classmethod
    async def connect(
        cls,
        connection_string: str = "",
        mavsdk_server_port: int = 50051,
        timeout: float = CONNECTION_TIMEOUT_S,
    ) -> "DummyVehicle":
        return cls()

    async def goto_coordinates(
        self,
        coordinates: "Coordinate",
        tolerance: float = 2.0,
        target_heading: Optional[float] = None,
    ) -> None:
        """No-op for dry-run."""
        pass

    def close(self) -> None:
        self._closed = True
        self._running = False
