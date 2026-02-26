"""
Drone vehicle for aerpawlib v2.
"""

from __future__ import annotations

import asyncio
import math
import time
from typing import Optional

from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

from ..constants import (
    DEFAULT_GOTO_TIMEOUT_S,
    DEFAULT_POSITION_TOLERANCE_M,
    DEFAULT_TAKEOFF_ALTITUDE_TOLERANCE,
    HEADING_TOLERANCE_DEG,
    MIN_ARM_TO_TAKEOFF_DELAY_S,
    POST_TAKEOFF_STABILIZATION_S,
    VELOCITY_UPDATE_DELAY_S,
)
from ..exceptions import (
    LandingError,
    NavigationError,
    NotArmableError,
    RTLError,
    TakeoffError,
    VelocityError,
)
from ..log import LogComponent, get_logger
from ..types import Coordinate, VectorNED
from .base import Vehicle, VehicleTask, _validate_tolerance, _wait_for_condition

logger = get_logger(LogComponent.DRONE)


def _normalize_heading(h: float) -> float:
    return h % 360


def _heading_diff(a: float, b: float) -> float:
    d = abs((a % 360) - (b % 360))
    return min(d, 360 - d)


class Drone(Vehicle):
    """Drone implementation for multirotors."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._current_heading: Optional[float] = None
        self._velocity_loop_active = False

    async def _initialize_prearm(self, should_postarm_init: bool = True) -> None:
        """Wait for pre-arm conditions. Call before run."""
        self._should_postarm_init = should_postarm_init
        logger.info("Drone: _initialize_prearm started (waiting for armable)")
        from ..constants import ARMABLE_TIMEOUT_S, ARMABLE_STATUS_LOG_INTERVAL_S, POLLING_DELAY_S

        start = time.monotonic()
        last_log = 0.0
        while not self._state.armable:
            if time.monotonic() - start > ARMABLE_TIMEOUT_S:
                logger.warning(
                    f"Timeout waiting for armable ({ARMABLE_TIMEOUT_S}s). "
                    f"Status: {self._get_health_summary()}"
                )
                break
            if time.monotonic() - last_log > ARMABLE_STATUS_LOG_INTERVAL_S:
                logger.debug(f"Waiting for armable... {self._get_health_summary()}")
                last_log = time.monotonic()
            await asyncio.sleep(POLLING_DELAY_S)
        logger.info("Drone: _initialize_prearm done (armable or timeout)")

    async def _initialize_postarm(self) -> None:
        """Arm and prepare for mission (SITL/standalone: auto-arm)."""
        from ..constants import ARMING_SEQUENCE_DELAY_S, POSITION_READY_TIMEOUT_S

        if not self._should_postarm_init:
            logger.debug("Drone: _initialize_postarm skipped (_should_postarm_init=False)")
            return
        logger.info("Drone: _initialize_postarm (waiting for armable, GPS fix, arming)")
        await _wait_for_condition(
            lambda: self._state.armable,
            timeout=30.0,
            timeout_message=f"Vehicle not armable: {self._get_health_summary()}",
        )
        await _wait_for_condition(
            lambda: self.gps.fix_type >= 3,
            timeout=POSITION_READY_TIMEOUT_S,
            timeout_message="No GPS 3D fix",
        )
        await self.set_armed(True)
        await asyncio.sleep(ARMING_SEQUENCE_DELAY_S)
        await _wait_for_condition(
            lambda: self._state.home_coords is not None,
            timeout=5.0,
            timeout_message="Home position not available",
        )
        logger.info("Drone: _initialize_postarm done (armed, home position set)")

    async def set_heading(
        self,
        heading: Optional[float],
        blocking: bool = True,
        lock_in: bool = True,
    ) -> None:
        """Command heading. None clears locked heading."""
        if blocking:
            await self.await_ready_to_move()
        if heading is None:
            self._current_heading = None
            return
        heading = _normalize_heading(heading)
        if lock_in:
            self._current_heading = heading
        if not blocking:
            return
        home = self.home_coords
        if home:
            offset = self.position - home  # VectorNED
            north_m, east_m = offset.north, offset.east
        else:
            north_m, east_m = 0.0, 0.0
        try:
            await self._system.offboard.set_position_ned(
                PositionNedYaw(
                    north_m, east_m, -self.position.alt, heading
                )
            )
            try:
                await self._system.offboard.start()
            except OffboardError:
                pass
            self._ready_to_move = (
                lambda s: _heading_diff(heading, s.heading) <= HEADING_TOLERANCE_DEG
            )
            await _wait_for_condition(lambda: self.done_moving())
        except (OffboardError, ActionError) as e:
            logger.warning(f"set_heading error: {e}")
        finally:
            try:
                await self._system.offboard.stop()
            except (OffboardError, ActionError):
                pass

    async def takeoff(
        self,
        altitude: float,
        min_alt_tolerance: float = DEFAULT_TAKEOFF_ALTITUDE_TOLERANCE,
    ) -> None:
        """Takeoff to altitude (m)."""
        logger.info(f"Drone: takeoff to {altitude}m (min_alt_tolerance={min_alt_tolerance})")
        await self.await_ready_to_move()
        time_since_arm = time.time() - self._state.last_arm_time
        if time_since_arm < MIN_ARM_TO_TAKEOFF_DELAY_S:
            delay = MIN_ARM_TO_TAKEOFF_DELAY_S - time_since_arm
            await asyncio.sleep(delay)  # Justified: min arm-to-takeoff delay
        if self._mission_start_time is None:
            self._mission_start_time = time.time()
        try:
            await self._system.action.set_takeoff_altitude(altitude)
            await self._system.action.takeoff()
            self._ready_to_move = (
                lambda s: s.position.alt >= altitude * min_alt_tolerance
            )
            await _wait_for_condition(lambda: self.done_moving())
            await asyncio.sleep(POST_TAKEOFF_STABILIZATION_S)  # Justified: stabilization
            logger.info(f"Drone: takeoff complete (altitude {altitude}m)")
        except ActionError as e:
            logger.error(f"Drone: takeoff failed: {e}")
            raise TakeoffError(str(e), original_error=e)

    async def land(self) -> None:
        """Land and wait for disarm."""
        logger.info("Drone: land")
        await self.await_ready_to_move()
        try:
            await self._system.action.land()
            await _wait_for_condition(
                lambda: not self.armed,
                poll_interval=0.05,
            )
            logger.info("Drone: land complete (disarmed)")
        except ActionError as e:
            logger.error(f"Drone: land failed: {e}")
            raise LandingError(str(e), original_error=e)

    async def return_to_launch(self) -> None:
        """RTL and wait for disarm."""
        logger.info("Drone: return_to_launch (RTL)")
        await self.await_ready_to_move()
        try:
            await self._system.action.return_to_launch()
            await _wait_for_condition(
                lambda: not self.armed,
                poll_interval=0.05,
            )
            logger.info("Drone: return_to_launch complete (disarmed)")
        except ActionError as e:
            logger.error(f"Drone: return_to_launch failed: {e}")
            raise RTLError(str(e), original_error=e)

    async def goto_coordinates(
        self,
        coordinates: Coordinate,
        tolerance: float = DEFAULT_POSITION_TOLERANCE_M,
        target_heading: Optional[float] = None,
        timeout: float = DEFAULT_GOTO_TIMEOUT_S,
        blocking: bool = True,
    ) -> Optional[VehicleTask]:
        """Goto coordinates. If blocking=False, returns VehicleTask."""
        logger.info(
            f"Drone: goto_coordinates ({coordinates.lat:.6f}, {coordinates.lon:.6f}, "
            f"alt={coordinates.alt}m) tolerance={tolerance}m, blocking={blocking}"
        )
        _validate_tolerance(tolerance, "tolerance")
        if target_heading is not None:
            await self.set_heading(target_heading, blocking=False)
        await self.await_ready_to_move()
        await self._stop_offboard()
        heading = (
            self._current_heading
            if self._current_heading is not None
            else self.position.bearing(coordinates)
        )
        if math.isnan(heading):
            heading = 0.0
        target_alt = coordinates.alt + self.home_amsl
        try:
            await self._system.action.goto_location(
                coordinates.lat, coordinates.lon, target_alt, heading
            )
        except ActionError as e:
            logger.error(f"Drone: goto_location failed: {e}")
            raise NavigationError(str(e), original_error=e)
        self._ready_to_move = lambda s: coordinates.distance(s.position) <= tolerance
        self._current_heading = None

        if blocking:
            await _wait_for_condition(
                lambda: self.done_moving(),
                timeout=timeout,
                timeout_message=f"Goto timed out within {timeout}s",
            )
            logger.debug("Drone: goto_coordinates complete (blocking)")
            return None

        handle = VehicleTask()
        logger.debug("Drone: goto_coordinates returning non-blocking VehicleTask")

        async def _on_cancel() -> None:
            await self.return_to_launch()

        handle.set_on_cancel(_on_cancel)

        initial_dist = coordinates.distance(self.position)

        async def _wait_arrival() -> None:
            try:
                await _wait_for_condition(
                    lambda: self.done_moving() or handle.is_cancelled(),
                    timeout=timeout,
                )
                if handle.is_cancelled():
                    handle.set_complete()
                    return
                handle.set_progress(1.0)
                handle.set_complete()
            except TimeoutError as e:
                handle.set_error(NavigationError(str(e), original_error=e))
            except Exception as e:
                handle.set_error(e)

        async def _progress_updater() -> None:
            while not handle.is_done():
                if handle.is_cancelled():
                    return
                d = coordinates.distance(self.position)
                if initial_dist > 0:
                    p = 1.0 - (d / initial_dist)
                    handle.set_progress(max(0, min(1, p)))
                await asyncio.sleep(0.2)  # Justified: progress polling interval

        t1 = asyncio.create_task(_wait_arrival())
        t2 = asyncio.create_task(_progress_updater())
        self._command_tasks.extend([t1, t2])
        return handle

    async def set_velocity(
        self,
        velocity: VectorNED,
        global_relative: bool = True,
        duration: Optional[float] = None,
    ) -> None:
        """Set velocity in NED frame."""
        logger.info(
            f"Drone: set_velocity NED=({velocity.north:.2f}, {velocity.east:.2f}, "
            f"{velocity.down:.2f}) m/s, duration={duration}s"
        )
        await self.await_ready_to_move()
        self._velocity_loop_active = False
        await asyncio.sleep(VELOCITY_UPDATE_DELAY_S)  # Let previous loop exit
        if not global_relative:
            velocity = velocity.rotate_by_angle(-self.heading)
        yaw = self._current_heading if self._current_heading is not None else self.heading
        try:
            await self._system.offboard.set_velocity_ned(
                VelocityNedYaw(
                    velocity.north, velocity.east, velocity.down, yaw
                )
            )
            try:
                await self._system.offboard.start()
            except OffboardError:
                pass
            self._ready_to_move = lambda _: True
            target_end = time.monotonic() + duration if duration else None

            async def _velocity_loop() -> None:
                try:
                    while self._velocity_loop_active:
                        if target_end and time.monotonic() > target_end:
                            self._velocity_loop_active = False
                            await self._system.offboard.set_velocity_ned(
                                VelocityNedYaw(0, 0, 0, yaw)
                            )
                            await asyncio.sleep(0.05)
                            await self._system.offboard.stop()
                            return
                        await asyncio.sleep(VELOCITY_UPDATE_DELAY_S)
                except Exception as e:
                    logger.error(f"Velocity loop error: {e}")
                    self._velocity_loop_active = False
                    try:
                        await self._system.offboard.set_velocity_ned(
                            VelocityNedYaw(0, 0, 0, 0)
                        )
                        await self._system.offboard.stop()
                    except Exception:
                        pass

            self._velocity_loop_active = True
            vel_task = asyncio.create_task(_velocity_loop())
            self._command_tasks.append(vel_task)
        except (OffboardError, ActionError) as e:
            raise VelocityError(str(e), original_error=e)

    async def _stop_offboard(self) -> None:
        """Stop offboard mode."""
        self._velocity_loop_active = False
        try:
            await self._system.offboard.set_velocity_ned(
                VelocityNedYaw(0, 0, 0, self.heading)
            )
            await self._system.offboard.stop()
        except Exception:
            logger.debug("Stop offboard (may not be in offboard)")

    async def _stop(self) -> None:
        await super()._stop()
        await self._stop_offboard()
