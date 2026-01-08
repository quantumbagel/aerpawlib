"""
Safety monitor for continuous flight monitoring.
"""
from __future__ import annotations

import asyncio
import logging
from typing import Callable, Dict, List, Optional, Set, TYPE_CHECKING

from .types import SafetyViolationType
from .limits import SafetyLimits

if TYPE_CHECKING:
    from ..vehicle import Vehicle

logger = logging.getLogger("aerpawlib.safety")


class SafetyMonitor:
    """
    Monitors vehicle state and enforces safety limits during flight.

    Runs as a background task and can:
    - Warn when battery is low
    - Automatically trigger RTL on critical battery
    - Log safety events
    - Monitor speed limits
    """

    def __init__(self, vehicle: "Vehicle", limits: SafetyLimits):
        self._vehicle = vehicle
        self._limits = limits
        self._running = False
        self._task: Optional[asyncio.Task] = None
        self._callbacks: Dict[SafetyViolationType, List[Callable]] = {}
        self._warnings_issued: Set[SafetyViolationType] = set()
        self._battery_rtl_triggered = False
        self._low_battery_warned = False

    def start(self) -> None:
        """Start the safety monitor."""
        if self._running:
            return
        self._running = True
        self._task = asyncio.create_task(self._monitor_loop())
        logger.info("Safety monitor started")

    def stop(self) -> None:
        """Stop the safety monitor."""
        self._running = False
        if self._task:
            self._task.cancel()
        logger.info("Safety monitor stopped")

    def on_violation(self, violation: SafetyViolationType, callback: Callable) -> None:
        """Register a callback for a specific violation type."""
        if violation not in self._callbacks:
            self._callbacks[violation] = []
        self._callbacks[violation].append(callback)

    async def _monitor_loop(self) -> None:
        """Main monitoring loop."""
        while self._running:
            try:
                await self._check_all()
                await asyncio.sleep(0.5)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Safety monitor error: {e}")
                await asyncio.sleep(1)

    async def _check_all(self) -> None:
        """Run all safety checks."""
        if self._limits.enable_battery_failsafe:
            await self._check_battery()

        if self._vehicle.state.is_in_air and self._limits.enable_speed_limits:
            await self._check_speed()

    async def _check_speed(self) -> None:
        """Check speed limits."""
        speed = self._vehicle.state.groundspeed
        if speed > self._limits.max_speed * 1.1:  # 10% tolerance
            await self._trigger_violation(
                SafetyViolationType.SPEED_TOO_HIGH,
                f"Speed {speed:.1f}m/s exceeds maximum {self._limits.max_speed}m/s"
            )

    async def _check_battery(self) -> None:
        """Check battery levels and trigger RTL if critical."""
        battery_pct = self._vehicle.battery.percentage

        if battery_pct <= self._limits.critical_battery_percent:
            if not self._battery_rtl_triggered:
                self._battery_rtl_triggered = True
                await self._trigger_violation(
                    SafetyViolationType.BATTERY_CRITICAL,
                    f"CRITICAL BATTERY: {battery_pct:.1f}% - Triggering automatic RTL!"
                )
                try:
                    logger.critical(f"BATTERY CRITICAL ({battery_pct:.1f}%) - Automatic RTL triggered!")
                    if self._vehicle.state.is_in_air:
                        await self._vehicle.rtl(wait=False)
                except Exception as e:
                    logger.error(f"Failed to trigger RTL on critical battery: {e}")

        elif battery_pct <= self._limits.min_battery_percent:
            if not self._low_battery_warned:
                self._low_battery_warned = True
                await self._trigger_violation(
                    SafetyViolationType.BATTERY_LOW,
                    f"Low battery warning: {battery_pct:.1f}% - Consider landing soon"
                )

    async def _trigger_violation(self, violation: SafetyViolationType, message: str) -> None:
        """Trigger a safety violation."""
        logger.warning(f"SAFETY: {message}")

        for callback in self._callbacks.get(violation, []):
            try:
                result = callback(violation, message)
                if asyncio.iscoroutine(result):
                    await result
            except Exception as e:
                logger.error(f"Violation callback error: {e}")

    def reset(self) -> None:
        """Reset warning state (call on landing)."""
        self._warnings_issued.clear()
        self._battery_rtl_triggered = False
        self._low_battery_warned = False


__all__ = ["SafetyMonitor"]

