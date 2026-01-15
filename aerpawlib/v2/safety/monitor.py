"""
Safety monitor for continuous flight monitoring.

This module provides real-time safety monitoring during flight operations.
It works in conjunction with the MAVLink filter infrastructure in AERPAW:

- The SafetyMonitor runs locally in the experimenter's script
- The MAVLink filter runs separately and intercepts all MAVLink messages
- Both provide safety enforcement at different layers

The SafetyMonitor provides:
- Battery percentage and voltage monitoring with automatic RTL
- Speed limit monitoring
- Event callbacks for safety violations
"""

from __future__ import annotations

import asyncio
import logging
from typing import Callable, Dict, List, Optional, Set, TYPE_CHECKING

from .types import SafetyViolationType
from .limits import SafetyLimits
from ..types import FlightMode, LandedState

if TYPE_CHECKING:
    from ..vehicle import Vehicle, Drone
    from ..aerpaw import OEOClient

logger = logging.getLogger("aerpawlib.safety")


class SafetyMonitor:
    """
    Monitors vehicle state and enforces safety limits during flight.

    Runs as a background task and can:
    - Warn when battery is low (percentage or voltage)
    - Automatically trigger RTL on critical battery
    - Log safety events
    - Monitor speed limits
    - Notify OEO of safety events (if OEO client is provided)

    Battery Monitoring:
        The monitor tracks both battery percentage and voltage (if configured).
        Thresholds are defined in SafetyLimits:
        - min_battery_percent: Warning threshold
        - critical_battery_percent: RTL threshold (percentage-based)
        - critical_voltage: RTL threshold (voltage-based, if configured)

    Example:
        limits = SafetyLimits(
            min_battery_percent=20.0,
            critical_battery_percent=10.0,
            critical_voltage=10.5,  # 3S battery critical voltage
        )
        monitor = SafetyMonitor(drone, limits)
        monitor.on_violation(SafetyViolationType.BATTERY_LOW, my_handler)
        monitor.start()
    """

    def __init__(
        self,
        vehicle: Vehicle | Drone,
        limits: SafetyLimits,
        oeo_client: Optional[OEOClient] = None,
    ):
        self._vehicle = vehicle
        self._limits = limits
        self._oeo = oeo_client
        self._running = False
        self._task: Optional[asyncio.Task] = None
        self._is_ready = asyncio.Event()
        self._callbacks: Dict[SafetyViolationType, List[Callable]] = {}
        self._warnings_issued: Set[SafetyViolationType] = set()
        self._battery_rtl_triggered = False
        self._low_battery_warned = False
        self._voltage_rtl_triggered = False

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

    def on_violation(
        self, violation: SafetyViolationType, callback: Callable
    ) -> None:
        """Register a callback for a specific violation type."""
        if violation not in self._callbacks:
            self._callbacks[violation] = []
        self._callbacks[violation].append(callback)

    @property
    def ready(self) -> bool:
        """Whether the monitor has received initial telemetry and is active."""
        return self._is_ready.is_set()

    async def wait_until_ready(self) -> None:
        """Wait until the safety monitor is ready."""
        await self._is_ready.wait()

    async def _monitor_loop(self) -> None:
        """Main monitoring loop."""
        # Wait until initial telemetry data has been received for relevant streams
        try:
            await self._wait_for_streams()
        except asyncio.CancelledError:
            return
        except Exception as e:
            logger.error(f"Error while waiting for safety streams: {e}")

        while self._running:
            try:
                await self._check_all()
                await asyncio.sleep(0.5)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Safety monitor error: {e}")
                await asyncio.sleep(1)

    async def _wait_for_streams(self) -> None:
        """
        Wait until all relevant telemetry streams have received data.

        This prevents false alarms or monitoring errors that might occur
        if the monitor starts before initial telemetry is received.
        """
        logger.info("Safety monitor: waiting for initial telemetry...")

        while self._running:
            streams_ready = True

            # 1. Battery (if failsafe enabled)
            if self._limits.enable_battery_failsafe:
                # We expect non-zero battery data
                if (
                    self._vehicle.battery.voltage < 1.0
                    and self._vehicle.battery.charge == 0.0
                ):
                    streams_ready = False
                    logger.debug("Safety monitor: waiting for battery data")

            # 2. State (Landed state and Flight mode)
            if (
                self._vehicle.state.landed_state == LandedState.UNKNOWN
                or self._vehicle.state.flight_mode == FlightMode.UNKNOWN
            ):
                streams_ready = False
                logger.debug(
                    "Safety monitor: waiting for state data (landed_state/flight_mode)"
                )

            # 3. GPS (if required)
            if self._limits.require_gps_fix:
                if not self._vehicle.gps.has_fix:
                    streams_ready = False
                    logger.debug("Safety monitor: waiting for GPS fix")

            if streams_ready:
                logger.info(
                    "Safety monitor: all relevant telemetry streams active"
                )
                self._is_ready.set()
                break

            await asyncio.sleep(0.5)

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
                f"Speed {speed:.1f}m/s exceeds maximum {self._limits.max_speed}m/s",
            )

    async def _check_battery(self) -> None:
        """
        Check battery levels and trigger RTL if critical.

        Monitors both percentage and voltage (if voltage threshold is configured).
        In AERPAW, battery levels are also closely monitored by safety pilots via
        their RC controllers, providing an additional layer of safety.
        """
        battery_pct = self._vehicle.battery.percentage
        battery_voltage = self._vehicle.battery.voltage

        # Skip battery checks if battery data hasn't been received yet (0.0% or very low voltage)
        # SITL and real vehicles should report valid battery data once connected
        if battery_pct == 0.0 and battery_voltage < 1.0:
            return

        # Check critical voltage first (more accurate than percentage on real batteries)
        if self._limits.critical_voltage is not None:
            if (
                battery_voltage <= self._limits.critical_voltage
                and not self._voltage_rtl_triggered
            ):
                self._voltage_rtl_triggered = True
                await self._trigger_violation(
                    SafetyViolationType.BATTERY_CRITICAL,
                    f"CRITICAL BATTERY VOLTAGE: {battery_voltage:.2f}V - Triggering automatic RTL!",
                )
                await self._trigger_rtl(
                    f"BATTERY VOLTAGE CRITICAL ({battery_voltage:.2f}V <= {self._limits.critical_voltage:.2f}V)"
                )
                return

        # Check critical percentage
        if battery_pct <= self._limits.critical_battery_percent:
            if not self._battery_rtl_triggered:
                self._battery_rtl_triggered = True
                await self._trigger_violation(
                    SafetyViolationType.BATTERY_CRITICAL,
                    f"CRITICAL BATTERY: {battery_pct:.1f}% - Triggering automatic RTL!",
                )
                await self._trigger_rtl(
                    f"BATTERY CRITICAL ({battery_pct:.1f}% <= {self._limits.critical_battery_percent}%)"
                )

        elif battery_pct <= self._limits.min_battery_percent:
            if not self._low_battery_warned:
                self._low_battery_warned = True
                await self._trigger_violation(
                    SafetyViolationType.BATTERY_LOW,
                    f"Low battery warning: {battery_pct:.1f}% - Consider landing soon",
                )

                # Notify OEO of low battery warning
                if self._oeo:
                    await self._oeo.notify_battery_low(
                        battery_pct, battery_voltage
                    )

    async def _trigger_rtl(self, reason: str) -> None:
        """Trigger RTL and notify OEO."""
        try:
            logger.critical(reason + " - Automatic RTL triggered!")
            if self._vehicle.state.is_in_air:
                await self._vehicle.rtl(wait=False)

            # Notify OEO of critical battery and RTL
            if self._oeo:
                await self._oeo.notify_battery_critical(
                    self._vehicle.battery.percentage,
                    self._vehicle.battery.voltage,
                    rtl_triggered=True,
                )
        except Exception as e:
            logger.error(f"Failed to trigger RTL: {e}")

    async def _trigger_violation(
        self, violation: SafetyViolationType, message: str
    ) -> None:
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
        self._voltage_rtl_triggered = False


__all__ = ["SafetyMonitor"]
