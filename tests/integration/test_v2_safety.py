"""Integration tests for aerpawlib v2 safety: PreflightChecks, SafetyMonitor. Requires SITL."""

import asyncio

import pytest

pytestmark = [pytest.mark.integration]


class TestPreflightChecks:
    """PreflightChecks.run_all."""

    @pytest.mark.asyncio
    async def test_run_all_passes_with_gps(self, connected_drone_v2):
        from aerpawlib.v2.safety import PreflightChecks

        connected_drone_v2._initialize_prearm(should_postarm_init=True)
        ok = await PreflightChecks.run_all(connected_drone_v2)
        assert ok is True


class TestSafetyMonitor:
    """SafetyMonitor violation callbacks."""

    @pytest.mark.asyncio
    async def test_monitor_violation_callback(self, connected_drone_v2):
        from aerpawlib.v2.safety import SafetyLimits, SafetyMonitor

        connected_drone_v2._initialize_prearm(should_postarm_init=True)
        limits = SafetyLimits(max_altitude_m=3, min_battery_percent=20)
        monitor = SafetyMonitor(limits)
        violations = []

        async def on_violation(event_type: str, message: str):
            violations.append((event_type, message))

        monitor.on_violation(on_violation)
        await connected_drone_v2.takeoff(5)
        await monitor.check_altitude(connected_drone_v2.position)
        await connected_drone_v2.land()
        assert len(violations) >= 1
        assert violations[0][0] == "ALTITUDE_EXCEEDED"
