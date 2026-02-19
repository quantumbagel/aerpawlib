"""Integration tests for aerpawlib v1 Rover. Requires Rover SITL (managed by pytest on port 14560)."""

import pytest

pytestmark = [pytest.mark.integration]


class TestRoverConnection:
    """Rover connection and telemetry."""

    @pytest.mark.asyncio
    async def test_connects(self, connected_rover):
        assert connected_rover.connected is True

    @pytest.mark.asyncio
    async def test_gps_fix(self, connected_rover):
        assert connected_rover.gps.fix_type >= 3

    @pytest.mark.asyncio
    async def test_position_valid(self, connected_rover):
        pos = connected_rover.position
        assert -90 <= pos.lat <= 90 and -180 <= pos.lon <= 180


class TestRoverNavigation:
    """Rover navigation."""

    @pytest.mark.asyncio
    async def test_goto_coordinates(self, connected_rover):
        from aerpawlib.v1.util import VectorNED

        connected_rover._initialize_prearm(should_postarm_init=True)
        start = connected_rover.position
        target = start + VectorNED(20, 0)
        await connected_rover.goto_coordinates(target, tolerance=3)
        dist = connected_rover.position.ground_distance(target)
        assert dist < 5

class TestRoverTelemetryExtended:
    """Extended telemetry checks for rover."""

    @pytest.mark.asyncio
    async def test_battery_valid(self, connected_rover):
        b = connected_rover.battery
        assert b.voltage > 0
        assert 0 <= b.level <= 100

    @pytest.mark.asyncio
    async def test_heading_valid(self, connected_rover):
        h = connected_rover.heading
        assert 0 <= h < 360

    @pytest.mark.asyncio
    async def test_attitude_accessible(self, connected_rover):
        a = connected_rover.attitude
        assert a is not None
        assert isinstance(a.pitch, float)
        assert isinstance(a.yaw, float)
        assert isinstance(a.roll, float)

    @pytest.mark.asyncio
    async def test_gps_satellites_nonnegative(self, connected_rover):
        g = connected_rover.gps
        assert g.satellites_visible >= 0





class TestRoverMultiWaypoint:
    """Sequential waypoint navigation for rover."""

    @pytest.mark.asyncio
    async def test_two_sequential_waypoints(self, connected_rover):
        from aerpawlib.v1.util import VectorNED
        connected_rover._initialize_prearm(should_postarm_init=True)
        start = connected_rover.position
        wp1 = start + VectorNED(15, 0)
        wp2 = start + VectorNED(15, 15)
        await connected_rover.goto_coordinates(wp1, tolerance=3)
        assert connected_rover.position.ground_distance(wp1) < 5
        await connected_rover.goto_coordinates(wp2, tolerance=3)
        assert connected_rover.position.ground_distance(wp2) < 5

    @pytest.mark.asyncio
    async def test_goto_with_target_heading_does_not_error(self, connected_rover):
        """Rover ignores target_heading but should not raise an error."""
        from aerpawlib.v1.util import VectorNED
        connected_rover._initialize_prearm(should_postarm_init=True)
        start = connected_rover.position
        target = start + VectorNED(10, 0)
        # Should complete without error even though heading is ignored for rovers
        await connected_rover.goto_coordinates(target, tolerance=3, target_heading=90)
        dist = connected_rover.position.ground_distance(target)
        assert dist < 5




