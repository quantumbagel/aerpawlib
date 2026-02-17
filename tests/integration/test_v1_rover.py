"""Integration tests for aerpawlib v1 Rover. Requires ArduRover SITL (managed by pytest on port 14560)."""

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
