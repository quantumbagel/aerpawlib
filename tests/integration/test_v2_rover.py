"""Integration tests for aerpawlib v2 Rover. Requires SITL."""

import pytest

pytestmark = [pytest.mark.integration]


class TestRoverConnection:
    """Rover connection and telemetry."""

    @pytest.mark.asyncio
    async def test_connects(self, connected_rover_v2):
        assert connected_rover_v2.connected is True

    @pytest.mark.asyncio
    async def test_gps_fix(self, connected_rover_v2):
        assert connected_rover_v2.gps.fix_type >= 3

    @pytest.mark.asyncio
    async def test_position_valid(self, connected_rover_v2):
        pos = connected_rover_v2.position
        assert -90 <= pos.lat <= 90 and -180 <= pos.lon <= 180

    @pytest.mark.asyncio
    async def test_heading_valid(self, connected_rover_v2):
        assert 0 <= connected_rover_v2.heading < 360

    @pytest.mark.asyncio
    async def test_battery_valid(self, connected_rover_v2):
        b = connected_rover_v2.battery
        assert b.voltage > 0 and 0 <= b.level <= 100

    @pytest.mark.asyncio
    async def test_home_coords_set_after_guided(self, connected_rover_v2):
        connected_rover_v2._initialize_prearm(should_postarm_init=True)
        home = connected_rover_v2.home_coords
        assert home is not None
        assert -90 <= home.lat <= 90
        assert -180 <= home.lon <= 180


class TestRoverNavigation:
    """Rover navigation."""

    @pytest.mark.asyncio
    async def test_goto_coordinates(self, connected_rover_v2):
        from aerpawlib.v2.types import VectorNED

        connected_rover_v2._initialize_prearm(should_postarm_init=True)
        start = connected_rover_v2.position
        target = start + VectorNED(15, 0, 0)
        await connected_rover_v2.goto_coordinates(target, tolerance=3)
        dist = connected_rover_v2.position.ground_distance(target)
        assert dist < 5
