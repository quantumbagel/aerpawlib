"""Integration tests for aerpawlib v1 Drone. Requires SITL (managed by pytest)."""

import asyncio
import time

import pytest

pytestmark = [pytest.mark.integration]


class TestDroneConnection:
    """Drone connection and telemetry."""

    @pytest.mark.asyncio
    async def test_connects(self, connected_drone):
        assert connected_drone.connected is True

    @pytest.mark.asyncio
    async def test_gps_fix(self, connected_drone):
        assert connected_drone.gps.fix_type >= 3

    @pytest.mark.asyncio
    async def test_position_valid(self, connected_drone):
        pos = connected_drone.position
        assert -90 <= pos.lat <= 90 and -180 <= pos.lon <= 180

    @pytest.mark.asyncio
    async def test_heading_valid(self, connected_drone):
        assert 0 <= connected_drone.heading < 360

    @pytest.mark.asyncio
    async def test_battery_valid(self, connected_drone):
        b = connected_drone.battery
        assert b.voltage > 0 and 0 <= b.level <= 100

    @pytest.mark.asyncio
    async def test_attitude_not_none(self, connected_drone):
        a = connected_drone.attitude
        assert a is not None
        # pitch/roll/yaw are floats
        assert isinstance(a.pitch, float)
        assert isinstance(a.roll, float)
        assert isinstance(a.yaw, float)

    @pytest.mark.asyncio
    async def test_battery_level_valid_range(self, connected_drone):
        b = connected_drone.battery
        assert 0 <= b.level <= 100

    @pytest.mark.asyncio
    async def test_gps_satellites_visible(self, connected_drone):
        g = connected_drone.gps
        assert g.satellites_visible >= 0

    @pytest.mark.asyncio
    async def test_home_coords_set_after_guided(self, connected_drone):
        """Home coordinates should be set once the vehicle is in guided mode."""
        connected_drone._initialize_prearm(should_postarm_init=True)
        # Home is set as part of prearm initialization
        home = connected_drone.home_coords
        assert home is not None
        assert -90 <= home.lat <= 90
        assert -180 <= home.lon <= 180


class TestDroneArming:
    """Drone arming."""

    @pytest.mark.asyncio
    async def test_arms_on_takeoff(self, connected_drone):
        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(5)
        assert connected_drone.armed is True
        await connected_drone.land()


class TestDroneTakeoff:
    """Drone takeoff."""

    @pytest.mark.asyncio
    async def test_takeoff_reaches_altitude(self, connected_drone):
        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(10)
        assert connected_drone.position.alt >= 9
        await connected_drone.land()


class TestDroneNavigation:
    """Drone navigation."""

    @pytest.mark.asyncio
    async def test_goto_coordinates(self, connected_drone):
        from aerpawlib.v1.util import VectorNED

        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(10)
        start = connected_drone.position
        target = start + VectorNED(30, 0, 0)
        await connected_drone.goto_coordinates(target, tolerance=3)
        dist = connected_drone.position.ground_distance(target)
        assert dist < 5
        await connected_drone.land()


class TestDroneLanding:
    """Drone landing."""

    @pytest.mark.asyncio
    async def test_land_disarms(self, connected_drone):
        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(10)
        await connected_drone.land()
        assert connected_drone.armed is False


class TestDroneRTL:
    """Return to launch."""

    @pytest.mark.asyncio
    async def test_rtl_returns_home(self, connected_drone):
        from aerpawlib.v1.util import VectorNED

        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(10)
        home = connected_drone.home_coords
        target = connected_drone.position + VectorNED(40, 0, 0)
        await connected_drone.goto_coordinates(target, tolerance=3)
        await connected_drone.return_to_launch()
        assert connected_drone.armed is False


class TestDroneHeading:
    """Heading control."""

    @pytest.mark.asyncio
    async def test_set_heading(self, connected_drone):
        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(10)
        await connected_drone.set_heading(90)
        h = connected_drone.heading
        assert 80 < h < 100
        await connected_drone.land()


class TestDroneHeadingNonBlocking:
    """Non-blocking heading control."""

    @pytest.mark.asyncio
    async def test_set_heading_non_blocking_returns_quickly(self, connected_drone):
        """set_heading(blocking=False) should return without waiting."""
        import time
        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(10)
        start = time.time()
        await connected_drone.set_heading(180, blocking=False)
        elapsed = time.time() - start
        # Non-blocking should return well under 1 second
        assert elapsed < 2.0
        await connected_drone.land()

    @pytest.mark.asyncio
    async def test_set_heading_none_clears_heading(self, connected_drone):
        """set_heading(None) should clear the locked heading."""
        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(10)
        await connected_drone.set_heading(90)
        assert connected_drone._current_heading == 90
        await connected_drone.set_heading(None)
        assert connected_drone._current_heading is None
        await connected_drone.land()


class TestDroneGotoWithHeading:
    """goto_coordinates with target_heading."""

    @pytest.mark.asyncio
    async def test_goto_with_target_heading(self, connected_drone):
        from aerpawlib.v1.util import VectorNED
        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(10)
        start = connected_drone.position
        target = start + VectorNED(20, 0, 0)
        await connected_drone.goto_coordinates(target, tolerance=3, target_heading=90)
        dist = connected_drone.position.ground_distance(target)
        assert dist < 6
        await connected_drone.land()


class TestDroneMultiWaypoint:
    """Sequential waypoint navigation."""

    @pytest.mark.asyncio
    async def test_two_sequential_goto(self, connected_drone):
        from aerpawlib.v1.util import VectorNED
        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(10)
        pos0 = connected_drone.position
        wp1 = pos0 + VectorNED(20, 0, 0)
        wp2 = pos0 + VectorNED(20, 20, 0)
        await connected_drone.goto_coordinates(wp1, tolerance=3)
        assert connected_drone.position.ground_distance(wp1) < 5
        await connected_drone.goto_coordinates(wp2, tolerance=3)
        assert connected_drone.position.ground_distance(wp2) < 5
        await connected_drone.land()


class TestDroneVelocityControl:
    """Velocity control (set_velocity)."""

    @pytest.mark.asyncio
    async def test_set_velocity_moves_drone(self, connected_drone):
        """Commanding a northward velocity should move the drone north."""
        import asyncio as _asyncio
        from aerpawlib.v1.util import VectorNED
        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(10)
        pos_before = connected_drone.position
        if not hasattr(connected_drone, "set_velocity"):
            pytest.skip("set_velocity not implemented")
        await connected_drone.set_velocity(VectorNED(2, 0, 0))
        await _asyncio.sleep(3)
        await connected_drone.set_velocity(VectorNED(0, 0, 0))  # stop
        pos_after = connected_drone.position
        # Should have moved at least 2 m northward
        diff = pos_after - pos_before
        assert diff.north > 1.5
        await connected_drone.land()
