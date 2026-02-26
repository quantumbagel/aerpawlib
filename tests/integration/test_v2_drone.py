"""Integration tests for aerpawlib v2 Drone. Requires SITL."""

import asyncio
import time

import pytest

pytestmark = [pytest.mark.integration]


class TestDroneConnection:
    """Drone connection and telemetry."""

    @pytest.mark.asyncio
    async def test_connects(self, connected_drone_v2):
        assert connected_drone_v2.connected is True

    @pytest.mark.asyncio
    async def test_gps_fix(self, connected_drone_v2):
        assert connected_drone_v2.gps.fix_type >= 3

    @pytest.mark.asyncio
    async def test_position_valid(self, connected_drone_v2):
        pos = connected_drone_v2.position
        assert -90 <= pos.lat <= 90 and -180 <= pos.lon <= 180

    @pytest.mark.asyncio
    async def test_heading_valid(self, connected_drone_v2):
        assert 0 <= connected_drone_v2.heading < 360

    @pytest.mark.asyncio
    async def test_battery_valid(self, connected_drone_v2):
        b = connected_drone_v2.battery
        assert b.voltage > 0 and 0 <= b.level <= 100

    @pytest.mark.asyncio
    async def test_attitude_not_none(self, connected_drone_v2):
        a = connected_drone_v2.attitude
        assert a is not None
        assert isinstance(a.pitch, float)
        assert isinstance(a.roll, float)
        assert isinstance(a.yaw, float)

    @pytest.mark.asyncio
    async def test_home_coords_set_after_guided(self, connected_drone_v2):
        connected_drone_v2._initialize_prearm(should_postarm_init=True)
        home = connected_drone_v2.home_coords
        assert home is not None
        assert -90 <= home.lat <= 90
        assert -180 <= home.lon <= 180


class TestDroneArming:
    """Drone arming."""

    @pytest.mark.asyncio
    async def test_arms_on_takeoff(self, connected_drone_v2):
        connected_drone_v2._initialize_prearm(should_postarm_init=True)
        await connected_drone_v2.takeoff(5)
        assert connected_drone_v2.armed is True
        await connected_drone_v2.land()


class TestDroneTakeoff:
    """Drone takeoff."""

    @pytest.mark.asyncio
    async def test_takeoff_reaches_altitude(self, connected_drone_v2):
        connected_drone_v2._initialize_prearm(should_postarm_init=True)
        await connected_drone_v2.takeoff(10)
        assert connected_drone_v2.position.alt >= 9
        await connected_drone_v2.land()


class TestDroneNavigation:
    """Drone navigation."""

    @pytest.mark.asyncio
    async def test_goto_coordinates(self, connected_drone_v2):
        from aerpawlib.v2.types import VectorNED

        connected_drone_v2._initialize_prearm(should_postarm_init=True)
        await connected_drone_v2.takeoff(10)
        start = connected_drone_v2.position
        target = start + VectorNED(30, 0, 0)
        await connected_drone_v2.goto_coordinates(target, tolerance=3)
        dist = connected_drone_v2.position.ground_distance(target)
        assert dist < 5
        await connected_drone_v2.land()


class TestDroneLanding:
    """Drone landing."""

    @pytest.mark.asyncio
    async def test_land_disarms(self, connected_drone_v2):
        connected_drone_v2._initialize_prearm(should_postarm_init=True)
        await connected_drone_v2.takeoff(10)
        await connected_drone_v2.land()
        assert connected_drone_v2.armed is False


class TestDroneRTL:
    """Return to launch."""

    @pytest.mark.asyncio
    async def test_rtl_returns_home(self, connected_drone_v2):
        from aerpawlib.v2.types import VectorNED

        connected_drone_v2._initialize_prearm(should_postarm_init=True)
        await connected_drone_v2.takeoff(10)
        home = connected_drone_v2.home_coords
        target = connected_drone_v2.position + VectorNED(40, 0, 0)
        await connected_drone_v2.goto_coordinates(target, tolerance=3)
        await connected_drone_v2.return_to_launch()
        assert connected_drone_v2.armed is False


class TestDroneVehicleTask:
    """Non-blocking VehicleTask."""

    @pytest.mark.asyncio
    async def test_goto_non_blocking_returns_handle(self, connected_drone_v2):
        from aerpawlib.v2.types import VectorNED

        connected_drone_v2._initialize_prearm(should_postarm_init=True)
        await connected_drone_v2.takeoff(10)
        start = connected_drone_v2.position
        target = start + VectorNED(15, 0, 0)
        handle = await connected_drone_v2.goto_coordinates(
            target, tolerance=3, blocking=False
        )
        assert handle is not None
        await handle.wait_done()
        await connected_drone_v2.land()


class TestDroneRunner:
    """Runner discovery and execution."""

    @pytest.mark.asyncio
    async def test_basic_runner_executes(self, connected_drone_v2):
        from aerpawlib.v2 import BasicRunner, Drone, VectorNED, entrypoint

        class MinimalRunner(BasicRunner):
            @entrypoint
            async def run(self, drone: Drone):
                await drone.takeoff(altitude=5)
                await drone.land()

        runner = MinimalRunner()
        await runner.run(connected_drone_v2)
