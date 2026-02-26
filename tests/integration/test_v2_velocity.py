"""Integration tests for aerpawlib v2 Drone velocity control. Requires SITL."""

import asyncio

import pytest

pytestmark = [pytest.mark.integration]


class TestDroneVelocity:
    """Drone set_velocity."""

    @pytest.mark.asyncio
    async def test_set_velocity_verify_movement(self, connected_drone_v2):
        from aerpawlib.v2.types import VectorNED

        connected_drone_v2._initialize_prearm(should_postarm_init=True)
        await connected_drone_v2.takeoff(10)
        start = connected_drone_v2.position
        await connected_drone_v2.set_velocity(VectorNED(2, 0, 0), duration=2.0)
        await asyncio.sleep(0.5)
        end = connected_drone_v2.position
        dist = start.ground_distance(end)
        assert dist > 1
        await connected_drone_v2.land()
