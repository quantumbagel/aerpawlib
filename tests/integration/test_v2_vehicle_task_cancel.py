"""Integration tests for aerpawlib v2 VehicleTask cancel. Requires SITL."""

import asyncio

import pytest

pytestmark = [pytest.mark.integration]


class TestVehicleTaskCancel:
    """Start goto, cancel, verify RTL and handle completion."""

    @pytest.mark.asyncio
    async def test_cancel_triggers_rtl(self, connected_drone_v2):
        from aerpawlib.v2.types import VectorNED

        connected_drone_v2._initialize_prearm(should_postarm_init=True)
        await connected_drone_v2.takeoff(10)
        start = connected_drone_v2.position
        target = start + VectorNED(50, 0, 0)
        handle = await connected_drone_v2.goto_coordinates(
            target, tolerance=3, blocking=False
        )
        assert handle is not None
        await asyncio.sleep(3)
        handle.cancel()
        await handle.wait_done()
        while connected_drone_v2.armed:
            await asyncio.sleep(0.5)
        assert connected_drone_v2.armed is False
