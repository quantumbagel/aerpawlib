"""Integration tests for aerpawlib v2 StateMachine. Requires SITL."""

import pytest

pytestmark = [pytest.mark.integration]


class TestStateMachineWithSITL:
    """StateMachine with SITL: states, timed_state, background."""

    @pytest.mark.asyncio
    async def test_state_machine_executes(self, connected_drone_v2):
        from aerpawlib.v2 import (
            Drone,
            StateMachine,
            VectorNED,
            background,
            state,
            timed_state,
        )

        class TestSM(StateMachine):
            def __init__(self):
                super().__init__()
                self._leg = 0

            @background
            async def log_pos(self, drone: Drone):
                import asyncio
                for _ in range(3):
                    _ = drone.position
                    await asyncio.sleep(0.5)

            @state(name="take_off", first=True)
            async def take_off(self, drone: Drone):
                await drone.takeoff(altitude=5)
                return "fly"

            @timed_state(name="fly", duration=1)
            async def fly(self, drone: Drone):
                target = drone.position + VectorNED(10, 0, 0)
                await drone.goto_coordinates(target, tolerance=3)
                return "land"

            @state(name="land")
            async def land(self, drone: Drone):
                await drone.land()
                return None

        sm = TestSM()
        await sm.run(connected_drone_v2)
        assert connected_drone_v2.armed is False
