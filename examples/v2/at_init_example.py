"""
at_init v2 Example - StateMachine with @at_init for pre-arm setup.

Run with:
    aerpawlib --api-version v2 --script examples.v2.at_init_example \
        --vehicle drone --conn udpin://127.0.0.1:14550
"""

import asyncio

from aerpawlib.v2 import (
    Drone,
    StateMachine,
    VectorNED,
    at_init,
    state,
    timed_state,
)


class AtInitMission(StateMachine):
    """State machine with @at_init for pre-arm setup."""

    def __init__(self):
        super().__init__()
        self._ready = False

    @at_init
    async def wait_for_gps_and_prearm(self, drone: Drone):
        """Run before arm: wait for GPS fix, log readiness."""
        print("[at_init] Waiting for GPS 3D fix...")
        while drone.gps.fix_type < 3:
            await asyncio.sleep(0.5)
        print("[at_init] GPS OK, ready to arm")
        self._ready = True

    @state(name="take_off", first=True)
    async def take_off(self, drone: Drone):
        assert self._ready
        await drone.takeoff(altitude=10)
        return "fly"

    @timed_state(name="fly", duration=2)
    async def fly(self, drone: Drone):
        target = drone.position + VectorNED(15, 0, 0)
        await drone.goto_coordinates(target)
        return "land"

    @state(name="land")
    async def land(self, drone: Drone):
        await drone.land()
        return None
