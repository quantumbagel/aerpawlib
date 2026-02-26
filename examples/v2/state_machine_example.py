"""
StateMachine v2 Example - States, timed_state, background

Run with:
    aerpawlib --api-version v2 --script examples.v2.state_machine_example \
        --vehicle drone --conn udpin://127.0.0.1:14550
"""

from aerpawlib.v2 import (
    Drone,
    StateMachine,
    VectorNED,
    background,
    state,
    timed_state,
)


class SquareStateMachine(StateMachine):
    """State machine with timed state and background task."""

    def __init__(self):
        super().__init__()
        self._legs = [
            VectorNED(10, 0, 0),
            VectorNED(0, 10, 0),
            VectorNED(-10, 0, 0),
            VectorNED(0, -10, 0),
        ]
        self._current_leg = 0

    @background
    async def log_position(self, drone: Drone):
        """Background: log position every 2 seconds."""
        import asyncio
        while True:
            pos = drone.position
            print(f"[bg] Position: {pos.lat:.6f}, {pos.lon:.6f}, alt={pos.alt:.1f}m")
            await asyncio.sleep(2)

    @state(name="start", first=True)
    async def start(self, drone: Drone):
        print("[state] start -> take_off")
        return "take_off"

    @state(name="take_off")
    async def take_off(self, drone: Drone):
        await drone.takeoff(altitude=10)
        print("[state] take_off -> at_position")
        return "at_position"

    @timed_state(name="at_position", duration=3)
    async def at_position(self, drone: Drone):
        start = drone.position
        wp = start + self._legs[self._current_leg]
        await drone.goto_coordinates(wp)
        self._current_leg += 1
        if self._current_leg >= len(self._legs):
            return "land"
        return "at_position"

    @state(name="land")
    async def land(self, drone: Drone):
        await drone.land()
        print("[state] land -> done")
        return None
