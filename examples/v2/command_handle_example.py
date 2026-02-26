"""
VehicleTask v2 Example - Non-blocking goto with progress and cancel

Run with:
    aerpawlib --api-version v2 --script examples.v2.command_handle_example \
        --vehicle drone --conn udpin://127.0.0.1:14550
"""

import asyncio

from aerpawlib.v2 import BasicRunner, Drone, VectorNED, entrypoint


class VehicleTaskDemo(BasicRunner):
    """Demonstrate non-blocking goto with VehicleTask."""

    @entrypoint
    async def run(self, drone: Drone):
        await drone.takeoff(altitude=10)
        start = drone.position
        target = start + VectorNED(50, 0, 0)

        print("[example] Starting non-blocking goto...")
        handle = await drone.goto_coordinates(
            target, tolerance=3, blocking=False
        )
        assert handle is not None

        # Poll progress until done
        for _ in range(60):
            await asyncio.sleep(1)
            print(f"[example] Progress: {handle.progress:.0%}")
            if handle.is_done():
                break
        await handle.wait_done()
        print("[example] Goto complete")
        await drone.land()
