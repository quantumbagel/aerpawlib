"""
VehicleTask cancel v2 Example - Start goto, cancel mid-flight (triggers RTL).

Run with:
    aerpawlib --api-version v2 --script examples.v2.command_handle_cancel_example \
        --vehicle drone --conn udpin://127.0.0.1:14550
"""

import asyncio

from aerpawlib.v2 import BasicRunner, Drone, VectorNED, entrypoint


class CancelGotoDemo(BasicRunner):
    """Start goto, cancel after a few seconds, verify RTL and handle completion."""

    @entrypoint
    async def run(self, drone: Drone):
        await drone.takeoff(altitude=10)
        start = drone.position
        target = start + VectorNED(80, 0, 0)

        print("[example] Starting non-blocking goto...")
        handle = await drone.goto_coordinates(
            target, tolerance=3, blocking=False
        )
        assert handle is not None

        # Cancel after ~5 seconds
        for i in range(6):
            await asyncio.sleep(1)
            print(f"[example] Progress: {handle.progress:.0%}")
            if i == 4:
                print("[example] Cancelling goto (RTL)...")
                handle.cancel()
                break

        await handle.wait_done()
        print("[example] Handle complete (RTL in progress)")
        # RTL lands automatically; wait for disarm
        while drone.armed:
            await asyncio.sleep(0.5)
        print("[example] Mission complete")
