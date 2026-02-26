"""
Velocity control v2 Example - Drone using set_velocity for manual flight.

Run with:
    aerpawlib --api-version v2 --script examples.v2.velocity_example \
        --vehicle drone --conn udpin://127.0.0.1:14550
"""

import asyncio

from aerpawlib.v2 import BasicRunner, Drone, VectorNED, entrypoint


class VelocityControlDemo(BasicRunner):
    """Demonstrate velocity control: hover, move north, stop."""

    @entrypoint
    async def run(self, drone: Drone):
        await drone.takeoff(altitude=10)
        print("[example] Hovering...")
        await drone.set_velocity(VectorNED(0, 0, 0), duration=2.0)
        print("[example] Moving north at 2 m/s...")
        await drone.set_velocity(VectorNED(2, 0, 0), duration=3.0)
        print("[example] Stopping...")
        await drone.set_velocity(VectorNED(0, 0, 0), duration=1.0)
        await asyncio.sleep(0.5)
        print("[example] Landing...")
        await drone.land()
        print("[example] Done")
