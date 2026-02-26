"""
Basic v2 API Example - Square Pattern Flight

Run with:
    aerpawlib --api-version v2 --script examples.v2.basic_example \
        --vehicle drone --conn udpin://127.0.0.1:14550
"""

from aerpawlib.v2 import BasicRunner, Drone, VectorNED, entrypoint


class SquareFlight(BasicRunner):
    """Fly in a square pattern."""

    @entrypoint
    async def run(self, drone: Drone):
        takeoff_altitude = 10
        square_size = 10

        print("[example] Starting square flight mission")
        await drone.takeoff(altitude=takeoff_altitude)
        print(f"[example] Takeoff complete, altitude: {drone.position.alt:.1f}m")

        start_position = drone.position
        waypoints = [
            ("North", VectorNED(square_size, 0, 0)),
            ("East", VectorNED(0, square_size, 0)),
            ("South", VectorNED(-square_size, 0, 0)),
            ("West", VectorNED(0, -square_size, 0)),
        ]

        current_position = start_position
        for direction, offset in waypoints:
            target = current_position + offset
            print(f"[example] Flying {direction} to {target}")
            await drone.goto_coordinates(target)
            current_position = target

        print("[example] Returning to start...")
        await drone.goto_coordinates(start_position)
        print("[example] Landing...")
        await drone.land()
        print("[example] Mission complete!")
