"""
Rover v2 Example - BasicRunner with Rover: 2D waypoints, no takeoff/land.

Run with:
    aerpawlib --api-version v2 --script examples.v2.rover_example \
        --vehicle rover --conn udpin://127.0.0.1:14560
"""

from aerpawlib.v2 import BasicRunner, Rover, VectorNED, entrypoint


class RoverWaypointMission(BasicRunner):
    """Drive rover through 2D waypoints."""

    @entrypoint
    async def run(self, rover: Rover):
        print("[example] Starting rover waypoint mission")
        start = rover.position
        waypoints = [
            start + VectorNED(20, 0, 0),
            start + VectorNED(20, 15, 0),
            start + VectorNED(5, 15, 0),
            start,
        ]
        for i, target in enumerate(waypoints):
            print(f"[example] Driving to waypoint {i + 1}/{len(waypoints)}")
            await rover.goto_coordinates(target, tolerance=2.5)
        print("[example] Mission complete!")
