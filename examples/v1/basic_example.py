"""
Basic v1 API Example - Square Pattern Flight

This script demonstrates the v1 API with a simple mission:
1. Take off to 10 meters
2. Fly in a square pattern (10m x 10m)
3. Return to start and land

Run with:
    aerpawlib --api-version v1 --script examples.v1.basic_example \
        --vehicle drone --conn udp:127.0.0.1:14550
"""

from aerpawlib.v1.runner import BasicRunner, entrypoint
from aerpawlib.v1.util import VectorNED
from aerpawlib.v1.vehicle import Drone


class SquareFlight(BasicRunner):
    """Fly in a square pattern."""

    @entrypoint
    async def run(self, drone: Drone):
        # Configuration
        takeoff_altitude = 10  # meters
        square_size = 10  # meters

        print(f"[example] Starting square flight mission")
        print(f"[example] Takeoff altitude: {takeoff_altitude}m")
        print(f"[example] Square size: {square_size}m x {square_size}m")

        # Take off
        print(f"[example] Taking off to {takeoff_altitude}m...")
        await drone.takeoff(takeoff_altitude)
        print(f"[example] Takeoff complete, altitude: {drone.position.alt:.1f}m")

        # Save starting position
        start_position = drone.position
        print(f"[example] Start position: {start_position}")

        # Fly square pattern: North -> East -> South -> West -> Back to start
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
            print(f"[example] Arrived at {direction} waypoint")

        # Return to start position
        print(f"[example] Returning to start position...")
        await drone.goto_coordinates(start_position)
        print(f"[example] Back at start position")

        # Land
        print(f"[example] Landing...")
        await drone.land()
        print(f"[example] Mission complete!")

