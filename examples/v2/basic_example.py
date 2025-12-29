"""
Example demonstrating the aerpawlib v2 API.

This example shows how to use the new Pythonic API for drone control.
To run this example, you need MAVSDK and a simulator (like PX4 SITL) running.

Usage:
    python -m examples.v2.basic_example
"""
import asyncio

from aerpawlib.v2 import (BasicRunner, Coordinate, Drone, entrypoint, sleep)


class SimpleSquareMission(BasicRunner):
    """
    A simple mission that flies a square pattern.

    This demonstrates:
    - Connection and arming
    - Takeoff
    - Navigation using coordinates
    - Using drone state properties
    - Landing
    """

    @entrypoint
    async def fly_square(self, drone: Drone):
        # Connect to the drone
        print("Connecting to drone...")
        await drone.connect()
        print(f"Connected! GPS: {drone.gps.quality} ({drone.gps.satellites} satellites)")

        # Wait for GPS lock
        while not drone.gps.has_fix:
            print("Waiting for GPS fix...")
            await sleep(1)

        # Arm and takeoff
        print("Arming...")
        await drone.arm()
        print(f"Armed: {drone.armed}")

        print("Taking off to 10m...")
        await drone.takeoff(altitude=10)
        print(f"Current altitude: {drone.state.altitude:.1f}m")

        # Get current position as starting point
        start = drone.position
        print(f"Starting position: {start}")

        # Define square corners (30m sides)
        corners = [
            start,  # Start
            Coordinate(start.latitude + 0.00027, start.longitude, 10),  # ~30m north
            Coordinate(start.latitude + 0.00027, start.longitude + 0.00036, 10),  # ~30m east
            Coordinate(start.latitude, start.longitude + 0.00036, 10),  # Back south
            start,  # Return to start
        ]

        # Fly the square
        for i, corner in enumerate(corners):
            print(f"Flying to corner {i+1}: {corner}")
            await drone.goto(coordinates=corner)
            print(f"  Arrived! Heading: {drone.state.heading:.0f}°, "
                  f"Speed: {drone.state.groundspeed:.1f}m/s, "
                  f"Battery: {drone.battery.percentage:.0f}%")

        # Land
        print("Landing...")
        await drone.land()
        print("Mission complete!")


class TelemetryDemo(BasicRunner):
    """
    Demonstrates accessing drone telemetry through the new API.
    """

    @entrypoint
    async def show_telemetry(self, drone: Drone):
        await drone.connect()

        print("\n=== Drone Telemetry Demo ===\n")

        # GPS information
        print("GPS Status:")
        print(f"  Satellites: {drone.gps.satellites}")
        print(f"  Quality: {drone.gps.quality}")
        print(f"  Has Fix: {drone.gps.has_fix}")

        # State information
        print("\nState:")
        print(f"  Position: {drone.state.position}")
        print(f"  Heading: {drone.state.heading:.1f}°")
        print(f"  Altitude: {drone.state.altitude:.1f}m (MSL)")
        print(f"  Relative Altitude: {drone.state.relative_altitude:.1f}m (AGL)")
        print(f"  Ground Speed: {drone.state.groundspeed:.1f}m/s")
        print(f"  Flight Mode: {drone.state.flight_mode.name}")
        print(f"  Landed State: {drone.state.landed_state.name}")

        # Attitude
        print("\nAttitude:")
        print(f"  Roll: {drone.state.attitude.roll_degrees:.1f}°")
        print(f"  Pitch: {drone.state.attitude.pitch_degrees:.1f}°")
        print(f"  Yaw: {drone.state.attitude.yaw_degrees:.1f}°")

        # Battery
        print("\nBattery:")
        print(f"  Voltage: {drone.battery.voltage:.1f}V")
        print(f"  Charge: {drone.battery.percentage:.0f}%")
        print(f"  Current: {drone.battery.current:.1f}A")

        # Vehicle info
        print("\nVehicle Info:")
        print(f"  Hardware UUID: {drone.info.hardware_uuid}")
        print(f"  Version: {drone.info.version}")
        print(f"  Vendor: {drone.info.vendor_name}")
        print(f"  Product: {drone.info.product_name}")

        # Convenience properties
        print("\nConvenience Properties:")
        print(f"  drone.armed: {drone.armed}")
        print(f"  drone.connected: {drone.connected}")
        print(f"  drone.is_armable: {drone.is_armable}")
        print(f"  drone.position: {drone.position}")
        print(f"  drone.heading: {drone.heading}")
        print(f"  drone.altitude: {drone.altitude}")
        print(f"  drone.velocity: {drone.velocity}")
        print(f"  drone.home: {drone.home}")


async def main():
    """Run the example mission."""
    # For simulation, use UDP
    drone = Drone("udp://:14540")

    mission = SimpleSquareMission()
    await mission.fly_square(drone)

    await drone.disconnect()


if __name__ == "__main__":
    asyncio.run(main())

