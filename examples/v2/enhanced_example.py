"""
Example demonstrating the enhanced aerpawlib v2 API features.

This example shows:
- Structured exception handling
- Automatic connection retry
- Telemetry recording
- Extended Coordinate methods

Usage:
    python -m examples.v2.enhanced_example
"""
from aerpawlib.v2 import (AbortError, AerpawlibError, BasicRunner, ConnectionTimeoutError, Coordinate, Drone,
                          GotoTimeoutError, NavigationError, NotArmableError, TakeoffError, TakeoffTimeoutError,
                          VectorNED, entrypoint)  # Exceptions for error handling


class EnhancedMission(BasicRunner):
    """
    A mission demonstrating enhanced v2 API features.

    Shows:
    - Automatic connection retry
    - Structured exception handling
    - Telemetry recording
    - Extended Coordinate methods
    """

    @entrypoint
    async def fly_mission(self, drone: Drone):
        # Connection with automatic retry
        print("Connecting to drone with retry support...")
        try:
            await drone.connect(
                timeout=30.0,
                auto_reconnect=True,
                retry_count=3,
                retry_delay=2.0
            )
            print(f"Connected! GPS: {drone.gps.quality} ({drone.gps.satellites} satellites)")
        except ConnectionTimeoutError as e:
            print(f"Connection failed after {e.max_attempts} attempts: {e}")
            return

        # Wait for GPS with error handling
        try:
            await drone.wait_for_gps_fix(min_satellites=6, timeout=60.0)
        except Exception as e:
            print(f"GPS fix timeout: {e}")
            return

        # Start telemetry recording
        print("Starting telemetry recording at 10Hz...")
        drone.start_recording(interval=0.1)

        try:
            # Arm with structured exception handling
            print("Arming...")
            try:
                await drone.arm()
            except NotArmableError as e:
                print(f"Cannot arm: {e}")
                print(f"  Reasons: {e.reasons}")
                return

            # Takeoff with specific error handling
            print("Taking off to 15m...")
            try:
                await drone.takeoff(altitude=15)
            except TakeoffTimeoutError as e:
                print(f"Takeoff timed out at {e.current_altitude:.1f}m (target: {e.target_altitude}m)")
                await drone.land()
                return
            except TakeoffError as e:
                print(f"Takeoff failed: {e.reason}")
                return

            # Use extended Coordinate methods
            home = drone.position
            print(f"Home position: {home}")

            # Create waypoints using from_relative
            wp1 = Coordinate.from_relative(home, VectorNED(50, 0, 0), "North 50m")
            wp2 = Coordinate.from_relative(home, VectorNED(50, 50, 0), "Northeast")
            wp3 = Coordinate.from_relative(home, VectorNED(0, 50, 0), "East 50m")

            # Generate a path back home with intermediate points
            return_path = wp3.path_to(home, num_points=3, include_endpoints=True)
            print(f"Return path has {len(return_path)} waypoints")

            # Fly to waypoints with structured error handling
            waypoints = [wp1, wp2, wp3] + return_path[1:]  # Skip wp3 duplicate

            for i, wp in enumerate(waypoints):
                print(f"Flying to waypoint {i+1}/{len(waypoints)}: {wp.name or 'unnamed'}")

                # Calculate distance
                distance = drone.position.distance_to(wp)
                print(f"  Distance: {distance:.1f}m")

                try:
                    await drone.goto(coordinates=wp, tolerance=2.0, timeout=60.0)
                    print(f"  Arrived! Heading: {drone.heading:.0f}°")
                except GotoTimeoutError as e:
                    print(f"  Timeout! {e.distance_remaining:.1f}m remaining")
                    # Continue to next waypoint
                except NavigationError as e:
                    print(f"  Navigation failed: {e.reason}")
                    break
                except AbortError:
                    print("  Mission aborted!")
                    break

            # Use midpoint method for demonstration
            mid = home.midpoint_to(wp2)
            print(f"Midpoint between home and WP2: {mid}")

            # Use interpolate method
            quarter = home.interpolate_to(wp1, 0.25)
            print(f"25% of the way to WP1: {quarter}")

        except AerpawlibError as e:
            # Catch-all for any aerpawlib error
            print(f"Mission error [{e.severity.name}]: {e.message}")
            if e.details:
                print(f"  Details: {e.details}")

        finally:
            # Stop recording and save
            point_count = drone.stop_recording()
            print(f"Stopped recording. Captured {point_count} data points")

            # Save flight log
            drone.save_flight_log("flight_log.json", format="json")
            print("Saved flight log to flight_log.json")

            # Land
            print("Landing...")
            try:
                await drone.land()
                print("Mission complete!")
            except Exception as e:
                print(f"Landing error: {e}")


class CoordinateDemo(BasicRunner):
    """
    Demonstrates the extended Coordinate methods without flying.
    """

    @entrypoint
    async def demo_coordinates(self):
        # Create a base coordinate
        base = Coordinate(35.7275, -78.6960, 0, "Home")
        print(f"Base: {base}")

        # from_relative - create coordinate from offset
        north_50m = Coordinate.from_relative(base, VectorNED(50, 0, 0), "North 50m")
        east_30m = Coordinate.from_relative(base, VectorNED(0, 30, -10), "East 30m, Up 10m")
        print(f"North 50m: {north_50m}")
        print(f"East 30m, Up 10m: {east_30m}")

        # Distance and bearing
        print(f"Distance to north_50m: {base.distance_to(north_50m):.1f}m")
        print(f"Bearing to east_30m: {base.bearing_to(east_30m):.1f}°")

        # midpoint_to
        midpoint = base.midpoint_to(north_50m, "Midpoint")
        print(f"Midpoint: {midpoint}")

        # interpolate_to
        quarter = base.interpolate_to(north_50m, 0.25, "Quarter")
        three_quarter = base.interpolate_to(north_50m, 0.75, "Three Quarter")
        print(f"25% point: {quarter}")
        print(f"75% point: {three_quarter}")

        # path_to - generate waypoints
        path = base.path_to(north_50m, num_points=5)
        print(f"\nPath with 5 points:")
        for i, point in enumerate(path):
            dist = base.distance_to(point)
            print(f"  Point {i+1}: {dist:.1f}m from base")

        # path_to without endpoints
        path_no_endpoints = base.path_to(north_50m, num_points=3, include_endpoints=False)
        print(f"\nPath with 3 interior points (no endpoints):")
        for i, point in enumerate(path_no_endpoints):
            dist = base.distance_to(point)
            print(f"  Point {i+1}: {dist:.1f}m from base")

        # with_altitude and with_name
        elevated = base.with_altitude(100)
        renamed = base.with_name("Launch Site")
        print(f"\nElevated: {elevated}")
        print(f"Renamed: {renamed}")


if __name__ == "__main__":
    # Run the coordinate demo (doesn't require actual drone)
    print("=== Coordinate Demo ===\n")
    demo = CoordinateDemo()
    # Note: In real usage, you would run the mission with the runner framework
    # asyncio.run(demo.run(drone))

