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
from aerpawlib.v2 import (
    AbortError,
    AerpawlibError,
    BasicRunner,
    ConnectionTimeoutError,
    Coordinate,
    Drone,
    GotoTimeoutError,
    NavigationError,
    NotArmableError,
    TakeoffError,
    TakeoffTimeoutError,
    VectorNED,
    entrypoint,
    # Logging
    get_logger,
    LogComponent,
)

# Get logger for user scripts (logging is configured by __main__.py)
logger = get_logger(LogComponent.USER)


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
        logger.info("Connecting to drone with retry support...")
        try:
            await drone.connect(
                timeout=30.0,
                retry_count=3,
                retry_delay=2.0
            )
            logger.info(f"Connected! GPS: {drone.gps.quality} ({drone.gps.satellites} satellites)")
        except ConnectionTimeoutError as e:
            logger.error(f"Connection failed after {e.max_attempts} attempts: {e}")
            return

        # Wait for GPS with error handling
        try:
            await drone.wait_for_gps_fix(min_satellites=6, timeout=60.0)
        except Exception as e:
            logger.error(f"GPS fix timeout: {e}")
            return

        # Start telemetry recording
        logger.info("Starting telemetry recording at 10Hz...")
        drone.start_recording(interval=0.1)

        try:
            # Arm with structured exception handling
            logger.info("Arming...")
            try:
                await drone.arm()
            except NotArmableError as e:
                logger.error(f"Cannot arm: {e}")
                logger.error(f"  Reasons: {e.reasons}")
                return

            # Takeoff with specific error handling
            logger.info("Taking off to 15m...")
            try:
                await drone.takeoff(altitude=15)
            except TakeoffTimeoutError as e:
                logger.error(f"Takeoff timed out at {e.current_altitude:.1f}m (target: {e.target_altitude}m)")
                await drone.land()
                return
            except TakeoffError as e:
                logger.error(f"Takeoff failed: {e.reason}")
                return

            # Use extended Coordinate methods
            home = drone.position
            logger.info(f"Home position: {home}")

            # Create waypoints using from_relative
            wp1 = Coordinate.from_relative(home, VectorNED(50, 0, 0), "North 50m")
            wp2 = Coordinate.from_relative(home, VectorNED(50, 50, 0), "Northeast")
            wp3 = Coordinate.from_relative(home, VectorNED(0, 50, 0), "East 50m")

            # Generate a path back home with intermediate points
            return_path = wp3.path_to(home, num_points=3, include_endpoints=True)
            logger.info(f"Return path has {len(return_path)} waypoints")

            # Fly to waypoints with structured error handling
            waypoints = [wp1, wp2, wp3] + return_path[1:]  # Skip wp3 duplicate

            for i, wp in enumerate(waypoints):
                logger.info(f"Flying to waypoint {i+1}/{len(waypoints)}: {wp.name or 'unnamed'}")

                # Calculate distance
                distance = drone.position.distance_to(wp)
                logger.info(f"  Distance: {distance:.1f}m")

                try:
                    await drone.goto(coordinates=wp, tolerance=2.0, timeout=60.0)
                    logger.info(f"  Arrived! Heading: {drone.heading:.0f}°")
                except GotoTimeoutError as e:
                    logger.warning(f"  Timeout! {e.distance_remaining:.1f}m remaining")
                    # Continue to next waypoint
                except NavigationError as e:
                    logger.error(f"  Navigation failed: {e.reason}")
                    break
                except AbortError:
                    logger.warning("  Mission aborted!")
                    break

            # Use midpoint method for demonstration
            mid = home.midpoint_to(wp2)
            logger.info(f"Midpoint between home and WP2: {mid}")

            # Use interpolate method
            quarter = home.interpolate_to(wp1, 0.25)
            logger.info(f"25% of the way to WP1: {quarter}")

        except AerpawlibError as e:
            # Catch-all for any aerpawlib error
            logger.error(f"Mission error [{e.severity.name}]: {e.message}")
            if e.details:
                logger.error(f"  Details: {e.details}")

        finally:
            # Stop recording and save
            point_count = drone.stop_recording()
            logger.info(f"Stopped recording. Captured {point_count} data points")

            # Save flight log
            drone.save_flight_log("flight_log.json", format="json")
            logger.info("Saved flight log to flight_log.json")

            # Land
            logger.info("Landing...")
            try:
                await drone.land()
                logger.info("Mission complete!")
            except Exception as e:
                logger.error(f"Landing error: {e}")


class CoordinateDemo(BasicRunner):
    """
    Demonstrates the extended Coordinate methods without flying.
    """

    @entrypoint
    async def demo_coordinates(self):
        # Create a base coordinate
        base = Coordinate(35.7275, -78.6960, 0, "Home")
        logger.info(f"Base: {base}")

        # from_relative - create coordinate from offset
        north_50m = Coordinate.from_relative(base, VectorNED(50, 0, 0), "North 50m")
        east_30m = Coordinate.from_relative(base, VectorNED(0, 30, -10), "East 30m, Up 10m")
        logger.info(f"North 50m: {north_50m}")
        logger.info(f"East 30m, Up 10m: {east_30m}")

        # Distance and bearing
        logger.info(f"Distance to north_50m: {base.distance_to(north_50m):.1f}m")
        logger.info(f"Bearing to east_30m: {base.bearing_to(east_30m):.1f}°")

        # midpoint_to
        midpoint = base.midpoint_to(north_50m, "Midpoint")
        logger.info(f"Midpoint: {midpoint}")

        # interpolate_to
        quarter = base.interpolate_to(north_50m, 0.25, "Quarter")
        three_quarter = base.interpolate_to(north_50m, 0.75, "Three Quarter")
        logger.info(f"25% point: {quarter}")
        logger.info(f"75% point: {three_quarter}")

        # path_to - generate waypoints
        path = base.path_to(north_50m, num_points=5)
        logger.info("Path with 5 points:")
        for i, point in enumerate(path):
            dist = base.distance_to(point)
            logger.info(f"  Point {i+1}: {dist:.1f}m from base")

        # path_to without endpoints
        path_no_endpoints = base.path_to(north_50m, num_points=3, include_endpoints=False)
        logger.info("Path with 3 interior points (no endpoints):")
        for i, point in enumerate(path_no_endpoints):
            dist = base.distance_to(point)
            logger.info(f"  Point {i+1}: {dist:.1f}m from base")

        # with_altitude and with_name
        elevated = base.with_altitude(100)
        renamed = base.with_name("Launch Site")
        logger.info(f"Elevated: {elevated}")
        logger.info(f"Renamed: {renamed}")
