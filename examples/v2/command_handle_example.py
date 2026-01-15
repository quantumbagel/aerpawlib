#!/usr/bin/env python3
"""
Example demonstrating CommandHandle for non-blocking command execution.

CommandHandle allows you to:
- Execute commands without blocking
- Monitor command progress
- Cancel commands mid-execution
- Check command status and results

This is useful for complex missions where you want to:
- Run multiple operations in parallel
- Monitor telemetry while commands execute
- Implement custom abort/cancellation logic
- Log progress during long-running operations
"""
import asyncio

from aerpawlib.v2 import (
    BasicRunner,
    CommandCancelledError,
    Coordinate,
    Drone,
    entrypoint,
    GotoTimeoutError,
    VectorNED,
    # Logging
    get_logger,
    LogComponent,
)

# Get logger for user scripts (logging is configured by __main__.py)
logger = get_logger(LogComponent.USER)


class BasicNonBlockingExample(BasicRunner):
    """Basic example of non-blocking goto with progress monitoring."""

    @entrypoint
    async def run(self, drone: Drone):
        await drone.arm()
        await drone.takeoff(altitude=10)

        # Start a non-blocking goto
        target = Coordinate(35.7275, -78.6960, 10, "Target")
        handle = await drone.goto(coordinates=target, wait=False)

        # Monitor progress while moving
        while handle.is_running:
            progress = handle.progress
            logger.debug(
                f"Distance to target: {progress.get('distance', '?'):.1f}m"
            )
            logger.debug(f"Elapsed time: {handle.elapsed_time:.1f}s")
            if handle.time_remaining:
                logger.debug(f"Time remaining: {handle.time_remaining:.1f}s")
            await asyncio.sleep(1)

        # Check result
        if handle.succeeded:
            logger.info("✓ Arrived at destination!")
        elif handle.was_cancelled:
            logger.warning("✗ Goto was cancelled")
        elif handle.timed_out:
            logger.warning(
                f"✗ Goto timed out after {handle.elapsed_time:.1f}s"
            )
        else:
            logger.error(f"✗ Goto failed: {handle.error}")

        await drone.land()


class CancellationExample(BasicRunner):
    """Example of cancelling a command mid-execution."""

    @entrypoint
    async def run(self, drone: Drone):
        await drone.arm()
        await drone.takeoff(altitude=10)

        # Start a long goto
        far_target = Coordinate(35.8, -78.7, 10, "Far Target")
        handle = await drone.goto(
            coordinates=far_target, timeout=600, wait=False
        )

        logger.info("Starting goto to far target...")

        # Cancel after 10 seconds
        await asyncio.sleep(10)
        logger.info("Cancelling goto...")
        await handle.cancel()

        # The drone should now be holding position
        logger.info(f"Command status: {handle.status.name}")
        logger.info(f"Total flight time: {handle.elapsed_time:.1f}s")

        await drone.land()


class ParallelOperationsExample(BasicRunner):
    """Example of running operations in parallel with a moving drone."""

    @entrypoint
    async def run(self, drone: Drone):
        await drone.arm()
        await drone.takeoff(altitude=10)

        # Start heading towards target
        target = Coordinate(35.7275, -78.6960, 10, "Target")
        goto_handle = await drone.goto(coordinates=target, wait=False)

        # Log telemetry while moving
        log = []
        while goto_handle.is_running:
            log.append(
                {
                    "time": goto_handle.elapsed_time,
                    "position": drone.position,
                    "distance": goto_handle.progress.get("distance"),
                    "altitude": drone.altitude,
                    "heading": drone.heading,
                    "battery": drone.battery.percentage,
                }
            )
            await asyncio.sleep(0.5)

        logger.info(f"Logged {len(log)} telemetry points")

        # Wait for any remaining completion
        try:
            await goto_handle
        except CommandCancelledError:
            pass

        await drone.land()


class OrbitWithProgressExample(BasicRunner):
    """Example of monitoring orbit progress."""

    @entrypoint
    async def run(self, drone: Drone):
        await drone.arm()
        await drone.takeoff(altitude=15)

        center = drone.position + VectorNED(50, 0, 0)  # 50m north

        # Start non-blocking orbit
        handle = await drone.orbit(
            center=center, radius=30, speed=5, revolutions=2, wait=False
        )

        logger.info("Starting orbit...")
        while handle.is_running:
            progress = handle.progress
            logger.debug(
                f"Orbit: {progress.get('revolutions_completed', 0):.2f} / "
                f"{progress.get('target_revolutions', 0):.1f} revolutions "
                f"({progress.get('progress_percent', 0):.1f}%)"
            )
            await asyncio.sleep(2)

        logger.info(f"Orbit complete! Total time: {handle.elapsed_time:.1f}s")
        await drone.land()


class AwaitHandleLaterExample(BasicRunner):
    """Example of starting a command and awaiting it later."""

    @entrypoint
    async def run(self, drone: Drone):
        await drone.arm()

        # Start takeoff without waiting
        takeoff_handle = await drone.takeoff(altitude=20, wait=False)

        # Do something else while taking off
        logger.info("Takeoff initiated, doing other setup...")
        await asyncio.sleep(2)
        logger.info(f"Current altitude: {drone.altitude:.1f}m (target: 20m)")

        # Now wait for takeoff to complete
        try:
            await takeoff_handle  # Same as: await takeoff_handle.wait()
            logger.info("Takeoff complete!")
        except Exception as e:
            logger.error(f"Takeoff failed: {e}")
            return

        # Continue with mission
        await drone.goto(latitude=35.7275, longitude=-78.6960)
        await drone.land()


class CommandResultExample(BasicRunner):
    """Example of using CommandResult for detailed information."""

    @entrypoint
    async def run(self, drone: Drone):
        await drone.arm()
        await drone.takeoff(altitude=10)

        target = Coordinate(35.7275, -78.6960, 10, "Target")
        handle = await drone.goto(coordinates=target, wait=False)

        # Wait for completion
        try:
            await handle.wait()
        except (GotoTimeoutError, CommandCancelledError):
            pass

        # Get detailed result
        result = handle.result()
        logger.info(f"Command: {result.command}")
        logger.info(f"Status: {result.status.name}")
        logger.info(f"Duration: {result.duration:.2f}s")
        logger.info(f"Succeeded: {result.succeeded}")
        logger.info(f"Details: {result.details}")

        if result.error:
            logger.error(f"Error: {result.error}")

        await drone.land()


class VelocityWithDurationExample(BasicRunner):
    """Example of timed velocity command with handle."""

    @entrypoint
    async def run(self, drone: Drone):
        await drone.arm()
        await drone.takeoff(altitude=10)

        # Move north for 10 seconds, non-blocking
        velocity = VectorNED(5, 0, 0)  # 5 m/s north
        handle = await drone.set_velocity(velocity, duration=10, wait=False)

        logger.info("Moving north for 10 seconds...")
        while handle.is_running:
            progress = handle.progress
            logger.debug(
                f"Time remaining: {progress.get('time_remaining', 0):.1f}s"
            )
            await asyncio.sleep(1)

        logger.info("Velocity command complete!")
        await drone.hold()
        await drone.land()

