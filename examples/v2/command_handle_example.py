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

from aerpawlib.v2 import (CommandCancelledError, Coordinate, Drone, GotoTimeoutError, VectorNED)


async def basic_non_blocking_example():
    """Basic example of non-blocking goto with progress monitoring."""
    async with Drone("udp://:14540") as drone:
        await drone.arm()
        await drone.takeoff(altitude=10)

        # Start a non-blocking goto
        target = Coordinate(35.7275, -78.6960, 10, "Target")
        handle = await drone.goto(coordinates=target, wait=False)

        # Monitor progress while moving
        while handle.is_running:
            progress = handle.progress
            print(f"Distance to target: {progress.get('distance', '?'):.1f}m")
            print(f"Elapsed time: {handle.elapsed_time:.1f}s")
            if handle.time_remaining:
                print(f"Time remaining: {handle.time_remaining:.1f}s")
            await asyncio.sleep(1)

        # Check result
        if handle.succeeded:
            print("✓ Arrived at destination!")
        elif handle.was_cancelled:
            print("✗ Goto was cancelled")
        elif handle.timed_out:
            print(f"✗ Goto timed out after {handle.elapsed_time:.1f}s")
        else:
            print(f"✗ Goto failed: {handle.error}")

        await drone.land()


async def cancellation_example():
    """Example of cancelling a command mid-execution."""
    async with Drone("udp://:14540") as drone:
        await drone.arm()
        await drone.takeoff(altitude=10)

        # Start a long goto
        far_target = Coordinate(35.8, -78.7, 10, "Far Target")
        handle = await drone.goto(coordinates=far_target, timeout=600, wait=False)

        print("Starting goto to far target...")

        # Cancel after 10 seconds
        await asyncio.sleep(10)
        print("Cancelling goto...")
        await handle.cancel()

        # The drone should now be holding position
        print(f"Command status: {handle.status.name}")
        print(f"Total flight time: {handle.elapsed_time:.1f}s")

        await drone.land()


async def parallel_operations_example():
    """Example of running operations in parallel with a moving drone."""
    async with Drone("udp://:14540") as drone:
        await drone.arm()
        await drone.takeoff(altitude=10)

        # Start heading towards target
        target = Coordinate(35.7275, -78.6960, 10, "Target")
        goto_handle = await drone.goto(coordinates=target, wait=False)

        # Log telemetry while moving
        log = []
        while goto_handle.is_running:
            log.append({
                "time": goto_handle.elapsed_time,
                "position": drone.position,
                "distance": goto_handle.progress.get("distance"),
                "altitude": drone.altitude,
                "heading": drone.heading,
                "battery": drone.battery.percentage,
            })
            await asyncio.sleep(0.5)

        print(f"Logged {len(log)} telemetry points")

        # Wait for any remaining completion
        try:
            await goto_handle
        except CommandCancelledError:
            pass

        await drone.land()


async def orbit_with_progress_example():
    """Example of monitoring orbit progress."""
    async with Drone("udp://:14540") as drone:
        await drone.arm()
        await drone.takeoff(altitude=15)

        center = drone.position + VectorNED(50, 0, 0)  # 50m north

        # Start non-blocking orbit
        handle = await drone.orbit(
            center=center,
            radius=30,
            speed=5,
            revolutions=2,
            wait=False
        )

        print("Starting orbit...")
        while handle.is_running:
            progress = handle.progress
            print(f"Orbit: {progress.get('revolutions_completed', 0):.2f} / "
                  f"{progress.get('target_revolutions', 0):.1f} revolutions "
                  f"({progress.get('progress_percent', 0):.1f}%)")
            await asyncio.sleep(2)

        print(f"Orbit complete! Total time: {handle.elapsed_time:.1f}s")
        await drone.land()


async def await_handle_later_example():
    """Example of starting a command and awaiting it later."""
    async with Drone("udp://:14540") as drone:
        await drone.arm()

        # Start takeoff without waiting
        takeoff_handle = await drone.takeoff(altitude=20, wait=False)

        # Do something else while taking off
        print("Takeoff initiated, doing other setup...")
        await asyncio.sleep(2)
        print(f"Current altitude: {drone.altitude:.1f}m (target: 20m)")

        # Now wait for takeoff to complete
        try:
            await takeoff_handle  # Same as: await takeoff_handle.wait()
            print("Takeoff complete!")
        except Exception as e:
            print(f"Takeoff failed: {e}")
            return

        # Continue with mission
        await drone.goto(latitude=35.7275, longitude=-78.6960)
        await drone.land()


async def command_result_example():
    """Example of using CommandResult for detailed information."""
    async with Drone("udp://:14540") as drone:
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
        print(f"Command: {result.command}")
        print(f"Status: {result.status.name}")
        print(f"Duration: {result.duration:.2f}s")
        print(f"Succeeded: {result.succeeded}")
        print(f"Details: {result.details}")

        if result.error:
            print(f"Error: {result.error}")

        await drone.land()


async def velocity_with_duration_example():
    """Example of timed velocity command with handle."""
    async with Drone("udp://:14540") as drone:
        await drone.arm()
        await drone.takeoff(altitude=10)

        # Move north for 10 seconds, non-blocking
        velocity = VectorNED(5, 0, 0)  # 5 m/s north
        handle = await drone.set_velocity(velocity, duration=10, wait=False)

        print("Moving north for 10 seconds...")
        while handle.is_running:
            progress = handle.progress
            print(f"Time remaining: {progress.get('time_remaining', 0):.1f}s")
            await asyncio.sleep(1)

        print("Velocity command complete!")
        await drone.hold()
        await drone.land()


if __name__ == "__main__":
    # Run one of the examples
    print("Running basic non-blocking example...")
    asyncio.run(basic_non_blocking_example())

