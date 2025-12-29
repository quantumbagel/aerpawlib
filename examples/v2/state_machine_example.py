"""
State machine example for aerpawlib v2 API.

This example demonstrates how to use the StateMachine pattern for
complex missions with multiple states and background tasks.
"""
import asyncio

from aerpawlib.v2 import (Coordinate, Drone, StateMachine, at_init, background, sleep, state, timed_state)


class SurveyMission(StateMachine):
    """
    A survey mission using the StateMachine pattern.

    States:
    1. preflight - Check systems
    2. takeoff - Take off to survey altitude
    3. survey - Visit each waypoint
    4. rtl - Return to launch

    Background tasks:
    - Log telemetry every second
    - Monitor battery level
    """

    def __init__(self):
        super().__init__()
        self.waypoints = []
        self.current_waypoint = 0
        self.survey_altitude = 15.0
        self.min_battery = 20.0  # Minimum battery percentage

    @at_init
    async def load_mission(self):
        """Load waypoints before the mission starts."""
        # In a real mission, you might load from a file:
        # self.waypoints = read_waypoints_from_plan("survey.plan")

        # For this example, we'll create waypoints manually
        # These are offset from the home position
        self.waypoints = [
            Coordinate(35.7275, -78.6960, self.survey_altitude, "Point A"),
            Coordinate(35.7280, -78.6960, self.survey_altitude, "Point B"),
            Coordinate(35.7280, -78.6955, self.survey_altitude, "Point C"),
            Coordinate(35.7275, -78.6955, self.survey_altitude, "Point D"),
        ]
        print(f"Loaded {len(self.waypoints)} survey waypoints")

    @background
    async def log_telemetry(self, drone: Drone):
        """Log telemetry data periodically."""
        print(f"[Telemetry] Alt: {drone.altitude:.1f}m, "
              f"Speed: {drone.state.groundspeed:.1f}m/s, "
              f"Heading: {drone.heading:.0f}°, "
              f"Battery: {drone.battery.percentage:.0f}%")
        await sleep(1)

    @background
    async def battery_monitor(self, drone: Drone):
        """Monitor battery and trigger RTL if low."""
        if drone.battery.percentage < self.min_battery:
            print(f"[WARNING] Battery low ({drone.battery.percentage:.0f}%)! Triggering RTL...")
            self.transition_to("emergency_rtl")
        await sleep(5)

    @state("preflight", first=True)
    async def preflight_check(self, drone: Drone):
        """Perform pre-flight checks."""
        print("\n=== Pre-flight Checks ===")

        # Check GPS
        if not drone.gps.has_fix:
            print("  [WAITING] GPS fix...")
            await sleep(1)
            return "preflight"  # Stay in this state
        print(f"  [OK] GPS: {drone.gps.quality} ({drone.gps.satellites} sats)")

        # Check battery
        if drone.battery.percentage < 30:
            print(f"  [FAIL] Battery too low: {drone.battery.percentage:.0f}%")
            return None  # Stop the mission
        print(f"  [OK] Battery: {drone.battery.percentage:.0f}%")

        # Check if armable
        if not drone.is_armable:
            print("  [WAITING] Vehicle not armable...")
            await sleep(1)
            return "preflight"
        print("  [OK] Vehicle is armable")

        print("Pre-flight checks passed!")
        return "takeoff"

    @state("takeoff")
    async def takeoff_state(self, drone: Drone):
        """Arm and takeoff."""
        print(f"\n=== Taking Off to {self.survey_altitude}m ===")

        await drone.arm()
        await drone.takeoff(altitude=self.survey_altitude)

        print(f"Takeoff complete! Altitude: {drone.altitude:.1f}m")
        return "survey"

    @state("survey")
    async def survey_state(self, drone: Drone):
        """Visit each survey waypoint."""
        if self.current_waypoint >= len(self.waypoints):
            print("\n=== Survey Complete ===")
            return "rtl"

        waypoint = self.waypoints[self.current_waypoint]
        print(f"\n=== Flying to {waypoint.name} ({self.current_waypoint + 1}/{len(self.waypoints)}) ===")

        await drone.goto(coordinates=waypoint, tolerance=2.0)

        # Simulate data collection
        print(f"  Collecting data at {waypoint.name}...")
        await sleep(2)

        self.current_waypoint += 1
        return "survey"  # Continue to next waypoint

    @state("rtl")
    async def return_to_launch(self, drone: Drone):
        """Return to launch position."""
        print("\n=== Returning to Launch ===")
        await drone.rtl()
        print("Mission complete! Landed safely.")
        return None  # Stop the state machine

    @state("emergency_rtl")
    async def emergency_return(self, drone: Drone):
        """Emergency return due to low battery."""
        print("\n=== EMERGENCY RTL ===")
        await drone.rtl()
        print("Emergency landing complete.")
        return None


class OrbitMission(StateMachine):
    """
    Demonstrates timed_state for continuous operations like orbiting.
    """

    def __init__(self, orbit_radius: float = 20.0, orbit_duration: float = 60.0):
        super().__init__()
        self.orbit_radius = orbit_radius
        self.orbit_duration = orbit_duration
        self.orbit_center = None

    @state("setup", first=True)
    async def setup(self, drone: Drone):
        """Initial setup and takeoff."""
        await drone.arm()
        await drone.takeoff(altitude=10)
        self.orbit_center = drone.position
        return "orbit"

    @timed_state("orbit", duration=60.0, loop=True)
    async def orbit_state(self, drone: Drone):
        """Orbit around the center point for the specified duration."""
        import math

        # Calculate next position on the orbit circle
        elapsed = drone.info.time_since_takeoff_ms / 1000.0
        angle = (elapsed * 10) % 360  # 10 degrees per second

        rad = math.radians(angle)
        self.orbit_radius * math.cos(rad)
        self.orbit_radius * math.sin(rad)

        # Set velocity to orbit
        from aerpawlib.v2 import VectorNED
        velocity = VectorNED(
            -self.orbit_radius * 0.5 * math.sin(rad),  # tangent velocity
            self.orbit_radius * 0.5 * math.cos(rad),
            0
        )

        await drone.set_velocity(velocity, heading=angle + 90)
        return "land"

    @state("land")
    async def land_state(self, drone: Drone):
        """Land after orbit complete."""
        await drone.land()
        return None


async def main():
    """Run the survey mission example."""
    drone = Drone("udp://:14540")
    await drone.connect()

    # Run the survey mission
    mission = SurveyMission()
    await mission.run(drone)

    await drone.disconnect()


if __name__ == "__main__":
    asyncio.run(main())

