"""
SITL Integration tests for drone operations.

These tests require ArduPilot SITL to be installed and run actual flight
operations in simulation.

Run with:
    pytest tests/integration/ -v --tb=short

Skip integration tests:
    pytest tests/unit/ -v

Environment variables:
    SITL_VERBOSE=1  - Show SITL output
    SITL_SPEEDUP=10 - Simulation speedup (default: 5)
"""

import pytest
import asyncio
import time
import os

# Skip all tests in this module if SITL is not available
pytestmark = [
    pytest.mark.integration,
    pytest.mark.slow,
]


class TestDroneConnection:
    """Tests for drone connection."""

    @pytest.mark.asyncio
    async def test_drone_connects(self, connected_drone):
        """Test drone connects successfully."""
        assert connected_drone.connected is True

    @pytest.mark.asyncio
    async def test_drone_has_gps_fix(self, connected_drone):
        """Test drone has GPS fix after connection."""
        assert connected_drone.gps.fix_type >= 3

    @pytest.mark.asyncio
    async def test_drone_position_valid(self, connected_drone):
        """Test drone reports valid position."""
        pos = connected_drone.position
        assert pos.lat != 0 or pos.lon != 0
        assert -90 <= pos.lat <= 90
        assert -180 <= pos.lon <= 180

    @pytest.mark.asyncio
    async def test_drone_heading_valid(self, connected_drone):
        """Test drone reports valid heading."""
        heading = connected_drone.heading
        assert 0 <= heading < 360

    @pytest.mark.asyncio
    async def test_drone_battery_valid(self, connected_drone):
        """Test drone reports battery status."""
        battery = connected_drone.battery
        assert battery.voltage > 0
        assert 0 <= battery.level <= 100


class TestDroneArming:
    """Tests for drone arming."""

    @pytest.mark.asyncio
    async def test_drone_arms_before_takeoff(self, connected_drone):
        """Test drone arms automatically when takeoff is called."""
        assert connected_drone.armed is False
        # Takeoff will trigger _initialize_postarm which arms
        await connected_drone.takeoff(5)
        assert connected_drone.armed is True
        await connected_drone.land()


class TestDroneTakeoff:
    """Tests for drone takeoff."""

    @pytest.mark.asyncio
    async def test_takeoff_reaches_altitude(self, connected_drone):
        """Test drone takes off to target altitude."""
        target_alt = 10  # meters

        # Initialize and takeoff
        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(target_alt)

        # Check altitude (with tolerance)
        assert connected_drone.position.alt >= target_alt * 0.9

        # Land
        await connected_drone.land()

    @pytest.mark.asyncio
    async def test_takeoff_different_altitudes(self, connected_drone):
        """Test drone can takeoff to different altitudes."""
        for target_alt in [5, 15]:
            connected_drone._initialize_prearm(should_postarm_init=True)
            await connected_drone.takeoff(target_alt)

            assert connected_drone.position.alt >= target_alt * 0.85

            await connected_drone.land()

            # Wait for disarm before next iteration
            timeout = 30
            start = time.time()
            while connected_drone.armed and (time.time() - start) < timeout:
                await asyncio.sleep(0.5)


class TestDroneNavigation:
    """Tests for drone navigation."""

    @pytest.mark.asyncio
    async def test_goto_coordinates(self, connected_drone):
        """Test drone navigates to coordinates."""
        from aerpawlib.v1.util import VectorNED

        # Setup
        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(10)

        # Get starting position and calculate target
        start_pos = connected_drone.position
        target = start_pos + VectorNED(50, 0, 0)  # 50m north

        # Navigate
        await connected_drone.goto_coordinates(target, tolerance=3)

        # Verify we're near target
        distance = connected_drone.position.ground_distance(target)
        assert distance < 5  # Within 5 meters

        # Cleanup
        await connected_drone.land()

    @pytest.mark.asyncio
    async def test_goto_multiple_waypoints(self, connected_drone):
        """Test drone navigates through multiple waypoints."""
        from aerpawlib.v1.util import VectorNED

        # Setup
        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(10)

        start_pos = connected_drone.position

        # Define waypoints (small square)
        waypoints = [
            start_pos + VectorNED(20, 0, 0),  # North
            start_pos + VectorNED(20, 20, 0),  # Northeast
            start_pos + VectorNED(0, 20, 0),  # East
            start_pos,  # Back to start
        ]

        for wp in waypoints:
            await connected_drone.goto_coordinates(wp, tolerance=3)
            distance = connected_drone.position.ground_distance(wp)
            assert distance < 5

        # Cleanup
        await connected_drone.land()


class TestDroneLanding:
    """Tests for drone landing."""

    @pytest.mark.asyncio
    async def test_land_disarms(self, connected_drone):
        """Test landing disarms the drone."""
        # Setup and takeoff
        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(10)

        assert connected_drone.armed is True

        # Land
        await connected_drone.land()

        # Should be disarmed
        assert connected_drone.armed is False

    @pytest.mark.asyncio
    async def test_land_reaches_ground(self, connected_drone):
        """Test drone reaches ground level after landing."""
        # Setup and takeoff
        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(10)

        # Land
        await connected_drone.land()

        # Altitude should be near zero
        assert connected_drone.position.alt < 1


class TestDroneRTL:
    """Tests for Return To Launch."""

    @pytest.mark.asyncio
    async def test_rtl_returns_home(self, connected_drone):
        """Test RTL returns drone to home location."""
        from aerpawlib.v1.util import VectorNED

        # Setup and takeoff
        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(10)

        # Record home position
        home = connected_drone.home_coords

        # Fly away
        target = connected_drone.position + VectorNED(50, 50, 0)
        await connected_drone.goto_coordinates(target, tolerance=3)

        # Verify we moved
        distance_from_home = connected_drone.position.ground_distance(home)
        assert distance_from_home > 30

        # RTL
        await connected_drone.return_to_launch()

        # Should be disarmed (RTL lands automatically)
        assert connected_drone.armed is False


class TestDroneHeading:
    """Tests for heading control."""

    @pytest.mark.asyncio
    async def test_set_heading(self, connected_drone):
        """Test drone can change heading."""
        # Setup and takeoff
        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(10)

        # Set heading to north
        await connected_drone.set_heading(0)

        # Allow some tolerance for heading
        heading = connected_drone.heading
        assert heading < 10 or heading > 350

        # Set heading to east
        await connected_drone.set_heading(90)

        heading = connected_drone.heading
        assert 80 < heading < 100

        # Cleanup
        await connected_drone.land()


class TestDroneVelocity:
    """Tests for velocity control."""

    @pytest.mark.asyncio
    async def test_set_velocity_moves_drone(self, connected_drone):
        """Test setting velocity moves the drone."""
        from aerpawlib.v1.util import VectorNED

        # Setup and takeoff
        connected_drone._initialize_prearm(should_postarm_init=True)
        await connected_drone.takeoff(10)

        start_pos = connected_drone.position

        # Move north at 5 m/s for 2 seconds
        await connected_drone.set_velocity(VectorNED(5, 0, 0), duration=2)

        # Wait for velocity command to complete
        await asyncio.sleep(3)

        # Should have moved approximately 10m north
        distance = start_pos.ground_distance(connected_drone.position)
        assert distance > 5  # At least some movement

        # Cleanup
        await connected_drone.land()
