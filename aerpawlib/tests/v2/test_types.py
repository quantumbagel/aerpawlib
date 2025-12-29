"""
Tests for the v2 aerpawlib types module.

These tests verify the core types: Coordinate, VectorNED, and state containers.
"""
import json
import os
import sys

import pytest

# Add the parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))

from aerpawlib.v2.types import (
    VectorNED,
    PositionNED,
    Coordinate,
    Attitude,
    FlightMode,
    LandedState,
    Waypoint,
)


class TestVectorNED:
    """Tests for VectorNED dataclass in v2 API."""

    def test_init_default(self):
        """Test VectorNED initialization with defaults."""
        v = VectorNED()
        assert v.north == 0.0
        assert v.east == 0.0
        assert v.down == 0.0

    def test_init_with_values(self):
        """Test VectorNED initialization with explicit values."""
        v = VectorNED(1.0, 2.0, 3.0)
        assert v.north == 1.0
        assert v.east == 2.0
        assert v.down == 3.0

    def test_addition(self):
        """Test vector addition."""
        v1 = VectorNED(1, 2, 3)
        v2 = VectorNED(4, 5, 6)
        result = v1 + v2
        assert result.north == 5
        assert result.east == 7
        assert result.down == 9

    def test_addition_type_error(self):
        """Test that adding non-VectorNED raises TypeError."""
        v = VectorNED(1, 2, 3)
        with pytest.raises(TypeError):
            v + 5

    def test_subtraction(self):
        """Test vector subtraction."""
        v1 = VectorNED(4, 5, 6)
        v2 = VectorNED(1, 2, 3)
        result = v1 - v2
        assert result.north == 3
        assert result.east == 3
        assert result.down == 3

    def test_scalar_multiplication(self):
        """Test scalar multiplication."""
        v = VectorNED(1, 2, 3)
        result = v * 2
        assert result.north == 2
        assert result.east == 4
        assert result.down == 6

    def test_scalar_multiplication_reverse(self):
        """Test reverse scalar multiplication."""
        v = VectorNED(1, 2, 3)
        result = 3 * v
        assert result.north == 3
        assert result.east == 6
        assert result.down == 9

    def test_negation(self):
        """Test vector negation."""
        v = VectorNED(1, 2, 3)
        result = -v
        assert result.north == -1
        assert result.east == -2
        assert result.down == -3

    def test_magnitude_3d(self):
        """Test 3D magnitude calculation."""
        v = VectorNED(3, 4, 0)
        assert v.magnitude() == 5.0

    def test_magnitude_2d(self):
        """Test 2D magnitude calculation (ignoring vertical)."""
        v = VectorNED(3, 4, 100)
        assert v.magnitude(ignore_vertical=True) == 5.0

    def test_hypot_alias(self):
        """Test that hypot is an alias for magnitude."""
        v = VectorNED(3, 4, 0)
        assert v.hypot() == v.magnitude()

    def test_normalize(self):
        """Test vector normalization."""
        v = VectorNED(3, 4, 0)
        normalized = v.normalize()
        assert abs(normalized.north - 0.6) < 0.0001
        assert abs(normalized.east - 0.8) < 0.0001
        assert abs(normalized.magnitude() - 1.0) < 0.0001

    def test_norm_alias(self):
        """Test that norm is an alias for normalize."""
        v = VectorNED(3, 4, 0)
        assert v.norm() == v.normalize()

    def test_normalize_zero_vector(self):
        """Test normalization of zero vector."""
        v = VectorNED(0, 0, 0)
        normalized = v.normalize()
        assert normalized.north == 0
        assert normalized.east == 0
        assert normalized.down == 0

    def test_rotate_by_angle(self):
        """Test rotation by angle."""
        v = VectorNED(1, 0, 0)
        rotated = v.rotate_by_angle(90)
        assert abs(rotated.north - 0.0) < 0.0001
        assert abs(rotated.east - (-1.0)) < 0.0001

    def test_heading(self):
        """Test heading calculation."""
        # North vector should have heading 0
        v_north = VectorNED(1, 0, 0)
        assert abs(v_north.heading() - 0) < 0.1 or abs(v_north.heading() - 360) < 0.1

        # East vector should have heading 90
        v_east = VectorNED(0, 1, 0)
        assert abs(v_east.heading() - 90) < 0.1

    def test_cross_product(self):
        """Test cross product calculation."""
        v1 = VectorNED(1, 0, 0)
        v2 = VectorNED(0, 1, 0)
        cross = v1.cross_product(v2)
        assert isinstance(cross, VectorNED)

    def test_dot_product(self):
        """Test dot product calculation."""
        v1 = VectorNED(1, 2, 3)
        v2 = VectorNED(4, 5, 6)
        dot = v1.dot_product(v2)
        assert dot == 1*4 + 2*5 + 3*6  # = 32

    def test_repr(self):
        """Test string representation."""
        v = VectorNED(1.5, 2.5, 3.5)
        r = repr(v)
        assert "1.5" in r
        assert "2.5" in r
        assert "3.5" in r


class TestPositionNED:
    """Tests for PositionNED dataclass."""

    def test_init_default(self):
        """Test PositionNED initialization with defaults."""
        p = PositionNED()
        assert p.north == 0.0
        assert p.east == 0.0
        assert p.down == 0.0

    def test_to_vector(self):
        """Test conversion to VectorNED."""
        p = PositionNED(1, 2, 3)
        v = p.to_vector()
        assert isinstance(v, VectorNED)
        assert v.north == 1
        assert v.east == 2
        assert v.down == 3

    def test_from_vector(self):
        """Test creation from VectorNED."""
        v = VectorNED(1, 2, 3)
        p = PositionNED.from_vector(v)
        assert p.north == 1
        assert p.east == 2
        assert p.down == 3


class TestCoordinate:
    """Tests for Coordinate dataclass in v2 API."""

    def test_init_basic(self):
        """Test basic Coordinate initialization."""
        c = Coordinate(35.0, -78.0)
        assert c.latitude == 35.0
        assert c.longitude == -78.0
        assert c.altitude == 0.0
        assert c.name is None

    def test_init_with_all_params(self):
        """Test Coordinate initialization with all parameters."""
        c = Coordinate(35.0, -78.0, 100.0, "Home")
        assert c.latitude == 35.0
        assert c.longitude == -78.0
        assert c.altitude == 100.0
        assert c.name == "Home"

    def test_lat_lon_alt_aliases(self):
        """Test lat/lon/alt property aliases."""
        c = Coordinate(35.0, -78.0, 100.0)
        assert c.lat == c.latitude
        assert c.lon == c.longitude
        assert c.alt == c.altitude

    def test_lat_lon_alt_setters(self):
        """Test lat/lon/alt property setters."""
        c = Coordinate(35.0, -78.0, 100.0)
        c.lat = 36.0
        c.lon = -79.0
        c.alt = 150.0
        assert c.latitude == 36.0
        assert c.longitude == -79.0
        assert c.altitude == 150.0

    def test_distance_to_same_point(self):
        """Test distance to same point is zero."""
        c = Coordinate(35.0, -78.0, 100)
        assert c.distance_to(c) < 0.01

    def test_distance_to_different_altitude(self):
        """Test distance with altitude difference."""
        c1 = Coordinate(35.0, -78.0, 0)
        c2 = Coordinate(35.0, -78.0, 100)
        distance = c1.distance_to(c2)
        assert abs(distance - 100) < 1

    def test_ground_distance_to(self):
        """Test ground distance ignores altitude."""
        c1 = Coordinate(35.0, -78.0, 0)
        c2 = Coordinate(35.0, -78.0, 1000)
        ground_distance = c1.ground_distance_to(c2)
        assert ground_distance < 0.01

    def test_bearing_to_east(self):
        """Test bearing to eastern point."""
        c1 = Coordinate(35.0, -78.0)
        c2 = Coordinate(35.0, -77.0)  # East
        bearing = c1.bearing_to(c2)
        assert abs(bearing - 90) < 5

    def test_bearing_to_north(self):
        """Test bearing to northern point."""
        c1 = Coordinate(35.0, -78.0)
        c2 = Coordinate(36.0, -78.0)  # North
        bearing = c1.bearing_to(c2)
        assert bearing < 5 or bearing > 355

    def test_offset_by(self):
        """Test offset by vector."""
        c = Coordinate(35.0, -78.0, 100)
        v = VectorNED(100, 0, 0)  # 100m north
        c_new = c.offset_by(v)
        assert c_new.latitude > c.latitude

    def test_vector_to(self):
        """Test vector between coordinates."""
        c1 = Coordinate(35.0, -78.0, 100)
        c2 = Coordinate(35.0, -78.0, 150)
        v = c1.vector_to(c2)
        assert isinstance(v, VectorNED)
        # Down component should reflect altitude difference
        assert abs(v.down - (-50)) < 1

    def test_add_vector(self):
        """Test adding vector to coordinate."""
        c = Coordinate(35.0, -78.0, 100)
        v = VectorNED(100, 50, -10)
        result = c + v
        assert isinstance(result, Coordinate)
        assert result.latitude > c.latitude
        assert result.altitude > c.altitude

    def test_subtract_vector(self):
        """Test subtracting vector from coordinate."""
        c = Coordinate(35.0, -78.0, 100)
        v = VectorNED(100, 0, 0)
        result = c - v
        assert isinstance(result, Coordinate)
        assert result.latitude < c.latitude

    def test_subtract_coordinate(self):
        """Test subtracting coordinate from coordinate."""
        c1 = Coordinate(35.0, -78.0, 100)
        c2 = Coordinate(35.0, -78.0, 150)
        result = c1 - c2
        assert isinstance(result, VectorNED)

    def test_to_json(self):
        """Test JSON serialization."""
        c = Coordinate(35.0, -78.0, 100.0, "Home")
        json_str = c.to_json()
        data = json.loads(json_str)
        assert data["latitude"] == 35.0
        assert data["longitude"] == -78.0
        assert data["altitude"] == 100.0
        assert data["name"] == "Home"

    def test_from_json(self):
        """Test JSON deserialization."""
        json_str = '{"latitude": 35.0, "longitude": -78.0, "altitude": 100.0, "name": "Home"}'
        c = Coordinate.from_json(json_str)
        assert c.latitude == 35.0
        assert c.longitude == -78.0
        assert c.altitude == 100.0
        assert c.name == "Home"

    def test_from_relative(self):
        """Test creating coordinate relative to base."""
        base = Coordinate(35.0, -78.0, 100)
        v = VectorNED(100, 50, -10)
        c = Coordinate.from_relative(base, v, "WP1")
        assert c.name == "WP1"
        assert c.latitude > base.latitude

    def test_midpoint_to(self):
        """Test midpoint calculation."""
        c1 = Coordinate(35.0, -78.0, 100)
        c2 = Coordinate(36.0, -78.0, 200)
        mid = c1.midpoint_to(c2)
        assert abs(mid.latitude - 35.5) < 0.01
        assert abs(mid.altitude - 150) < 1

    def test_interpolate_to(self):
        """Test interpolation between coordinates."""
        c1 = Coordinate(35.0, -78.0, 100)
        c2 = Coordinate(36.0, -78.0, 200)
        quarter = c1.interpolate_to(c2, 0.25)
        assert abs(quarter.latitude - 35.25) < 0.01


class TestFlightMode:
    """Tests for FlightMode enum."""

    def test_flight_modes_exist(self):
        """Test that expected flight modes exist."""
        assert FlightMode.MANUAL
        assert FlightMode.STABILIZED
        assert FlightMode.ALTITUDE
        assert FlightMode.POSITION
        assert FlightMode.OFFBOARD
        assert FlightMode.HOLD
        assert FlightMode.MISSION
        assert FlightMode.RETURN_TO_LAUNCH
        assert FlightMode.LAND
        assert FlightMode.TAKEOFF
        assert FlightMode.UNKNOWN


class TestLandedState:
    """Tests for LandedState enum."""

    def test_landed_states_exist(self):
        """Test that expected landed states exist."""
        assert LandedState.UNKNOWN
        assert LandedState.ON_GROUND
        assert LandedState.IN_AIR
        assert LandedState.TAKING_OFF
        assert LandedState.LANDING


class TestAttitude:
    """Tests for Attitude dataclass."""

    def test_attitude_defaults(self):
        """Test Attitude default values."""
        a = Attitude()
        assert a.pitch == 0.0
        assert a.roll == 0.0
        assert a.yaw == 0.0

    def test_attitude_with_values(self):
        """Test Attitude with explicit values (radians)."""
        a = Attitude(pitch=0.1745, roll=0.0873, yaw=0.7854)  # ~10, 5, 45 degrees
        assert abs(a.pitch - 0.1745) < 0.0001
        assert abs(a.roll - 0.0873) < 0.0001
        assert abs(a.yaw - 0.7854) < 0.0001

    def test_attitude_degree_properties(self):
        """Test Attitude degree conversion properties."""
        import math
        a = Attitude(pitch=math.radians(10), roll=math.radians(5), yaw=math.radians(45))
        assert abs(a.pitch_degrees - 10.0) < 0.1
        assert abs(a.roll_degrees - 5.0) < 0.1
        assert abs(a.yaw_degrees - 45.0) < 0.1

    def test_attitude_heading(self):
        """Test Attitude heading property."""
        import math
        a = Attitude(yaw=math.radians(90))
        assert abs(a.heading - 90) < 0.1


class TestWaypoint:
    """Tests for Waypoint dataclass."""

    def test_waypoint_basic(self):
        """Test basic Waypoint creation."""
        coord = Coordinate(35.0, -78.0, 100)
        wp = Waypoint(coordinate=coord)
        assert wp.coordinate == coord
        assert wp.speed is None

    def test_waypoint_with_speed(self):
        """Test Waypoint with speed parameter."""
        coord = Coordinate(35.0, -78.0, 100)
        wp = Waypoint(coordinate=coord, speed=10.0)
        assert wp.speed == 10.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

