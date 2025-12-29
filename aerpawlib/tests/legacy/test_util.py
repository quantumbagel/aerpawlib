"""
Tests for the legacy aerpawlib utility classes (VectorNED, Coordinate).

These tests verify the core math and spatial operations used across the library.
"""
import os
import sys

import pytest

# Add the parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))

from aerpawlib.legacy.util import VectorNED, Coordinate


class TestVectorNED:
    """Tests for VectorNED class operations."""

    def test_init_default(self):
        """Test VectorNED initialization with default down value."""
        v = VectorNED(1.0, 2.0)
        assert v.north == 1.0
        assert v.east == 2.0
        assert v.down == 0.0

    def test_init_with_down(self):
        """Test VectorNED initialization with explicit down value."""
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
        with pytest.raises(TypeError):
            v + "string"

    def test_subtraction(self):
        """Test vector subtraction."""
        v1 = VectorNED(4, 5, 6)
        v2 = VectorNED(1, 2, 3)
        result = v1 - v2
        assert result.north == 3
        assert result.east == 3
        assert result.down == 3

    def test_subtraction_type_error(self):
        """Test that subtracting non-VectorNED raises TypeError."""
        v = VectorNED(1, 2, 3)
        with pytest.raises(TypeError):
            v - 5

    def test_scalar_multiplication(self):
        """Test scalar multiplication."""
        v = VectorNED(1, 2, 3)
        result = v * 2
        assert result.north == 2
        assert result.east == 4
        assert result.down == 6

    def test_scalar_multiplication_reverse(self):
        """Test reverse scalar multiplication (scalar * vector)."""
        v = VectorNED(1, 2, 3)
        result = 3 * v
        assert result.north == 3
        assert result.east == 6
        assert result.down == 9

    def test_scalar_multiplication_type_error(self):
        """Test that multiplying by non-numeric raises TypeError."""
        v = VectorNED(1, 2, 3)
        with pytest.raises(TypeError):
            v * "string"

    def test_hypot_3d(self):
        """Test 3D magnitude calculation."""
        v = VectorNED(3, 4, 0)
        assert v.hypot() == 5.0

        v2 = VectorNED(1, 2, 2)
        assert v2.hypot() == 3.0

    def test_hypot_2d(self):
        """Test 2D magnitude calculation (ignoring down)."""
        v = VectorNED(3, 4, 100)
        assert v.hypot(ignore_down=True) == 5.0

    def test_norm(self):
        """Test vector normalization."""
        v = VectorNED(3, 4, 0)
        normalized = v.norm()
        assert abs(normalized.north - 0.6) < 0.0001
        assert abs(normalized.east - 0.8) < 0.0001
        assert normalized.down == 0.0
        assert abs(normalized.hypot() - 1.0) < 0.0001

    def test_norm_zero_vector(self):
        """Test normalization of zero vector returns zero vector."""
        v = VectorNED(0, 0, 0)
        normalized = v.norm()
        assert normalized.north == 0
        assert normalized.east == 0
        assert normalized.down == 0

    def test_rotate_by_angle_90(self):
        """Test rotation by 90 degrees."""
        v = VectorNED(1, 0, 0)
        rotated = v.rotate_by_angle(90)
        assert abs(rotated.north - 0.0) < 0.0001
        assert abs(rotated.east - (-1.0)) < 0.0001

    def test_rotate_by_angle_180(self):
        """Test rotation by 180 degrees."""
        v = VectorNED(1, 0, 0)
        rotated = v.rotate_by_angle(180)
        assert abs(rotated.north - (-1.0)) < 0.0001
        assert abs(rotated.east - 0.0) < 0.0001

    def test_rotate_by_angle_preserves_down(self):
        """Test that rotation preserves the down component."""
        v = VectorNED(1, 0, 5)
        rotated = v.rotate_by_angle(90)
        assert rotated.down == 5

    def test_cross_product(self):
        """Test cross product calculation."""
        v1 = VectorNED(1, 0, 0)
        v2 = VectorNED(0, 1, 0)
        cross = v1.cross_product(v2)
        # Cross product should be in the down direction for north x east
        assert isinstance(cross, VectorNED)

    def test_cross_product_type_error(self):
        """Test that cross product with non-VectorNED raises TypeError."""
        v = VectorNED(1, 0, 0)
        with pytest.raises(TypeError):
            v.cross_product([0, 1, 0])

    def test_str_representation(self):
        """Test string representation."""
        v = VectorNED(1.5, 2.5, 3.5)
        s = str(v)
        assert "1.5" in s
        assert "2.5" in s
        assert "3.5" in s


class TestCoordinate:
    """Tests for Coordinate class operations."""

    def test_init_default(self):
        """Test Coordinate initialization with default altitude."""
        c = Coordinate(35.0, -78.0)
        assert c.lat == 35.0
        assert c.lon == -78.0
        assert c.alt == 0.0

    def test_init_with_altitude(self):
        """Test Coordinate initialization with explicit altitude."""
        c = Coordinate(35.0, -78.0, 100.0)
        assert c.lat == 35.0
        assert c.lon == -78.0
        assert c.alt == 100.0

    def test_add_vector(self):
        """Test adding VectorNED to Coordinate."""
        c = Coordinate(35.7275, -78.6960, 10)
        v = VectorNED(100, 0, 0)  # 100m north
        result = c + v
        assert isinstance(result, Coordinate)
        # The resulting latitude should be greater (moved north)
        assert result.lat > c.lat
        # Longitude should be roughly the same
        assert abs(result.lon - c.lon) < 0.001

    def test_add_type_error(self):
        """Test that adding non-VectorNED to Coordinate raises TypeError."""
        c = Coordinate(35.0, -78.0)
        with pytest.raises(TypeError):
            c + 5
        with pytest.raises(TypeError):
            c + [100, 0, 0]

    def test_subtract_coordinate(self):
        """Test subtracting Coordinate from Coordinate gives VectorNED."""
        c1 = Coordinate(35.7275, -78.6960, 10)
        c2 = Coordinate(35.7275, -78.6960, 20)
        result = c1 - c2
        assert isinstance(result, VectorNED)
        # Same lat/lon, so north and east should be near zero
        assert abs(result.north) < 1
        assert abs(result.east) < 1
        # Altitude difference should be reflected in down
        assert abs(result.down - 10) < 0.1

    def test_subtract_vector(self):
        """Test subtracting VectorNED from Coordinate."""
        c = Coordinate(35.7275, -78.6960, 10)
        v = VectorNED(100, 0, 0)  # 100m north
        result = c - v
        assert isinstance(result, Coordinate)
        # The resulting latitude should be less (moved south)
        assert result.lat < c.lat

    def test_ground_distance(self):
        """Test ground distance calculation between coordinates."""
        c1 = Coordinate(35.7275, -78.6960, 0)
        c2 = Coordinate(35.7275, -78.6960, 0)
        # Same point should have zero distance
        assert c1.ground_distance(c2) < 0.01

        # Points 1 degree apart in latitude (about 111km)
        c3 = Coordinate(36.7275, -78.6960, 0)
        distance = c1.ground_distance(c3)
        # Should be approximately 111km (111000m)
        assert 110000 < distance < 112000

    def test_ground_distance_type_error(self):
        """Test that ground_distance with non-Coordinate raises TypeError."""
        c = Coordinate(35.0, -78.0)
        with pytest.raises(TypeError):
            c.ground_distance([35.5, -78.5])

    def test_distance(self):
        """Test 3D distance calculation between coordinates."""
        c1 = Coordinate(35.7275, -78.6960, 0)
        c2 = Coordinate(35.7275, -78.6960, 100)
        # Same lat/lon, 100m altitude difference
        distance = c1.distance(c2)
        assert abs(distance - 100) < 1

    def test_distance_type_error(self):
        """Test that distance with non-Coordinate raises TypeError."""
        c = Coordinate(35.0, -78.0)
        with pytest.raises(TypeError):
            c.distance([35.5, -78.5])


class TestCoordinateVectorInteraction:
    """Tests for interactions between Coordinate and VectorNED."""

    def test_round_trip_offset(self):
        """Test that offset and difference are inverses."""
        c1 = Coordinate(35.7275, -78.6960, 100)
        v = VectorNED(50, 75, -10)
        c2 = c1 + v
        v_back = c1 - c2
        # Should be approximately the negative of original vector
        assert abs(v_back.north + v.north) < 1
        assert abs(v_back.east + v.east) < 1
        assert abs(v_back.down + v.down) < 1

    def test_vector_magnitude_matches_distance(self):
        """Test that vector magnitude matches coordinate distance."""
        c1 = Coordinate(35.7275, -78.6960, 100)
        c2 = Coordinate(35.7280, -78.6965, 150)
        v = c1 - c2
        distance = c1.distance(c2)
        vector_mag = v.hypot()
        # Should be approximately equal (small errors due to Earth curvature)
        assert abs(distance - vector_mag) < distance * 0.01  # Within 1%


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

