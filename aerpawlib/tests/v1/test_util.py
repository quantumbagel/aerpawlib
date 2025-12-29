"""
Tests for the v1 aerpawlib utility classes (VectorNED, Coordinate).

These tests verify the v1 API maintains backward compatibility with legacy
while using MAVSDK under the hood.
"""
import os
import sys

import pytest

# Add the parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))

from aerpawlib.v1.util import VectorNED, Coordinate


class TestVectorNED:
    """Tests for VectorNED class operations in v1 API."""

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

    def test_hypot_3d(self):
        """Test 3D magnitude calculation."""
        v = VectorNED(3, 4, 0)
        assert v.hypot() == 5.0

    def test_hypot_2d(self):
        """Test 2D magnitude calculation (ignoring down)."""
        v = VectorNED(3, 4, 100)
        assert v.hypot(ignore_down=True) == 5.0

    def test_norm(self):
        """Test vector normalization."""
        v = VectorNED(3, 4, 0)
        normalized = v.norm()
        assert abs(normalized.hypot() - 1.0) < 0.0001

    def test_norm_zero_vector(self):
        """Test normalization of zero vector."""
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

    def test_cross_product(self):
        """Test cross product calculation."""
        v1 = VectorNED(1, 0, 0)
        v2 = VectorNED(0, 1, 0)
        cross = v1.cross_product(v2)
        assert isinstance(cross, VectorNED)


class TestCoordinate:
    """Tests for Coordinate class operations in v1 API."""

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
        assert result.lat > c.lat

    def test_subtract_coordinate(self):
        """Test subtracting Coordinate from Coordinate."""
        c1 = Coordinate(35.7275, -78.6960, 10)
        c2 = Coordinate(35.7275, -78.6960, 20)
        result = c1 - c2
        assert isinstance(result, VectorNED)

    def test_ground_distance(self):
        """Test ground distance calculation."""
        c1 = Coordinate(35.7275, -78.6960, 0)
        c2 = Coordinate(35.7275, -78.6960, 0)
        assert c1.ground_distance(c2) < 0.01

    def test_distance(self):
        """Test 3D distance calculation."""
        c1 = Coordinate(35.7275, -78.6960, 0)
        c2 = Coordinate(35.7275, -78.6960, 100)
        distance = c1.distance(c2)
        assert abs(distance - 100) < 1


class TestV1ApiCompatibility:
    """Tests to verify v1 API is compatible with legacy."""

    def test_vector_operations_same_as_legacy(self):
        """Test that v1 VectorNED operations match expected behavior."""
        v1 = VectorNED(3, 4, 0)
        v2 = VectorNED(1, 2, 0)

        # Test operations
        add_result = v1 + v2
        sub_result = v1 - v2
        mul_result = v1 * 2

        assert add_result.north == 4
        assert add_result.east == 6
        assert sub_result.north == 2
        assert sub_result.east == 2
        assert mul_result.north == 6
        assert mul_result.east == 8

    def test_coordinate_operations_same_as_legacy(self):
        """Test that v1 Coordinate operations match expected behavior."""
        c = Coordinate(35.7275, -78.6960, 50)
        v = VectorNED(100, 50, -10)

        # Add vector to coordinate
        c_new = c + v
        assert c_new.lat > c.lat  # Moved north

        # Get vector between coordinates
        v_diff = c - c_new
        assert isinstance(v_diff, VectorNED)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

