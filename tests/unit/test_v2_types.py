"""Unit tests for aerpawlib v2 Coordinate and VectorNED."""

import math
import pytest

from aerpawlib.v2.types import Coordinate, VectorNED


class TestVectorNED:
    """VectorNED operations."""

    def test_hypot(self):
        v = VectorNED(3, 4, 0)
        assert v.hypot() == 5.0

    def test_hypot_ignore_down(self):
        v = VectorNED(3, 4, 10)
        assert v.hypot(ignore_down=True) == 5.0

    def test_norm(self):
        v = VectorNED(3, 4, 0)
        n = v.norm()
        assert abs(n.hypot() - 1.0) < 1e-9
        assert abs(n.north - 0.6) < 1e-9
        assert abs(n.east - 0.8) < 1e-9

    def test_norm_zero(self):
        v = VectorNED(0, 0, 0)
        n = v.norm()
        assert n.north == 0 and n.east == 0 and n.down == 0

    def test_rotate_by_angle(self):
        v = VectorNED(1, 0, 0)
        r = v.rotate_by_angle(-90)
        assert abs(r.north - 0) < 1e-6
        assert abs(r.east - 1) < 1e-6

    def test_add(self):
        a = VectorNED(1, 2, 3)
        b = VectorNED(4, 5, 6)
        c = a + b
        assert c.north == 5 and c.east == 7 and c.down == 9

    def test_sub(self):
        a = VectorNED(5, 5, 5)
        b = VectorNED(2, 3, 4)
        c = a - b
        assert c.north == 3 and c.east == 2 and c.down == 1

    def test_mul_scalar(self):
        v = VectorNED(1, 2, 3)
        w = v * 2
        assert w.north == 2 and w.east == 4 and w.down == 6

    def test_rmul_scalar(self):
        v = VectorNED(1, 2, 3)
        w = 2 * v
        assert w.north == 2 and w.east == 4 and w.down == 6


class TestCoordinate:
    """Coordinate operations."""

    def test_ground_distance(self):
        a = Coordinate(35.727, -78.696, 0)
        b = Coordinate(35.728, -78.696, 0)
        d = a.ground_distance(b)
        assert 100 < d < 150

    def test_distance_3d(self):
        a = Coordinate(35.727, -78.696, 0)
        b = Coordinate(35.727, -78.696, 100)
        d = a.distance(b)
        assert abs(d - 100) < 1

    def test_add_vector(self):
        c = Coordinate(35.727, -78.696, 0)
        v = VectorNED(100, 0, 0)
        d = c + v
        assert d.lat > c.lat
        assert abs(d.lon - c.lon) < 0.001

    def test_bearing(self):
        a = Coordinate(35.727, -78.696, 0)
        b = Coordinate(35.728, -78.696, 0)
        bearing = a.bearing(b)
        assert 0 <= bearing < 360

    def test_ground_distance_type_error(self):
        a = Coordinate(35.727, -78.696, 0)
        with pytest.raises(TypeError):
            a.ground_distance((35.728, -78.696, 0))
