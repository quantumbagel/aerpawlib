"""Unit tests for aerpawlib v1 util module (Coordinate, VectorNED, plan, geofence)."""

import json
import math
import os
import tempfile

import pytest

from aerpawlib.v1.constants import (
    PLAN_CMD_RTL,
    PLAN_CMD_TAKEOFF,
    PLAN_CMD_WAYPOINT,
    PLAN_CMD_SPEED,
    DEFAULT_WAYPOINT_SPEED,
)
from aerpawlib.v1.util import (
    Coordinate,
    VectorNED,
    do_intersect,
    doIntersect,
    get_location_from_waypoint,
    inside,
    lies_on_segment,
    liesOnSegment,
    orientation,
    read_from_plan,
    read_from_plan_complete,
    read_geofence,
)


class TestVectorNED:
    """VectorNED creation and operations."""

    def test_create(self):
        v = VectorNED(1.0, 2.0, 3.0)
        assert v.north == 1.0 and v.east == 2.0 and v.down == 3.0

    def test_default_down(self):
        v = VectorNED(1.0, 2.0)
        assert v.down == 0

    def test_add(self):
        v1, v2 = VectorNED(1, 2, 3), VectorNED(4, 5, 6)
        r = v1 + v2
        assert r.north == 5 and r.east == 7 and r.down == 9

    def test_subtract(self):
        v1, v2 = VectorNED(5, 7, 9), VectorNED(1, 2, 3)
        r = v1 - v2
        assert r.north == 4 and r.east == 5 and r.down == 6

    def test_multiply_scalar(self):
        v = VectorNED(2, 3, 4)
        assert (v * 2).north == 4 and (v * 2).east == 6

    def test_rmul(self):
        v = VectorNED(2, 3, 4)
        assert (3 * v).north == 6

    def test_hypot(self):
        assert VectorNED(3, 4, 0).hypot() == 5.0
        assert VectorNED(0, 0, 0).hypot() == 0

    def test_hypot_ignore_down(self):
        assert VectorNED(3, 4, 100).hypot(ignore_down=True) == 5.0

    def test_norm(self):
        v = VectorNED(3, 4, 0)
        n = v.norm()
        assert abs(n.hypot() - 1.0) < 1e-10

    def test_norm_zero(self):
        assert VectorNED(0, 0, 0).norm().hypot() == 0

    def test_rotate_by_angle(self):
        v = VectorNED(1, 0, 0)
        r = v.rotate_by_angle(90)
        assert abs(r.north) < 1e-10 and abs(r.east - (-1)) < 1e-10

    def test_cross_product(self):
        n, e = VectorNED(1, 0, 0), VectorNED(0, 1, 0)
        c = n.cross_product(e)
        assert abs(c.down - 1) < 1e-10

    def test_add_invalid_raises(self):
        with pytest.raises(TypeError):
            VectorNED(1, 2, 3) + 5

    def test_subtract_invalid_raises(self):
        with pytest.raises(TypeError):
            VectorNED(1, 2, 3) - 5


class TestCoordinate:
    """Coordinate creation and operations."""

    def test_create(self):
        c = Coordinate(35.7274, -78.6960, 100.0)
        assert c.lat == 35.7274 and c.lon == -78.6960 and c.alt == 100.0

    def test_default_alt(self):
        c = Coordinate(35.7274, -78.6960)
        assert c.alt == 0

    def test_distance_same(self):
        c = Coordinate(35.7274, -78.6960, 100)
        assert c.distance(c) == 0

    def test_ground_distance_ignores_alt(self):
        c1 = Coordinate(35.7274, -78.6960, 0)
        c2 = Coordinate(35.7274, -78.6960, 100)
        assert c1.ground_distance(c2) < 1

    def test_bearing_north(self):
        c1 = Coordinate(35.7274, -78.6960, 0)
        c2 = Coordinate(35.7374, -78.6960, 0)
        b = c1.bearing(c2)
        assert abs(b) < 1 or abs(b - 360) < 1

    def test_add_vector(self):
        c = Coordinate(35.7274, -78.6960, 0)
        v = VectorNED(100, 0, 0)
        r = c + v
        assert r.lat > c.lat and abs(r.lon - c.lon) < 1e-6

    def test_subtract_coordinates_gives_vector(self):
        c1 = Coordinate(35.7274, -78.6960, 100)
        c2 = Coordinate(35.7274, -78.6960, 0)
        r = c1 - c2
        assert isinstance(r, VectorNED)

    def test_to_json(self):
        c = Coordinate(35.7274, -78.6960, 100)
        j = json.loads(c.toJson())
        assert j["lat"] == 35.7274 and j["lon"] == -78.6960


class TestPlanFile:
    """Plan file reading."""

    @pytest.fixture
    def sample_plan(self):
        data = {
            "fileType": "Plan",
            "mission": {
                "items": [
                    {"command": PLAN_CMD_TAKEOFF, "params": [0, 0, 0, 0, 35.7274, -78.6960, 10], "doJumpId": 1},
                    {"command": PLAN_CMD_WAYPOINT, "params": [0, 0, 0, 0, 35.7284, -78.6960, 20], "doJumpId": 2},
                    {"command": PLAN_CMD_RTL, "params": [0, 0, 0, 0, 0, 0, 0], "doJumpId": 3},
                ]
            },
        }
        with tempfile.NamedTemporaryFile(mode="w", suffix=".plan", delete=False) as f:
            json.dump(data, f)
            path = f.name
        yield path
        os.unlink(path)

    def test_read_from_plan(self, sample_plan):
        wps = read_from_plan(sample_plan)
        assert len(wps) == 3
        assert wps[0][0] == PLAN_CMD_TAKEOFF
        assert wps[0][1] == 35.7274 and wps[0][3] == 10

    def test_read_from_plan_wrong_type(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
            json.dump({"fileType": "NotAPlan", "mission": {"items": []}}, f)
            path = f.name
        try:
            with pytest.raises(Exception, match="Wrong file type"):
                read_from_plan(path)
        finally:
            os.unlink(path)

    def test_get_location_from_waypoint(self, sample_plan):
        wps = read_from_plan(sample_plan)
        c = get_location_from_waypoint(wps[0])
        assert isinstance(c, Coordinate)
        assert c.lat == 35.7274 and c.alt == 10

    def test_read_from_plan_complete(self, sample_plan):
        wps = read_from_plan_complete(sample_plan)
        assert len(wps) == 3
        assert "id" in wps[0] and "pos" in wps[0] and "wait_for" in wps[0]


class TestGeofence:
    """Geofence and geometry functions."""

    @pytest.fixture
    def square_geofence(self):
        return [
            {"lon": -78.70, "lat": 35.72},
            {"lon": -78.70, "lat": 35.74},
            {"lon": -78.68, "lat": 35.74},
            {"lon": -78.68, "lat": 35.72},
            {"lon": -78.70, "lat": 35.72},
        ]

    def test_inside_true(self, square_geofence):
        assert inside(-78.69, 35.73, square_geofence) is True

    def test_inside_false(self, square_geofence):
        assert inside(-78.50, 35.73, square_geofence) is False

    def test_orientation_colinear(self):
        assert orientation(0, 0, 1, 1, 2, 2) == 0

    def test_orientation_clockwise(self):
        assert orientation(0, 0, 1, 1, 1, 0) == 1

    def test_do_intersect_crossing(self):
        assert doIntersect(0, 0, 10, 10, 0, 10, 10, 0) is True

    def test_do_intersect_parallel(self):
        assert doIntersect(0, 0, 10, 0, 0, 1, 10, 1) is False

    def test_lies_on_segment_true(self):
        assert liesOnSegment(0, 0, 1, 1, 2, 2) is True

    def test_lies_on_segment_false(self):
        assert liesOnSegment(0, 0, 3, 3, 2, 2) is False


class TestReadGeofence:
    """read_geofence from KML."""

    @pytest.fixture
    def minimal_kml(self):
        kml = """<?xml version="1.0"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
<Placemark>
<Polygon>
<outerBoundaryIs>
<LinearRing>
<coordinates>-78.70,35.72,0 -78.70,35.74,0 -78.68,35.74,0 -78.68,35.72,0 -78.70,35.72,0</coordinates>
</LinearRing>
</outerBoundaryIs>
</Polygon>
</Placemark>
</Document>
</kml>"""
        with tempfile.NamedTemporaryFile(mode="wb", suffix=".kml", delete=False) as f:
            f.write(kml.encode())
            path = f.name
        yield path
        os.unlink(path)

    def test_read_geofence_returns_polygon(self, minimal_kml):
        poly = read_geofence(minimal_kml)
        assert len(poly) >= 4
        assert all("lat" in p and "lon" in p for p in poly)


class TestVectorNEDExtended:
    def test_str_format(self):
        v = VectorNED(1.0, 2.0, 3.0)
        s = str(v)
        assert "1.0" in s and "2.0" in s and "3.0" in s

    def test_mul_float(self):
        v = VectorNED(1.5, 2.5, 3.5)
        r = v * 2.0
        assert abs(r.north - 3.0) < 1e-10
        assert abs(r.east - 5.0) < 1e-10
        assert abs(r.down - 7.0) < 1e-10

    def test_rmul_float(self):
        v = VectorNED(1.0, 0.0, 0.0)
        r = 3.0 * v
        assert abs(r.north - 3.0) < 1e-10

    def test_mul_invalid_type_raises(self):
        with pytest.raises(TypeError):
            VectorNED(1, 2, 3) * "oops"

    def test_cross_product_non_vector_raises(self):
        with pytest.raises(TypeError):
            VectorNED(1, 0, 0).cross_product(42)

    def test_cross_product_antiparallel(self):
        """cross product of anti-parallel vectors should be zero."""
        v1 = VectorNED(1, 0, 0)
        v2 = VectorNED(-1, 0, 0)
        c = v1.cross_product(v2)
        assert abs(c.north) < 1e-10 and abs(c.east) < 1e-10 and abs(c.down) < 1e-10

    def test_hypot_3d(self):
        # 1, 1, 1 => sqrt(3)
        v = VectorNED(1, 1, 1)
        assert abs(v.hypot() - math.sqrt(3)) < 1e-10

    def test_norm_direction_preserved(self):
        v = VectorNED(3, 4, 0)
        n = v.norm()
        assert abs(n.north - 0.6) < 1e-10
        assert abs(n.east - 0.8) < 1e-10

    def test_rotate_by_angle_180(self):
        v = VectorNED(1, 0, 0)
        r = v.rotate_by_angle(180)
        # Rotated 180° → (-1, 0, 0) direction (allow floating point tolerance)
        assert abs(r.north - (-1)) < 1e-10
        assert abs(r.east) < 1e-10

    def test_down_preserved_in_rotation(self):
        v = VectorNED(1, 0, 5)
        r = v.rotate_by_angle(90)
        assert abs(r.down - 5) < 1e-10


class TestCoordinateExtended:
    def test_str_format(self):
        c = Coordinate(35.0, -78.0, 100.0)
        s = str(c)
        assert "35.0" in s and "-78.0" in s and "100.0" in s

    def test_distance_3d_altitude(self):
        """3D distance should account for altitude difference."""
        c1 = Coordinate(35.7274, -78.6960, 0)
        c2 = Coordinate(35.7274, -78.6960, 100)
        d = c1.distance(c2)
        # Two co-located points with 100 m altitude difference
        assert abs(d - 100.0) < 0.5  # within 0.5 m

    def test_distance_same_point_is_zero(self):
        c = Coordinate(35.0, -78.0, 50)
        assert c.distance(c) == 0

    def test_distance_invalid_type(self):
        with pytest.raises(TypeError):
            Coordinate(0, 0).distance((0, 0))

    def test_ground_distance_invalid_type(self):
        with pytest.raises(TypeError):
            Coordinate(0, 0).ground_distance(42)

    def test_bearing_wrap_360_false(self):
        """With wrap_360=False, bearing can be negative or >360."""
        c1 = Coordinate(35.0, -78.0)
        c2 = Coordinate(34.9, -78.0)   # south of c1 → bearing ~180
        b_wrapped = c1.bearing(c2, wrap_360=True)
        b_unwrapped = c1.bearing(c2, wrap_360=False)
        # wrap_360=True result is in [0, 360)
        assert 0 <= b_wrapped < 360
        # Both should represent the same compass direction (mod 360)
        assert abs((b_wrapped - b_unwrapped) % 360) < 1e-6 or abs(b_unwrapped - b_wrapped) < 1e-6

    def test_bearing_coincident_points_returns_zero(self):
        c = Coordinate(35.7274, -78.6960, 0)
        assert c.bearing(c) == 0.0

    def test_bearing_invalid_type(self):
        with pytest.raises(TypeError):
            Coordinate(0, 0).bearing("north")

    def test_add_vector_altitude(self):
        """Adding a vector with down component changes altitude."""
        c = Coordinate(35.0, -78.0, 50.0)
        v = VectorNED(0, 0, 10)   # down=10 → altitude decreases by 10
        r = c + v
        assert abs(r.alt - 40.0) < 1e-6

    def test_add_invalid_type_raises(self):
        with pytest.raises(TypeError):
            Coordinate(0, 0, 0) + 5

    def test_sub_vector_moves_opposite(self):
        """Subtracting a NED vector moves in the opposite direction."""
        c = Coordinate(35.0, -78.0, 100.0)
        v = VectorNED(0, 100, 0)   # east 100 m
        r = c - v                  # should move west
        assert r.lon < c.lon

    def test_sub_coordinate_gives_vector(self):
        c1 = Coordinate(35.7274, -78.6960, 100)
        c2 = Coordinate(35.7274, -78.6960, 0)
        diff = c1 - c2
        assert isinstance(diff, VectorNED)
        # down component: o.alt - self.alt = 0 - 100 = -100
        assert abs(diff.down - (-100)) < 1.0

    def test_sub_invalid_type_raises(self):
        with pytest.raises(TypeError):
            Coordinate(0, 0, 0) - 99

    def test_to_json_and_toJson_equivalent(self):
        c = Coordinate(35.7274, -78.6960, 100.0)
        j1 = json.loads(c.to_json())
        j2 = json.loads(c.toJson())
        assert j1 == j2

    def test_to_json_contains_all_fields(self):
        c = Coordinate(1.0, 2.0, 3.0)
        j = json.loads(c.to_json())
        assert j["lat"] == 1.0
        assert j["lon"] == 2.0
        assert j["alt"] == 3.0

    def test_add_vector_north_increases_lat(self):
        c = Coordinate(35.0, -78.0, 0)
        r = c + VectorNED(1000, 0, 0)
        assert r.lat > c.lat

    def test_add_vector_east_increases_lon(self):
        c = Coordinate(35.0, -78.0, 0)
        r = c + VectorNED(0, 1000, 0)
        assert r.lon > c.lon


class TestPlanFileExtended:
    @pytest.fixture
    def plan_with_speed_change(self):
        """Plan with a PLAN_CMD_SPEED item between waypoints."""
        data = {
            "fileType": "Plan",
            "mission": {
                "items": [
                    {
                        "command": PLAN_CMD_TAKEOFF,
                        "params": [0, 0, 0, 0, 35.72, -78.69, 10],
                        "doJumpId": 1,
                    },
                    {
                        "command": PLAN_CMD_SPEED,
                        "params": [0, 12.0, 0, 0, 0, 0, 0],
                        "doJumpId": 2,
                    },
                    {
                        "command": PLAN_CMD_WAYPOINT,
                        "params": [0, 0, 0, 0, 35.73, -78.69, 20],
                        "doJumpId": 3,
                    },
                    {
                        "command": PLAN_CMD_RTL,
                        "params": [0, 0, 0, 0, 0, 0, 0],
                        "doJumpId": 4,
                    },
                ]
            },
        }
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".plan", delete=False
        ) as f:
            json.dump(data, f)
            path = f.name
        yield path
        os.unlink(path)

    def test_speed_change_applied_to_following_waypoints(self, plan_with_speed_change):
        wps = read_from_plan(plan_with_speed_change)
        # SPEED item is not yielded as a waypoint itself
        assert len(wps) == 3  # takeoff, waypoint, RTL
        # First waypoint uses default speed
        assert wps[0][5] == DEFAULT_WAYPOINT_SPEED
        # Subsequent waypoints after the speed change
        assert wps[1][5] == 12.0
        assert wps[2][5] == 12.0

    def test_read_from_plan_complete_speed_change(self, plan_with_speed_change):
        wps = read_from_plan_complete(plan_with_speed_change)
        assert len(wps) == 3
        assert wps[0]["speed"] == DEFAULT_WAYPOINT_SPEED
        assert wps[1]["speed"] == 12.0
        assert wps[2]["speed"] == 12.0

    def test_read_from_plan_complete_has_wait_for(self, plan_with_speed_change):
        wps = read_from_plan_complete(plan_with_speed_change)
        for wp in wps:
            assert "wait_for" in wp

    def test_read_from_plan_empty_mission(self):
        data = {"fileType": "Plan", "mission": {"items": []}}
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".plan", delete=False
        ) as f:
            json.dump(data, f)
            path = f.name
        try:
            wps = read_from_plan(path)
            assert wps == []
        finally:
            os.unlink(path)

    def test_get_location_from_waypoint_rtl_zeros(self):
        """RTL waypoint has lat=0, lon=0, alt=0 by convention."""
        data = {
            "fileType": "Plan",
            "mission": {
                "items": [
                    {
                        "command": PLAN_CMD_RTL,
                        "params": [0, 0, 0, 0, 0, 0, 0],
                        "doJumpId": 1,
                    }
                ]
            },
        }
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".plan", delete=False
        ) as f:
            json.dump(data, f)
            path = f.name
        try:
            wps = read_from_plan(path)
            c = get_location_from_waypoint(wps[0])
            assert c.lat == 0 and c.lon == 0 and c.alt == 0
        finally:
            os.unlink(path)


class TestGeometryExtended:
    """Covers orientation CCW, do_intersect collinear, lies_on_segment corners."""

    def test_orientation_counterclockwise(self):
        # Counter-clockwise triplet
        assert orientation(0, 0, 1, 0, 0, 1) == 2

    def test_orientation_clockwise(self):
        # Clockwise: going right then down-right
        assert orientation(0, 0, 4, 4, 4, 0) == 1

    def test_orientation_colinear(self):
        assert orientation(0, 0, 1, 1, 2, 2) == 0

    def test_lies_on_segment_endpoint(self):
        # Point Q is exactly at endpoint P
        assert lies_on_segment(0, 0, 0, 0, 10, 10) is True

    def test_lies_on_segment_other_endpoint(self):
        # Point Q is exactly at endpoint R
        assert lies_on_segment(0, 0, 10, 10, 10, 10) is True

    def test_lies_on_segment_outside(self):
        # Q lies beyond the segment
        assert lies_on_segment(0, 0, 15, 15, 10, 10) is False

    def test_liesOnSegment_alias(self):
        """liesOnSegment is an alias for lies_on_segment."""
        assert liesOnSegment(0, 0, 5, 5, 10, 10) == lies_on_segment(0, 0, 5, 5, 10, 10)

    def test_do_intersect_t_shape(self):
        """T-shape: one segment's endpoint touches the middle of another."""
        # Horizontal segment (0,5)-(10,5), vertical segment (5,0)-(5,5) ending at midpoint
        assert do_intersect(0, 5, 10, 5, 5, 0, 5, 5) is True

    def test_do_intersect_collinear_non_overlapping(self):
        # Collinear but no overlap
        assert do_intersect(0, 0, 1, 0, 2, 0, 3, 0) is False

    def test_doIntersect_alias(self):
        """doIntersect is an alias for do_intersect."""
        assert doIntersect(0, 0, 10, 10, 0, 10, 10, 0) == do_intersect(
            0, 0, 10, 10, 0, 10, 10, 0
        )

    def test_inside_on_boundary_behaviour(self):
        """Point on the boundary – result can be True or False, just must not crash."""
        fence = [
            {"lon": 0, "lat": 0},
            {"lon": 0, "lat": 10},
            {"lon": 10, "lat": 10},
            {"lon": 10, "lat": 0},
            {"lon": 0, "lat": 0},
        ]
        # Just ensure no exception
        result = inside(0, 5, fence)
        assert isinstance(result, bool)

    def test_inside_with_triangle_geofence(self):
        triangle = [
            {"lon": 0, "lat": 0},
            {"lon": 10, "lat": 0},
            {"lon": 5, "lat": 10},
            {"lon": 0, "lat": 0},
        ]
        assert inside(5, 3, triangle) is True   # inside
        assert inside(0, 11, triangle) is False  # outside above apex

    def test_inside_empty_fence_returns_false(self):
        assert inside(0, 0, []) is False

