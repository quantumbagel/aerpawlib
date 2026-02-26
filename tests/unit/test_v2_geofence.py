"""Unit tests for aerpawlib v2 geofence module."""

import os
import tempfile
import pytest

from aerpawlib.v2.geofence import inside, do_intersect, read_geofence, polygon_edges


class TestInside:
    """inside(lon, lat, geofence) point-in-polygon."""

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

    def test_inside_empty_returns_false(self):
        assert inside(0, 0, []) is False

    def test_inside_triangle(self):
        triangle = [
            {"lon": 0, "lat": 0},
            {"lon": 10, "lat": 0},
            {"lon": 5, "lat": 10},
            {"lon": 0, "lat": 0},
        ]
        assert inside(5, 3, triangle) is True
        assert inside(0, 11, triangle) is False


class TestDoIntersect:
    """do_intersect segment-segment intersection."""

    def test_crossing(self):
        assert do_intersect(0, 0, 10, 10, 0, 10, 10, 0) is True

    def test_parallel(self):
        assert do_intersect(0, 0, 10, 0, 0, 1, 10, 1) is False

    def test_t_shape(self):
        assert do_intersect(0, 5, 10, 5, 5, 0, 5, 5) is True

    def test_collinear_non_overlapping(self):
        assert do_intersect(0, 0, 1, 0, 2, 0, 3, 0) is False


class TestPolygonEdges:
    """polygon_edges yields consecutive edge pairs."""

    def test_square_edges(self):
        square = [
            {"lon": 0, "lat": 0},
            {"lon": 0, "lat": 1},
            {"lon": 1, "lat": 1},
            {"lon": 1, "lat": 0},
        ]
        edges = list(polygon_edges(square))
        assert len(edges) == 4
        assert edges[0][0]["lon"] == 0 and edges[0][0]["lat"] == 0
        assert edges[0][1]["lon"] == 0 and edges[0][1]["lat"] == 1


class TestReadGeofence:
    """read_geofence from KML file."""

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
        with tempfile.NamedTemporaryFile(
            mode="wb", suffix=".kml", delete=False
        ) as f:
            f.write(kml.encode())
            path = f.name
        yield path
        os.unlink(path)

    def test_read_geofence_returns_polygon(self, minimal_kml):
        poly = read_geofence(minimal_kml)
        assert len(poly) >= 4
        assert all("lat" in p and "lon" in p for p in poly)
