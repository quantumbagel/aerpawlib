"""
Geofence utilities for aerpawlib v2 API.

This module provides geofence-related functions for reading KML files,
checking if points are inside polygons, and detecting line segment intersections.

Uses the Shapely library for reliable geometric operations.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Union, Optional

from pykml import parser
from shapely.geometry import Point, Polygon as ShapelyPolygon, LineString
from shapely.ops import nearest_points

from .types import Coordinate
from .exceptions import GeofenceError, GeofenceReadError, InvalidPolygonError

# Use modular logging system
from .logging import get_logger, LogComponent

logger = get_logger(LogComponent.GEOFENCE)


@dataclass(frozen=True, slots=True)
class GeofencePoint:
    """
    An immutable point in a geofence polygon.

    Attributes:
        longitude: Longitude in degrees (-180 to 180)
        latitude: Latitude in degrees (-90 to 90)
    """

    longitude: float
    latitude: float

    def __post_init__(self) -> None:
        """Validate coordinates are within valid ranges."""
        if not -180 <= self.longitude <= 180:
            raise ValueError(
                f"Longitude must be between -180 and 180, got {self.longitude}"
            )
        if not -90 <= self.latitude <= 90:
            raise ValueError(
                f"Latitude must be between -90 and 90, got {self.latitude}"
            )

    @property
    def lon(self) -> float:
        """Alias for longitude."""
        return self.longitude

    @property
    def lat(self) -> float:
        """Alias for latitude."""
        return self.latitude

    def to_coordinate(self, altitude: float = 0.0) -> Coordinate:
        """Convert to a Coordinate with the given altitude."""
        return Coordinate(
            latitude=self.latitude, longitude=self.longitude, altitude=altitude
        )

    def to_shapely(self) -> Point:
        """Convert to a Shapely Point."""
        return Point(self.longitude, self.latitude)

    def __repr__(self) -> str:
        return (
            f"GeofencePoint(lon={self.longitude:.6f}, lat={self.latitude:.6f})"
        )


# Type aliases
Polygon = Sequence[GeofencePoint]
PointLike = Union[Coordinate, GeofencePoint]


def is_inside_polygon(point: PointLike, polygon: Polygon) -> bool:
    """
    Check if a point is inside a polygon.

    Uses Shapely's robust point-in-polygon algorithm.

    Args:
        point: The point to check (Coordinate or GeofencePoint)
        polygon: A sequence of GeofencePoint defining the polygon

    Returns:
        True if the point is inside the polygon, False otherwise.

    Raises:
        InvalidPolygonError: If the polygon has fewer than 3 vertices.

    Example:
        >>> geofence = read_geofence("boundary.kml")
        >>> point = Coordinate(35.7277, -78.6967, 0)
        >>> if is_inside_polygon(point, geofence):
        ...     print("Point is inside geofence")
    """
    shapely_point = _to_shapely_point(point)
    shapely_poly = _to_shapely_polygon(polygon)

    # contains() returns True if point is strictly inside
    # We also check touches() for points on the boundary
    return shapely_poly.contains(shapely_point) or shapely_poly.touches(
        shapely_point
    )


def is_on_polygon_boundary(point: PointLike, polygon: Polygon) -> bool:
    """
    Check if a point is on the boundary of a polygon.

    Args:
        point: The point to check
        polygon: A sequence of GeofencePoint defining the polygon

    Returns:
        True if the point is on the polygon boundary, False otherwise.
    """
    shapely_point = _to_shapely_point(point)
    shapely_poly = _to_shapely_polygon(polygon)
    return shapely_poly.boundary.contains(shapely_point)


def segments_intersect(
    p1: PointLike, q1: PointLike, p2: PointLike, q2: PointLike
) -> bool:
    """
    Check if line segment p1-q1 intersects with segment p2-q2.

    Uses Shapely's robust intersection algorithm.

    Args:
        p1: First endpoint of segment 1
        q1: Second endpoint of segment 1
        p2: First endpoint of segment 2
        q2: Second endpoint of segment 2

    Returns:
        True if the segments intersect, False otherwise.

    Example:
        >>> start = Coordinate(35.7, -78.6, 0)
        >>> end = Coordinate(35.8, -78.5, 0)
        >>> edge_start = GeofencePoint(-78.55, 35.75)
        >>> edge_end = GeofencePoint(-78.65, 35.75)
        >>> if segments_intersect(start, end, edge_start, edge_end):
        ...     print("Path crosses geofence!")
    """
    line1 = LineString(
        [
            (_to_shapely_point(p1).x, _to_shapely_point(p1).y),
            (_to_shapely_point(q1).x, _to_shapely_point(q1).y),
        ]
    )
    line2 = LineString(
        [
            (_to_shapely_point(p2).x, _to_shapely_point(p2).y),
            (_to_shapely_point(q2).x, _to_shapely_point(q2).y),
        ]
    )

    return line1.intersects(line2)


def path_crosses_polygon(
    start: PointLike, end: PointLike, polygon: Polygon, *, closed: bool = True
) -> bool:
    """
    Check if a path from start to end crosses any edge of a polygon.

    Uses Shapely's robust intersection algorithm.

    Args:
        start: Starting coordinate of the path
        end: Ending coordinate of the path
        polygon: A sequence of GeofencePoint defining the polygon
        closed: If True, treats polygon as closed (ignored, Shapely always closes)

    Returns:
        True if the path crosses any polygon edge, False otherwise.

    Raises:
        InvalidPolygonError: If the polygon has fewer than 3 vertices.

    Example:
        >>> geofence = read_geofence("boundary.kml")
        >>> start = drone.position
        >>> target = Coordinate(35.8, -78.5, 10)
        >>> if path_crosses_polygon(start, target, geofence):
        ...     print("Path would cross geofence boundary!")
    """
    path = LineString(
        [
            (_to_shapely_point(start).x, _to_shapely_point(start).y),
            (_to_shapely_point(end).x, _to_shapely_point(end).y),
        ]
    )
    shapely_poly = _to_shapely_polygon(polygon)

    # Check if path crosses the polygon boundary
    return path.crosses(shapely_poly.boundary) or path.intersects(
        shapely_poly.boundary
    )


def is_path_valid(
    start: PointLike,
    end: PointLike,
    include_fence: Polygon,
    exclude_fences: Sequence[Polygon] = (),
) -> bool:
    """
    Check if a path is valid given inclusion and exclusion geofences.

    A valid path:
    - Starts and ends inside the include fence
    - Does not cross the include fence boundary
    - Does not start or end in any exclude fence
    - Does not cross into any exclude fence

    Args:
        start: Starting point
        end: Ending point
        include_fence: The polygon that the path must stay within
        exclude_fences: Polygons that the path must avoid

    Returns:
        True if the path is valid, False otherwise.
    """
    # Create path as LineString
    path = LineString(
        [
            (_to_shapely_point(start).x, _to_shapely_point(start).y),
            (_to_shapely_point(end).x, _to_shapely_point(end).y),
        ]
    )

    # Create include polygon
    include_poly = _to_shapely_polygon(include_fence)

    # Check start and end are inside include fence
    start_point = _to_shapely_point(start)
    end_point = _to_shapely_point(end)

    if not include_poly.contains(start_point):
        return False
    if not include_poly.contains(end_point):
        return False

    # Check entire path stays within include fence
    if not include_poly.contains(path):
        return False

    # Check path doesn't enter any exclude fence
    for fence in exclude_fences:
        exclude_poly = _to_shapely_polygon(fence)

        # Path is invalid if start or end is in an exclude zone
        if exclude_poly.contains(start_point):
            return False
        if exclude_poly.contains(end_point):
            return False

        # Path is invalid if it intersects an exclude zone
        if path.intersects(exclude_poly):
            return False

    return True


def polygon_area(polygon: Polygon) -> float:
    """
    Calculate the area of a polygon.

    Note: This returns the area in square degrees. For actual area in
    square meters, use polygon_area_meters() or convert using geopy.

    Args:
        polygon: A sequence of GeofencePoint defining the polygon

    Returns:
        The absolute area of the polygon in square degrees.

    Raises:
        InvalidPolygonError: If the polygon has fewer than 3 vertices.
    """
    shapely_poly = _to_shapely_polygon(polygon)
    return abs(shapely_poly.area)


def polygon_centroid(polygon: Polygon) -> GeofencePoint:
    """
    Calculate the centroid (center of mass) of a polygon.

    Uses Shapely's robust centroid calculation.

    Args:
        polygon: A sequence of GeofencePoint defining the polygon

    Returns:
        The centroid as a GeofencePoint.

    Raises:
        InvalidPolygonError: If the polygon has fewer than 3 vertices.
    """
    shapely_poly = _to_shapely_polygon(polygon)
    centroid = shapely_poly.centroid
    return GeofencePoint(longitude=centroid.x, latitude=centroid.y)


def closest_point_on_polygon(
    point: PointLike, polygon: Polygon
) -> GeofencePoint:
    """
    Find the closest point on a polygon boundary to a given point.

    Args:
        point: The point to find the closest boundary point to
        polygon: A sequence of GeofencePoint defining the polygon

    Returns:
        The closest point on the polygon boundary as a GeofencePoint.
    """
    shapely_point = _to_shapely_point(point)
    shapely_poly = _to_shapely_polygon(polygon)

    # Get the nearest point on the polygon boundary
    _, nearest = nearest_points(shapely_point, shapely_poly.boundary)

    return GeofencePoint(longitude=nearest.x, latitude=nearest.y)


def distance_to_polygon_edge(point: PointLike, polygon: Polygon) -> float:
    """
    Calculate the minimum distance from a point to the polygon boundary.

    Note: This returns distance in degrees. For actual distance in meters,
    use distance_to_polygon_edge_meters() or convert using geopy.

    Args:
        point: The point to measure from
        polygon: A sequence of GeofencePoint defining the polygon

    Returns:
        The minimum distance in degrees to the polygon boundary.

    Raises:
        InvalidPolygonError: If the polygon has fewer than 3 vertices.
    """
    shapely_point = _to_shapely_point(point)
    shapely_poly = _to_shapely_polygon(polygon)

    return shapely_point.distance(shapely_poly.boundary)


def is_polygon_clockwise(polygon: Polygon) -> bool:
    """
    Determine if a polygon's vertices are ordered clockwise.

    Uses Shapely's signed area calculation.

    Args:
        polygon: A sequence of GeofencePoint defining the polygon

    Returns:
        True if vertices are clockwise, False if counterclockwise.

    Raises:
        InvalidPolygonError: If the polygon has fewer than 3 vertices.
    """
    shapely_poly = _to_shapely_polygon(polygon)

    # Shapely returns positive area for counter-clockwise polygons
    # We use the exterior ring to check orientation
    return not shapely_poly.exterior.is_ccw


def buffer_polygon(polygon: Polygon, distance: float) -> List[GeofencePoint]:
    """
    Create a buffered (expanded or shrunk) version of a polygon.

    Args:
        polygon: The original polygon
        distance: Buffer distance in degrees (positive = expand, negative = shrink)

    Returns:
        A new polygon representing the buffered shape.
    """
    shapely_poly = _to_shapely_polygon(polygon)
    buffered = shapely_poly.buffer(distance)

    # Extract coordinates from the buffered polygon
    if buffered.is_empty:
        return []

    coords = list(buffered.exterior.coords)
    return [
        GeofencePoint(longitude=x, latitude=y) for x, y in coords[:-1]
    ]  # Exclude closing point


def simplify_polygon(
    polygon: Polygon, tolerance: float = 0.0001
) -> List[GeofencePoint]:
    """
    Simplify a polygon by removing vertices while preserving shape.

    Uses the Douglas-Peucker algorithm via Shapely.

    Args:
        polygon: The polygon to simplify
        tolerance: Maximum deviation from original shape in degrees

    Returns:
        A simplified polygon.
    """
    shapely_poly = _to_shapely_polygon(polygon)
    simplified = shapely_poly.simplify(tolerance, preserve_topology=True)

    coords = list(simplified.exterior.coords)
    return [GeofencePoint(longitude=x, latitude=y) for x, y in coords[:-1]]


def polygon_bounds(polygon: Polygon) -> tuple[float, float, float, float]:
    """
    Get the bounding box of a polygon.

    Args:
        polygon: The polygon to get bounds for

    Returns:
        Tuple of (min_lon, min_lat, max_lon, max_lat)
    """
    shapely_poly = _to_shapely_polygon(polygon)
    return shapely_poly.bounds


def polygons_overlap(polygon1: Polygon, polygon2: Polygon) -> bool:
    """
    Check if two polygons overlap.

    Args:
        polygon1: First polygon
        polygon2: Second polygon

    Returns:
        True if the polygons overlap, False otherwise.
    """
    poly1 = _to_shapely_polygon(polygon1)
    poly2 = _to_shapely_polygon(polygon2)
    return poly1.intersects(poly2)


def polygon_intersection(
    polygon1: Polygon, polygon2: Polygon
) -> Optional[List[GeofencePoint]]:
    """
    Calculate the intersection of two polygons.

    Args:
        polygon1: First polygon
        polygon2: Second polygon

    Returns:
        The intersection polygon, or None if polygons don't intersect.
    """
    poly1 = _to_shapely_polygon(polygon1)
    poly2 = _to_shapely_polygon(polygon2)

    intersection = poly1.intersection(poly2)

    if intersection.is_empty:
        return None

    if intersection.geom_type != "Polygon":
        # Could be MultiPolygon, LineString, etc.
        return None

    coords = list(intersection.exterior.coords)
    return [GeofencePoint(longitude=x, latitude=y) for x, y in coords[:-1]]


def _to_shapely_point(point: PointLike) -> Point:
    """Internal helper to convert PointLike to Shapely Point."""
    if isinstance(point, Coordinate):
        return Point(point.longitude, point.latitude)
    elif isinstance(point, GeofencePoint):
        return Point(point.longitude, point.latitude)
    else:
        raise TypeError(
            f"Expected Coordinate or GeofencePoint, got {type(point)}"
        )


def _to_shapely_polygon(polygon: Polygon) -> ShapelyPolygon:
    """Internal helper to convert Polygon to Shapely Polygon."""
    if len(polygon) < 3:
        raise InvalidPolygonError("Polygon must have at least 3 vertices")
    return ShapelyPolygon([(p.longitude, p.latitude) for p in polygon])


def validate_polygon(polygon: Polygon) -> None:
    """
    Validate that a polygon is a valid geometric shape.

    Args:
        polygon: A sequence of GeofencePoint

    Raises:
        InvalidPolygonError: If the polygon is invalid.
    """
    if len(polygon) < 3:
        raise InvalidPolygonError("Polygon must have at least 3 vertices")

    try:
        poly = _to_shapely_polygon(polygon)
        if not poly.is_valid:
            raise InvalidPolygonError(
                "Polygon is not a valid geometric shape (self-intersecting?)"
            )
    except Exception as e:
        if isinstance(e, InvalidPolygonError):
            raise
        raise InvalidPolygonError(f"Geometric validation failed: {e}")


def read_geofence(file_path: Union[str, Path]) -> List[GeofencePoint]:
    """
    Read a geofence from a KML file.

    Args:
        file_path: Path to the KML file

    Returns:
        A list of GeofencePoint defining the boundary.

    Raises:
        GeofenceReadError: If the file cannot be read or parsed.
    """
    path = Path(file_path)
    if not path.exists():
        raise GeofenceReadError(f"KML file not found: {file_path}")

    try:
        with open(path, "rb") as f:
            root = parser.parse(f).getroot()

        # Look for LinearRing or Polygon coordinates
        namespace = {"kml": "http://www.opengis.net/kml/2.2"}
        coords_elements = root.xpath("//kml:coordinates", namespaces=namespace)

        if not coords_elements:
            raise GeofenceReadError("No coordinates found in KML file")

        # Extract coordinates from the first coordinates element
        # KML coordinates are typically: lon,lat,alt lon,lat,alt ...
        coords_str = str(coords_elements[0].text).strip()
        points = []
        for pair in coords_str.split():
            parts = pair.split(",")
            if len(parts) >= 2:
                points.append(
                    GeofencePoint(
                        longitude=float(parts[0]), latitude=float(parts[1])
                    )
                )

        if len(points) < 3:
            raise GeofenceReadError(
                f"Geofence must have at least 3 points, found {len(points)}"
            )

        return points

    except Exception as e:
        raise GeofenceReadError(f"Failed to parse KML: {e}")


__all__ = [
    # Types
    "GeofencePoint",
    "Polygon",
    "PointLike",
    # Exceptions
    "GeofenceError",
    "GeofenceReadError",
    "InvalidPolygonError",
    # Core functions
    "read_geofence",
    "validate_polygon",
    "is_inside_polygon",
    "is_on_polygon_boundary",
    "segments_intersect",
    "path_crosses_polygon",
    "is_path_valid",
    # Utility functions
    "polygon_area",
    "polygon_centroid",
    "closest_point_on_polygon",
    "distance_to_polygon_edge",
    "is_polygon_clockwise",
    "buffer_polygon",
    "simplify_polygon",
    "polygon_bounds",
    "polygons_overlap",
    "polygon_intersection",
]
