"""
Geofence utilities for aerpawlib v2 API.

This module provides geofence-related functions for reading KML files,
checking if points are inside polygons, and detecting line segment intersections.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Union, Protocol, runtime_checkable

from pykml import parser

from .types import Coordinate

# Use modular logging system
from .logging import get_logger, LogComponent
logger = get_logger(LogComponent.GEOFENCE)

# Tolerance for floating point comparisons
_FLOAT_TOLERANCE = 1e-10


@runtime_checkable
class GeoPoint(Protocol):
    """Protocol for any object that has longitude and latitude properties."""

    @property
    def longitude(self) -> float:
        """Longitude in degrees."""
        ...

    @property
    def latitude(self) -> float:
        """Latitude in degrees."""
        ...


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
            raise ValueError(f"Longitude must be between -180 and 180, got {self.longitude}")
        if not -90 <= self.latitude <= 90:
            raise ValueError(f"Latitude must be between -90 and 90, got {self.latitude}")

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
            latitude=self.latitude,
            longitude=self.longitude,
            altitude=altitude
        )

    def __repr__(self) -> str:
        return f"GeofencePoint(lon={self.longitude:.6f}, lat={self.latitude:.6f})"


# Type aliases
Polygon = Sequence[GeofencePoint]
PointLike = Union[Coordinate, GeofencePoint]


class GeofenceError(Exception):
    """Base exception for geofence-related errors."""
    pass


class GeofenceReadError(GeofenceError):
    """Raised when reading a geofence file fails."""
    pass


class InvalidPolygonError(GeofenceError):
    """Raised when a polygon is invalid (e.g., too few points)."""
    pass


def _extract_coords(point: PointLike) -> tuple[float, float]:
    """
    Extract longitude and latitude from a point-like object.

    Args:
        point: A Coordinate, GeofencePoint, or any object with lon/lat properties

    Returns:
        Tuple of (longitude, latitude)
    """
    # Try lon/lat first (both Coordinate and GeofencePoint have these)
    if hasattr(point, 'lon') and hasattr(point, 'lat'):
        return point.lon, point.lat
    # Fall back to full names
    if hasattr(point, 'longitude') and hasattr(point, 'latitude'):
        return point.longitude, point.latitude
    raise TypeError(f"Cannot extract coordinates from {type(point).__name__}")


def read_geofence(file_path: str | Path) -> List[GeofencePoint]:
    """
    Read a geofence from a KML file.

    Args:
        file_path: Path to the KML file containing the geofence polygon.

    Returns:
        A list of GeofencePoint objects representing the polygon vertices.

    Raises:
        GeofenceReadError: If the file cannot be read or parsed.
        FileNotFoundError: If the file does not exist.

    Example:
        >>> geofence = read_geofence("boundary.kml")
        >>> print(f"Geofence has {len(geofence)} vertices")
    """
    path = Path(file_path)

    if not path.exists():
        raise FileNotFoundError(f"Geofence file not found: {path}")

    try:
        with path.open("rb") as f:
            root = parser.fromstring(f.read())
    except Exception as e:
        raise GeofenceReadError(f"Failed to parse KML file: {e}") from e

    try:
        coordinates_string = (
            root.Document.Placemark.Polygon.outerBoundaryIs.LinearRing.coordinates.text
        )
    except AttributeError as e:
        raise GeofenceReadError(
            "KML file does not contain expected polygon structure. "
            "Expected: Document/Placemark/Polygon/outerBoundaryIs/LinearRing/coordinates"
        ) from e

    if not coordinates_string or not coordinates_string.strip():
        raise GeofenceReadError("KML file contains empty coordinates")

    coordinates_list = coordinates_string.split()

    polygon: List[GeofencePoint] = []
    for i, coord_str in enumerate(coordinates_list):
        parts = coord_str.split(",")
        if len(parts) < 2:
            raise GeofenceReadError(
                f"Invalid coordinate format at index {i}: '{coord_str}'. "
                "Expected 'longitude,latitude' or 'longitude,latitude,altitude'"
            )
        try:
            point = GeofencePoint(
                longitude=float(parts[0]),
                latitude=float(parts[1])
            )
            polygon.append(point)
        except ValueError as e:
            raise GeofenceReadError(
                f"Invalid coordinate values at index {i}: '{coord_str}'"
            ) from e

    if len(polygon) < 3:
        raise InvalidPolygonError(
            f"Polygon must have at least 3 vertices, got {len(polygon)}"
        )

    logger.debug(f"Loaded geofence with {len(polygon)} vertices from {path}")
    return polygon


def validate_polygon(polygon: Polygon) -> None:
    """
    Validate that a polygon is valid for geofence operations.

    Args:
        polygon: The polygon to validate

    Raises:
        InvalidPolygonError: If the polygon is invalid
    """
    if len(polygon) < 3:
        raise InvalidPolygonError(
            f"Polygon must have at least 3 vertices, got {len(polygon)}"
        )


def is_inside_polygon(point: PointLike, polygon: Polygon) -> bool:
    """
    Check if a point is inside a polygon using ray-casting algorithm.

    Based on https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html

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
    validate_polygon(polygon)

    lon, lat = _extract_coords(point)

    inside = False
    n = len(polygon)
    j = n - 1

    for i in range(n):
        lon_i = polygon[i].longitude
        lat_i = polygon[i].latitude
        lon_j = polygon[j].longitude
        lat_j = polygon[j].latitude

        # Avoid division by zero
        if abs(lat_j - lat_i) < _FLOAT_TOLERANCE:
            j = i
            continue

        intersect = ((lat_i > lat) != (lat_j > lat)) and (
            lon < (lon_j - lon_i) * (lat - lat_i) / (lat_j - lat_i) + lon_i
        )
        if intersect:
            inside = not inside
        j = i

    return inside


def _lies_on_segment(
    seg_start_x: float, seg_start_y: float,
    point_x: float, point_y: float,
    seg_end_x: float, seg_end_y: float
) -> bool:
    """
    Check if a point lies on a line segment.

    Args:
        seg_start_x, seg_start_y: Start point of the segment
        point_x, point_y: The point to check
        seg_end_x, seg_end_y: End point of the segment

    Returns:
        True if the point lies on the segment (within tolerance)
    """
    return (
        (point_x <= max(seg_start_x, seg_end_x) + _FLOAT_TOLERANCE) and
        (point_x >= min(seg_start_x, seg_end_x) - _FLOAT_TOLERANCE) and
        (point_y <= max(seg_start_y, seg_end_y) + _FLOAT_TOLERANCE) and
        (point_y >= min(seg_start_y, seg_end_y) - _FLOAT_TOLERANCE)
    )


def _orientation(
    px: float, py: float,
    qx: float, qy: float,
    rx: float, ry: float
) -> int:
    """
    Find the orientation of an ordered triplet (p, q, r).

    Returns:
        0: Collinear points
        1: Clockwise
        2: Counterclockwise
    """
    val = (qy - py) * (rx - qx) - (qx - px) * (ry - qy)

    # Use tolerance for collinearity check
    if abs(val) < _FLOAT_TOLERANCE:
        return 0  # Collinear
    return 1 if val > 0 else 2


def segments_intersect(
    p1: PointLike,
    q1: PointLike,
    p2: PointLike,
    q2: PointLike
) -> bool:
    """
    Check if line segment p1-q1 intersects with segment p2-q2.

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
    # Extract coordinates
    px, py = _extract_coords(p1)
    qx, qy = _extract_coords(q1)
    rx, ry = _extract_coords(p2)
    sx, sy = _extract_coords(q2)

    o1 = _orientation(px, py, qx, qy, rx, ry)
    o2 = _orientation(px, py, qx, qy, sx, sy)
    o3 = _orientation(rx, ry, sx, sy, px, py)
    o4 = _orientation(rx, ry, sx, sy, qx, qy)

    # General case
    if (o1 != o2) and (o3 != o4):
        return True

    # Special cases (collinear points)
    if o1 == 0 and _lies_on_segment(px, py, rx, ry, qx, qy):
        return True
    if o2 == 0 and _lies_on_segment(px, py, sx, sy, qx, qy):
        return True
    if o3 == 0 and _lies_on_segment(rx, ry, px, py, sx, sy):
        return True
    if o4 == 0 and _lies_on_segment(rx, ry, qx, qy, sx, sy):
        return True

    return False


def path_crosses_polygon(
    start: PointLike,
    end: PointLike,
    polygon: Polygon,
    *,
    closed: bool = True
) -> bool:
    """
    Check if a path from start to end crosses any edge of a polygon.

    Args:
        start: Starting coordinate of the path
        end: Ending coordinate of the path
        polygon: A sequence of GeofencePoint defining the polygon
        closed: If True, also check the edge from last point to first point

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
    validate_polygon(polygon)

    n = len(polygon)

    # Check all edges including the closing edge if closed=True
    edges_to_check = n if closed else n - 1

    for i in range(edges_to_check):
        j = (i + 1) % n  # Wraps around for closing edge
        if segments_intersect(start, end, polygon[i], polygon[j]):
            return True

    return False


def is_path_valid(
    start: PointLike,
    end: PointLike,
    include_fence: Polygon,
    exclude_fences: Sequence[Polygon] = ()
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
    # Check start and end are inside include fence
    if not is_inside_polygon(start, include_fence):
        return False
    if not is_inside_polygon(end, include_fence):
        return False

    # Check path doesn't cross include fence
    if path_crosses_polygon(start, end, include_fence):
        return False

    # Check path doesn't enter any exclude fence
    for fence in exclude_fences:
        # Path is invalid if start or end is in an exclude zone
        if is_inside_polygon(start, fence):
            return False
        if is_inside_polygon(end, fence):
            return False
        # Path is invalid if it crosses into an exclude zone
        if path_crosses_polygon(start, end, fence):
            return False

    return True


def polygon_area(polygon: Polygon) -> float:
    """
    Calculate the area of a polygon using the Shoelace formula.

    Note: This returns the area in square degrees, which is only useful for
    relative comparisons. For actual area in square meters, you need to
    account for the Earth's curvature.

    Args:
        polygon: A sequence of GeofencePoint defining the polygon

    Returns:
        The absolute area of the polygon in square degrees.

    Raises:
        InvalidPolygonError: If the polygon has fewer than 3 vertices.
    """
    validate_polygon(polygon)

    n = len(polygon)
    area = 0.0

    for i in range(n):
        j = (i + 1) % n
        area += polygon[i].longitude * polygon[j].latitude
        area -= polygon[j].longitude * polygon[i].latitude

    return abs(area) / 2.0


def polygon_centroid(polygon: Polygon) -> GeofencePoint:
    """
    Calculate the centroid (center of mass) of a polygon.

    Args:
        polygon: A sequence of GeofencePoint defining the polygon

    Returns:
        The centroid as a GeofencePoint.

    Raises:
        InvalidPolygonError: If the polygon has fewer than 3 vertices.
    """
    validate_polygon(polygon)

    n = len(polygon)
    cx = 0.0
    cy = 0.0
    signed_area = 0.0

    for i in range(n):
        j = (i + 1) % n
        cross = (polygon[i].longitude * polygon[j].latitude -
                 polygon[j].longitude * polygon[i].latitude)
        signed_area += cross
        cx += (polygon[i].longitude + polygon[j].longitude) * cross
        cy += (polygon[i].latitude + polygon[j].latitude) * cross

    signed_area *= 0.5

    if abs(signed_area) < _FLOAT_TOLERANCE:
        # Degenerate polygon, return average of vertices
        avg_lon = sum(p.longitude for p in polygon) / n
        avg_lat = sum(p.latitude for p in polygon) / n
        return GeofencePoint(longitude=avg_lon, latitude=avg_lat)

    cx /= (6.0 * signed_area)
    cy /= (6.0 * signed_area)

    return GeofencePoint(longitude=cx, latitude=cy)


def closest_point_on_segment(
    point: PointLike,
    seg_start: PointLike,
    seg_end: PointLike
) -> tuple[float, float]:
    """
    Find the closest point on a line segment to a given point.

    Args:
        point: The point to find the closest segment point to
        seg_start: Start of the line segment
        seg_end: End of the line segment

    Returns:
        Tuple of (longitude, latitude) of the closest point on the segment.
    """
    px, py = _extract_coords(point)
    ax, ay = _extract_coords(seg_start)
    bx, by = _extract_coords(seg_end)

    # Vector from a to b
    abx = bx - ax
    aby = by - ay

    # Vector from a to p
    apx = px - ax
    apy = py - ay

    # Project ap onto ab
    ab_squared = abx * abx + aby * aby

    if ab_squared < _FLOAT_TOLERANCE:
        # Segment is essentially a point
        return ax, ay

    t = (apx * abx + apy * aby) / ab_squared

    # Clamp t to [0, 1] to stay on segment
    t = max(0.0, min(1.0, t))

    return ax + t * abx, ay + t * aby


def distance_to_polygon_edge(point: PointLike, polygon: Polygon) -> float:
    """
    Calculate the minimum distance from a point to any edge of a polygon.

    Note: This returns distance in degrees. For actual distance in meters,
    you need to convert using an appropriate Earth model.

    Args:
        point: The point to measure from
        polygon: A sequence of GeofencePoint defining the polygon

    Returns:
        The minimum distance in degrees to any polygon edge.

    Raises:
        InvalidPolygonError: If the polygon has fewer than 3 vertices.
    """
    validate_polygon(polygon)

    px, py = _extract_coords(point)
    min_dist_sq = float('inf')
    n = len(polygon)

    for i in range(n):
        j = (i + 1) % n
        cx, cy = closest_point_on_segment(point, polygon[i], polygon[j])
        dist_sq = (px - cx) ** 2 + (py - cy) ** 2
        min_dist_sq = min(min_dist_sq, dist_sq)

    return min_dist_sq ** 0.5


def is_polygon_clockwise(polygon: Polygon) -> bool:
    """
    Determine if a polygon's vertices are ordered clockwise.

    Args:
        polygon: A sequence of GeofencePoint defining the polygon

    Returns:
        True if vertices are clockwise, False if counterclockwise.

    Raises:
        InvalidPolygonError: If the polygon has fewer than 3 vertices.
    """
    validate_polygon(polygon)

    # Use the signed area formula - negative = clockwise
    n = len(polygon)
    signed_area = 0.0

    for i in range(n):
        j = (i + 1) % n
        signed_area += polygon[i].longitude * polygon[j].latitude
        signed_area -= polygon[j].longitude * polygon[i].latitude

    return signed_area < 0


# Backward compatibility aliases with deprecation warnings
def readGeofence(file_path: str | Path) -> List[GeofencePoint]:
    """
    Deprecated: Use read_geofence instead.
    """
    import warnings
    warnings.warn(
        "readGeofence is deprecated, use read_geofence instead",
        DeprecationWarning,
        stacklevel=2
    )
    return read_geofence(file_path)


def inside(point: PointLike, polygon: Polygon) -> bool:
    """
    Deprecated: Use is_inside_polygon instead.
    """
    import warnings
    warnings.warn(
        "inside is deprecated, use is_inside_polygon instead",
        DeprecationWarning,
        stacklevel=2
    )
    return is_inside_polygon(point, polygon)


def doIntersect(
    p1_lon: float, p1_lat: float,
    q1_lon: float, q1_lat: float,
    p2_lon: float, p2_lat: float,
    q2_lon: float, q2_lat: float
) -> bool:
    """
    Deprecated: Use segments_intersect instead.

    Check if segment (p1_lon,p1_lat)-(q1_lon,q1_lat) intersects with
    segment (p2_lon,p2_lat)-(q2_lon,q2_lat).
    """
    import warnings
    warnings.warn(
        "doIntersect is deprecated, use segments_intersect instead",
        DeprecationWarning,
        stacklevel=2
    )
    p1 = GeofencePoint(longitude=p1_lon, latitude=p1_lat)
    q1 = GeofencePoint(longitude=q1_lon, latitude=q1_lat)
    p2 = GeofencePoint(longitude=p2_lon, latitude=p2_lat)
    q2 = GeofencePoint(longitude=q2_lon, latitude=q2_lat)
    return segments_intersect(p1, q1, p2, q2)


__all__ = [
    # Types
    "GeofencePoint",
    "Polygon",
    "PointLike",
    "GeoPoint",
    # Exceptions
    "GeofenceError",
    "GeofenceReadError",
    "InvalidPolygonError",
    # Core functions
    "read_geofence",
    "validate_polygon",
    "is_inside_polygon",
    "segments_intersect",
    "path_crosses_polygon",
    "is_path_valid",
    # Utility functions
    "polygon_area",
    "polygon_centroid",
    "closest_point_on_segment",
    "distance_to_polygon_edge",
    "is_polygon_clockwise",
    # Backward compatibility (deprecated)
    "readGeofence",
    "inside",
    "doIntersect",
]

