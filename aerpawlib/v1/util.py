"""
Types and functions commonly used throughout the aerpawlib v1 framework.

This version has been updated to remove DroneKit dependencies while
maintaining API compatibility.

@author: Julian Reder (quantumbagel)
"""

import json
import math
from typing import List, Tuple

from pykml import parser


class VectorNED:
    """
    Representation of a difference between two coordinates (used for expressing
    relative motion). Makes use of NED (north, east, down) scheme.

    Units are expressed in meters
    """

    north: float
    east: float
    down: float

    def __init__(self, north: float, east: float, down: float = 0):
        self.north = north
        self.east = east
        self.down = down

    def rotate_by_angle(self, angle: float):
        """
        Transform this VectorNED and rotate it by a certain angle, provided in
        degrees.

        ex: VectorNED(1, 0, 0).rotate_by_angle(90) -> VectorNed(0, -1, 0)
        ex: VectorNED(1, 0, 0).rotate_by_angle(45) -> VectorNed(0.707, -0.707, 0)
        """
        rads = angle / 180 * math.pi

        east = self.east * math.cos(rads) - self.north * math.sin(rads)
        north = self.east * math.sin(rads) + self.north * math.cos(rads)

        return VectorNED(north, east, self.down)

    def cross_product(self, o):
        """
        find the cross product of this and the other vector (this x o)
        """
        if not isinstance(o, VectorNED):
            raise TypeError()
        return VectorNED(
            self.east * o.down + self.down * o.east,
            self.down * o.north - self.north * o.down,
            self.north * o.east - self.east * o.north,
        )

    def hypot(self, ignore_down: bool = False):
        """
        find the distance of this VectorNED, optionally ignoring any changes in
        height
        """
        if ignore_down:
            return math.hypot(self.north, self.east)
        else:
            return math.sqrt(self.north**2 + self.east**2 + self.down**2)

    def norm(self):
        """
        returns a normalized version of this vector in 3d space, with a magnitude
        equal to 1

        if the zero vector, returns the zero vector
        """
        hypot = self.hypot()
        if hypot == 0:
            return VectorNED(0, 0, 0)
        return (1 / hypot) * self

    def __add__(self, o):
        if not isinstance(o, VectorNED):
            raise TypeError()
        return VectorNED(
            self.north + o.north, self.east + o.east, self.down + o.down
        )

    def __sub__(self, o):
        if not isinstance(o, VectorNED):
            raise TypeError()
        return VectorNED(
            self.north - o.north, self.east - o.east, self.down - o.down
        )

    def __mul__(self, o):
        if not (isinstance(o, float) or isinstance(o, int)):
            raise TypeError()
        return VectorNED(self.north * o, self.east * o, self.down * o)

    __rmul__ = __mul__

    def __str__(self) -> str:
        return (
            "(" + ",".join(map(str, [self.north, self.east, self.down])) + ")"
        )


class Coordinate:
    """
    An absolute point in space making use of lat, lon, and an altitude (over
    the home location of the vehicle).

    `lat`/`lon` should be expressed in *degrees*, while `alt` is in *meters*
    (relative to the takeoff/home location)

    `Coordinate`s can be added or subtracted with `VectorNED`s using Python's
    respective operators to calculate a new `Coordinate` relative to the
    original.

    `Coordinate`s can be subtracted from each other using Python's subtraction
    operator to calculate the vector difference between them, which is returned
    as a `VectorNED`
    """

    lat: float
    lon: float
    alt: float

    def __init__(self, lat: float, lon: float, alt: float = 0):
        self.lat = lat
        self.lon = lon
        self.alt = alt

    def ground_distance(self, other) -> float:
        """
        Get the ground distance (in meters) between this `Coordinate` and
        another `Coordinate`.

        Makes use of `Coordinate.distance` under the hood
        """
        if not isinstance(other, Coordinate):
            raise TypeError()

        other = Coordinate(other.lat, other.lon, self.alt)
        return self.distance(other)

    def distance(self, other) -> float:
        """
        Get the true distance (in meters) between this `Coordinate` and another
        `Coordinate`. Unlike `Coordinate.ground_distance`, this function also
        takes the altitude into account.

        The implementation used here makes use of Haversine Distance -- this
        should be extremely accurate (max err < 3m) for any distances used
        within the scope of the AERPAW program. (<10km)
        """
        if not isinstance(other, Coordinate):
            raise TypeError()

        # calculation uses haversine distance
        d2r = math.pi / 180
        dlon = (other.lon - self.lon) * d2r
        dlat = (other.lat - self.lat) * d2r
        a = math.pow(math.sin(dlat / 2), 2) + math.cos(
            self.lat * d2r
        ) * math.cos(other.lat * d2r) * math.pow(math.sin(dlon / 2), 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        d = 6367 * c
        return math.hypot(d * 1000, other.alt - self.alt)

    def bearing(self, other, wrap_360: bool = True) -> float:
        """
        Calculate the bearing (angle) between two `Coordinates`, and return it
        in degrees
        """
        if not isinstance(other, Coordinate):
            raise TypeError()

        d_lat = other.lat - self.lat
        d_lon = other.lon - self.lon
        bearing = 90 + math.atan2(-d_lat, d_lon) * 57.2957795
        if wrap_360:
            bearing %= 360
        return bearing

    def __add__(self, o):
        if isinstance(o, VectorNED):
            north = o.north
            east = o.east
            alt = -o.down
        else:
            raise TypeError()

        earth_radius = 6378137.0
        d_lat = north / earth_radius
        d_lon = east / (earth_radius * math.cos(math.pi * self.lat / 180))
        new_lat = self.lat + (d_lat * 180 / math.pi)
        new_lon = self.lon + (d_lon * 180 / math.pi)

        return Coordinate(new_lat, new_lon, self.alt + alt)

    def __sub__(self, o):
        if isinstance(o, VectorNED):
            return self + VectorNED(-o.north, -o.east, -o.down)
        elif isinstance(o, Coordinate):
            lat_mid = (self.lat + o.lat) * math.pi / 360

            d_lat = self.lat - o.lat
            d_lon = self.lon - o.lon

            return VectorNED(
                d_lat
                * (
                    111132.954
                    - 559.822 * math.cos(2 * lat_mid)
                    + 1.175 * math.cos(4 * lat_mid)
                ),
                d_lon * (111132.954 * math.cos(lat_mid)),
                o.alt - self.alt,
            )
        else:
            raise TypeError()

    def __str__(self):
        return f"({self.lat},{self.lon},{self.alt})"

    def toJson(self):
        """Return a JSON string representing this Coordinate object"""
        return json.dumps(self, default=lambda o: o.__dict__)


# Waypoint type alias
Waypoint = Tuple[
    int, float, float, float, int, float
]  # command, x, y, z, waypoint_id, speed

_DEFAULT_WAYPOINT_SPEED = 5  # m/s

_PLAN_CMD_TAKEOFF = 22
_PLAN_CMD_WAYPOINT = 16
_PLAN_CMD_RTL = 20
_PLAN_CMD_SPEED = 178


def read_from_plan(
    path: str, default_speed: float = _DEFAULT_WAYPOINT_SPEED
) -> List[Waypoint]:
    """
    Helper function to read a provided .plan file (passed in as `path`) into a
    list of `Waypoint`s that can then be used to run waypoint-based missions.

    This function has really only been tested with `.plan` files generated by
    QGroundControl.
    """
    waypoints = []
    with open(path) as f:
        data = json.load(f)
    if data["fileType"] != "Plan":
        raise Exception("Wrong file type -- use a .plan file.")
    current_speed = default_speed
    for item in data["mission"]["items"]:
        command = item["command"]
        if command in [_PLAN_CMD_TAKEOFF, _PLAN_CMD_WAYPOINT, _PLAN_CMD_RTL]:
            x, y, z = item["params"][4:7]
            waypoint_id = item["doJumpId"]
            waypoints.append((command, x, y, z, waypoint_id, current_speed))
        elif command in [_PLAN_CMD_SPEED]:
            current_speed = item["params"][1]
    return waypoints


def get_location_from_waypoint(waypoint: Waypoint) -> Coordinate:
    """
    Helper to get a Coordinate from a `Waypoint`
    """
    return Coordinate(waypoint[1], waypoint[2], waypoint[3])


def read_from_plan_complete(
    path: str, default_speed: float = _DEFAULT_WAYPOINT_SPEED
):
    """
    Helper to read from a .plan file and gather all fields from each waypoint

    This can then be used for more advanced .plan file based missions
    """
    waypoints = []
    with open(path) as f:
        data = json.load(f)
    if data["fileType"] != "Plan":
        raise Exception("Wrong file type -- use a .plan file.")
    current_speed = default_speed
    for item in data["mission"]["items"]:
        command = item["command"]
        if command in [_PLAN_CMD_SPEED]:
            current_speed = item["params"][1]
        elif command in [_PLAN_CMD_TAKEOFF, _PLAN_CMD_WAYPOINT, _PLAN_CMD_RTL]:
            x, y, z = item["params"][4:7]
            waypoint_id = item["doJumpId"]
            delay = item["params"][0]
            waypoints.append(
                {
                    "id": waypoint_id,
                    "command": command,
                    "pos": [x, y, z],
                    "wait_for": delay,
                    "speed": current_speed,
                }
            )
    return waypoints


def readGeofence(filePath):
    """
    Reads geofence kml file to create array of coordinates representing geofence
    """
    root = parser.fromstring(open(filePath, "rb").read())
    coordinates_string = (
        root.Document.Placemark.Polygon.outerBoundaryIs.LinearRing.coordinates.text
    )
    coordinates_list = coordinates_string.split()
    polygon = []
    for str in coordinates_list:
        point = {
            "lon": float(str.split(",")[0]),
            "lat": float(str.split(",")[1]),
        }
        polygon.append(point)
    return polygon


def inside(lon, lat, geofence):
    """
    ray-casting algorithm based on
    https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
    """
    inside = False
    i = 0
    j = len(geofence) - 1

    while i < len(geofence):
        loni = geofence[i]["lon"]
        lati = geofence[i]["lat"]
        lonj = geofence[j]["lon"]
        latj = geofence[j]["lat"]

        intersect = ((lati > lat) != (latj > lat)) and (
            lon < (lonj - loni) * (lat - lati) / (latj - lati) + loni
        )
        if intersect:
            inside = not inside
        j = i
        i += 1

    return inside


def liesOnSegment(px, py, qx, qy, rx, ry):
    if (
        (qx <= max(px, rx))
        and (qx >= min(px, rx))
        and (qy <= max(py, ry))
        and (qy >= min(py, ry))
    ):
        return True
    return False


def orientation(px, py, qx, qy, rx, ry):
    """
    Find the orientation of an ordered triplet (p,q,r)
    Returns:
    0 : Colinear points
    1 : Clockwise points
    2 : Counterclockwise
    """
    val = (float(qy - py) * (rx - qx)) - (float(qx - px) * (ry - qy))
    if val > 0:
        return 1  # Clockwise
    elif val < 0:
        return 2  # Counterclockwise
    else:
        return 0  # Colinear


def doIntersect(px, py, qx, qy, rx, ry, sx, sy):
    """
    Returns true if segment pq intersects with rs. Else false.
    """
    o1 = orientation(px, py, qx, qy, rx, ry)
    o2 = orientation(px, py, qx, qy, sx, sy)
    o3 = orientation(rx, ry, sx, sy, px, py)
    o4 = orientation(rx, ry, sx, sy, qx, qy)

    # General case
    if (o1 != o2) and (o3 != o4):
        return True

    # Special Cases
    if (o1 == 0) and liesOnSegment(px, py, rx, ry, qx, qy):
        return True
    if (o2 == 0) and liesOnSegment(px, py, sx, sy, qx, qy):
        return True
    if (o3 == 0) and liesOnSegment(rx, ry, px, py, sx, sy):
        return True
    if (o4 == 0) and liesOnSegment(rx, ry, qx, qy, sx, sy):
        return True

    return False
