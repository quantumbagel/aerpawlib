"""
Core types for aerpawlib v2 API.

This module provides data classes for representing vehicle state,
coordinates, and telemetry information.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Optional, List
from enum import Enum, auto


class FlightMode(Enum):
    """Flight modes available for vehicles."""

    MANUAL = auto()
    STABILIZED = auto()
    ALTITUDE = auto()
    POSITION = auto()
    OFFBOARD = auto()
    HOLD = auto()
    MISSION = auto()
    RETURN_TO_LAUNCH = auto()
    LAND = auto()
    TAKEOFF = auto()
    FOLLOW_ME = auto()
    UNKNOWN = auto()


class LandedState(Enum):
    """Landed state of the vehicle."""

    UNKNOWN = auto()
    ON_GROUND = auto()
    IN_AIR = auto()
    TAKING_OFF = auto()
    LANDING = auto()


@dataclass
class VectorNED:
    """
    Representation of a difference between two coordinates (used for expressing
    relative motion). Makes use of NED (north, east, down) scheme.

    Units are expressed in meters.
    """

    north: float = 0.0
    east: float = 0.0
    down: float = 0.0

    def rotate_by_angle(self, angle: float) -> VectorNED:
        """
        Transform this VectorNED and rotate it by a certain angle, provided in degrees.

        Args:
            angle: The angle to rotate by in degrees

        Returns:
            A new rotated VectorNED

        Examples:
            >>> VectorNED(1, 0, 0).rotate_by_angle(90)
            VectorNED(north=0.0, east=-1.0, down=0.0)
        """
        rads = math.radians(angle)
        east = self.east * math.cos(rads) - self.north * math.sin(rads)
        north = self.east * math.sin(rads) + self.north * math.cos(rads)
        return VectorNED(north, east, self.down)

    def cross_product(self, other: VectorNED) -> VectorNED:
        """Find the cross product of this and another vector (self x other)."""
        if not isinstance(other, VectorNED):
            raise TypeError(
                "Can only compute cross product with another VectorNED"
            )
        return VectorNED(
            self.east * other.down + self.down * other.east,
            self.down * other.north - self.north * other.down,
            self.north * other.east - self.east * other.north,
        )

    def dot_product(self, other: VectorNED) -> float:
        """Compute the dot product of this and another vector."""
        if not isinstance(other, VectorNED):
            raise TypeError(
                "Can only compute dot product with another VectorNED"
            )
        return (
            self.north * other.north
            + self.east * other.east
            + self.down * other.down
        )

    def magnitude(self, ignore_vertical: bool = False) -> float:
        """
        Get the magnitude (length) of this vector.

        Args:
            ignore_vertical: If True, only considers horizontal components

        Returns:
            The magnitude in meters
        """
        if ignore_vertical:
            return math.hypot(self.north, self.east)
        return math.sqrt(self.north**2 + self.east**2 + self.down**2)

    def normalize(self) -> VectorNED:
        """
        Returns a normalized version of this vector with magnitude 1.
        Returns zero vector if input is zero.
        """
        mag = self.magnitude()
        if mag == 0:
            return VectorNED(0, 0, 0)
        return VectorNED(self.north / mag, self.east / mag, self.down / mag)

    def heading(self) -> float:
        """Get the compass heading of this vector in degrees (0-360, north=0)."""
        return (90 - math.degrees(math.atan2(self.north, self.east))) % 360

    def __add__(self, other: VectorNED) -> VectorNED:
        if not isinstance(other, VectorNED):
            raise TypeError("Can only add VectorNED to VectorNED")
        return VectorNED(
            self.north + other.north,
            self.east + other.east,
            self.down + other.down,
        )

    def __sub__(self, other: VectorNED) -> VectorNED:
        if not isinstance(other, VectorNED):
            raise TypeError("Can only subtract VectorNED from VectorNED")
        return VectorNED(
            self.north - other.north,
            self.east - other.east,
            self.down - other.down,
        )

    def __mul__(self, scalar: float) -> VectorNED:
        if not isinstance(scalar, (int, float)):
            raise TypeError("Can only multiply VectorNED by a scalar")
        return VectorNED(
            self.north * scalar, self.east * scalar, self.down * scalar
        )

    def __rmul__(self, scalar: float) -> VectorNED:
        return self.__mul__(scalar)

    def __neg__(self) -> VectorNED:
        return VectorNED(-self.north, -self.east, -self.down)

    def __repr__(self) -> str:
        return f"VectorNED(north={self.north}, east={self.east}, down={self.down})"


# Alias for clarity in position contexts
@dataclass
class PositionNED:
    """
    Position in NED frame relative to a reference point (usually home).

    This is functionally identical to VectorNED but provides clearer semantics
    when representing absolute positions rather than relative movements.
    """

    north: float = 0.0
    east: float = 0.0
    down: float = 0.0

    def to_vector(self) -> VectorNED:
        """Convert to VectorNED for math operations."""
        return VectorNED(self.north, self.east, self.down)

    @classmethod
    def from_vector(cls, vector: VectorNED) -> PositionNED:
        """Create from a VectorNED."""
        return cls(vector.north, vector.east, vector.down)


@dataclass
class Coordinate:
    """
    An absolute point in space using latitude, longitude, and altitude.

    This coordinate system uses:
    - `latitude`: degrees (-90 to 90)
    - `longitude`: degrees (-180 to 180)
    - `altitude`: meters relative to home/takeoff location
    - `name`: optional identifier for the coordinate

    Coordinates support arithmetic with VectorNED for relative movement calculations.
    """

    latitude: float
    longitude: float
    altitude: float = 0.0
    name: Optional[str] = None

    # Aliases for convenience
    @property
    def lat(self) -> float:
        """Alias for latitude."""
        return self.latitude

    @lat.setter
    def lat(self, value: float):
        self.latitude = value

    @property
    def lon(self) -> float:
        """Alias for longitude."""
        return self.longitude

    @lon.setter
    def lon(self, value: float):
        self.longitude = value

    @property
    def alt(self) -> float:
        """Alias for altitude."""
        return self.altitude

    @alt.setter
    def alt(self, value: float):
        self.altitude = value

    def distance_to(self, other: Coordinate) -> float:
        """
        Get the 3D distance to another Coordinate in meters.

        Uses Haversine formula for ground distance calculation.
        Accurate to within 3m for distances under 10km.
        """
        if not isinstance(other, Coordinate):
            raise TypeError("Can only compute distance to another Coordinate")

        d2r = math.pi / 180
        dlon = (other.longitude - self.longitude) * d2r
        dlat = (other.latitude - self.latitude) * d2r
        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(self.latitude * d2r)
            * math.cos(other.latitude * d2r)
            * math.sin(dlon / 2) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        ground_distance = 6367000 * c  # Earth radius in meters
        return math.hypot(ground_distance, other.altitude - self.altitude)

    def ground_distance_to(self, other: Coordinate) -> float:
        """Get the ground distance (ignoring altitude) to another Coordinate in meters."""
        if not isinstance(other, Coordinate):
            raise TypeError("Can only compute distance to another Coordinate")
        other_at_same_alt = Coordinate(
            other.latitude, other.longitude, self.altitude
        )
        return self.distance_to(other_at_same_alt)

    def bearing_to(self, other: Coordinate, wrap_360: bool = True) -> float:
        """
        Calculate the bearing (heading) to another Coordinate in degrees.

        Args:
            other: Target coordinate
            wrap_360: If True, returns value in 0-360 range

        Returns:
            Bearing in degrees (0 = North, 90 = East)
        """
        if not isinstance(other, Coordinate):
            raise TypeError("Can only compute bearing to another Coordinate")

        d_lat = other.latitude - self.latitude
        d_lon = other.longitude - self.longitude
        bearing = 90 + math.degrees(math.atan2(-d_lat, d_lon))
        if wrap_360:
            bearing %= 360
        return bearing

    def offset_by(self, vector: VectorNED) -> Coordinate:
        """
        Create a new Coordinate offset by the given vector.

        Args:
            vector: The NED vector to offset by

        Returns:
            A new Coordinate at the offset position
        """
        earth_radius = 6378137.0
        d_lat = vector.north / earth_radius
        d_lon = vector.east / (
            earth_radius * math.cos(math.radians(self.latitude))
        )
        new_lat = self.latitude + math.degrees(d_lat)
        new_lon = self.longitude + math.degrees(d_lon)
        return Coordinate(
            new_lat, new_lon, self.altitude - vector.down, self.name
        )

    def vector_to(self, other: Coordinate) -> VectorNED:
        """
        Get the NED vector from this coordinate to another.

        Args:
            other: Target coordinate

        Returns:
            VectorNED representing the offset
        """
        if not isinstance(other, Coordinate):
            raise TypeError("Can only compute vector to another Coordinate")

        lat_mid = math.radians((self.latitude + other.latitude) / 2)
        d_lat = other.latitude - self.latitude
        d_lon = other.longitude - self.longitude

        north = d_lat * (
            111132.954
            - 559.822 * math.cos(2 * lat_mid)
            + 1.175 * math.cos(4 * lat_mid)
        )
        east = d_lon * (111132.954 * math.cos(lat_mid))
        down = self.altitude - other.altitude

        return VectorNED(north, east, down)

    def __add__(self, vector: VectorNED) -> Coordinate:
        if isinstance(vector, VectorNED):
            return self.offset_by(vector)
        raise TypeError("Can only add VectorNED to Coordinate")

    def __sub__(self, other) -> VectorNED | Coordinate:
        if isinstance(other, VectorNED):
            return self.offset_by(-other)
        elif isinstance(other, Coordinate):
            return self.vector_to(other)
        raise TypeError(
            "Can only subtract VectorNED or Coordinate from Coordinate"
        )

    def to_json(self) -> str:
        """Return a JSON string representing this Coordinate."""
        return json.dumps(
            {
                "latitude": self.latitude,
                "longitude": self.longitude,
                "altitude": self.altitude,
                "name": self.name,
            }
        )

    @classmethod
    def from_json(cls, json_str: str) -> Coordinate:
        """Create a Coordinate from a JSON string."""
        data = json.loads(json_str)
        return cls(
            latitude=data["latitude"],
            longitude=data["longitude"],
            altitude=data.get("altitude", 0.0),
            name=data.get("name"),
        )

    @classmethod
    def from_relative(
        cls, base: Coordinate, vector: VectorNED, name: Optional[str] = None
    ) -> Coordinate:
        """
        Create a new Coordinate relative to a base coordinate.

        Args:
            base: The reference coordinate
            vector: NED offset from the base
            name: Optional name for the new coordinate

        Returns:
            New Coordinate at the offset position

        Example:
            >>> home = Coordinate(35.7275, -78.6960, 0, "Home")
            >>> waypoint = Coordinate.from_relative(home, VectorNED(100, 50, -10), "WP1")
        """
        result = base.offset_by(vector)
        if name:
            result = Coordinate(
                result.latitude, result.longitude, result.altitude, name
            )
        return result

    def midpoint_to(
        self, other: Coordinate, name: Optional[str] = None
    ) -> Coordinate:
        """
        Calculate the midpoint between this coordinate and another.

        Args:
            other: The other coordinate
            name: Optional name for the midpoint

        Returns:
            Coordinate at the midpoint

        Example:
            >>> start = Coordinate(35.7275, -78.6960, 10)
            >>> end = Coordinate(35.7285, -78.6950, 20)
            >>> mid = start.midpoint_to(end, "Midpoint")
        """
        return self.interpolate_to(other, 0.5, name)

    def interpolate_to(
        self, other: Coordinate, fraction: float, name: Optional[str] = None
    ) -> Coordinate:
        """
        Interpolate between this coordinate and another.

        Args:
            other: The target coordinate
            fraction: Interpolation factor (0.0 = this, 1.0 = other)
            name: Optional name for the result

        Returns:
            Coordinate at the interpolated position

        Example:
            >>> start = Coordinate(35.7275, -78.6960, 10)
            >>> end = Coordinate(35.7285, -78.6950, 20)
            >>> quarter = start.interpolate_to(end, 0.25)  # 25% of the way
        """
        if not isinstance(other, Coordinate):
            raise TypeError("Can only interpolate to another Coordinate")

        lat = self.latitude + (other.latitude - self.latitude) * fraction
        lon = self.longitude + (other.longitude - self.longitude) * fraction
        alt = self.altitude + (other.altitude - self.altitude) * fraction
        return Coordinate(lat, lon, alt, name)

    def path_to(
        self,
        other: Coordinate,
        num_points: int,
        include_endpoints: bool = True,
    ) -> List[Coordinate]:
        """
        Generate a path of evenly spaced points to another coordinate.

        Args:
            other: The target coordinate
            num_points: Number of points to generate
            include_endpoints: If True, include start and end points

        Returns:
            List of Coordinates along the path

        Example:
            >>> start = Coordinate(35.7275, -78.6960, 10, "Start")
            >>> end = Coordinate(35.7285, -78.6950, 10, "End")
            >>> path = start.path_to(end, 5)  # Returns 5 points including start/end
        """
        if num_points < 2:
            raise ValueError("num_points must be at least 2")

        if not isinstance(other, Coordinate):
            raise TypeError("Can only create path to another Coordinate")

        points = []
        if include_endpoints:
            for i in range(num_points):
                fraction = i / (num_points - 1)
                points.append(self.interpolate_to(other, fraction))
        else:
            for i in range(1, num_points + 1):
                fraction = i / (num_points + 1)
                points.append(self.interpolate_to(other, fraction))

        return points

    def with_altitude(self, altitude: float) -> Coordinate:
        """
        Return a copy of this coordinate with a different altitude.

        Args:
            altitude: New altitude in meters

        Returns:
            New Coordinate with the specified altitude
        """
        return Coordinate(self.latitude, self.longitude, altitude, self.name)

    def with_name(self, name: str) -> Coordinate:
        """
        Return a copy of this coordinate with a different name.

        Args:
            name: New name for the coordinate

        Returns:
            New Coordinate with the specified name
        """
        return Coordinate(self.latitude, self.longitude, self.altitude, name)

    def __repr__(self) -> str:
        name_str = f", name='{self.name}'" if self.name else ""
        return f"Coordinate(lat={self.latitude}, lon={self.longitude}, alt={self.altitude}{name_str})"


@dataclass
class Attitude:
    """
    Vehicle attitude in radians.

    - roll: rotation around forward axis (positive = right wing down)
    - pitch: rotation around lateral axis (positive = nose up)
    - yaw: rotation around vertical axis (positive = clockwise from above, 0 = North)
    """

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    @property
    def roll_degrees(self) -> float:
        """Roll in degrees."""
        return math.degrees(self.roll)

    @property
    def pitch_degrees(self) -> float:
        """Pitch in degrees."""
        return math.degrees(self.pitch)

    @property
    def yaw_degrees(self) -> float:
        """Yaw in degrees."""
        return math.degrees(self.yaw)

    @property
    def heading(self) -> float:
        """Compass heading in degrees (0-360)."""
        return self.yaw_degrees % 360


@dataclass
class GPSInfo:
    """GPS status information."""

    satellites: int = 0
    fix_type: int = 0  # 0-1: no fix, 2: 2D fix, 3: 3D fix

    @property
    def quality(self) -> str:
        """Human-readable GPS quality description."""
        if self.fix_type == 0:
            return "No GPS"
        elif self.fix_type == 1:
            return "No Fix"
        elif self.fix_type == 2:
            return "2D Fix"
        elif self.fix_type == 3:
            return "3D Fix"
        elif self.fix_type == 4:
            return "DGPS"
        elif self.fix_type == 5:
            return "RTK Float"
        elif self.fix_type == 6:
            return "RTK Fixed"
        return "Unknown"

    @property
    def has_fix(self) -> bool:
        """True if GPS has at least a 2D fix."""
        return self.fix_type >= 2


@dataclass
class BatteryInfo:
    """Battery status information."""

    id: int = 0
    voltage: float = 0.0  # Volts
    current: float = 0.0  # Amperes
    charge: float = 0.0  # 0.0 to 1.0 (decimal percentage)
    temperature: Optional[float] = None  # Celsius
    consumption: float = 0.0  # Ampere-hours consumed

    @property
    def percentage(self) -> float:
        """Battery percentage (0-100)."""
        return self.charge * 100


@dataclass
class DroneInfo:
    """Static vehicle information."""

    hardware_uuid: str = ""
    legacy_uuid: str = ""
    vendor_id: int = 0
    vendor_name: str = ""
    product_id: int = 0
    product_name: str = ""
    version: str = ""


@dataclass
class FlightInfo:
    """Flight timing information."""

    time_since_boot_ms: int = 0
    flight_id: str = ""
    time_since_arm_ms: int = 0
    time_since_takeoff_ms: int = 0


@dataclass
class DroneState:
    """
    Complete snapshot of drone state.

    This provides a clean interface to access all vehicle telemetry through
    organized properties.
    """

    heading: float = 0.0  # degrees, 0 = North
    velocity: VectorNED = field(default_factory=VectorNED)
    attitude: Attitude = field(default_factory=Attitude)
    altitude: float = 0.0  # absolute altitude in meters
    relative_altitude: float = 0.0  # altitude above home in meters
    latitude: float = 0.0
    longitude: float = 0.0
    groundspeed: float = 0.0  # m/s
    airspeed: float = 0.0  # m/s
    climb_rate: float = 0.0  # m/s (positive = ascending)
    flight_mode: FlightMode = FlightMode.UNKNOWN
    landed_state: LandedState = LandedState.UNKNOWN

    @property
    def position(self) -> Coordinate:
        """Current position as a Coordinate."""
        return Coordinate(
            self.latitude, self.longitude, self.relative_altitude
        )

    @property
    def speed(self) -> float:
        """3D speed in m/s."""
        return self.velocity.magnitude()

    @property
    def horizontal_speed(self) -> float:
        """Horizontal speed in m/s."""
        return self.velocity.magnitude(ignore_vertical=True)


@dataclass
class Waypoint:
    """
    A waypoint for mission planning.

    Includes position, speed, and hold time configuration.
    """

    coordinate: Coordinate
    speed: float = 5.0  # m/s
    hold_time: float = 0.0  # seconds to hold at waypoint
    acceptance_radius: float = 2.0  # meters
    pass_through: bool = False  # if True, don't stop at waypoint

    def __repr__(self) -> str:
        return f"Waypoint({self.coordinate}, speed={self.speed}m/s)"


def read_waypoints_from_plan(
    path: str, default_speed: float = 5.0
) -> List[Waypoint]:
    """
    Read waypoints from a QGroundControl .plan file.

    Args:
        path: Path to the .plan file
        default_speed: Default speed if not specified in file

    Returns:
        List of Waypoint objects
    """
    _PLAN_CMD_TAKEOFF = 22
    _PLAN_CMD_WAYPOINT = 16
    _PLAN_CMD_RTL = 20
    _PLAN_CMD_SPEED = 178

    waypoints = []
    with open(path) as f:
        data = json.load(f)

    if data["fileType"] != "Plan":
        raise ValueError("Wrong file type -- use a .plan file.")

    current_speed = default_speed

    for item in data["mission"]["items"]:
        command = item["command"]

        if command == _PLAN_CMD_SPEED:
            current_speed = item["params"][1]
        elif command in [_PLAN_CMD_TAKEOFF, _PLAN_CMD_WAYPOINT, _PLAN_CMD_RTL]:
            lat, lon, alt = item["params"][4:7]
            hold_time = item["params"][0]
            waypoint = Waypoint(
                coordinate=Coordinate(lat, lon, alt),
                speed=current_speed,
                hold_time=hold_time,
            )
            waypoints.append(waypoint)

    return waypoints
