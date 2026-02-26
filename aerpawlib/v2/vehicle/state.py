"""
Vehicle state for aerpawlib v2.

Plain attributes updated by telemetry (no ThreadSafeValue).
"""

from __future__ import annotations

import math
import time
from typing import Optional

from ..types import Attitude, Battery, Coordinate, GPSInfo, VectorNED


class VehicleState:
    """Mutable state updated by telemetry subscriptions."""

    def __init__(self) -> None:
        # Position
        self._position_lat: float = 0.0
        self._position_lon: float = 0.0
        self._position_alt: float = 0.0
        self._position_abs_alt: float = 0.0
        # Velocity
        self._velocity_ned: VectorNED = VectorNED(0, 0, 0)
        # Attitude / heading
        self._attitude: Attitude = Attitude(0.0, 0.0, 0.0)
        self._heading_deg: float = 0.0
        # Battery
        self._battery: Battery = Battery(0.0, 0.0, 0)
        # GPS
        self._gps: GPSInfo = GPSInfo(0, 0)
        # Home
        self._home: Optional[Coordinate] = None
        self._home_abs_alt: float = 0.0
        # Armed / mode
        self._armed: bool = False
        self._armable: bool = False
        self._mode: str = "UNKNOWN"
        self._last_arm_time: float = 0.0
        self._armed_telemetry_received: bool = False

    @property
    def position(self) -> Coordinate:
        return Coordinate(
            self._position_lat, self._position_lon, self._position_alt
        )

    @property
    def home_coords(self) -> Optional[Coordinate]:
        return self._home

    @property
    def home_amsl(self) -> float:
        return self._home_abs_alt

    @property
    def velocity(self) -> VectorNED:
        return self._velocity_ned

    @property
    def heading(self) -> float:
        return self._heading_deg % 360

    @property
    def attitude(self) -> Attitude:
        return self._attitude

    @property
    def battery(self) -> Battery:
        return self._battery

    @property
    def gps(self) -> GPSInfo:
        return self._gps

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def armable(self) -> bool:
        return self._armable

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def last_arm_time(self) -> float:
        return self._last_arm_time

    def update_position(
        self, lat: float, lon: float, rel_alt: float, abs_alt: float
    ) -> None:
        self._position_lat = lat
        self._position_lon = lon
        self._position_alt = rel_alt
        self._position_abs_alt = abs_alt

    def update_attitude(self, roll: float, pitch: float, yaw: float) -> None:
        self._attitude = Attitude(roll, pitch, yaw)
        self._heading_deg = math.degrees(yaw) % 360

    def update_velocity(self, north: float, east: float, down: float) -> None:
        self._velocity_ned = VectorNED(north, east, down)

    def update_gps(self, fix_type: int, satellites: int) -> None:
        self._gps = GPSInfo(fix_type, satellites)

    def update_battery(self, voltage: float, current: float, level: int) -> None:
        self._battery = Battery(voltage, current, level)

    def update_mode(self, mode: str) -> None:
        self._mode = mode

    def update_armed(self, armed: bool) -> None:
        old = self._armed
        self._armed = armed
        self._armed_telemetry_received = True
        if armed and not old:
            self._last_arm_time = time.time()

    def update_armable(
        self,
        global_ok: bool,
        home_ok: bool,
        armable: bool,
    ) -> None:
        self._armable = global_ok and home_ok and armable

    def update_home(self, lat: float, lon: float, rel_alt: float, abs_alt: float) -> None:
        self._home = Coordinate(lat, lon, rel_alt)
        self._home_abs_alt = abs_alt
