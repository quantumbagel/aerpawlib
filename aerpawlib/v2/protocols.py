"""
Protocol definitions for aerpawlib v2 API.

Provides Protocol classes for type-safe duck typing, enabling proper
mock implementations that stay in sync with real classes.
"""
from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional, Protocol, runtime_checkable

from .types import Coordinate, VectorNED, Attitude, FlightMode, LandedState


@runtime_checkable
class GPSProtocol(Protocol):
    """Protocol for GPS state containers."""
    satellites: int
    fix_type: int

    @property
    def has_fix(self) -> bool: ...

    @property
    def quality(self) -> str: ...


@runtime_checkable
class BatteryProtocol(Protocol):
    """Protocol for battery state containers."""
    voltage: float
    charge: float

    @property
    def percentage(self) -> float: ...

    @property
    def is_low(self) -> bool: ...

    @property
    def is_critical(self) -> bool: ...


@runtime_checkable
class StateProtocol(Protocol):
    """Protocol for vehicle state containers."""
    heading: float
    velocity: VectorNED
    attitude: Attitude
    altitude: float
    relative_altitude: float
    latitude: float
    longitude: float
    groundspeed: float
    airspeed: float
    climb_rate: float
    flight_mode: FlightMode
    landed_state: LandedState

    @property
    def position(self) -> Coordinate: ...

    @property
    def speed(self) -> float: ...

    @property
    def is_in_air(self) -> bool: ...


@runtime_checkable
class VehicleProtocol(Protocol):
    """
    Protocol defining the interface for vehicle classes.

    Use this to type-hint variables that could be either a real Drone
    or a MockDrone, enabling proper static type checking while allowing
    test mocks.

    Example:
        def fly_mission(vehicle: VehicleProtocol) -> None:
            await vehicle.takeoff(altitude=10)
            await vehicle.goto(latitude=51.5, longitude=-0.1)
            await vehicle.land()
    """
    state: StateProtocol
    gps: GPSProtocol
    battery: BatteryProtocol

    @property
    def connected(self) -> bool: ...

    @property
    def connection_healthy(self) -> bool: ...

    @property
    def armed(self) -> bool: ...

    @property
    def is_armable(self) -> bool: ...

    @property
    def home(self) -> Optional[Coordinate]: ...

    @property
    def position(self) -> Coordinate: ...

    @property
    def heading(self) -> float: ...

    @property
    def velocity(self) -> VectorNED: ...

    @property
    def altitude(self) -> float: ...

    @property
    def is_in_air(self) -> bool: ...

    async def connect(self, timeout: float = 30.0, auto_reconnect: bool = False) -> bool: ...

    async def disconnect(self) -> None: ...

    async def arm(self, force: bool = False) -> bool: ...

    async def disarm(self, force: bool = False) -> bool: ...

    async def takeoff(self, altitude: float = 5.0, wait: bool = True) -> bool: ...

    async def land(self, wait: bool = True) -> bool: ...

    async def goto(
        self,
        latitude: Optional[float] = None,
        longitude: Optional[float] = None,
        altitude: Optional[float] = None,
        coordinates: Optional[Coordinate] = None,
        speed: Optional[float] = None,
        heading: Optional[float] = None,
    ): ...

    async def rtl(self, wait: bool = True) -> bool: ...

    async def hold(self) -> None: ...

    def on(self, event: Any, callback: Callable) -> None: ...

    def off(self, event: Any, callback: Callable) -> None: ...


__all__ = [
    "GPSProtocol",
    "BatteryProtocol",
    "StateProtocol",
    "VehicleProtocol",
]

