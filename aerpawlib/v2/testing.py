"""
Mock vehicle classes for testing aerpawlib v2 API.

Uses Protocol for interface compliance, ensuring mocks stay in sync with real classes.
"""
from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

from .types import Coordinate, VectorNED, Attitude, FlightMode, LandedState, Waypoint


@dataclass
class MockState:
    """Mutable state for mock vehicle."""
    latitude: float = 35.7275
    longitude: float = -78.6960
    altitude: float = 0.0
    relative_altitude: float = 0.0
    heading: float = 0.0
    groundspeed: float = 0.0
    airspeed: float = 0.0
    climb_rate: float = 0.0
    velocity: VectorNED = field(default_factory=VectorNED)
    attitude: Attitude = field(default_factory=Attitude)
    flight_mode: FlightMode = FlightMode.MANUAL
    landed_state: LandedState = LandedState.ON_GROUND

    @property
    def position(self) -> Coordinate:
        return Coordinate(self.latitude, self.longitude, self.altitude)

    @property
    def speed(self) -> float:
        return self.velocity.magnitude()

    @property
    def is_in_air(self) -> bool:
        return self.landed_state == LandedState.IN_AIR


@dataclass
class MockGPS:
    """Mock GPS container."""
    satellites: int = 12
    fix_type: int = 3

    @property
    def has_fix(self) -> bool:
        return self.fix_type >= 2

    @property
    def quality(self) -> str:
        return {0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix"}.get(self.fix_type, "Unknown")


@dataclass
class MockBattery:
    """Mock battery container."""
    voltage: float = 16.8
    charge: float = 1.0

    @property
    def percentage(self) -> float:
        return self.charge * 100

    @property
    def is_low(self) -> bool:
        return self.charge < 0.20

    @property
    def is_critical(self) -> bool:
        return self.charge < 0.10


class MockDrone:
    """
    Mock drone for testing mission logic without actual hardware.

    Implements VehicleProtocol for type safety.
    """

    def __init__(self, connection: str = "mock://"):
        self._connection_string = connection
        self._connected = False
        self._armed = False
        self._is_armable = True

        self.state = MockState()
        self.gps = MockGPS()
        self.battery = MockBattery()

        self._home: Optional[Coordinate] = None
        self._simulated_speed: float = 5.0
        self._waypoints: List[Waypoint] = []

        self._callbacks: Dict[str, List[Callable]] = {}

        # Failure injection
        self._fail_on_arm = False
        self._fail_on_takeoff = False
        self._fail_on_goto = False

    async def __aenter__(self) -> "MockDrone":
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb) -> None:
        await self.disconnect()

    # Properties matching VehicleProtocol
    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def connection_healthy(self) -> bool:
        return self._connected

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def is_armable(self) -> bool:
        return self._is_armable

    @property
    def home(self) -> Optional[Coordinate]:
        return self._home

    @property
    def position(self) -> Coordinate:
        return self.state.position

    @property
    def heading(self) -> float:
        return self.state.heading

    @property
    def velocity(self) -> VectorNED:
        return self.state.velocity

    @property
    def altitude(self) -> float:
        return self.state.altitude

    @property
    def is_in_air(self) -> bool:
        return self.state.is_in_air

    # Connection
    async def connect(self, timeout: float = 30.0, auto_reconnect: bool = False, **kwargs) -> bool:
        await asyncio.sleep(0.01)
        self._connected = True
        self._home = self.position
        await self._trigger_callbacks("on_connect")
        return True

    async def disconnect(self) -> None:
        self._connected = False
        await self._trigger_callbacks("on_disconnect")

    # Events
    def on(self, event: Any, callback: Callable) -> None:
        event_name = event.value if hasattr(event, 'value') else str(event)
        if event_name not in self._callbacks:
            self._callbacks[event_name] = []
        self._callbacks[event_name].append(callback)

    def off(self, event: Any, callback: Callable) -> None:
        event_name = event.value if hasattr(event, 'value') else str(event)
        if event_name in self._callbacks and callback in self._callbacks[event_name]:
            self._callbacks[event_name].remove(callback)

    async def _trigger_callbacks(self, event: str, *args, **kwargs) -> None:
        for callback in self._callbacks.get(event, []):
            result = callback(*args, **kwargs)
            if asyncio.iscoroutine(result):
                await result

    # Basic operations
    async def arm(self, force: bool = False) -> bool:
        if self._fail_on_arm:
            from .exceptions import ArmError
            raise ArmError("Simulated arm failure")
        if not force and not self._is_armable:
            from .exceptions import NotArmableError
            raise NotArmableError()
        await asyncio.sleep(0.01)
        self._armed = True
        await self._trigger_callbacks("on_arm")
        return True

    async def disarm(self, force: bool = False) -> bool:
        await asyncio.sleep(0.01)
        self._armed = False
        self.state.landed_state = LandedState.ON_GROUND
        await self._trigger_callbacks("on_disarm")
        return True

    async def takeoff(self, altitude: float = 5.0, wait: bool = True) -> bool:
        if self._fail_on_takeoff:
            from .exceptions import TakeoffError
            raise TakeoffError(target_altitude=altitude)
        if not self._armed:
            await self.arm()

        self.state.landed_state = LandedState.TAKING_OFF
        self.state.flight_mode = FlightMode.TAKEOFF

        if wait:
            for i in range(10):
                self.state.altitude = altitude * (i + 1) / 10
                self.state.relative_altitude = self.state.altitude
                await asyncio.sleep(0.05)

        self.state.altitude = altitude
        self.state.relative_altitude = altitude
        self.state.landed_state = LandedState.IN_AIR
        self.state.flight_mode = FlightMode.HOLD
        return True

    async def land(self, wait: bool = True) -> bool:
        self.state.landed_state = LandedState.LANDING
        self.state.flight_mode = FlightMode.LAND

        if wait:
            start_alt = self.state.altitude
            for i in range(10):
                self.state.altitude = start_alt * (9 - i) / 10
                self.state.relative_altitude = self.state.altitude
                await asyncio.sleep(0.05)

        self.state.altitude = 0
        self.state.relative_altitude = 0
        self.state.landed_state = LandedState.ON_GROUND
        self._armed = False
        return True

    async def rtl(self, wait: bool = True) -> bool:
        if self._home:
            await self.goto(coordinates=self._home)
        await self.land(wait=wait)
        return True

    async def goto(
        self,
        latitude: Optional[float] = None,
        longitude: Optional[float] = None,
        altitude: Optional[float] = None,
        coordinates: Optional[Coordinate] = None,
        speed: Optional[float] = None,
        heading: Optional[float] = None,
    ) -> None:
        if self._fail_on_goto:
            from .exceptions import NavigationError
            raise NavigationError()

        if coordinates is not None:
            target = coordinates
        elif latitude is not None and longitude is not None:
            target = Coordinate(latitude, longitude, altitude or self.state.altitude)
        else:
            raise ValueError("Must provide coordinates or latitude/longitude")

        self.state.flight_mode = FlightMode.POSITION
        travel_speed = speed or self._simulated_speed
        distance = self.position.distance_to(target)
        steps = max(int(distance / travel_speed * 10), 5)
        start = self.position

        for i in range(steps):
            frac = (i + 1) / steps
            self.state.latitude = start.latitude + (target.latitude - start.latitude) * frac
            self.state.longitude = start.longitude + (target.longitude - start.longitude) * frac
            self.state.altitude = start.altitude + (target.altitude - start.altitude) * frac
            self.state.relative_altitude = self.state.altitude
            self.state.groundspeed = travel_speed
            await asyncio.sleep(0.02)

        self.state.groundspeed = 0
        self.state.flight_mode = FlightMode.HOLD
        if heading is not None:
            self.state.heading = heading % 360

    async def hold(self) -> None:
        self.state.flight_mode = FlightMode.HOLD
        self.state.groundspeed = 0

    # Test utilities
    def set_position(self, position: Coordinate) -> None:
        self.state.latitude = position.latitude
        self.state.longitude = position.longitude
        self.state.altitude = position.altitude
        self.state.relative_altitude = position.altitude

    def set_battery(self, percentage: float) -> None:
        self.battery.charge = percentage / 100

    def set_gps(self, satellites: int, fix_type: int = 3) -> None:
        self.gps.satellites = satellites
        self.gps.fix_type = fix_type

    def set_armable(self, armable: bool) -> None:
        self._is_armable = armable

    def inject_arm_failure(self, fail: bool = True) -> None:
        self._fail_on_arm = fail

    def inject_takeoff_failure(self, fail: bool = True) -> None:
        self._fail_on_takeoff = fail

    def inject_navigation_failure(self, fail: bool = True) -> None:
        self._fail_on_goto = fail

    async def wait_for_gps_fix(self, min_satellites: int = 6, timeout: float = 60.0) -> bool:
        if self.gps.has_fix and self.gps.satellites >= min_satellites:
            return True
        from .exceptions import TimeoutError
        raise TimeoutError("GPS fix timeout", operation="wait_for_gps_fix", timeout=timeout)

    async def wait_for_armable(self, timeout: float = 60.0) -> bool:
        if self._is_armable:
            return True
        from .exceptions import TimeoutError
        raise TimeoutError("Armable timeout", operation="wait_for_armable", timeout=timeout)


class MockRover(MockDrone):
    """Mock rover for testing ground vehicle mission logic."""

    async def takeoff(self, altitude: float = 5.0, wait: bool = True) -> bool:
        return True  # No-op for rovers

    async def land(self, wait: bool = True) -> bool:
        await self.hold()
        self._armed = False
        return True


__all__ = ["MockDrone", "MockRover", "MockState", "MockGPS", "MockBattery"]

