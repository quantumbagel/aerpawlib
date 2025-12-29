"""
Mock vehicle classes for testing aerpawlib v2 API.

This module provides MockDrone and MockRover classes that implement the same
interface as the real vehicle classes, but without requiring MAVSDK or an
actual vehicle connection. This enables unit testing of mission logic.

Example:
    from aerpawlib.v2.testing import MockDrone, MockScenario

    async def test_mission():
        drone = MockDrone()
        drone.set_position(Coordinate(35.7275, -78.6960, 0))

        await drone.connect()
        await drone.arm()
        await drone.takeoff(altitude=10)

        assert drone.altitude >= 9.5
        assert drone.armed
"""
from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from typing import Optional, List, Callable, Dict, Any

from .types import (
    Coordinate,
    VectorNED,
    Attitude,
    FlightMode,
    LandedState,
    Waypoint,
)


@dataclass
class MockState:
    """Mutable state for mock vehicle."""
    latitude: float = 35.7275
    longitude: float = -78.6960
    altitude: float = 0.0
    heading: float = 0.0
    groundspeed: float = 0.0
    velocity: VectorNED = field(default_factory=VectorNED)
    attitude: Attitude = field(default_factory=Attitude)
    flight_mode: FlightMode = FlightMode.MANUAL
    landed_state: LandedState = LandedState.ON_GROUND

    @property
    def position(self) -> Coordinate:
        return Coordinate(self.latitude, self.longitude, self.altitude)


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
        if self.fix_type >= 3:
            return "3D Fix"
        elif self.fix_type >= 2:
            return "2D Fix"
        return "No Fix"


@dataclass
class MockBattery:
    """Mock battery container."""
    voltage: float = 16.8
    charge: float = 1.0  # 0-1

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

    Simulates drone behavior including:
    - Connection (instant success)
    - Arming/disarming
    - Takeoff/landing (with simulated altitude changes)
    - Navigation (with simulated position changes)
    - Telemetry (configurable state)

    Example:
        async def test_takeoff():
            drone = MockDrone()
            await drone.connect()
            await drone.arm()
            await drone.takeoff(altitude=10)

            assert drone.armed
            assert drone.altitude >= 9.5
            assert drone.state.landed_state == LandedState.IN_AIR
    """

    def __init__(self, connection: str = "mock://"):
        self._connection_string = connection
        self._connected = False
        self._armed = False
        self._is_armable = True

        # State containers
        self.state = MockState()
        self.gps = MockGPS()
        self.battery = MockBattery()

        # Home position
        self._home: Optional[Coordinate] = None

        # Movement simulation
        self._simulated_speed: float = 5.0  # m/s
        self._waypoints: List[Waypoint] = []
        self._current_heading: Optional[float] = None

        # Callbacks
        self._callbacks: Dict[str, List[Callable]] = {
            "on_connect": [],
            "on_disconnect": [],
            "on_reconnect": [],
            "on_arm": [],
            "on_disarm": [],
            "on_low_battery": [],
            "on_critical_battery": [],
            "on_mode_change": [],
        }

        # Recording
        self._recording = False
        self._flight_log: List[Dict[str, Any]] = []

        # Failure injection for testing
        self._fail_on_arm = False
        self._fail_on_takeoff = False
        self._fail_on_goto = False

    # Context manager
    async def __aenter__(self) -> "MockDrone":
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb) -> None:
        await self.disconnect()

    # Connection
    async def connect(
        self
    ) -> bool:
        """Simulate connection (always succeeds unless configured to fail)."""
        await asyncio.sleep(0.01)  # Simulate brief delay
        self._connected = True
        self._home = self.position
        await self._trigger_callbacks("on_connect")
        return True

    async def disconnect(self) -> None:
        """Simulate disconnection."""
        self._connected = False
        await self._trigger_callbacks("on_disconnect")

    # Properties
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
        return self.state.landed_state == LandedState.IN_AIR

    # Event callbacks
    def on(self, event: str, callback: Callable) -> None:
        if event in self._callbacks:
            self._callbacks[event].append(callback)

    def off(self, event: str, callback: Callable) -> None:
        if event in self._callbacks and callback in self._callbacks[event]:
            self._callbacks[event].remove(callback)

    async def _trigger_callbacks(self, event: str, *args, **kwargs) -> None:
        for callback in self._callbacks.get(event, []):
            result = callback(*args, **kwargs)
            if asyncio.iscoroutine(result):
                await result

    # Basic operations
    async def arm(self, force: bool = False) -> bool:
        """Simulate arming."""
        if self._fail_on_arm:
            from .exceptions import ArmError
            raise ArmError("Simulated arm failure")

        if not force and not self._is_armable:
            from .exceptions import NotArmableError
            raise NotArmableError("Vehicle not armable")

        await asyncio.sleep(0.01)
        self._armed = True
        await self._trigger_callbacks("on_arm")
        return True

    async def disarm(self) -> bool:
        """Simulate disarming."""
        await asyncio.sleep(0.01)
        self._armed = False
        self.state.landed_state = LandedState.ON_GROUND
        await self._trigger_callbacks("on_disarm")
        return True

    async def takeoff(self, altitude: float = 5.0, wait: bool = True) -> bool:
        """Simulate takeoff."""
        if self._fail_on_takeoff:
            from .exceptions import TakeoffError
            raise TakeoffError("Simulated takeoff failure", target_altitude=altitude)

        if not self._armed:
            await self.arm()

        self.state.landed_state = LandedState.TAKING_OFF
        self.state.flight_mode = FlightMode.TAKEOFF

        if wait:
            # Simulate gradual altitude increase
            steps = 10
            for i in range(steps):
                self.state.altitude = altitude * (i + 1) / steps
                await asyncio.sleep(0.05)

        self.state.altitude = altitude
        self.state.landed_state = LandedState.IN_AIR
        self.state.flight_mode = FlightMode.HOLD
        return True

    async def land(self, wait: bool = True) -> bool:
        """Simulate landing."""
        self.state.landed_state = LandedState.LANDING
        self.state.flight_mode = FlightMode.LAND

        if wait:
            # Simulate gradual descent
            start_alt = self.state.altitude
            steps = 10
            for i in range(steps):
                self.state.altitude = start_alt * (steps - i - 1) / steps
                await asyncio.sleep(0.05)

        self.state.altitude = 0
        self.state.landed_state = LandedState.ON_GROUND
        self._armed = False
        return True

    async def rtl(self, wait: bool = True) -> bool:
        """Simulate return to launch."""
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
        heading: Optional[float] = None
    ):
        """Simulate navigation to a location."""
        if self._fail_on_goto:
            from .exceptions import NavigationError
            raise NavigationError("Simulated navigation failure")

        # Determine target
        if coordinates is not None:
            target = coordinates
        elif latitude is not None and longitude is not None:
            target = Coordinate(
                latitude,
                longitude,
                altitude if altitude is not None else self.state.altitude
            )
        else:
            raise ValueError("Must provide either coordinates or latitude/longitude")

        # Simulate movement
        self.state.flight_mode = FlightMode.POSITION
        distance = self.position.distance_to(target)
        travel_speed = speed or self._simulated_speed
        travel_time = distance / travel_speed

        # Simulate gradual position change
        steps = max(int(travel_time * 10), 5)
        start_pos = self.position

        for i in range(steps):
            fraction = (i + 1) / steps
            self.state.latitude = start_pos.latitude + (target.latitude - start_pos.latitude) * fraction
            self.state.longitude = start_pos.longitude + (target.longitude - start_pos.longitude) * fraction
            self.state.altitude = start_pos.altitude + (target.altitude - start_pos.altitude) * fraction
            self.state.groundspeed = travel_speed
            await asyncio.sleep(0.02)

        self.state.groundspeed = 0
        self.state.flight_mode = FlightMode.HOLD

        if heading is not None:
            self.state.heading = heading % 360
        else:
            self.state.heading = start_pos.bearing_to(target)

    async def set_heading(self, degrees: float):
        """Simulate heading change."""
        self.state.heading = degrees % 360

    async def hold(self):
        """Simulate position hold."""
        self.state.flight_mode = FlightMode.HOLD
        self.state.groundspeed = 0

    async def abort(self, rtl: bool = True):
        """Simulate abort."""
        if rtl:
            await self.rtl(wait=False)
        else:
            await self.hold()

    # Telemetry recording
    def start_recording(self) -> None:
        self._recording = True
        self._flight_log.clear()

    def stop_recording(self) -> int:
        self._recording = False
        return len(self._flight_log)

    def get_flight_log(self) -> List[Dict[str, Any]]:
        return self._flight_log.copy()

    def save_flight_log(self, path: str, format: str = "json") -> None:
        pass  # No-op for mock

    def clear_flight_log(self) -> None:
        self._flight_log.clear()

    # Waypoint management
    def add_waypoint(self, waypoint: Coordinate | Waypoint):
        if isinstance(waypoint, Coordinate):
            self._waypoints.append(Waypoint(coordinate=waypoint))
        else:
            self._waypoints.append(waypoint)

    def add_waypoints(self, waypoints: List[Coordinate | Waypoint]):
        for wp in waypoints:
            self.add_waypoint(wp)

    def clear_waypoints(self):
        self._waypoints.clear()

    async def execute_waypoints(self, tolerance: float = 2.0):
        for wp in self._waypoints:
            await self.goto(coordinates=wp.coordinate)
        self._waypoints.clear()

    # Utility methods for testing
    def set_position(self, position: Coordinate) -> None:
        """Set the mock drone's position."""
        self.state.latitude = position.latitude
        self.state.longitude = position.longitude
        self.state.altitude = position.altitude

    def set_battery(self, percentage: float) -> None:
        """Set battery level (0-100)."""
        self.battery.charge = percentage / 100

    def set_gps(self, satellites: int, fix_type: int = 3) -> None:
        """Set GPS state."""
        self.gps.satellites = satellites
        self.gps.fix_type = fix_type

    def set_armable(self, armable: bool) -> None:
        """Set whether the drone is armable."""
        self._is_armable = armable

    def inject_arm_failure(self, fail: bool = True) -> None:
        """Configure arm to fail for testing error handling."""
        self._fail_on_arm = fail

    def inject_takeoff_failure(self, fail: bool = True) -> None:
        """Configure takeoff to fail for testing error handling."""
        self._fail_on_takeoff = fail

    def inject_navigation_failure(self, fail: bool = True) -> None:
        """Configure navigation to fail for testing error handling."""
        self._fail_on_goto = fail

    async def wait(self, seconds: float):
        """Wait for specified duration."""
        await asyncio.sleep(seconds)

    async def wait_for_gps_fix(self, min_satellites: int = 6, timeout: float = 60.0) -> bool:
        """Simulate waiting for GPS fix."""
        if self.gps.has_fix and self.gps.satellites >= min_satellites:
            return True
        from .exceptions import TimeoutError
        raise TimeoutError(f"GPS fix timeout", operation="wait_for_gps_fix", timeout=timeout)

    async def wait_for_armable(self, timeout: float = 60.0) -> bool:
        """Simulate waiting for armable state."""
        if self._is_armable:
            return True
        from .exceptions import TimeoutError
        raise TimeoutError("Armable timeout", operation="wait_for_armable", timeout=timeout)


class MockRover(MockDrone):
    """
    Mock rover for testing ground vehicle mission logic.

    Inherits from MockDrone but removes air-specific functionality.
    """

    async def takeoff(self, altitude: float = 5.0, wait: bool = True) -> bool:
        """Rovers don't take off - this is a no-op."""
        return True

    async def land(self, wait: bool = True) -> bool:
        """Rovers don't land - just stop."""
        await self.hold()
        self._armed = False
        return True


__all__ = [
    "MockDrone",
    "MockRover",
    "MockState",
    "MockGPS",
    "MockBattery",
]

