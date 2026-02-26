"""Unit tests for aerpawlib v2 MockVehicle."""

import pytest

from aerpawlib.v2.testing import MockVehicle
from aerpawlib.v2.types import Coordinate, Battery, GPSInfo


class TestMockVehicle:
    """MockVehicle properties and behavior."""

    def test_default_position(self):
        mv = MockVehicle()
        assert mv.position is not None
        assert isinstance(mv.position, Coordinate)
        assert -90 <= mv.position.lat <= 90
        assert -180 <= mv.position.lon <= 180

    def test_default_home_coords(self):
        mv = MockVehicle()
        assert mv.home_coords is not None
        assert isinstance(mv.home_coords, Coordinate)

    def test_default_battery(self):
        mv = MockVehicle()
        assert mv.battery is not None
        assert isinstance(mv.battery, Battery)
        assert 0 <= mv.battery.level <= 100
        assert mv.battery.voltage > 0

    def test_default_gps(self):
        mv = MockVehicle()
        assert mv.gps is not None
        assert isinstance(mv.gps, GPSInfo)
        assert mv.gps.fix_type >= 3

    def test_default_heading(self):
        mv = MockVehicle()
        assert isinstance(mv.heading, (int, float))
        assert 0 <= mv.heading < 360

    def test_default_armed(self):
        mv = MockVehicle()
        assert mv.armed is False

    def test_default_connected(self):
        mv = MockVehicle()
        assert mv.connected is True

    def test_custom_position(self):
        c = Coordinate(40.0, -74.0, 50.0)
        mv = MockVehicle(position=c)
        assert mv.position.lat == 40.0
        assert mv.position.lon == -74.0
        assert mv.position.alt == 50.0

    def test_custom_armed(self):
        mv = MockVehicle(armed=True)
        assert mv.armed is True

    def test_custom_connected(self):
        mv = MockVehicle(connected=False)
        assert mv.connected is False

    def test_heartbeat_tick_no_op(self):
        mv = MockVehicle()
        mv.heartbeat_tick()  # should not raise
