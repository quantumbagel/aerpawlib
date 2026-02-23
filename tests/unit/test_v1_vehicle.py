"""Unit tests for aerpawlib v1 vehicle related components."""

import math
import socket
from unittest.mock import patch

import pytest

from aerpawlib.v1.exceptions import PortInUseError
from aerpawlib.v1.util import Coordinate
from aerpawlib.v1.vehicle import (Drone, DummyVehicle, Rover, _AttitudeCompat, _BatteryCompat, _GPSInfoCompat,
                                  _VersionCompat)
from aerpawlib.v1.vehicles.core_vehicle import _parse_udp_connection_port


class TestBatteryCompat:
    def test_defaults(self):
        b = _BatteryCompat()
        assert b.voltage == 0.0
        assert b.current == 0.0
        assert b.level == 0

    def test_str_contains_fields(self):
        b = _BatteryCompat()
        b.voltage = 12.4
        b.current = 1.5
        b.level = 75
        s = str(b)
        assert "12.4" in s
        assert "1.5" in s
        assert "75" in s

    def test_repr_contains_fields(self):
        b = _BatteryCompat()
        b.voltage = 11.1
        r = repr(b)
        assert "11.1" in r

    def test_assignment(self):
        b = _BatteryCompat()
        b.voltage = 16.8
        b.current = 2.3
        b.level = 50
        assert b.voltage == 16.8
        assert b.current == 2.3
        assert b.level == 50


class TestGPSInfoCompat:
    def test_defaults(self):
        g = _GPSInfoCompat()
        assert g.fix_type == 0
        assert g.satellites_visible == 0

    def test_str_contains_fields(self):
        g = _GPSInfoCompat()
        g.fix_type = 3
        g.satellites_visible = 12
        s = str(g)
        assert "3" in s
        assert "12" in s

    def test_repr_contains_fields(self):
        g = _GPSInfoCompat()
        g.fix_type = 2
        r = repr(g)
        assert "2" in r

    def test_assignment(self):
        g = _GPSInfoCompat()
        g.fix_type = 3
        g.satellites_visible = 9
        assert g.fix_type == 3
        assert g.satellites_visible == 9


class TestAttitudeCompat:
    def test_defaults(self):
        a = _AttitudeCompat()
        assert a.pitch == 0.0
        assert a.roll == 0.0
        assert a.yaw == 0.0

    def test_str_contains_fields(self):
        a = _AttitudeCompat()
        a.pitch = 0.1
        a.yaw = 1.57
        a.roll = -0.05
        s = str(a)
        assert "0.1" in s

    def test_repr_contains_fields(self):
        a = _AttitudeCompat()
        a.pitch = 0.3
        r = repr(a)
        assert "0.3" in r

    def test_assignment(self):
        a = _AttitudeCompat()
        a.yaw = math.pi / 2
        assert abs(a.yaw - math.pi / 2) < 1e-10


class TestVersionCompat:
    def test_defaults(self):
        v = _VersionCompat()
        assert v.major is None
        assert v.minor is None
        assert v.patch is None
        assert v.release is None

    def test_str_format(self):
        v = _VersionCompat()
        v.major = 4
        v.minor = 3
        v.patch = 1
        s = str(v)
        assert "4" in s and "3" in s and "1" in s

    def test_repr_contains_fields(self):
        v = _VersionCompat()
        v.major = 4
        r = repr(v)
        assert "4" in r

    def test_assignment(self):
        v = _VersionCompat()
        v.major = 4
        v.minor = 2
        v.patch = 3
        v.release = "stable"
        assert v.major == 4
        assert v.minor == 2
        assert v.patch == 3
        assert v.release == "stable"


class TestDummyVehicleUnit:
    """Basic unit tests for DummyVehicle (no async needed for some)."""

    def test_creates(self):
        v = DummyVehicle()
        assert v is not None

    def test_close_noop(self):
        v = DummyVehicle()
        v.close()

    def test_close_is_idempotent(self):
        v = DummyVehicle()
        v.close()
        v.close()  # Second call should not raise

    def test_initialize_prearm_noop(self):
        v = DummyVehicle()
        v._initialize_prearm(should_postarm_init=True)

    def test_initialize_prearm_multiple_times(self):
        v = DummyVehicle()
        v._initialize_prearm(should_postarm_init=False)
        v._initialize_prearm(should_postarm_init=True)

    @pytest.mark.asyncio
    async def test_initialize_postarm_noop(self):
        v = DummyVehicle()
        await v._initialize_postarm()

    @pytest.mark.asyncio
    async def test_initialize_postarm_is_async(self):
        v = DummyVehicle()
        result = await v._initialize_postarm()
        assert result is None  # noop, just must not raise


class TestParseUdpConnectionPort:
    """Parse UDP connection strings used by aerpawlib/MAVSDK."""

    def test_udp_listen_all(self):
        assert _parse_udp_connection_port("udp://:14540") == ("0.0.0.0", 14540)

    def test_udp_host_port(self):
        assert _parse_udp_connection_port("udp://127.0.0.1:14550") == ("127.0.0.1", 14550)

    def test_udpin_listen_all(self):
        assert _parse_udp_connection_port("udpin://:14540") == ("0.0.0.0", 14540)

    def test_udpin_host_port(self):
        assert _parse_udp_connection_port("udpin://127.0.0.1:14551") == ("127.0.0.1", 14551)

    def test_udpin_explicit_bind(self):
        assert _parse_udp_connection_port("udpin://0.0.0.0:14540") == ("0.0.0.0", 14540)

    def test_udpin_ipv6(self):
        assert _parse_udp_connection_port("udpin://[::1]:14540") == ("::1", 14540)

    def test_udpout_returns_none(self):
        assert _parse_udp_connection_port("udpout://192.168.1.12:14550") is None

    def test_serial_returns_none(self):
        assert _parse_udp_connection_port("serial:///dev/ttyUSB0:57600") is None

    def test_tcp_returns_none(self):
        assert _parse_udp_connection_port("tcp://localhost:5760") is None


class TestPortInUse:
    """Port-in-use fails fast instead of hanging."""

    def test_udp_port_in_use_raises_immediately(self):
        """When UDP port from connection string is in use, Drone raises PortInUseError immediately."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("0.0.0.0", 0))
        except (PermissionError, OSError):
            pytest.skip("Cannot bind socket in this environment")
        port = sock.getsockname()[1]
        try:
            with pytest.raises(PortInUseError, match="already in use"):
                Drone(f"udp://:{port}")
        finally:
            sock.close()


class TestVehicleValidationUnit:
    """Unit tests for vehicle command validation (no SITL needed)."""


    @pytest.mark.asyncio
    @patch("aerpawlib.v1.vehicles.core_vehicle.Vehicle._connect_sync", return_value=None)
    @patch("aerpawlib.v1.vehicles.rover.Rover.__init__", return_value=None)
    async def test_rover_goto_validation(self, mock_rover_init, mock_connect):
        rover = Rover("udp://:14540")
        with pytest.raises(ValueError, match="at least"):
            await Rover.goto_coordinates(rover, Coordinate(0, 0), tolerance=0.0)

    @pytest.mark.asyncio
    @patch("aerpawlib.v1.vehicles.core_vehicle.Vehicle._connect_sync", return_value=None)
    @patch("aerpawlib.v1.vehicles.drone.Drone.__init__", return_value=None)
    async def test_drone_goto_validation(self, mock_drone_init, mock_connect):
        drone = Drone("udp://:14540")
        with pytest.raises(ValueError, match="at least"):
            await Drone.goto_coordinates(drone, Coordinate(0, 0), tolerance=0.0)
