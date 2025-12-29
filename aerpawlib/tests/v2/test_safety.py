"""
Tests for the v2 aerpawlib safety module.

These tests verify safety limits, validation functions, and safety monitoring.
"""
import os
import sys

import pytest

# Add the parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))

from aerpawlib.v2.safety import (
    SafetyLimits,
    SafetyViolationType,
    RequestType,
    VehicleType,
    validate_coordinate,
    validate_altitude,
    validate_speed,
    validate_velocity,
    validate_timeout,
    validate_tolerance,
)
from aerpawlib.v2.types import Coordinate, VectorNED


class TestSafetyLimits:
    """Tests for SafetyLimits configuration."""

    def test_default_safety_limits(self):
        """Test default SafetyLimits values."""
        limits = SafetyLimits()
        assert limits.max_speed == 15.0
        assert limits.max_vertical_speed == 5.0
        assert limits.min_battery_percent == 20.0
        assert limits.critical_battery_percent == 10.0
        assert limits.require_gps_fix is True
        assert limits.min_satellites == 6
        assert limits.enable_speed_limits is True
        assert limits.enable_battery_failsafe is True
        assert limits.enable_parameter_validation is True
        assert limits.enable_preflight_checks is True
        assert limits.auto_clamp_values is True

    def test_safety_limits_validate_valid(self):
        """Test that default limits pass validation."""
        limits = SafetyLimits()
        errors = limits.validate()
        assert len(errors) == 0

    def test_safety_limits_validate_invalid_speed(self):
        """Test validation catches invalid max_speed."""
        limits = SafetyLimits(max_speed=-1)
        errors = limits.validate()
        assert any("max_speed" in e for e in errors)

    def test_safety_limits_validate_invalid_vertical_speed(self):
        """Test validation catches invalid max_vertical_speed."""
        limits = SafetyLimits(max_vertical_speed=0)
        errors = limits.validate()
        assert any("max_vertical_speed" in e for e in errors)

    def test_safety_limits_validate_invalid_battery(self):
        """Test validation catches invalid battery thresholds."""
        limits = SafetyLimits(min_battery_percent=150)
        errors = limits.validate()
        assert any("min_battery_percent" in e for e in errors)

    def test_safety_limits_validate_battery_order(self):
        """Test validation catches critical >= min battery."""
        limits = SafetyLimits(
            min_battery_percent=20,
            critical_battery_percent=25  # Should be less than min
        )
        errors = limits.validate()
        assert any("critical_battery_percent must be less" in e for e in errors)

    def test_safety_limits_permissive(self):
        """Test permissive preset."""
        limits = SafetyLimits.permissive()
        assert limits.max_speed == 30.0
        assert limits.enable_battery_failsafe is False
        assert limits.enable_preflight_checks is False
        assert limits.auto_clamp_values is False

    def test_safety_limits_restrictive(self):
        """Test restrictive preset."""
        limits = SafetyLimits.restrictive()
        assert limits.max_speed == 5.0
        assert limits.max_vertical_speed == 2.0
        assert limits.min_battery_percent == 30.0
        assert limits.min_satellites == 8

    def test_safety_limits_disabled(self):
        """Test disabled preset."""
        limits = SafetyLimits.disabled()
        assert limits.enable_speed_limits is False
        assert limits.enable_battery_failsafe is False
        assert limits.enable_parameter_validation is False
        assert limits.enable_preflight_checks is False


class TestSafetyEnums:
    """Tests for safety-related enums."""

    def test_safety_violation_types(self):
        """Test SafetyViolationType enum values exist."""
        assert SafetyViolationType.SPEED_TOO_HIGH
        assert SafetyViolationType.VERTICAL_SPEED_TOO_HIGH
        assert SafetyViolationType.BATTERY_LOW
        assert SafetyViolationType.BATTERY_CRITICAL
        assert SafetyViolationType.GPS_POOR
        assert SafetyViolationType.NO_GPS_FIX
        assert SafetyViolationType.INVALID_COORDINATE
        assert SafetyViolationType.INVALID_ALTITUDE
        assert SafetyViolationType.GEOFENCE_VIOLATION
        assert SafetyViolationType.NO_GO_ZONE_VIOLATION

    def test_request_types(self):
        """Test RequestType enum values exist."""
        assert RequestType.SERVER_STATUS
        assert RequestType.VALIDATE_WAYPOINT
        assert RequestType.VALIDATE_SPEED
        assert RequestType.VALIDATE_TAKEOFF
        assert RequestType.VALIDATE_LANDING

    def test_vehicle_types(self):
        """Test VehicleType enum values exist."""
        assert VehicleType.ROVER
        assert VehicleType.COPTER
        assert VehicleType.ROVER.value == "rover"
        assert VehicleType.COPTER.value == "copter"


class TestValidateCoordinate:
    """Tests for validate_coordinate function."""

    def test_valid_coordinate(self):
        """Test validation of valid coordinate."""
        coord = Coordinate(35.0, -78.0, 100)
        result = validate_coordinate(coord)
        assert result.passed is True

    def test_invalid_latitude_too_high(self):
        """Test validation catches latitude > 90."""
        coord = Coordinate(95.0, -78.0, 100)
        result = validate_coordinate(coord)
        assert result.passed is False

    def test_invalid_latitude_too_low(self):
        """Test validation catches latitude < -90."""
        coord = Coordinate(-95.0, -78.0, 100)
        result = validate_coordinate(coord)
        assert result.passed is False

    def test_invalid_longitude_too_high(self):
        """Test validation catches longitude > 180."""
        coord = Coordinate(35.0, 190.0, 100)
        result = validate_coordinate(coord)
        assert result.passed is False

    def test_invalid_longitude_too_low(self):
        """Test validation catches longitude < -180."""
        coord = Coordinate(35.0, -190.0, 100)
        result = validate_coordinate(coord)
        assert result.passed is False


class TestValidateAltitude:
    """Tests for validate_altitude function."""

    def test_valid_altitude(self):
        """Test validation of valid altitude."""
        result = validate_altitude(100)
        assert result.passed is True

    def test_altitude_too_low(self):
        """Test validation catches altitude too low (below -100m)."""
        result = validate_altitude(-200)
        assert result.passed is False

    def test_altitude_too_high(self):
        """Test validation catches altitude too high (above 10000m)."""
        result = validate_altitude(15000)
        assert result.passed is False


class TestValidateSpeed:
    """Tests for validate_speed function."""

    def test_valid_speed(self):
        """Test validation of valid speed."""
        limits = SafetyLimits()
        result = validate_speed(10, limits)
        assert result.passed is True

    def test_speed_too_high(self):
        """Test validation catches speed exceeding limit."""
        limits = SafetyLimits(max_speed=15)
        result = validate_speed(20, limits)
        assert result.passed is False

    def test_negative_speed(self):
        """Test validation catches negative speed."""
        limits = SafetyLimits()
        result = validate_speed(-5, limits)
        assert result.passed is False


class TestValidateVelocity:
    """Tests for validate_velocity function."""

    def test_valid_velocity(self):
        """Test validation of valid velocity vector."""
        limits = SafetyLimits()
        velocity = VectorNED(3, 4, 0)  # 5 m/s horizontal
        result = validate_velocity(velocity, limits)
        assert result.passed is True

    def test_velocity_too_fast_horizontal(self):
        """Test validation catches horizontal speed exceeding limit."""
        limits = SafetyLimits(max_speed=5)
        velocity = VectorNED(6, 8, 0)  # 10 m/s horizontal
        result = validate_velocity(velocity, limits)
        assert result.passed is False

    def test_velocity_too_fast_vertical(self):
        """Test validation catches vertical speed exceeding limit."""
        limits = SafetyLimits(max_vertical_speed=3)
        velocity = VectorNED(0, 0, 5)  # 5 m/s vertical
        result = validate_velocity(velocity, limits)
        assert result.passed is False


class TestValidateTimeout:
    """Tests for validate_timeout function."""

    def test_valid_timeout(self):
        """Test validation of valid timeout."""
        result = validate_timeout(30)
        assert result.passed is True

    def test_negative_timeout(self):
        """Test validation catches negative timeout."""
        result = validate_timeout(-10)
        assert result.passed is False

    def test_none_timeout(self):
        """Test validation of None timeout (means no timeout)."""
        result = validate_timeout(None)
        assert result.passed is True


class TestValidateTolerance:
    """Tests for validate_tolerance function."""

    def test_valid_tolerance(self):
        """Test validation of valid tolerance."""
        result = validate_tolerance(1.0)
        assert result.passed is True

    def test_zero_tolerance(self):
        """Test validation catches zero tolerance."""
        result = validate_tolerance(0)
        assert result.passed is False

    def test_negative_tolerance(self):
        """Test validation catches negative tolerance."""
        result = validate_tolerance(-1)
        assert result.passed is False


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

