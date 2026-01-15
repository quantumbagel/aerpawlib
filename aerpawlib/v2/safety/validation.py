"""
Parameter validation functions for aerpawlib v2 API.
"""

from __future__ import annotations

import logging
import math
from typing import Optional, TYPE_CHECKING

from .types import SafetyViolationType, SafetyCheckResult, PreflightCheckResult
from .limits import SafetyLimits

if TYPE_CHECKING:
    from ..vehicle import Vehicle

logger = logging.getLogger("aerpawlib.safety")


def _validate_number(
    value: Optional[float],
    name: str,
    violation_type: SafetyViolationType,
    *,
    allow_none: bool = False,
    min_val: Optional[float] = None,
    max_val: Optional[float] = None,
    positive: bool = False,
) -> SafetyCheckResult:
    """Generic number validation with configurable constraints."""
    if value is None:
        return (
            SafetyCheckResult.ok()
            if allow_none
            else SafetyCheckResult.fail(
                violation_type, f"{name} cannot be None"
            )
        )

    if math.isnan(value) or math.isinf(value):
        return SafetyCheckResult.fail(
            violation_type, f"{name} is invalid (NaN or Inf)"
        )

    if positive and value <= 0:
        return SafetyCheckResult.fail(
            violation_type, f"{name} must be positive"
        )

    if min_val is not None and value < min_val:
        return SafetyCheckResult.fail(
            violation_type, f"{name} {value} is below minimum {min_val}"
        )

    if max_val is not None and value > max_val:
        return SafetyCheckResult.fail(
            violation_type, f"{name} {value} exceeds maximum {max_val}"
        )

    return SafetyCheckResult.ok()


def validate_coordinate(coord, name: str = "coordinate") -> SafetyCheckResult:
    """Validate a coordinate has reasonable lat/lon values."""
    if coord is None:
        return SafetyCheckResult.fail(
            SafetyViolationType.INVALID_COORDINATE, f"{name} cannot be None"
        )

    lat = getattr(coord, "latitude", getattr(coord, "lat", None))
    lon = getattr(coord, "longitude", getattr(coord, "lon", None))

    if lat is None or lon is None:
        return SafetyCheckResult.fail(
            SafetyViolationType.INVALID_COORDINATE,
            f"{name} missing latitude or longitude",
        )

    for val, what, min_v, max_v in [
        (lat, "latitude", -90, 90),
        (lon, "longitude", -180, 180),
    ]:
        if math.isnan(val) or math.isinf(val):
            return SafetyCheckResult.fail(
                SafetyViolationType.INVALID_COORDINATE,
                f"{name} {what} is invalid",
            )
        if not min_v <= val <= max_v:
            return SafetyCheckResult.fail(
                SafetyViolationType.INVALID_COORDINATE,
                f"{name} {what} {val} out of range ({min_v} to {max_v})",
            )

    return SafetyCheckResult.ok()


def validate_altitude(
    altitude: float, name: str = "altitude"
) -> SafetyCheckResult:
    """Validate an altitude value is reasonable (-100m to 10000m)."""
    return _validate_number(
        altitude,
        name,
        SafetyViolationType.INVALID_ALTITUDE,
        min_val=-100,
        max_val=10000,
    )


def validate_speed(
    speed: float, limits: SafetyLimits, name: str = "speed"
) -> SafetyCheckResult:
    """Validate a speed value against safety limits."""
    result = _validate_number(
        speed,
        name,
        SafetyViolationType.INVALID_SPEED,
        allow_none=True,
        min_val=0,
    )
    if not result.passed or speed is None:
        return result

    if limits.enable_speed_limits and speed > limits.max_speed:
        return SafetyCheckResult.fail(
            SafetyViolationType.SPEED_TOO_HIGH,
            f"{name} {speed}m/s exceeds maximum {limits.max_speed}m/s",
            value=speed,
            limit=limits.max_speed,
        )
    return SafetyCheckResult.ok()


def validate_velocity(
    velocity, limits: SafetyLimits, name: str = "velocity"
) -> SafetyCheckResult:
    """Validate a velocity vector against safety limits."""
    if velocity is None:
        return SafetyCheckResult.fail(
            SafetyViolationType.INVALID_PARAMETER, f"{name} cannot be None"
        )

    if not limits.enable_speed_limits:
        return SafetyCheckResult.ok()

    h_speed = velocity.magnitude(ignore_vertical=True)
    v_speed = abs(velocity.down)

    if h_speed > limits.max_speed:
        return SafetyCheckResult.fail(
            SafetyViolationType.SPEED_TOO_HIGH,
            f"Horizontal {name} {h_speed:.1f}m/s exceeds max {limits.max_speed}m/s",
            value=h_speed,
            limit=limits.max_speed,
        )
    if v_speed > limits.max_vertical_speed:
        return SafetyCheckResult.fail(
            SafetyViolationType.VERTICAL_SPEED_TOO_HIGH,
            f"Vertical {name} {v_speed:.1f}m/s exceeds max {limits.max_vertical_speed}m/s",
            value=v_speed,
            limit=limits.max_vertical_speed,
        )
    return SafetyCheckResult.ok()


def validate_timeout(
    timeout: float, name: str = "timeout"
) -> SafetyCheckResult:
    """Validate a timeout value is reasonable (0 to 3600s)."""
    return _validate_number(
        timeout,
        name,
        SafetyViolationType.INVALID_PARAMETER,
        allow_none=True,
        positive=True,
        max_val=3600,
    )


def validate_tolerance(
    tolerance: float, name: str = "tolerance"
) -> SafetyCheckResult:
    """Validate a tolerance/acceptance radius value (0.1m minimum)."""
    result = _validate_number(
        tolerance,
        name,
        SafetyViolationType.INVALID_PARAMETER,
        allow_none=True,
        positive=True,
    )
    if not result.passed or tolerance is None:
        return result
    if tolerance < 0.1:
        return SafetyCheckResult.fail(
            SafetyViolationType.INVALID_PARAMETER,
            f"{name} {tolerance}m is too small (min 0.1m)",
        )
    return SafetyCheckResult.ok()


def clamp_speed(
    speed: Optional[float], limits: SafetyLimits
) -> Optional[float]:
    """Clamp a speed value to within safety limits."""
    if speed is None:
        return None
    if not limits.enable_speed_limits:
        return max(0.0, speed)
    clamped = min(limits.max_speed, max(0.0, speed))
    if clamped != speed:
        logger.warning(
            f"Speed clamped: {speed:.1f}m/s -> {clamped:.1f}m/s (max: {limits.max_speed}m/s)"
        )
    return clamped


def clamp_velocity(velocity, limits: SafetyLimits):
    """Clamp a velocity vector to within safety limits, preserving direction."""
    from ..types import VectorNED

    if velocity is None or not limits.enable_speed_limits:
        return velocity

    h_speed = velocity.magnitude(ignore_vertical=True)
    v_speed = abs(velocity.down)

    north, east, down = velocity.north, velocity.east, velocity.down

    if h_speed > limits.max_speed and h_speed > 0:
        scale = limits.max_speed / h_speed
        north, east = velocity.north * scale, velocity.east * scale
        logger.warning(
            f"Horizontal velocity scaled: {h_speed:.1f}m/s -> {limits.max_speed}m/s"
        )

    if v_speed > limits.max_vertical_speed:
        sign = 1 if velocity.down > 0 else -1
        down = sign * limits.max_vertical_speed
        logger.warning(
            f"Vertical velocity clamped: {velocity.down:.1f}m/s -> {down:.1f}m/s"
        )

    if (
        north != velocity.north
        or east != velocity.east
        or down != velocity.down
    ):
        return VectorNED(north, east, down)
    return velocity


async def run_preflight_checks(
    vehicle: "Vehicle", limits: SafetyLimits
) -> PreflightCheckResult:
    """
    Run comprehensive pre-flight safety checks.

    This includes checks that would be performed before takeoff in the AERPAW
    environment, where operators and safety pilots verify vehicle state via
    the OEO parameter checker.

    Checks performed:
    - Safety configuration validity
    - GPS fix quality and satellite count
    - Battery level vs takeoff threshold (not just flight thresholds)
    - Battery voltage (if thresholds are configured)
    - Vehicle connection status

    Args:
        vehicle: The vehicle to check
        limits: Safety limits configuration

    Returns:
        PreflightCheckResult with all check results and warnings
    """
    result = PreflightCheckResult()

    # Check safety limits configuration
    config_errors = limits.validate()
    if config_errors:
        result.add_check(
            "config",
            SafetyCheckResult.fail(
                SafetyViolationType.INVALID_PARAMETER,
                f"Invalid safety config: {', '.join(config_errors)}",
            ),
        )
    else:
        result.add_check("config", SafetyCheckResult.ok())

    # Check GPS
    if limits.require_gps_fix:
        has_fix = getattr(vehicle.gps, "has_fix", vehicle.gps.fix_type >= 2)
        satellites = getattr(
            vehicle.gps,
            "satellites",
            getattr(vehicle.gps, "satellites_visible", 0),
        )
        gps_ok = has_fix and satellites >= limits.min_satellites

        if not gps_ok:
            result.add_check(
                "gps",
                SafetyCheckResult.fail(
                    SafetyViolationType.GPS_POOR,
                    f"GPS: {satellites} satellites (need {limits.min_satellites}), fix: {has_fix}",
                ),
            )
        else:
            result.add_check("gps", SafetyCheckResult.ok())

    # Check battery percentage against TAKEOFF threshold (not just flight threshold)
    battery_pct = vehicle.battery.percentage
    battery_voltage = vehicle.battery.voltage

    # Skip battery checks if telemetry hasn't been received (0% and 0V)
    if battery_pct == 0.0 and battery_voltage == 0.0:
        result.add_warning(
            "Battery telemetry not yet received - skipping battery preflight check"
        )
        result.add_check("battery_level", SafetyCheckResult.ok())
    else:
        if battery_pct <= limits.critical_battery_percent:
            result.add_check(
                "battery_level",
                SafetyCheckResult.fail(
                    SafetyViolationType.BATTERY_CRITICAL,
                    f"Battery critically low: {battery_pct:.1f}%",
                ),
            )
        elif battery_pct < limits.min_takeoff_battery_percent:
            result.add_check(
                "battery_level",
                SafetyCheckResult.fail(
                    SafetyViolationType.BATTERY_INSUFFICIENT_FOR_TAKEOFF,
                    f"Battery too low for takeoff: {battery_pct:.1f}% (min: {limits.min_takeoff_battery_percent}%)",
                ),
            )
        else:
            result.add_check("battery_level", SafetyCheckResult.ok())

        # Add warning if battery is above takeoff threshold but should be monitored
        if limits.min_takeoff_battery_percent <= battery_pct < 50:
            result.add_warning(
                f"Battery at {battery_pct:.1f}% - consider charging before long flights"
            )

    # Check battery voltage if thresholds are configured and we have telemetry
    if limits.min_takeoff_voltage is not None:
        if battery_pct == 0.0 and battery_voltage == 0.0:
            # Already handled above
            pass
        elif battery_voltage < limits.min_takeoff_voltage:
            result.add_check(
                "battery_voltage",
                SafetyCheckResult.fail(
                    SafetyViolationType.BATTERY_INSUFFICIENT_FOR_TAKEOFF,
                    f"Battery voltage too low for takeoff: {battery_voltage:.2f}V (min: {limits.min_takeoff_voltage:.2f}V)",
                ),
            )
        else:
            result.add_check("battery_voltage", SafetyCheckResult.ok())

    # Check for critical voltage
    if limits.critical_voltage is not None:
        voltage = vehicle.battery.voltage
        if voltage < limits.critical_voltage:
            result.add_check(
                "critical_voltage",
                SafetyCheckResult.fail(
                    SafetyViolationType.BATTERY_CRITICAL,
                    f"Battery voltage critical: {voltage:.2f}V (critical: {limits.critical_voltage:.2f}V)",
                ),
            )
        else:
            result.add_check("critical_voltage", SafetyCheckResult.ok())

    # Check connection
    if hasattr(vehicle, "_connected"):
        if not vehicle._connected:
            result.add_check(
                "connection",
                SafetyCheckResult.fail(
                    SafetyViolationType.INVALID_PARAMETER,
                    "Not connected to vehicle",
                ),
            )
        else:
            result.add_check("connection", SafetyCheckResult.ok())

    # Check heartbeat health
    if hasattr(vehicle, "connection_healthy"):
        if not vehicle.connection_healthy:
            result.add_check(
                "heartbeat",
                SafetyCheckResult.fail(
                    SafetyViolationType.INVALID_PARAMETER,
                    "Heartbeat not healthy - check connection",
                ),
            )
        else:
            result.add_check("heartbeat", SafetyCheckResult.ok())

    if result.passed:
        logger.info(f"Pre-flight checks passed ({len(result.checks)} checks)")
    else:
        logger.warning(f"Pre-flight checks FAILED: {result.failed_checks}")

    return result


__all__ = [
    "validate_coordinate",
    "validate_altitude",
    "validate_speed",
    "validate_velocity",
    "validate_timeout",
    "validate_tolerance",
    "clamp_speed",
    "clamp_velocity",
    "run_preflight_checks",
]
