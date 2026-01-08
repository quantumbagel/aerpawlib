"""
Safety types for aerpawlib v2 API.

Contains enums and result types for safety operations.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, List, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    pass  # No imports needed here for now


class SafetyViolationType(Enum):
    """Types of safety limit violations."""
    SPEED_TOO_HIGH = auto()
    VERTICAL_SPEED_TOO_HIGH = auto()
    BATTERY_LOW = auto()
    BATTERY_CRITICAL = auto()
    GPS_POOR = auto()
    NO_GPS_FIX = auto()
    INVALID_COORDINATE = auto()
    INVALID_ALTITUDE = auto()
    INVALID_SPEED = auto()
    INVALID_PARAMETER = auto()
    PREFLIGHT_FAILED = auto()
    GEOFENCE_VIOLATION = auto()
    NO_GO_ZONE_VIOLATION = auto()
    PATH_LEAVES_GEOFENCE = auto()
    PATH_ENTERS_NO_GO_ZONE = auto()
    ALTITUDE_OUT_OF_BOUNDS = auto()


class RequestType(Enum):
    """Types of safety checker requests."""
    SERVER_STATUS = "server_status_req"
    VALIDATE_WAYPOINT = "validate_waypoint_req"
    VALIDATE_SPEED = "validate_change_speed_req"
    VALIDATE_TAKEOFF = "validate_takeoff_req"
    VALIDATE_LANDING = "validate_landing_req"


class VehicleType(Enum):
    """Supported vehicle types."""
    ROVER = "rover"
    COPTER = "copter"


@dataclass
class ValidationResult:
    """
    Result of a safety validation check from the server.

    Attributes:
        valid: Whether the command passed validation
        message: Error message if invalid, empty string if valid
        request_type: The type of request that was validated
    """
    valid: bool
    message: str = ""
    request_type: Optional[RequestType] = None

    def __bool__(self) -> bool:
        return self.valid


@dataclass
class SafetyCheckResult:
    """Result of a client-side safety check."""
    passed: bool
    violation: Optional[SafetyViolationType] = None
    message: str = ""
    value: Optional[float] = None
    limit: Optional[float] = None

    def __bool__(self) -> bool:
        return self.passed

    @classmethod
    def ok(cls) -> "SafetyCheckResult":
        """Create a passing result."""
        return cls(passed=True)

    @classmethod
    def fail(
        cls,
        violation: SafetyViolationType,
        message: str,
        value: Optional[float] = None,
        limit: Optional[float] = None,
    ) -> "SafetyCheckResult":
        """Create a failing result."""
        return cls(passed=False, violation=violation, message=message, value=value, limit=limit)

    @classmethod
    def from_validation_result(cls, result: ValidationResult) -> "SafetyCheckResult":
        """Convert a ValidationResult from safety checker server."""
        if result.valid:
            return cls.ok()

        # Determine violation type from message content
        violation = SafetyViolationType.GEOFENCE_VIOLATION
        msg = result.message.lower()

        if "outside" in msg and "geofence" in msg:
            violation = SafetyViolationType.GEOFENCE_VIOLATION
        elif "no-go" in msg or "no go" in msg:
            violation = (SafetyViolationType.PATH_ENTERS_NO_GO_ZONE
                        if "path" in msg else SafetyViolationType.NO_GO_ZONE_VIOLATION)
        elif "leaves" in msg and "geofence" in msg:
            violation = SafetyViolationType.PATH_LEAVES_GEOFENCE
        elif "altitude" in msg:
            violation = SafetyViolationType.ALTITUDE_OUT_OF_BOUNDS
        elif "speed" in msg:
            violation = SafetyViolationType.SPEED_TOO_HIGH

        return cls.fail(violation, result.message)


class PreflightCheckResult:
    """Result of pre-flight safety checks."""

    def __init__(self):
        self.checks: Dict[str, SafetyCheckResult] = {}
        self.warnings: List[str] = []

    @property
    def passed(self) -> bool:
        """True if all checks passed."""
        return all(check.passed for check in self.checks.values())

    @property
    def failed_checks(self) -> List[str]:
        """List of failed check names."""
        return [name for name, check in self.checks.items() if not check.passed]

    def add_check(self, name: str, result: SafetyCheckResult) -> None:
        """Add a check result."""
        self.checks[name] = result

    def add_warning(self, message: str) -> None:
        """Add a warning message."""
        self.warnings.append(message)

    def __str__(self) -> str:
        if self.passed:
            msg = f"Pre-flight checks PASSED ({len(self.checks)} checks)"
            if self.warnings:
                msg += f" with {len(self.warnings)} warning(s)"
            return msg
        return f"Pre-flight checks FAILED: {', '.join(self.failed_checks)}"

    def __bool__(self) -> bool:
        return self.passed

    def summary(self) -> str:
        """Get a detailed summary of all checks."""
        lines = ["Pre-flight Check Results:", "=" * 40]
        for name, check in self.checks.items():
            status = "✓ PASS" if check.passed else "✗ FAIL"
            line = f"  {status}: {name}"
            if not check.passed and check.message:
                line += f" - {check.message}"
            lines.append(line)
        if self.warnings:
            lines.extend(["", "Warnings:"] + [f"  ⚠ {w}" for w in self.warnings])
        lines.extend(["=" * 40, f"Result: {'PASSED' if self.passed else 'FAILED'}"])
        return "\n".join(lines)


__all__ = [
    "SafetyViolationType",
    "RequestType",
    "VehicleType",
    "ValidationResult",
    "SafetyCheckResult",
    "PreflightCheckResult",
]

