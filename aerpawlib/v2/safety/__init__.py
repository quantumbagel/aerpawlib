"""
Safety subpackage for aerpawlib v2 API.

This package is organized into focused modules:
- types: Enums and result types
- limits: SafetyLimits and SafetyConfig dataclasses
- validation: Parameter validation functions
- checker: ZMQ client/server for geofence validation (requires pyzmq)
- monitor: Continuous flight safety monitoring
- oeo: OEO notification interface
- connection: Connection handling for failures and disconnects
"""

from __future__ import annotations

# Types - always available
from .types import (
    SafetyViolationType,
    RequestType,
    VehicleType,
    ValidationResult,
    SafetyCheckResult,
    PreflightCheckResult,
    DisconnectReason,
)

# Limits/config - always available
from .limits import SafetyLimits, SafetyConfig

# Validation functions - always available
from .validation import (
    validate_coordinate,
    validate_altitude,
    validate_speed,
    validate_velocity,
    validate_timeout,
    validate_tolerance,
    clamp_speed,
    clamp_velocity,
    run_preflight_checks,
)

# Monitor - always available
from .monitor import SafetyMonitor


# Connection handling - always available
from .connection import (
    ConnectionState,
    ConnectionEvent,
    ConnectionHandler,
)

# Checker requires ZMQ
from .checker import (
    SafetyCheckerClient,
    SafetyCheckerServer,
    validate_waypoint_with_checker,
    validate_speed_with_checker,
    validate_takeoff_with_checker,
)


# Re-export exceptions for convenience (from the main exceptions module)
from ..exceptions import (
    SafetyError,
    GeofenceViolationError,
    SpeedLimitExceededError,
    ParameterValidationError,
    PreflightCheckError,
)

__all__ = [
    # Enums
    "SafetyViolationType",
    "RequestType",
    "VehicleType",
    "DisconnectReason",
    # Configuration
    "SafetyLimits",
    "SafetyConfig",
    # Result types
    "ValidationResult",
    "SafetyCheckResult",
    "PreflightCheckResult",
    # Client/Server (may be None if ZMQ not available)
    "SafetyCheckerClient",
    "SafetyCheckerServer",
    # Monitoring
    "SafetyMonitor",
    # Connection handling
    "ConnectionState",
    "ConnectionEvent",
    "ConnectionHandler",
    # Validation functions
    "validate_coordinate",
    "validate_altitude",
    "validate_speed",
    "validate_velocity",
    "validate_timeout",
    "validate_tolerance",
    # Server validation functions
    "validate_waypoint_with_checker",
    "validate_speed_with_checker",
    "validate_takeoff_with_checker",
    # Clamping functions
    "clamp_speed",
    "clamp_velocity",
    # Pre-flight checks
    "run_preflight_checks",
    # Exceptions (re-exported)
    "SafetyError",
    "PreflightCheckError",
    "ParameterValidationError",
    "SpeedLimitExceededError",
    "GeofenceViolationError",
]
