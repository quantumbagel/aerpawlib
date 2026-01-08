"""
Exception hierarchy for aerpawlib v2 API.

Simplified exception structure using a few base classes with error codes,
instead of 30+ individual exception classes.
"""
from __future__ import annotations

from enum import Enum, auto
from typing import Any, Dict, Optional


class ErrorCode(Enum):
    """Error codes for categorizing exceptions."""
    # Connection errors (1xx)
    CONNECTION_FAILED = 100
    CONNECTION_TIMEOUT = 101
    HEARTBEAT_LOST = 102
    RECONNECTION_FAILED = 103

    # Command errors (2xx)
    COMMAND_FAILED = 200
    ARM_FAILED = 201
    DISARM_FAILED = 202
    TAKEOFF_FAILED = 203
    LANDING_FAILED = 204
    NAVIGATION_FAILED = 205
    MODE_CHANGE_FAILED = 206
    OFFBOARD_FAILED = 207

    # Timeout errors (3xx)
    OPERATION_TIMEOUT = 300
    GOTO_TIMEOUT = 301
    TAKEOFF_TIMEOUT = 302
    LANDING_TIMEOUT = 303

    # Abort errors (4xx)
    OPERATION_ABORTED = 400
    USER_ABORT = 401
    COMMAND_CANCELLED = 402
    SAFETY_ABORT = 403

    # Safety errors (5xx)
    SAFETY_VIOLATION = 500
    GEOFENCE_VIOLATION = 501
    ALTITUDE_VIOLATION = 502
    SPEED_VIOLATION = 503
    PARAMETER_INVALID = 504

    # Preflight errors (6xx)
    PREFLIGHT_FAILED = 600
    GPS_NOT_READY = 601
    BATTERY_LOW = 602
    NOT_ARMABLE = 603

    # State machine errors (7xx)
    STATE_MACHINE_ERROR = 700
    INVALID_STATE = 701
    NO_INITIAL_STATE = 702
    MULTIPLE_INITIAL_STATES = 703


class ErrorSeverity(Enum):
    """Severity level of an error."""
    WARNING = auto()
    ERROR = auto()
    CRITICAL = auto()
    FATAL = auto()


class AerpawlibError(Exception):
    """
    Base exception for all aerpawlib errors.

    All errors have:
    - message: Human-readable description
    - code: ErrorCode for programmatic handling
    - severity: How serious the error is
    - details: Additional context as key-value pairs
    - recoverable: Whether the operation can be retried
    """

    def __init__(
        self,
        message: str,
        code: ErrorCode = ErrorCode.COMMAND_FAILED,
        severity: ErrorSeverity = ErrorSeverity.ERROR,
        recoverable: bool = True,
        **details: Any
    ):
        self.message = message
        self.code = code
        self.severity = severity
        self.recoverable = recoverable
        self.details: Dict[str, Any] = {k: v for k, v in details.items() if v is not None}
        super().__init__(message)

    def __str__(self) -> str:
        base = f"[{self.severity.name}] {self.message}"
        if self.details:
            detail_str = ", ".join(f"{k}={v}" for k, v in self.details.items())
            base += f" ({detail_str})"
        return base


# Specific exception classes for common error categories

class ConnectionError(AerpawlibError):
    """Vehicle connection failed or lost."""

    def __init__(
        self,
        message: str = "Connection failed",
        code: ErrorCode = ErrorCode.CONNECTION_FAILED,
        address: Optional[str] = None,
        timeout: Optional[float] = None,
        attempt: Optional[int] = None,
        **details: Any
    ):
        super().__init__(
            message, code, ErrorSeverity.ERROR, True,
            address=address, timeout=timeout, attempt=attempt, **details
        )


class CommandError(AerpawlibError):
    """Vehicle command failed."""

    def __init__(
        self,
        message: str = "Command failed",
        code: ErrorCode = ErrorCode.COMMAND_FAILED,
        command: Optional[str] = None,
        reason: Optional[str] = None,
        **details: Any
    ):
        super().__init__(
            message, code, ErrorSeverity.ERROR, True,
            command=command, reason=reason, **details
        )


class TimeoutError(AerpawlibError):
    """Operation timed out."""

    def __init__(
        self,
        message: str = "Operation timed out",
        code: ErrorCode = ErrorCode.OPERATION_TIMEOUT,
        operation: Optional[str] = None,
        timeout: Optional[float] = None,
        **details: Any
    ):
        super().__init__(
            message, code, ErrorSeverity.ERROR, True,
            operation=operation, timeout=timeout, **details
        )


class AbortError(AerpawlibError):
    """Operation aborted."""

    def __init__(
        self,
        message: str = "Operation aborted",
        code: ErrorCode = ErrorCode.OPERATION_ABORTED,
        operation: Optional[str] = None,
        reason: Optional[str] = None,
        **details: Any
    ):
        super().__init__(
            message, code, ErrorSeverity.WARNING, True,
            operation=operation, reason=reason, **details
        )


class SafetyError(AerpawlibError):
    """Safety-related error."""

    def __init__(
        self,
        message: str = "Safety violation",
        code: ErrorCode = ErrorCode.SAFETY_VIOLATION,
        violation: Optional[str] = None,
        **details: Any
    ):
        super().__init__(
            message, code, ErrorSeverity.WARNING, True,
            violation=violation, **details
        )


class PreflightError(AerpawlibError):
    """Pre-flight check failed."""

    def __init__(
        self,
        message: str = "Pre-flight check failed",
        code: ErrorCode = ErrorCode.PREFLIGHT_FAILED,
        check_name: Optional[str] = None,
        reason: Optional[str] = None,
        **details: Any
    ):
        super().__init__(
            message, code, ErrorSeverity.ERROR, True,
            check_name=check_name, reason=reason, **details
        )


class StateMachineError(AerpawlibError):
    """State machine error."""

    def __init__(
        self,
        message: str = "State machine error",
        code: ErrorCode = ErrorCode.STATE_MACHINE_ERROR,
        current_state: Optional[str] = None,
        **details: Any
    ):
        super().__init__(
            message, code, ErrorSeverity.ERROR, True,
            current_state=current_state, **details
        )


# Factory functions for specific error types (replacing 30+ individual classes)

def ConnectionTimeoutError(
    timeout: float = 30.0,
    address: Optional[str] = None,
    attempt: Optional[int] = None,
    max_attempts: Optional[int] = None
) -> ConnectionError:
    """Create a connection timeout error."""
    return ConnectionError(
        "Connection timed out",
        ErrorCode.CONNECTION_TIMEOUT,
        address=address,
        timeout=timeout,
        attempt=attempt,
        max_attempts=max_attempts
    )


def HeartbeatLostError(last_heartbeat: Optional[float] = None) -> ConnectionError:
    """Create a heartbeat lost error."""
    return ConnectionError(
        "Lost heartbeat from vehicle",
        ErrorCode.HEARTBEAT_LOST,
        seconds_since_heartbeat=last_heartbeat
    )


def ReconnectionError(
    address: Optional[str] = None,
    attempt: Optional[int] = None,
    max_attempts: Optional[int] = None
) -> ConnectionError:
    """Create a reconnection failed error."""
    err = ConnectionError(
        "Failed to reconnect to vehicle",
        ErrorCode.RECONNECTION_FAILED,
        address=address,
        attempt=attempt,
        max_attempts=max_attempts
    )
    err.recoverable = False
    return err


def ArmError(message: str = "Failed to arm vehicle", reason: Optional[str] = None) -> CommandError:
    """Create an arm failed error."""
    return CommandError(message, ErrorCode.ARM_FAILED, command="arm", reason=reason)


def DisarmError(message: str = "Failed to disarm vehicle", reason: Optional[str] = None) -> CommandError:
    """Create a disarm failed error."""
    return CommandError(message, ErrorCode.DISARM_FAILED, command="disarm", reason=reason)


def TakeoffError(
    message: str = "Takeoff failed",
    target_altitude: Optional[float] = None,
    current_altitude: Optional[float] = None,
    reason: Optional[str] = None
) -> CommandError:
    """Create a takeoff failed error."""
    return CommandError(
        message, ErrorCode.TAKEOFF_FAILED, command="takeoff", reason=reason,
        target_altitude=target_altitude, current_altitude=current_altitude
    )


def LandingError(
    message: str = "Landing failed",
    current_altitude: Optional[float] = None,
    reason: Optional[str] = None
) -> CommandError:
    """Create a landing failed error."""
    err = CommandError(
        message, ErrorCode.LANDING_FAILED, command="land", reason=reason,
        current_altitude=current_altitude
    )
    err.severity = ErrorSeverity.CRITICAL
    return err


def NavigationError(
    message: str = "Navigation command failed",
    target: Optional[Any] = None,
    current_position: Optional[Any] = None,
    reason: Optional[str] = None
) -> CommandError:
    """Create a navigation error."""
    return CommandError(
        message, ErrorCode.NAVIGATION_FAILED, command="goto", reason=reason,
        target=str(target) if target else None,
        current_position=str(current_position) if current_position else None
    )


def ModeChangeError(
    message: str = "Failed to change flight mode",
    target_mode: Optional[str] = None,
    current_mode: Optional[str] = None,
    reason: Optional[str] = None
) -> CommandError:
    """Create a mode change error."""
    return CommandError(
        message, ErrorCode.MODE_CHANGE_FAILED, command="set_mode", reason=reason,
        target_mode=target_mode, current_mode=current_mode
    )


def OffboardError(message: str = "Offboard mode operation failed", reason: Optional[str] = None) -> CommandError:
    """Create an offboard mode error."""
    return CommandError(message, ErrorCode.OFFBOARD_FAILED, command="offboard", reason=reason)


def GotoTimeoutError(
    timeout: Optional[float] = None,
    target: Optional[Any] = None,
    distance_remaining: Optional[float] = None
) -> TimeoutError:
    """Create a goto timeout error."""
    return TimeoutError(
        "Goto operation timed out",
        ErrorCode.GOTO_TIMEOUT,
        operation="goto",
        timeout=timeout,
        target=str(target) if target else None,
        distance_remaining=distance_remaining
    )


def TakeoffTimeoutError(
    timeout: Optional[float] = None,
    target_altitude: Optional[float] = None,
    current_altitude: Optional[float] = None
) -> TimeoutError:
    """Create a takeoff timeout error."""
    return TimeoutError(
        "Takeoff timed out",
        ErrorCode.TAKEOFF_TIMEOUT,
        operation="takeoff",
        timeout=timeout,
        target_altitude=target_altitude,
        current_altitude=current_altitude
    )


def LandingTimeoutError(
    timeout: Optional[float] = None,
    current_altitude: Optional[float] = None
) -> TimeoutError:
    """Create a landing timeout error."""
    err = TimeoutError(
        "Landing timed out",
        ErrorCode.LANDING_TIMEOUT,
        operation="land",
        timeout=timeout,
        current_altitude=current_altitude
    )
    err.severity = ErrorSeverity.CRITICAL
    return err


def UserAbortError(operation: Optional[str] = None) -> AbortError:
    """Create a user abort error."""
    return AbortError("User aborted operation", ErrorCode.USER_ABORT, operation=operation, reason="user_request")


def CommandCancelledError(command: Optional[str] = None) -> AbortError:
    """Create a command cancelled error."""
    return AbortError(
        f"Command was cancelled" + (f": {command}" if command else ""),
        ErrorCode.COMMAND_CANCELLED,
        reason="cancelled",
        command=command
    )


def SafetyAbortError(violation: Optional[str] = None) -> AbortError:
    """Create a safety abort error."""
    err = AbortError(
        "Safety system triggered abort",
        ErrorCode.SAFETY_ABORT,
        reason="safety_violation",
        violation=violation
    )
    err.severity = ErrorSeverity.CRITICAL
    return err


def GeofenceViolationError(
    message: str = "Command would violate geofence",
    current_position: Optional[Any] = None,
    target_position: Optional[Any] = None,
    geofence_name: Optional[str] = None
) -> SafetyError:
    """Create a geofence violation error."""
    return SafetyError(
        message,
        ErrorCode.GEOFENCE_VIOLATION,
        current_position=str(current_position) if current_position else None,
        target_position=str(target_position) if target_position else None,
        geofence=geofence_name
    )


def AltitudeViolationError(
    message: str = "Command would violate altitude limits",
    target_altitude: Optional[float] = None,
    min_altitude: Optional[float] = None,
    max_altitude: Optional[float] = None
) -> SafetyError:
    """Create an altitude violation error."""
    return SafetyError(
        message,
        ErrorCode.ALTITUDE_VIOLATION,
        target_altitude=target_altitude,
        min_altitude=min_altitude,
        max_altitude=max_altitude
    )


def SpeedViolationError(
    message: str = "Command would violate speed limits",
    requested_speed: Optional[float] = None,
    max_speed: Optional[float] = None
) -> SafetyError:
    """Create a speed violation error."""
    return SafetyError(
        message,
        ErrorCode.SPEED_VIOLATION,
        requested_speed=requested_speed,
        max_speed=max_speed
    )


def SpeedLimitExceededError(
    message: str = "Speed limit exceeded",
    value: Optional[float] = None,
    limit: Optional[float] = None
) -> SafetyError:
    """Create a speed limit exceeded error."""
    return SafetyError(message, ErrorCode.SPEED_VIOLATION, value=value, limit=limit)


def ParameterValidationError(
    message: str = "Invalid parameter",
    parameter: Optional[str] = None,
    value: Optional[Any] = None
) -> SafetyError:
    """Create a parameter validation error."""
    return SafetyError(
        message,
        ErrorCode.PARAMETER_INVALID,
        parameter=parameter,
        value=str(value) if value is not None else None
    )


def PreflightCheckError(
    message: str = "Pre-flight checks failed",
    result: Optional[Any] = None
) -> PreflightError:
    """Create a preflight check error."""
    failed_checks = None
    if result is not None and hasattr(result, 'failed_checks'):
        failed_checks = result.failed_checks
        if message == "Pre-flight checks failed" and failed_checks:
            message = f"Pre-flight checks failed: {', '.join(failed_checks)}"
    return PreflightError(message, ErrorCode.PREFLIGHT_FAILED, failed_checks=failed_checks)


def GPSError(
    message: str = "GPS not ready",
    satellites: Optional[int] = None,
    fix_type: Optional[int] = None
) -> PreflightError:
    """Create a GPS error."""
    return PreflightError(message, ErrorCode.GPS_NOT_READY, check_name="gps", satellites=satellites, fix_type=fix_type)


def BatteryError(
    message: str = "Battery level too low",
    percentage: Optional[float] = None,
    minimum_required: Optional[float] = None
) -> PreflightError:
    """Create a battery error."""
    return PreflightError(
        message, ErrorCode.BATTERY_LOW, check_name="battery",
        percentage=percentage, minimum_required=minimum_required
    )


def NotArmableError(message: str = "Vehicle is not ready to arm", reasons: Optional[list] = None) -> PreflightError:
    """Create a not armable error."""
    return PreflightError(message, ErrorCode.NOT_ARMABLE, check_name="armable", reasons=reasons)


def InvalidStateError(
    message: str = "Invalid state transition",
    current_state: Optional[str] = None,
    target_state: Optional[str] = None,
    available_states: Optional[list] = None
) -> StateMachineError:
    """Create an invalid state error."""
    return StateMachineError(
        message, ErrorCode.INVALID_STATE, current_state=current_state,
        target_state=target_state, available_states=available_states
    )


def NoInitialStateError() -> StateMachineError:
    """Create a no initial state error."""
    err = StateMachineError("No initial state defined", ErrorCode.NO_INITIAL_STATE)
    err.recoverable = False
    return err


def MultipleInitialStatesError(states: Optional[list] = None) -> StateMachineError:
    """Create a multiple initial states error."""
    err = StateMachineError("Multiple initial states defined", ErrorCode.MULTIPLE_INITIAL_STATES, initial_states=states)
    err.recoverable = False
    return err


__all__ = [
    # Enums
    "ErrorCode",
    "ErrorSeverity",
    # Base classes
    "AerpawlibError",
    "ConnectionError",
    "CommandError",
    "TimeoutError",
    "AbortError",
    "SafetyError",
    "PreflightError",
    "StateMachineError",
    # Factory functions (for backwards compatibility with old class names)
    "ConnectionTimeoutError",
    "HeartbeatLostError",
    "ReconnectionError",
    "ArmError",
    "DisarmError",
    "TakeoffError",
    "LandingError",
    "NavigationError",
    "ModeChangeError",
    "OffboardError",
    "GotoTimeoutError",
    "TakeoffTimeoutError",
    "LandingTimeoutError",
    "UserAbortError",
    "CommandCancelledError",
    "SafetyAbortError",
    "GeofenceViolationError",
    "AltitudeViolationError",
    "SpeedViolationError",
    "SpeedLimitExceededError",
    "ParameterValidationError",
    "PreflightCheckError",
    "GPSError",
    "BatteryError",
    "NotArmableError",
    "InvalidStateError",
    "NoInitialStateError",
    "MultipleInitialStatesError",
]

