"""
Custom exceptions for aerpawlib v1.

Provides a meaningful exception hierarchy for better error handling
while maintaining backward compatibility with existing code that catches
generic Exception.

All exceptions inherit from AerpawlibError, which inherits from Exception,
so existing catch blocks will continue to work.

@author: Julian Reder
"""

from typing import Optional, Any


class AerpawlibError(Exception):
    """
    Base exception for all aerpawlib errors.

    All custom exceptions inherit from this, allowing users to catch
    either specific exceptions or all aerpawlib errors with a single handler.

    Attributes:
        message: Human-readable error description
        original_error: The underlying exception that caused this error, if any
    """

    def __init__(
        self, message: str, original_error: Optional[Exception] = None
    ):
        self.message = message
        self.original_error = original_error
        super().__init__(message)

    def __str__(self) -> str:
        if self.original_error:
            return f"{self.message} (caused by: {self.original_error})"
        return self.message


# =============================================================================
# Connection Errors
# =============================================================================


class ConnectionError(AerpawlibError):
    """Base class for connection-related errors."""

    pass


class ConnectionTimeoutError(ConnectionError):
    """Raised when connection to the vehicle times out."""

    def __init__(self, timeout_seconds: float, message: Optional[str] = None):
        self.timeout_seconds = timeout_seconds
        msg = (
            message or f"Connection timed out after {timeout_seconds} seconds"
        )
        super().__init__(msg)


class HeartbeatLostError(ConnectionError):
    """Raised when the vehicle heartbeat is lost."""

    def __init__(
        self, last_heartbeat_age: float = 0.0, message: Optional[str] = None
    ):
        self.last_heartbeat_age = last_heartbeat_age
        msg = (
            message
            or f"Vehicle heartbeat lost (last heartbeat {last_heartbeat_age:.1f}s ago)"
        )
        super().__init__(msg)


class MAVSDKNotInstalledError(ConnectionError):
    """Raised when MAVSDK is not installed."""

    def __init__(self):
        super().__init__(
            "MAVSDK is not installed. Install with: pip install mavsdk"
        )


# =============================================================================
# AERPAW Platform Errors
# =============================================================================


class AERPAWPlatformError(AerpawlibError):
    """Base class for AERPAW platform-specific errors."""

    pass


class NotInAERPAWEnvironmentError(AERPAWPlatformError):
    """Raised when AERPAW platform features are used outside the platform."""

    def __init__(self, feature: str):
        super().__init__(
            f"'{feature}' requires AERPAW platform environment. "
            "This feature is not available in standalone/SITL mode."
        )
        self.feature = feature


# =============================================================================
# Command Errors
# =============================================================================


class CommandError(AerpawlibError):
    """Base class for command execution errors."""

    pass


class ArmError(CommandError):
    """Raised when arming the vehicle fails."""

    def __init__(
        self,
        reason: str = "Unknown",
        original_error: Optional[Exception] = None,
    ):
        super().__init__(f"Failed to arm vehicle: {reason}", original_error)


class DisarmError(CommandError):
    """Raised when disarming the vehicle fails."""

    def __init__(
        self,
        reason: str = "Unknown",
        original_error: Optional[Exception] = None,
    ):
        super().__init__(f"Failed to disarm vehicle: {reason}", original_error)


class TakeoffError(CommandError):
    """Raised when takeoff fails."""

    def __init__(
        self,
        reason: str = "Unknown",
        original_error: Optional[Exception] = None,
    ):
        super().__init__(f"Takeoff failed: {reason}", original_error)


class LandingError(CommandError):
    """Raised when landing fails."""

    def __init__(
        self,
        reason: str = "Unknown",
        original_error: Optional[Exception] = None,
    ):
        super().__init__(f"Landing failed: {reason}", original_error)


class NavigationError(CommandError):
    """Raised when navigation/goto command fails."""

    def __init__(
        self,
        reason: str = "Unknown",
        original_error: Optional[Exception] = None,
    ):
        super().__init__(f"Navigation failed: {reason}", original_error)


class VelocityError(CommandError):
    """Raised when velocity command fails."""

    def __init__(
        self,
        reason: str = "Unknown",
        original_error: Optional[Exception] = None,
    ):
        super().__init__(f"Set velocity failed: {reason}", original_error)


class HeadingError(CommandError):
    """Raised when set heading command fails."""

    def __init__(
        self,
        reason: str = "Unknown",
        original_error: Optional[Exception] = None,
    ):
        super().__init__(f"Set heading failed: {reason}", original_error)


class RTLError(CommandError):
    """Raised when return-to-launch fails."""

    def __init__(
        self,
        reason: str = "Unknown",
        original_error: Optional[Exception] = None,
    ):
        super().__init__(f"Return to launch failed: {reason}", original_error)


# =============================================================================
# State Errors
# =============================================================================


class StateError(AerpawlibError):
    """Base class for vehicle state errors."""

    pass


class NotArmableError(StateError):
    """Raised when attempting to arm a vehicle that is not ready."""

    def __init__(self, reason: str = "Vehicle not in armable state"):
        super().__init__(f"Cannot arm: {reason}")


class NotConnectedError(StateError):
    """Raised when attempting to command a disconnected vehicle."""

    def __init__(self, message: str = "Vehicle is not connected"):
        super().__init__(message)


class AbortedError(StateError):
    """Raised when an operation was aborted."""

    def __init__(self, message: str = "Operation was aborted"):
        super().__init__(message)


# =============================================================================
# Validation Errors
# =============================================================================


class ValidationError(AerpawlibError):
    """Base class for input validation errors."""

    pass


class InvalidToleranceError(ValidationError):
    """Raised when an invalid tolerance value is provided."""

    def __init__(self, value: float, min_val: float, max_val: float):
        super().__init__(
            f"Invalid tolerance {value}: must be between {min_val} and {max_val}"
        )
        self.value = value
        self.min_val = min_val
        self.max_val = max_val


class InvalidAltitudeError(ValidationError):
    """Raised when an invalid altitude is provided."""

    def __init__(self, value: float, min_val: float, max_val: float):
        super().__init__(
            f"Invalid altitude {value}m: must be between {min_val}m and {max_val}m"
        )
        self.value = value
        self.min_val = min_val
        self.max_val = max_val


class InvalidSpeedError(ValidationError):
    """Raised when an invalid speed is provided."""

    def __init__(self, value: float, min_val: float, max_val: float):
        super().__init__(
            f"Invalid speed {value} m/s: must be between {min_val} and {max_val} m/s"
        )
        self.value = value
        self.min_val = min_val
        self.max_val = max_val


# =============================================================================
# Feature Errors
# =============================================================================


class NotImplementedForVehicleError(AerpawlibError):
    """Raised when a feature is not available for a vehicle type."""

    def __init__(self, feature: str, vehicle_type: str):
        super().__init__(
            f"'{feature}' is not implemented for {vehicle_type} vehicles"
        )
        self.feature = feature
        self.vehicle_type = vehicle_type


# =============================================================================
# State Machine Errors
# =============================================================================


class StateMachineError(AerpawlibError):
    """Base class for state machine errors."""

    pass


class NoEntrypointError(StateMachineError):
    """Raised when a BasicRunner has no @entrypoint declared."""

    def __init__(self):
        super().__init__("No @entrypoint declared")


class InvalidStateError(StateMachineError):
    """Raised when state machine enters an unknown state."""

    def __init__(self, state_name: str, available_states: list):
        super().__init__(
            f"Illegal state '{state_name}'. Available states: {available_states}"
        )
        self.state_name = state_name
        self.available_states = available_states


class NoInitialStateError(StateMachineError):
    """Raised when state machine has no initial state."""

    def __init__(self):
        super().__init__("There is no initial state")


class MultipleInitialStatesError(StateMachineError):
    """Raised when state machine has multiple initial states."""

    def __init__(self):
        super().__init__("There may only be one initial state")


class InvalidStateNameError(StateMachineError):
    """Raised when a state is given an empty name."""

    def __init__(self):
        super().__init__("State name cannot be empty string")
