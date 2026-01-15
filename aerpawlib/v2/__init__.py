"""
aerpawlib v2 API - MAVSDK-based vehicle control.

This module uses lazy imports to reduce startup time and coupling.
Import only what you need, or use the top-level imports for convenience.

Example:
    # Import what you need
    from aerpawlib.v2 import Drone, Coordinate

    # Or import everything (lazy-loaded)
    from aerpawlib.v2 import *
"""

from __future__ import annotations


# These are always immediately available (lightweight)
from .types import (
    Coordinate,
    VectorNED,
    PositionNED,
    Attitude,
    Waypoint,
    DroneState,
    GPSInfo,
    BatteryInfo,
    DroneInfo,
    FlightInfo,
    FlightMode,
    LandedState,
    read_waypoints_from_plan,
)

# Lazy import registry
_lazy_imports = {
    # Vehicles
    "Vehicle": ".vehicle",
    "Drone": ".vehicle",
    "Rover": ".vehicle",
    "CommandHandle": ".vehicle",
    "CommandStatus": ".vehicle",
    "CommandResult": ".vehicle",
    # Safety (subpackage)
    "SafetyViolationType": ".safety",
    "RequestType": ".safety",
    "VehicleType": ".safety",
    "SafetyLimits": ".safety",
    "SafetyConfig": ".safety",
    "ValidationResult": ".safety",
    "SafetyCheckResult": ".safety",
    "PreflightCheckResult": ".safety",
    "SafetyCheckerClient": ".safety",
    "SafetyCheckerServer": ".safety",
    "SafetyMonitor": ".safety",
    "validate_coordinate": ".safety",
    "validate_altitude": ".safety",
    "validate_speed": ".safety",
    "validate_velocity": ".safety",
    "validate_timeout": ".safety",
    "validate_tolerance": ".safety",
    "validate_waypoint_with_checker": ".safety",
    "validate_speed_with_checker": ".safety",
    "validate_takeoff_with_checker": ".safety",
    "clamp_speed": ".safety",
    "clamp_velocity": ".safety",
    "run_preflight_checks": ".safety",
    "DisconnectReason": ".safety",
    # Connection handling
    "ConnectionState": ".safety",
    "ConnectionEvent": ".safety",
    "ConnectionHandler": ".safety",
    # Runners
    "Runner": ".runner",
    "BasicRunner": ".runner",
    "StateMachine": ".runner",
    "entrypoint": ".runner",
    "state": ".runner",
    "timed_state": ".runner",
    "background": ".runner",
    "at_init": ".runner",
    "sleep": ".runner",
    "in_background": ".runner",
    # Platform
    "AERPAWPlatform": ".aerpaw",
    "AERPAWConfig": ".aerpaw",
    "MessageSeverity": ".aerpaw",
    "AERPAWConnectionError": ".aerpaw",
    "AERPAWCheckpointError": ".aerpaw",
    "OEOClient": ".aerpaw",
    "NotificationSeverity": ".aerpaw",
    "NotificationType": ".aerpaw",
    "OEONotification": ".aerpaw",
    # ZMQ
    "ZMQPublisher": ".zmqutil",
    "ZMQSubscriber": ".zmqutil",
    "ZMQMessage": ".zmqutil",
    "ZMQProxyConfig": ".zmqutil",
    "MessageType": ".zmqutil",
    "run_zmq_proxy": ".zmqutil",
    # Geofence
    "GeofencePoint": ".geofence",
    "Polygon": ".geofence",
    "read_geofence": ".geofence",
    "is_inside_polygon": ".geofence",
    "segments_intersect": ".geofence",
    "path_crosses_polygon": ".geofence",
    # Testing
    "MockDrone": ".testing",
    "MockRover": ".testing",
    "MockState": ".testing",
    "MockGPS": ".testing",
    "MockBattery": ".testing",
    # Logging
    "LogLevel": ".logging",
    "LogComponent": ".logging",
    "ColoredFormatter": ".logging",
    "configure_logging": ".logging",
    "get_logger": ".logging",
    "set_level": ".logging",
    "log_call": ".logging",
    "log_timing": ".logging",
    # Protocols
    "VehicleProtocol": ".protocols",
    "GPSProtocol": ".protocols",
    "BatteryProtocol": ".protocols",
    "StateProtocol": ".protocols",
    # Exceptions
    "AerpawlibError": ".exceptions",
    "ErrorCode": ".exceptions",
    "ErrorSeverity": ".exceptions",
    "ConnectionError": ".exceptions",
    "CommandError": ".exceptions",
    "TimeoutError": ".exceptions",
    "AbortError": ".exceptions",
    "SafetyError": ".exceptions",
    "PreflightError": ".exceptions",
    "StateMachineError": ".exceptions",
    "ConnectionTimeoutError": ".exceptions",
    "HeartbeatLostError": ".exceptions",
    "ReconnectionError": ".exceptions",
    "ArmError": ".exceptions",
    "DisarmError": ".exceptions",
    "TakeoffError": ".exceptions",
    "LandingError": ".exceptions",
    "NavigationError": ".exceptions",
    "ModeChangeError": ".exceptions",
    "OffboardError": ".exceptions",
    "GotoTimeoutError": ".exceptions",
    "TakeoffTimeoutError": ".exceptions",
    "LandingTimeoutError": ".exceptions",
    "UserAbortError": ".exceptions",
    "CommandCancelledError": ".exceptions",
    "SafetyAbortError": ".exceptions",
    "GeofenceViolationError": ".exceptions",
    "AltitudeViolationError": ".exceptions",
    "SpeedViolationError": ".exceptions",
    "SpeedLimitExceededError": ".exceptions",
    "ParameterValidationError": ".exceptions",
    "PreflightCheckError": ".exceptions",
    "GPSError": ".exceptions",
    "BatteryError": ".exceptions",
    "NotArmableError": ".exceptions",
    "InvalidStateError": ".exceptions",
    "NoInitialStateError": ".exceptions",
    "MultipleInitialStatesError": ".exceptions",
}


def __getattr__(name: str):
    """Lazy import handler."""
    if name in _lazy_imports:
        module_name = _lazy_imports[name]
        import importlib

        module = importlib.import_module(module_name, __package__)
        value = getattr(module, name)
        # Cache it for next time
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    """List available attributes for tab completion."""
    return list(__all__)


# Generate __all__ from lazy imports + eager imports
__all__ = [
    # Types (eager)
    "Coordinate",
    "VectorNED",
    "PositionNED",
    "Attitude",
    "Waypoint",
    "DroneState",
    "GPSInfo",
    "BatteryInfo",
    "DroneInfo",
    "FlightInfo",
    "FlightMode",
    "LandedState",
    "read_waypoints_from_plan",
    # Everything else (lazy)
    *_lazy_imports.keys(),
]
