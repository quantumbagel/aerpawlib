"""
Safety features for aerpawlib v2 API.

This module provides all safety features including:
- Configurable safety limits (SafetyLimits)
- Parameter validation functions
- Pre-flight checks
- Continuous safety monitoring (SafetyMonitor)
- Geofence validation (SafetyCheckerClient, SafetyCheckerServer)
- ZMQ-based client-server architecture for external validation

Example:
    # Use default safety limits (recommended for beginners)
    drone = Drone("udp://:14540")

    # Customize safety limits
    drone = Drone("udp://:14540", safety_limits=SafetyLimits(
        max_speed=10,
        min_battery_percent=25
    ))

    # Use restrictive preset for beginners
    drone = Drone("udp://:14540", safety_limits=SafetyLimits.restrictive())

    # Run pre-flight checks before arming
    result = await drone.preflight_check()
    if not result:
        print(f"Pre-flight failed: {result.failed_checks}")

    # With external safety checker server for geofence validation
    async with SafetyCheckerClient("localhost", 14580) as checker:
        drone = Drone("udp://:14540", safety_checker=checker)
        await drone.connect()
        # Commands are now validated against geofences before execution
"""
from __future__ import annotations

import asyncio
import json
import logging
import math
import zlib
from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set, TYPE_CHECKING, Tuple

import yaml

try:
    import zmq
    import zmq.asyncio
    ZMQ_AVAILABLE = True
    ZMQ_ASYNC_AVAILABLE = True
except ImportError:
    try:
        import zmq
        ZMQ_AVAILABLE = True
        ZMQ_ASYNC_AVAILABLE = False
    except ImportError:
        ZMQ_AVAILABLE = False
        ZMQ_ASYNC_AVAILABLE = False
        zmq = None

if TYPE_CHECKING:
    from .vehicle import Vehicle
    from .types import Coordinate, VectorNED

# Import geofence functions
from .geofence import (
    Polygon,
    read_geofence,
    is_inside_polygon,
    path_crosses_polygon,
)
from .types import Coordinate

# Import exception classes from exceptions.py (single source of truth)
from .exceptions import (
    SafetyError,
    GeofenceViolationError,
    SpeedLimitExceededError,
    ParameterValidationError,
    PreflightCheckError,
)

# Use modular logging system
from .logging import get_logger, LogComponent
logger = get_logger(LogComponent.SAFETY)


# ============================================================================
# Enums
# ============================================================================

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
    # Geofence violations
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


# ============================================================================
# Safety Limits Configuration
# ============================================================================

@dataclass
class SafetyLimits:
    """
    Safety limits configuration for vehicle operations.

    These limits help prevent common mistakes that could damage the drone
    or cause crashes. All limits can be adjusted or disabled as needed.

    Attributes:
        max_speed: Maximum horizontal speed in m/s (default: 15 m/s)
        max_vertical_speed: Maximum vertical speed in m/s (default: 5 m/s)
        min_battery_percent: Minimum battery before warning (default: 20%)
        critical_battery_percent: Critical battery triggers RTL (default: 10%)
        require_gps_fix: Require GPS fix before arming (default: True)
        min_satellites: Minimum satellites for GPS fix (default: 6)
        enable_speed_limits: Enable speed limit checks (default: True)
        enable_battery_failsafe: Enable automatic RTL on low battery (default: True)
        enable_parameter_validation: Validate all command parameters (default: True)
        enable_preflight_checks: Require pre-flight checks before arm (default: True)
        auto_clamp_values: Automatically clamp values to safe ranges (default: True)

    Example:
        # Use defaults (recommended for beginners)
        drone = Drone("udp://:14540")

        # Customize limits
        drone = Drone("udp://:14540", safety_limits=SafetyLimits(
            max_speed=10,
            min_battery_percent=25
        ))

        # Use preset configurations
        drone = Drone("udp://:14540", safety_limits=SafetyLimits.restrictive())
    """
    # Speed limits
    max_speed: float = 15.0           # m/s horizontal
    max_vertical_speed: float = 5.0   # m/s vertical

    # Battery limits
    min_battery_percent: float = 20.0      # warning threshold
    critical_battery_percent: float = 10.0  # RTL threshold

    # GPS requirements
    require_gps_fix: bool = True
    min_satellites: int = 6

    # Feature toggles
    enable_speed_limits: bool = True
    enable_battery_failsafe: bool = True
    enable_parameter_validation: bool = True
    enable_preflight_checks: bool = True
    auto_clamp_values: bool = True  # Clamp values instead of rejecting

    def validate(self) -> List[str]:
        """
        Validate the safety limits configuration.

        Returns:
            List of validation error messages (empty if valid)
        """
        errors = []

        if self.max_speed <= 0:
            errors.append("max_speed must be positive")
        if self.max_vertical_speed <= 0:
            errors.append("max_vertical_speed must be positive")
        if not 0 <= self.min_battery_percent <= 100:
            errors.append("min_battery_percent must be between 0 and 100")
        if not 0 <= self.critical_battery_percent <= 100:
            errors.append("critical_battery_percent must be between 0 and 100")
        if self.critical_battery_percent >= self.min_battery_percent:
            errors.append("critical_battery_percent must be less than min_battery_percent")
        if self.min_satellites < 0:
            errors.append("min_satellites cannot be negative")

        return errors

    @classmethod
    def permissive(cls) -> "SafetyLimits":
        """
        Create permissive safety limits for advanced users.

        Disables most automatic interventions but keeps basic validation.
        """
        return cls(
            max_speed=30.0,
            max_vertical_speed=10.0,
            enable_battery_failsafe=False,
            enable_preflight_checks=False,
            auto_clamp_values=False,
        )

    @classmethod
    def restrictive(cls) -> "SafetyLimits":
        """
        Create restrictive safety limits for beginners or indoor use.

        Very conservative limits to minimize risk of crashes.
        """
        return cls(
            max_speed=5.0,
            max_vertical_speed=2.0,
            min_battery_percent=30.0,
            critical_battery_percent=20.0,
            min_satellites=8,
        )

    @classmethod
    def disabled(cls) -> "SafetyLimits":
        """
        Create safety limits with all checks disabled.

        WARNING: Only use this if you know what you're doing!
        """
        return cls(
            max_speed=100.0,
            max_vertical_speed=50.0,
            enable_speed_limits=False,
            enable_battery_failsafe=False,
            enable_parameter_validation=False,
            enable_preflight_checks=False,
            auto_clamp_values=False,
        )

    @classmethod
    def from_safety_config(cls, config: "SafetyConfig") -> "SafetyLimits":
        """
        Create SafetyLimits from a SafetyConfig (YAML-based configuration).

        This allows reusing the same YAML configuration file for both
        the SafetyCheckerServer and client-side SafetyLimits.

        Args:
            config: SafetyConfig loaded from YAML

        Returns:
            SafetyLimits instance with values from the config

        Example:
            config = SafetyConfig.from_yaml("mission_config.yaml")
            limits = SafetyLimits.from_safety_config(config)
            drone = Drone("udp://:14540", safety_limits=limits)
        """
        return cls(
            max_speed=config.max_speed,
            max_vertical_speed=config.max_speed,  # Use same as horizontal if not specified
            # Battery and GPS settings use defaults since SafetyConfig doesn't have them
        )


# ============================================================================
# Safety Config (YAML-based configuration for server)
# ============================================================================

@dataclass
class SafetyConfig:
    """
    Safety configuration loaded from YAML for the safety checker server.

    Attributes:
        vehicle_type: Type of vehicle (rover or copter)
        max_speed: Maximum allowed speed in m/s
        min_speed: Minimum allowed speed in m/s
        include_geofences: List of geofence polygons (allowed areas)
        exclude_geofences: List of no-go zone polygons
        max_altitude: Maximum altitude for copters (meters)
        min_altitude: Minimum altitude for copters (meters)
    """
    vehicle_type: VehicleType
    max_speed: float
    min_speed: float
    include_geofences: List[Polygon] = field(default_factory=list)
    exclude_geofences: List[Polygon] = field(default_factory=list)
    max_altitude: Optional[float] = None
    min_altitude: Optional[float] = None

    @classmethod
    def from_yaml(cls, config_path: str | Path) -> "SafetyConfig":
        """
        Load safety configuration from a YAML file.

        Args:
            config_path: Path to the configuration YAML file

        Returns:
            SafetyConfig instance

        Raises:
            ValueError: If required parameters are missing or invalid
        """
        config_path = Path(config_path)
        config_dir = config_path.parent

        with config_path.open("r") as f:
            config = yaml.safe_load(f)

        # Validate required parameters
        required_params = [
            "vehicle_type", "max_speed", "min_speed",
            "include_geofences", "exclude_geofences"
        ]
        for param in required_params:
            if param not in config:
                raise ValueError(f"Required parameter '{param}' not found in {config_path}")

        # Validate vehicle type
        try:
            vehicle_type = VehicleType(config["vehicle_type"])
        except ValueError:
            valid_types = [t.value for t in VehicleType]
            raise ValueError(
                f"Invalid vehicle_type '{config['vehicle_type']}'. "
                f"Must be one of {valid_types}"
            )

        # Copter-specific validation
        max_alt = None
        min_alt = None
        if vehicle_type == VehicleType.COPTER:
            if "max_alt" not in config:
                raise ValueError("Copter requires 'max_alt' parameter")
            if "min_alt" not in config:
                raise ValueError("Copter requires 'min_alt' parameter")
            max_alt = float(config["max_alt"])
            min_alt = float(config["min_alt"])

        # Load geofences
        include_geofences = [
            read_geofence(config_dir / geofence_file)
            for geofence_file in config["include_geofences"]
        ]
        exclude_geofences = [
            read_geofence(config_dir / geofence_file)
            for geofence_file in config["exclude_geofences"]
        ]

        return cls(
            vehicle_type=vehicle_type,
            max_speed=float(config["max_speed"]),
            min_speed=float(config["min_speed"]),
            include_geofences=include_geofences,
            exclude_geofences=exclude_geofences,
            max_altitude=max_alt,
            min_altitude=min_alt,
        )


# ============================================================================
# Validation Result Types
# ============================================================================

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
        """Allow using result directly in if statements."""
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
    def from_validation_result(cls, result: ValidationResult) -> "SafetyCheckResult":
        """
        Convert a ValidationResult from safety checker server to a SafetyCheckResult.

        Args:
            result: ValidationResult from SafetyCheckerClient

        Returns:
            SafetyCheckResult with equivalent status and message
        """
        if result.valid:
            return cls(passed=True)

        # Determine violation type from message content
        violation = SafetyViolationType.GEOFENCE_VIOLATION  # Default
        msg = result.message.lower()

        if "outside" in msg and "geofence" in msg:
            violation = SafetyViolationType.GEOFENCE_VIOLATION
        elif "no-go" in msg or "no go" in msg:
            if "path" in msg:
                violation = SafetyViolationType.PATH_ENTERS_NO_GO_ZONE
            else:
                violation = SafetyViolationType.NO_GO_ZONE_VIOLATION
        elif "leaves" in msg and "geofence" in msg:
            violation = SafetyViolationType.PATH_LEAVES_GEOFENCE
        elif "altitude" in msg:
            violation = SafetyViolationType.ALTITUDE_OUT_OF_BOUNDS
        elif "speed" in msg:
            violation = SafetyViolationType.SPEED_TOO_HIGH

        return cls(
            passed=False,
            violation=violation,
            message=result.message
        )


class PreflightCheckResult:
    """
    Result of pre-flight safety checks.

    Attributes:
        checks: Dictionary of check names to results
        warnings: List of warning messages
    """

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

    def add_check(self, name: str, result: SafetyCheckResult):
        """Add a check result."""
        self.checks[name] = result

    def add_warning(self, message: str):
        """Add a warning message."""
        self.warnings.append(message)
        logger.warning(f"Pre-flight warning: {message}")

    def __str__(self) -> str:
        if self.passed:
            msg = f"Pre-flight checks PASSED ({len(self.checks)} checks)"
            if self.warnings:
                msg += f" with {len(self.warnings)} warning(s)"
            return msg
        else:
            failed = ", ".join(self.failed_checks)
            return f"Pre-flight checks FAILED: {failed}"

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
            lines.append("")
            lines.append("Warnings:")
            for warning in self.warnings:
                lines.append(f"  ⚠ {warning}")

        lines.append("=" * 40)
        lines.append(f"Result: {'PASSED' if self.passed else 'FAILED'}")

        return "\n".join(lines)


# ============================================================================
# ZMQ Message Serialization
# ============================================================================

def _serialize_message(data: dict) -> bytes:
    """Compress and serialize a message."""
    json_str = json.dumps(data)
    return zlib.compress(json_str.encode("utf-8"))


def _deserialize_message(data: bytes) -> dict:
    """Decompress and deserialize a message."""
    json_str = zlib.decompress(data).decode("utf-8")
    return json.loads(json_str)


# ============================================================================
# Safety Checker Client (connects to external server)
# ============================================================================

class SafetyCheckerClient:
    """
    Async client for the safety checker server.

    Provides geofence validation via a ZMQ connection to a SafetyCheckerServer.

    Example:
        async with SafetyCheckerClient("localhost", 14580) as checker:
            result = await checker.validate_waypoint(current, target)
            if not result:
                print(f"Blocked: {result.message}")
    """

    def __init__(self, address: str = "localhost", port: int = 14580):
        """
        Initialize the safety checker client.

        Args:
            address: Server address
            port: Server port
        """
        if not ZMQ_AVAILABLE:
            raise ImportError("ZMQ is required for SafetyCheckerClient. Install with: pip install pyzmq")

        self._address = address
        self._port = port
        self._context: Optional[Any] = None
        self._socket: Optional[Any] = None
        self._connected = False

    async def connect(self):
        """Connect to the safety checker server."""
        if ZMQ_ASYNC_AVAILABLE:
            self._context = zmq.asyncio.Context()
            self._socket = self._context.socket(zmq.REQ)
        else:
            self._context = zmq.Context()
            self._socket = self._context.socket(zmq.REQ)

        self._socket.connect(f"tcp://{self._address}:{self._port}")
        self._connected = True

    async def disconnect(self):
        """Disconnect from the server."""
        if self._socket:
            self._socket.close()
            self._socket = None
        if self._context:
            self._context.term()
            self._context = None
        self._connected = False

    async def __aenter__(self) -> "SafetyCheckerClient":
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.disconnect()
        return False

    async def _send_request(
        self,
        request_type: RequestType,
        params: Optional[list] = None
    ) -> ValidationResult:
        """Send a request to the server and get the response."""
        if not self._connected:
            raise RuntimeError("Not connected to safety checker server")

        request = {
            "request_function": request_type.value,
            "params": params,
        }
        serialized = _serialize_message(request)

        if ZMQ_ASYNC_AVAILABLE:
            await self._socket.send(serialized)
            response_data = await self._socket.recv()
        else:
            self._socket.send(serialized)
            response_data = self._socket.recv()

        response = _deserialize_message(response_data)
        logger.debug(f"Safety checker response: {response}")

        return ValidationResult(
            valid=response.get("result", False),
            message=response.get("message", ""),
            request_type=request_type
        )

    async def check_server_status(self) -> ValidationResult:
        """
        Check if the safety checker server is running.

        Returns:
            ValidationResult with valid=True if server is responding.
        """
        return await self._send_request(RequestType.SERVER_STATUS)

    async def validate_waypoint(
        self,
        current_position: Coordinate,
        target_position: Coordinate
    ) -> ValidationResult:
        """
        Validate a waypoint command.

        Checks that:
        - Target is inside at least one geofence
        - Target is not in any no-go zones
        - Path doesn't cross geofence boundaries
        - Path doesn't enter any no-go zones
        - (For copters) Altitude is within limits

        Args:
            current_position: Current vehicle position
            target_position: Target waypoint

        Returns:
            ValidationResult indicating if the waypoint is safe
        """
        params = [
            current_position.to_json(),
            target_position.to_json()
        ]
        return await self._send_request(RequestType.VALIDATE_WAYPOINT, params)

    async def validate_speed(self, speed: float) -> ValidationResult:
        """
        Validate a speed change command.

        Args:
            speed: The requested speed in m/s

        Returns:
            ValidationResult indicating if the speed is within limits
        """
        return await self._send_request(RequestType.VALIDATE_SPEED, [speed])

    async def validate_takeoff(
        self,
        altitude: float,
        latitude: float,
        longitude: float
    ) -> ValidationResult:
        """
        Validate a takeoff command.

        Args:
            altitude: Target takeoff altitude
            latitude: Current latitude
            longitude: Current longitude

        Returns:
            ValidationResult indicating if takeoff is allowed
        """
        return await self._send_request(
            RequestType.VALIDATE_TAKEOFF,
            [altitude, latitude, longitude]
        )

    async def validate_landing(
        self,
        latitude: float,
        longitude: float
    ) -> ValidationResult:
        """
        Validate a landing command.

        Args:
            latitude: Current latitude
            longitude: Current longitude

        Returns:
            ValidationResult indicating if landing at this location is allowed
        """
        return await self._send_request(
            RequestType.VALIDATE_LANDING,
            [latitude, longitude]
        )


# ============================================================================
# Safety Checker Server
# ============================================================================

class SafetyCheckerServer:
    """
    Safety checker server that validates vehicle commands.

    This server should run in a separate process and validates commands
    based on geofence boundaries, speed limits, and altitude restrictions.

    Example:
        config = SafetyConfig.from_yaml("vehicle_config.yaml")
        server = SafetyCheckerServer(config, port=14580)
        await server.serve()  # Blocks until stopped
    """

    def __init__(
        self,
        config: SafetyConfig | str | Path,
        port: int = 14580
    ):
        """
        Initialize the safety checker server.

        Args:
            config: SafetyConfig instance or path to config YAML file
            port: Port to listen on
        """
        if not ZMQ_AVAILABLE:
            raise ImportError("ZMQ is required for SafetyCheckerServer. Install with: pip install pyzmq")

        if isinstance(config, (str, Path)):
            self._config = SafetyConfig.from_yaml(config)
        else:
            self._config = config

        self._port = port
        self._context: Optional[Any] = None
        self._socket: Optional[Any] = None
        self._takeoff_location: Optional[Coordinate] = None
        self._running = False

        # Request handler mapping
        self._handlers = {
            RequestType.SERVER_STATUS: self._handle_server_status,
            RequestType.VALIDATE_WAYPOINT: self._handle_validate_waypoint,
            RequestType.VALIDATE_SPEED: self._handle_validate_speed,
            RequestType.VALIDATE_TAKEOFF: self._handle_validate_takeoff,
            RequestType.VALIDATE_LANDING: self._handle_validate_landing,
        }

    def _create_response(
        self,
        request_type: RequestType,
        result: bool,
        message: str = ""
    ) -> bytes:
        """Create a serialized response message."""
        response = {
            "request_function": request_type.value,
            "result": result,
            "message": message
        }
        return _serialize_message(response)

    def _handle_server_status(self) -> Tuple[bool, str]:
        """Handle server status request."""
        return (True, "")

    def _handle_validate_waypoint(
        self,
        current_json: str,
        target_json: str
    ) -> Tuple[bool, str]:
        """Handle waypoint validation request."""
        current = Coordinate.from_json(current_json)
        target = Coordinate.from_json(target_json)

        logger.debug(f"Validating waypoint: {target}")

        # Check altitude for copters
        if self._config.vehicle_type == VehicleType.COPTER:
            if self._config.min_altitude is not None and target.altitude < self._config.min_altitude:
                return (
                    False,
                    f"Invalid waypoint. Altitude of {target.altitude}m is below "
                    f"minimum of {self._config.min_altitude}m. ABORTING!"
                )
            if self._config.max_altitude is not None and target.altitude > self._config.max_altitude:
                return (
                    False,
                    f"Invalid waypoint. Altitude of {target.altitude}m exceeds "
                    f"maximum of {self._config.max_altitude}m. ABORTING!"
                )

        # Check if target is inside at least one geofence
        inside_geofence = False
        active_geofence = None
        for geofence in self._config.include_geofences:
            if is_inside_polygon(target, geofence):
                inside_geofence = True
                active_geofence = geofence
                break

        if not inside_geofence:
            return (
                False,
                f"Invalid waypoint. Waypoint ({target.latitude},{target.longitude}) "
                f"is outside of the geofence. ABORTING!"
            )

        # Check if target is in any no-go zone
        for zone in self._config.exclude_geofences:
            if is_inside_polygon(target, zone):
                return (
                    False,
                    f"Invalid waypoint. Waypoint ({target.latitude},{target.longitude}) "
                    f"is inside a no-go zone. ABORTING!"
                )

        # Check if path crosses geofence boundary
        if active_geofence and path_crosses_polygon(current, target, active_geofence):
            return (
                False,
                f"Invalid waypoint. Path from ({current.latitude},{current.longitude}) "
                f"to waypoint ({target.latitude},{target.longitude}) leaves geofence. ABORTING!"
            )

        # Check if path enters any no-go zone
        for zone in self._config.exclude_geofences:
            if path_crosses_polygon(current, target, zone):
                return (
                    False,
                    f"Invalid waypoint. Path from ({current.latitude},{current.longitude}) "
                    f"to waypoint ({target.latitude},{target.longitude}) enters no-go zone. ABORTING!"
                )

        return (True, "")

    def _handle_validate_speed(self, speed: float) -> Tuple[bool, str]:
        """Handle speed validation request."""
        if speed > self._config.max_speed:
            return (
                False,
                f"Invalid speed ({speed}) greater than maximum ({self._config.max_speed})"
            )
        if speed < self._config.min_speed:
            return (
                False,
                f"Invalid speed ({speed}) less than minimum ({self._config.min_speed})"
            )
        return (True, "")

    def _handle_validate_takeoff(
        self,
        altitude: float,
        latitude: float,
        longitude: float
    ) -> Tuple[bool, str]:
        """Handle takeoff validation request."""
        # Check altitude for copters
        if self._config.vehicle_type == VehicleType.COPTER:
            if self._config.min_altitude is not None and altitude < self._config.min_altitude:
                return (False, f"Invalid takeoff altitude of {altitude}m.")
            if self._config.max_altitude is not None and altitude > self._config.max_altitude:
                return (False, f"Invalid takeoff altitude of {altitude}m.")

        # Save takeoff location for landing validation
        self._takeoff_location = Coordinate(latitude, longitude, 0)
        return (True, "")

    def _handle_validate_landing(
        self,
        latitude: float,
        longitude: float
    ) -> Tuple[bool, str]:
        """Handle landing validation request."""
        if self._takeoff_location is None:
            return (False, "No takeoff location recorded. Cannot validate landing.")

        current = Coordinate(latitude, longitude, 0)
        distance = self._takeoff_location.ground_distance_to(current)

        if distance > 5:
            return (
                False,
                f"Invalid landing location. Must be within 5 meters of takeoff location. "
                f"Attempted landing location ({latitude},{longitude}) is {distance:.2f} meters "
                f"from takeoff location."
            )
        return (True, "")

    async def serve(self):
        """
        Start the server and process requests.

        This method blocks until stop() is called.
        """
        if ZMQ_ASYNC_AVAILABLE:
            self._context = zmq.asyncio.Context()
            self._socket = self._context.socket(zmq.REP)
        else:
            self._context = zmq.Context()
            self._socket = self._context.socket(zmq.REP)

        self._socket.bind(f"tcp://*:{self._port}")
        self._running = True

        logger.info(f"Safety checker server waiting for messages on port {self._port}")

        try:
            while self._running:
                if ZMQ_ASYNC_AVAILABLE:
                    raw_msg = await self._socket.recv()
                else:
                    # For sync ZMQ, use polling to allow stopping
                    try:
                        self._socket.setsockopt(zmq.RCVTIMEO, 1000)
                        raw_msg = self._socket.recv()
                    except zmq.Again:
                        continue

                message = _deserialize_message(raw_msg)
                logger.debug(f"Received request: {message}")

                try:
                    request_type = RequestType(message["request_function"])
                    handler = self._handlers.get(request_type)

                    if handler is None:
                        response = self._create_response(
                            request_type, False,
                            f"Unimplemented function request <{request_type.value}>"
                        )
                    else:
                        params = message.get("params") or []
                        result, msg = handler(*params)
                        response = self._create_response(request_type, result, msg)

                except (KeyError, ValueError) as e:
                    response = _serialize_message({
                        "result": False,
                        "message": f"Invalid request: {e}"
                    })
                except Exception as e:
                    _serialize_message({
                        "result": False,
                        "message": f"Server error: {e}"
                    })
                    raise

                if ZMQ_ASYNC_AVAILABLE:
                    await self._socket.send(response)
                else:
                    self._socket.send(response)

        finally:
            self._socket.close()
            self._context.term()

    def stop(self):
        """Stop the server."""
        self._running = False

    def start_server(self, port: Optional[int] = None):
        """
        Start the server synchronously (blocking).

        For backward compatibility. For new code, prefer using serve() with asyncio.

        Args:
            port: Port to listen on (overrides constructor port)
        """
        if port is not None:
            self._port = port

        asyncio.run(self.serve())


# ============================================================================
# Safety Monitor - Continuous monitoring during flight
# ============================================================================

class SafetyMonitor:
    """
    Monitors vehicle state and enforces safety limits during flight.

    This runs as a background task and can:
    - Warn when battery is low
    - Automatically trigger RTL on critical battery
    - Log safety events
    - Monitor speed limits
    """

    def __init__(self, vehicle: "Vehicle", limits: SafetyLimits):
        self._vehicle = vehicle
        self._limits = limits
        self._running = False
        self._task: Optional[asyncio.Task] = None
        self._callbacks: Dict[SafetyViolationType, List[Callable]] = {}
        self._warnings_issued: Set[SafetyViolationType] = set()
        self._battery_rtl_triggered = False
        self._low_battery_warned = False

    def start(self):
        """Start the safety monitor."""
        if self._running:
            return
        self._running = True
        self._task = asyncio.create_task(self._monitor_loop())
        logger.info("Safety monitor started")

    def stop(self):
        """Stop the safety monitor."""
        self._running = False
        if self._task:
            self._task.cancel()
        logger.info("Safety monitor stopped")

    def on_violation(self, violation: SafetyViolationType, callback: Callable):
        """
        Register a callback for a specific violation type.

        Args:
            violation: The type of violation to listen for
            callback: Function to call (can be async)
        """
        if violation not in self._callbacks:
            self._callbacks[violation] = []
        self._callbacks[violation].append(callback)

    async def _monitor_loop(self):
        """Main monitoring loop."""
        while self._running:
            try:
                await self._check_all()
                await asyncio.sleep(0.5)  # Check twice per second
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Safety monitor error: {e}")
                await asyncio.sleep(1)

    async def _check_all(self):
        """Run all safety checks."""
        # Always check battery
        if self._limits.enable_battery_failsafe:
            await self._check_battery()

        # Only check speed when in air
        if self._vehicle.state.is_in_air and self._limits.enable_speed_limits:
            await self._check_speed()

    async def _check_speed(self):
        """Check speed limits."""
        speed = self._vehicle.state.groundspeed

        if speed > self._limits.max_speed * 1.1:  # 10% tolerance
            await self._trigger_violation(
                SafetyViolationType.SPEED_TOO_HIGH,
                f"Speed {speed:.1f}m/s exceeds maximum {self._limits.max_speed}m/s"
            )

    async def _check_battery(self):
        """Check battery levels and trigger RTL if critical."""
        battery_pct = self._vehicle.battery.percentage

        # Critical battery - auto RTL
        if battery_pct <= self._limits.critical_battery_percent:
            if not self._battery_rtl_triggered:
                self._battery_rtl_triggered = True
                await self._trigger_violation(
                    SafetyViolationType.BATTERY_CRITICAL,
                    f"CRITICAL BATTERY: {battery_pct:.1f}% - Triggering automatic RTL!"
                )
                # Trigger automatic RTL
                try:
                    logger.critical(f"BATTERY CRITICAL ({battery_pct:.1f}%) - Automatic RTL triggered!")
                    if self._vehicle.state.is_in_air:
                        await self._vehicle.rtl(wait=False)
                except Exception as e:
                    logger.error(f"Failed to trigger RTL on critical battery: {e}")

        # Low battery warning
        elif battery_pct <= self._limits.min_battery_percent:
            if not self._low_battery_warned:
                self._low_battery_warned = True
                await self._trigger_violation(
                    SafetyViolationType.BATTERY_LOW,
                    f"Low battery warning: {battery_pct:.1f}% - Consider landing soon"
                )

    async def _trigger_violation(self, violation: SafetyViolationType, message: str):
        """Trigger a safety violation."""
        logger.warning(f"SAFETY: {message}")

        # Call registered callbacks
        for callback in self._callbacks.get(violation, []):
            try:
                result = callback(violation, message)
                if asyncio.iscoroutine(result):
                    await result
            except Exception as e:
                logger.error(f"Violation callback error: {e}")

    def reset(self):
        """Reset warning state (call on landing)."""
        self._warnings_issued.clear()
        self._battery_rtl_triggered = False
        self._low_battery_warned = False


# ============================================================================
# Parameter Validation Functions
# ============================================================================

def validate_coordinate(coord, name: str = "coordinate") -> SafetyCheckResult:
    """
    Validate a coordinate has reasonable values.

    Checks for:
    - Not None
    - Valid latitude range (-90 to 90)
    - Valid longitude range (-180 to 180)
    - Non-NaN values

    Args:
        coord: The coordinate to validate
        name: Name for error messages

    Returns:
        SafetyCheckResult indicating pass/fail
    """
    if coord is None:
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_COORDINATE,
            message=f"{name} cannot be None"
        )

    lat = getattr(coord, 'latitude', getattr(coord, 'lat', None))
    lon = getattr(coord, 'longitude', getattr(coord, 'lon', None))

    if lat is None or lon is None:
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_COORDINATE,
            message=f"{name} missing latitude or longitude"
        )

    if math.isnan(lat) or math.isnan(lon):
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_COORDINATE,
            message=f"{name} contains NaN values"
        )

    if math.isinf(lat) or math.isinf(lon):
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_COORDINATE,
            message=f"{name} contains infinite values"
        )

    if not -90 <= lat <= 90:
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_COORDINATE,
            message=f"{name} latitude {lat} is out of range (-90 to 90)"
        )

    if not -180 <= lon <= 180:
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_COORDINATE,
            message=f"{name} longitude {lon} is out of range (-180 to 180)"
        )

    return SafetyCheckResult(passed=True)


def validate_altitude(altitude: float, name: str = "altitude") -> SafetyCheckResult:
    """
    Validate an altitude value is reasonable.

    Args:
        altitude: The altitude to validate
        name: Name for error messages

    Returns:
        SafetyCheckResult indicating pass/fail
    """
    if altitude is None:
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_ALTITUDE,
            message=f"{name} cannot be None"
        )

    if math.isnan(altitude) or math.isinf(altitude):
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_ALTITUDE,
            message=f"{name} is invalid (NaN or Inf)"
        )

    # Basic sanity check - altitude shouldn't be extremely negative
    if altitude < -100:
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_ALTITUDE,
            message=f"{name} {altitude}m seems unreasonably low"
        )

    # Basic sanity check - altitude shouldn't be extremely high
    if altitude > 10000:
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_ALTITUDE,
            message=f"{name} {altitude}m seems unreasonably high"
        )

    return SafetyCheckResult(passed=True)


def validate_speed(
    speed: float,
    limits: SafetyLimits,
    name: str = "speed"
) -> SafetyCheckResult:
    """
    Validate a speed value against safety limits.

    Args:
        speed: The speed to validate
        limits: Safety limits to check against
        name: Name for error messages

    Returns:
        SafetyCheckResult indicating pass/fail
    """
    if speed is None:
        return SafetyCheckResult(passed=True)  # None means use default

    if math.isnan(speed) or math.isinf(speed):
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_SPEED,
            message=f"{name} is invalid (NaN or Inf)"
        )

    if speed < 0:
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_SPEED,
            message=f"{name} cannot be negative"
        )

    if limits.enable_speed_limits and speed > limits.max_speed:
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.SPEED_TOO_HIGH,
            message=f"{name} {speed}m/s exceeds maximum {limits.max_speed}m/s",
            value=speed,
            limit=limits.max_speed
        )

    return SafetyCheckResult(passed=True)


def validate_velocity(
    velocity,
    limits: SafetyLimits,
    name: str = "velocity"
) -> SafetyCheckResult:
    """
    Validate a velocity vector against safety limits.

    Args:
        velocity: VectorNED to validate
        limits: Safety limits to check against
        name: Name for error messages

    Returns:
        SafetyCheckResult indicating pass/fail
    """
    if velocity is None:
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_PARAMETER,
            message=f"{name} cannot be None"
        )

    horizontal_speed = velocity.magnitude(ignore_vertical=True)
    vertical_speed = abs(velocity.down)

    if limits.enable_speed_limits:
        if horizontal_speed > limits.max_speed:
            return SafetyCheckResult(
                passed=False,
                violation=SafetyViolationType.SPEED_TOO_HIGH,
                message=f"Horizontal {name} {horizontal_speed:.1f}m/s exceeds max {limits.max_speed}m/s",
                value=horizontal_speed,
                limit=limits.max_speed
            )

        if vertical_speed > limits.max_vertical_speed:
            return SafetyCheckResult(
                passed=False,
                violation=SafetyViolationType.VERTICAL_SPEED_TOO_HIGH,
                message=f"Vertical {name} {vertical_speed:.1f}m/s exceeds max {limits.max_vertical_speed}m/s",
                value=vertical_speed,
                limit=limits.max_vertical_speed
            )

    return SafetyCheckResult(passed=True)


def validate_timeout(timeout: float, name: str = "timeout") -> SafetyCheckResult:
    """
    Validate a timeout value is reasonable.

    Args:
        timeout: The timeout to validate
        name: Name for error messages

    Returns:
        SafetyCheckResult indicating pass/fail
    """
    if timeout is None:
        return SafetyCheckResult(passed=True)

    if math.isnan(timeout) or math.isinf(timeout):
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_PARAMETER,
            message=f"{name} is invalid (NaN or Inf)"
        )

    if timeout <= 0:
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_PARAMETER,
            message=f"{name} must be positive"
        )

    if timeout > 3600:  # 1 hour
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_PARAMETER,
            message=f"{name} {timeout}s seems unreasonably long (max 1 hour)"
        )

    return SafetyCheckResult(passed=True)


def validate_tolerance(tolerance: float, name: str = "tolerance") -> SafetyCheckResult:
    """
    Validate a tolerance/acceptance radius value.

    Args:
        tolerance: The tolerance to validate
        name: Name for error messages

    Returns:
        SafetyCheckResult indicating pass/fail
    """
    if tolerance is None:
        return SafetyCheckResult(passed=True)

    if math.isnan(tolerance) or math.isinf(tolerance):
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_PARAMETER,
            message=f"{name} is invalid (NaN or Inf)"
        )

    if tolerance <= 0:
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_PARAMETER,
            message=f"{name} must be positive"
        )

    if tolerance < 0.1:
        return SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_PARAMETER,
            message=f"{name} {tolerance}m is too small (min 0.1m)"
        )

    return SafetyCheckResult(passed=True)


# ============================================================================
# Value Clamping Functions
# ============================================================================

def clamp_speed(speed: Optional[float], limits: SafetyLimits) -> Optional[float]:
    """
    Clamp a speed value to within safety limits.

    Args:
        speed: Desired speed
        limits: Safety limits

    Returns:
        Speed clamped to within limits
    """
    if speed is None:
        return None

    if not limits.enable_speed_limits:
        return max(0.0, speed)

    clamped = min(limits.max_speed, max(0.0, speed))
    if clamped != speed:
        logger.warning(f"Speed clamped: {speed:.1f}m/s -> {clamped:.1f}m/s (max: {limits.max_speed}m/s)")
    return clamped


def clamp_velocity(velocity, limits: SafetyLimits):
    """
    Clamp a velocity vector to within safety limits.

    Preserves direction while limiting magnitude.

    Args:
        velocity: Desired velocity (VectorNED)
        limits: Safety limits

    Returns:
        Velocity with magnitude limited
    """
    from .types import VectorNED

    if velocity is None:
        return velocity

    if not limits.enable_speed_limits:
        return velocity

    horizontal_speed = velocity.magnitude(ignore_vertical=True)
    vertical_speed = abs(velocity.down)

    north = velocity.north
    east = velocity.east
    down = velocity.down

    # Scale horizontal if needed
    if horizontal_speed > limits.max_speed and horizontal_speed > 0:
        scale = limits.max_speed / horizontal_speed
        north = velocity.north * scale
        east = velocity.east * scale
        logger.warning(f"Horizontal velocity scaled: {horizontal_speed:.1f}m/s -> {limits.max_speed}m/s")

    # Clamp vertical if needed
    if vertical_speed > limits.max_vertical_speed:
        sign = 1 if velocity.down > 0 else -1
        down = sign * limits.max_vertical_speed
        logger.warning(f"Vertical velocity clamped: {velocity.down:.1f}m/s -> {down:.1f}m/s")

    if north != velocity.north or east != velocity.east or down != velocity.down:
        return VectorNED(north, east, down)

    return velocity


# ============================================================================
# Pre-flight Check Functions
# ============================================================================

async def run_preflight_checks(vehicle: "Vehicle", limits: SafetyLimits) -> PreflightCheckResult:
    """
    Run comprehensive pre-flight safety checks.

    Checks include:
    - GPS fix quality and satellite count
    - Battery level
    - Safety limits configuration validity
    - Connection health

    Args:
        vehicle: The vehicle to check
        limits: Safety limits configuration

    Returns:
        PreflightCheckResult with all check results
    """
    result = PreflightCheckResult()

    # Check safety limits configuration
    config_errors = limits.validate()
    if config_errors:
        result.add_check("config", SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.INVALID_PARAMETER,
            message=f"Invalid safety config: {', '.join(config_errors)}"
        ))
    else:
        result.add_check("config", SafetyCheckResult(passed=True))

    # Check GPS
    if limits.require_gps_fix:
        has_fix = getattr(vehicle.gps, 'has_fix', vehicle.gps.fix_type >= 2)
        satellites = getattr(vehicle.gps, 'satellites',
                           getattr(vehicle.gps, 'satellites_visible', 0))

        gps_ok = has_fix and satellites >= limits.min_satellites

        if not gps_ok:
            result.add_check("gps", SafetyCheckResult(
                passed=False,
                violation=SafetyViolationType.GPS_POOR,
                message=f"GPS: {satellites} satellites (need {limits.min_satellites}), fix: {has_fix}"
            ))
        else:
            result.add_check("gps", SafetyCheckResult(passed=True))

    # Check battery
    battery_pct = vehicle.battery.percentage

    if battery_pct <= limits.critical_battery_percent:
        result.add_check("battery", SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.BATTERY_CRITICAL,
            message=f"Battery critically low: {battery_pct:.1f}%"
        ))
    elif battery_pct <= limits.min_battery_percent:
        result.add_check("battery", SafetyCheckResult(
            passed=False,
            violation=SafetyViolationType.BATTERY_LOW,
            message=f"Battery too low: {battery_pct:.1f}% (min: {limits.min_battery_percent}%)"
        ))
    else:
        result.add_check("battery", SafetyCheckResult(passed=True))

    # Add warnings for marginal conditions
    if limits.min_battery_percent < battery_pct < 50:
        result.add_warning(f"Battery at {battery_pct:.1f}% - consider charging before long flights")

    # Check connection (if available)
    if hasattr(vehicle, '_connected'):
        if not vehicle._connected:
            result.add_check("connection", SafetyCheckResult(
                passed=False,
                message="Not connected to vehicle"
            ))
        else:
            result.add_check("connection", SafetyCheckResult(passed=True))

    # Log the result
    if result.passed:
        logger.info(f"Pre-flight checks passed ({len(result.checks)} checks)")
    else:
        logger.warning(f"Pre-flight checks FAILED: {result.failed_checks}")

    return result


# ============================================================================
# Waypoint Validation with SafetyCheckerClient
# ============================================================================

async def validate_waypoint_with_checker(
    safety_checker: SafetyCheckerClient,
    current_position: Coordinate,
    target_position: Coordinate,
    raise_on_fail: bool = True
) -> SafetyCheckResult:
    """Validate a waypoint against the safety checker server."""
    try:
        result = await safety_checker.validate_waypoint(current_position, target_position)
        check_result = SafetyCheckResult.from_validation_result(result)

        if not check_result.passed and raise_on_fail:
            raise GeofenceViolationError(
                check_result.message,
                check_result.violation or SafetyViolationType.GEOFENCE_VIOLATION,
                current_position,
                target_position
            )

        return check_result

    except GeofenceViolationError:
        raise
    except Exception as e:
        logger.error(f"Safety checker validation failed: {e}")
        return SafetyCheckResult(passed=False, message=f"Safety checker error: {e}")


async def validate_speed_with_checker(
    safety_checker: SafetyCheckerClient,
    speed: float,
    raise_on_fail: bool = True
) -> SafetyCheckResult:
    """Validate a speed change against the safety checker server."""
    try:
        result = await safety_checker.validate_speed(speed)
        check_result = SafetyCheckResult.from_validation_result(result)

        if not check_result.passed and raise_on_fail:
            raise SpeedLimitExceededError(check_result.message, speed, 0)

        return check_result

    except SpeedLimitExceededError:
        raise
    except Exception as e:
        logger.error(f"Safety checker speed validation failed: {e}")
        return SafetyCheckResult(passed=False, message=f"Safety checker error: {e}")


async def validate_takeoff_with_checker(
    safety_checker: SafetyCheckerClient,
    altitude: float,
    latitude: float,
    longitude: float,
    raise_on_fail: bool = True
) -> SafetyCheckResult:
    """Validate a takeoff against the safety checker server."""
    try:
        result = await safety_checker.validate_takeoff(altitude, latitude, longitude)
        check_result = SafetyCheckResult.from_validation_result(result)

        if not check_result.passed and raise_on_fail:
            raise SafetyError(check_result.message, SafetyViolationType.ALTITUDE_OUT_OF_BOUNDS)

        return check_result

    except SafetyError:
        raise
    except Exception as e:
        logger.error(f"Safety checker takeoff validation failed: {e}")
        return SafetyCheckResult(passed=False, message=f"Safety checker error: {e}")



# ============================================================================
# Backward Compatibility Exports
# ============================================================================

SERVER_STATUS_REQ = RequestType.SERVER_STATUS.value
VALIDATE_WAYPOINT_REQ = RequestType.VALIDATE_WAYPOINT.value
VALIDATE_CHANGE_SPEED_REQ = RequestType.VALIDATE_SPEED.value
VALIDATE_TAKEOFF_REQ = RequestType.VALIDATE_TAKEOFF.value
VALIDATE_LANDING_REQ = RequestType.VALIDATE_LANDING.value


def serialize_request(request_function: str, params: list) -> bytes:
    """Serialize a safety checker request (backward compatibility)."""
    return _serialize_message({
        "request_function": request_function,
        "params": params,
    })


def serialize_response(request_function: str, result: bool, message: str = "") -> bytes:
    """Serialize a safety checker response (backward compatibility)."""
    return _serialize_message({
        "request_function": request_function,
        "result": result,
        "message": message
    })


def serialize_msg(raw_json: str) -> bytes:
    """Compress JSON message using zlib (backward compatibility)."""
    return zlib.compress(raw_json.encode("utf-8"))


def deserialize_msg(compressed_msg: bytes) -> dict:
    """Decompress JSON message using zlib (backward compatibility)."""
    return _deserialize_message(compressed_msg)


# ============================================================================
# Exports
# ============================================================================

__all__ = [
    # Enums
    "SafetyViolationType",
    "RequestType",
    "VehicleType",
    # Configuration
    "SafetyLimits",
    "SafetyConfig",
    # Result types
    "ValidationResult",
    "SafetyCheckResult",
    "PreflightCheckResult",
    # Client/Server
    "SafetyCheckerClient",
    "SafetyCheckerServer",
    # Monitoring
    "SafetyMonitor",
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
    # Exceptions
    "SafetyError",
    "PreflightCheckError",
    "ParameterValidationError",
    "SpeedLimitExceededError",
    "GeofenceViolationError",
    # Backward compatibility
    "SERVER_STATUS_REQ",
    "VALIDATE_WAYPOINT_REQ",
    "VALIDATE_CHANGE_SPEED_REQ",
    "VALIDATE_TAKEOFF_REQ",
    "VALIDATE_LANDING_REQ",
    "serialize_request",
    "serialize_response",
    "serialize_msg",
    "deserialize_msg",
]
