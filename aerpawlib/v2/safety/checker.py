"""
Safety checker client/server for aerpawlib v2 API.

Provides ZMQ-based geofence validation through a client-server architecture.
ZMQ is a hard requirement for this module - it will fail fast if not available.
"""

from __future__ import annotations

import json
import zlib
from pathlib import Path
from typing import Any, Optional, Tuple, TYPE_CHECKING

# Fail fast on ZMQ - it's required for this module
try:
    import zmq
    import zmq.asyncio
except ImportError as e:
    raise ImportError(
        "ZMQ is required for safety checker client/server. "
        "Install with: pip install pyzmq"
    ) from e

from .types import (
    RequestType,
    ValidationResult,
    SafetyCheckResult,
    SafetyViolationType,
)
from .limits import SafetyConfig, VehicleType

if TYPE_CHECKING:
    from ..types import Coordinate

from ..logging import get_logger, LogComponent

logger = get_logger(LogComponent.SAFETY)


def _serialize_message(data: dict) -> bytes:
    """Compress and serialize a message."""
    return zlib.compress(json.dumps(data).encode("utf-8"))


def _deserialize_message(data: bytes) -> dict:
    """Decompress and deserialize a message."""
    return json.loads(zlib.decompress(data).decode("utf-8"))


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
        self._address = address
        self._port = port
        self._context: Optional[zmq.asyncio.Context] = None
        self._socket: Optional[Any] = None
        self._connected = False

    async def connect(self) -> None:
        """Connect to the safety checker server."""
        self._context = zmq.asyncio.Context()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.connect(f"tcp://{self._address}:{self._port}")
        self._connected = True

    async def disconnect(self) -> None:
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

    async def __aexit__(self, exc_type, exc_val, exc_tb) -> None:
        await self.disconnect()

    async def _send_request(
        self, request_type: RequestType, params: Optional[list] = None
    ) -> ValidationResult:
        """Send a request to the server and get the response."""
        if not self._connected:
            raise RuntimeError("Not connected to safety checker server")

        request = {"request_function": request_type.value, "params": params}
        await self._socket.send(_serialize_message(request))
        response = _deserialize_message(await self._socket.recv())
        logger.debug(f"Safety checker response: {response}")

        return ValidationResult(
            valid=response.get("result", False),
            message=response.get("message", ""),
            request_type=request_type,
        )

    async def check_server_status(self) -> ValidationResult:
        """Check if the safety checker server is running."""
        return await self._send_request(RequestType.SERVER_STATUS)

    async def validate_waypoint(
        self, current: "Coordinate", target: "Coordinate"
    ) -> ValidationResult:
        """Validate a waypoint command against geofences."""
        return await self._send_request(
            RequestType.VALIDATE_WAYPOINT,
            [current.to_json(), target.to_json()],
        )

    async def validate_speed(self, speed: float) -> ValidationResult:
        """Validate a speed change command."""
        return await self._send_request(RequestType.VALIDATE_SPEED, [speed])

    async def validate_takeoff(
        self, altitude: float, latitude: float, longitude: float
    ) -> ValidationResult:
        """Validate a takeoff command."""
        return await self._send_request(
            RequestType.VALIDATE_TAKEOFF, [altitude, latitude, longitude]
        )

    async def validate_landing(
        self, latitude: float, longitude: float
    ) -> ValidationResult:
        """Validate a landing command."""
        return await self._send_request(
            RequestType.VALIDATE_LANDING, [latitude, longitude]
        )


class SafetyCheckerServer:
    """
    Safety checker server that validates vehicle commands.

    Validates commands based on geofence boundaries, speed limits, and altitude restrictions.

    Example:
        config = SafetyConfig.from_yaml("vehicle_config.yaml")
        server = SafetyCheckerServer(config, port=14580)
        await server.serve()
    """

    def __init__(self, config: SafetyConfig | str | Path, port: int = 14580):
        if isinstance(config, (str, Path)):
            self._config = SafetyConfig.from_yaml(config)
        else:
            self._config = config

        self._port = port
        self._context: Optional[zmq.asyncio.Context] = None
        self._socket: Optional[Any] = None
        self._takeoff_location: Optional["Coordinate"] = None
        self._running = False

        self._handlers = {
            RequestType.SERVER_STATUS: self._handle_server_status,
            RequestType.VALIDATE_WAYPOINT: self._handle_validate_waypoint,
            RequestType.VALIDATE_SPEED: self._handle_validate_speed,
            RequestType.VALIDATE_TAKEOFF: self._handle_validate_takeoff,
            RequestType.VALIDATE_LANDING: self._handle_validate_landing,
        }

    def _create_response(
        self, request_type: RequestType, result: bool, message: str = ""
    ) -> bytes:
        """Create a serialized response message."""
        return _serialize_message(
            {
                "request_function": request_type.value,
                "result": result,
                "message": message,
            }
        )

    def _handle_server_status(self) -> Tuple[bool, str]:
        return (True, "")

    def _handle_validate_waypoint(
        self, current_json: str, target_json: str
    ) -> Tuple[bool, str]:
        from ..types import Coordinate
        from ..geofence import is_inside_polygon, path_crosses_polygon

        current = Coordinate.from_json(current_json)
        target = Coordinate.from_json(target_json)
        logger.debug(f"Validating waypoint: {target}")

        # Check altitude for copters
        if self._config.vehicle_type == VehicleType.COPTER:
            if (
                self._config.min_altitude is not None
                and target.altitude < self._config.min_altitude
            ):
                return (
                    False,
                    f"Altitude {target.altitude}m below minimum {self._config.min_altitude}m",
                )
            if (
                self._config.max_altitude is not None
                and target.altitude > self._config.max_altitude
            ):
                return (
                    False,
                    f"Altitude {target.altitude}m exceeds maximum {self._config.max_altitude}m",
                )

        # Check geofences
        inside_geofence = False
        active_geofence = None
        for geofence in self._config.include_geofences:
            if is_inside_polygon(target, geofence):
                inside_geofence, active_geofence = True, geofence
                break

        if not inside_geofence:
            return (
                False,
                f"Waypoint ({target.latitude},{target.longitude}) is outside of the geofence",
            )

        for zone in self._config.exclude_geofences:
            if is_inside_polygon(target, zone):
                return (
                    False,
                    f"Waypoint ({target.latitude},{target.longitude}) is inside a no-go zone",
                )

        if active_geofence and path_crosses_polygon(
            current, target, active_geofence
        ):
            return (
                False,
                f"Path from ({current.latitude},{current.longitude}) to ({target.latitude},{target.longitude}) leaves geofence",
            )

        for zone in self._config.exclude_geofences:
            if path_crosses_polygon(current, target, zone):
                return (False, "Path enters no-go zone")

        return (True, "")

    def _handle_validate_speed(self, speed: float) -> Tuple[bool, str]:
        if speed > self._config.max_speed:
            return (
                False,
                f"Speed {speed} exceeds maximum {self._config.max_speed}",
            )
        if speed < self._config.min_speed:
            return (
                False,
                f"Speed {speed} below minimum {self._config.min_speed}",
            )
        return (True, "")

    def _handle_validate_takeoff(
        self, altitude: float, latitude: float, longitude: float
    ) -> Tuple[bool, str]:
        from ..types import Coordinate

        if self._config.vehicle_type == VehicleType.COPTER:
            if (
                self._config.min_altitude is not None
                and altitude < self._config.min_altitude
            ):
                return (False, f"Takeoff altitude {altitude}m below minimum")
            if (
                self._config.max_altitude is not None
                and altitude > self._config.max_altitude
            ):
                return (False, f"Takeoff altitude {altitude}m exceeds maximum")

        self._takeoff_location = Coordinate(latitude, longitude, 0)
        return (True, "")

    def _handle_validate_landing(
        self, latitude: float, longitude: float
    ) -> Tuple[bool, str]:
        from ..types import Coordinate

        if self._takeoff_location is None:
            return (False, "No takeoff location recorded")

        current = Coordinate(latitude, longitude, 0)
        distance = self._takeoff_location.ground_distance_to(current)
        if distance > 5:
            return (
                False,
                f"Landing location must be within 5m of takeoff ({distance:.1f}m away)",
            )
        return (True, "")

    async def serve(self) -> None:
        """Start the server and process requests."""
        self._context = zmq.asyncio.Context()
        self._socket = self._context.socket(zmq.REP)
        self._socket.bind(f"tcp://*:{self._port}")
        self._running = True

        logger.info(f"Safety checker server listening on port {self._port}")

        try:
            while self._running:
                raw_msg = await self._socket.recv()
                message = _deserialize_message(raw_msg)
                logger.debug(f"Received request: {message}")

                try:
                    request_type = RequestType(message["request_function"])
                    handler = self._handlers.get(request_type)

                    if handler is None:
                        response = self._create_response(
                            request_type,
                            False,
                            f"Unknown request: {request_type.value}",
                        )
                    else:
                        params = message.get("params") or []
                        result, msg = handler(*params)
                        response = self._create_response(
                            request_type, result, msg
                        )

                except (KeyError, ValueError) as e:
                    response = _serialize_message(
                        {"result": False, "message": f"Invalid request: {e}"}
                    )

                await self._socket.send(response)
        finally:
            self._socket.close()
            self._context.term()

    def stop(self) -> None:
        """Stop the server."""
        self._running = False


# Helper functions for validation with checker


async def validate_waypoint_with_checker(
    checker: SafetyCheckerClient,
    current: "Coordinate",
    target: "Coordinate",
    raise_on_fail: bool = True,
) -> SafetyCheckResult:
    """Validate a waypoint against the safety checker server."""
    from ..exceptions import GeofenceViolationError

    try:
        result = await checker.validate_waypoint(current, target)
        check_result = SafetyCheckResult.from_validation_result(result)

        if not check_result.passed and raise_on_fail:
            raise GeofenceViolationError(message=check_result.message)

        return check_result
    except GeofenceViolationError:
        raise
    except Exception as e:
        logger.error(f"Safety checker validation failed: {e}")
        return SafetyCheckResult.fail(
            SafetyViolationType.GEOFENCE_VIOLATION,
            f"Safety checker error: {e}",
        )


async def validate_speed_with_checker(
    checker: SafetyCheckerClient, speed: float, raise_on_fail: bool = True
) -> SafetyCheckResult:
    """Validate a speed change against the safety checker server."""
    from ..exceptions import SpeedLimitExceededError

    try:
        result = await checker.validate_speed(speed)
        check_result = SafetyCheckResult.from_validation_result(result)

        if not check_result.passed and raise_on_fail:
            raise SpeedLimitExceededError(message=check_result.message)

        return check_result
    except SpeedLimitExceededError:
        raise
    except Exception as e:
        logger.error(f"Safety checker speed validation failed: {e}")
        return SafetyCheckResult.fail(
            SafetyViolationType.SPEED_TOO_HIGH, f"Safety checker error: {e}"
        )


async def validate_takeoff_with_checker(
    checker: SafetyCheckerClient,
    altitude: float,
    latitude: float,
    longitude: float,
    raise_on_fail: bool = True,
) -> SafetyCheckResult:
    """Validate a takeoff against the safety checker server."""
    from ..exceptions import SafetyError

    try:
        result = await checker.validate_takeoff(altitude, latitude, longitude)
        check_result = SafetyCheckResult.from_validation_result(result)

        if not check_result.passed and raise_on_fail:
            raise SafetyError(message=check_result.message)

        return check_result
    except SafetyError:
        raise
    except Exception as e:
        logger.error(f"Safety checker takeoff validation failed: {e}")
        return SafetyCheckResult.fail(
            SafetyViolationType.ALTITUDE_OUT_OF_BOUNDS,
            f"Safety checker error: {e}",
        )


__all__ = [
    "SafetyCheckerClient",
    "SafetyCheckerServer",
    "validate_waypoint_with_checker",
    "validate_speed_with_checker",
    "validate_takeoff_with_checker",
]
