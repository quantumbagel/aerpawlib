"""
SafetyCheckerClient for aerpawlib v2.

Async ZMQ client for external geofence validation.
"""

from __future__ import annotations

import json
import zlib
from typing import Tuple

import zmq
import zmq.asyncio

from ..constants import (
    SAFETY_CHECKER_REQUEST_TIMEOUT_S,
    SERVER_STATUS_REQ,
    VALIDATE_WAYPOINT_REQ,
    VALIDATE_CHANGE_SPEED_REQ,
    VALIDATE_TAKEOFF_REQ,
    VALIDATE_LANDING_REQ,
)
from ..log import LogComponent, get_logger
from ..types import Coordinate

logger = get_logger(LogComponent.SAFETY)


def _serialize_request(request_function: str, params: list) -> bytes:
    raw = json.dumps({"request_function": request_function, "params": params})
    return zlib.compress(raw.encode("utf-8"))


def _deserialize_response(data: bytes) -> dict:
    raw = zlib.decompress(data).decode("utf-8")
    return json.loads(raw)


class SafetyCheckerClient:
    """Async client for SafetyCheckerServer via ZMQ."""

    def __init__(
        self,
        addr: str,
        port: int,
        timeout_s: float = SAFETY_CHECKER_REQUEST_TIMEOUT_S,
    ) -> None:
        self._addr = addr
        self._port = port
        self._timeout_s = timeout_s
        logger.info(f"SafetyCheckerClient: connecting to {addr}:{port} (timeout={timeout_s}s)")
        self._ctx = zmq.asyncio.Context()
        self._socket = self._ctx.socket(zmq.REQ)
        timeout_ms = int(timeout_s * 1000)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self._socket.connect(f"tcp://{addr}:{port}")

    def close(self) -> None:
        logger.debug("SafetyCheckerClient: closing connection")
        self._socket.close()
        self._ctx.term()

    async def _send_request(self, msg: bytes) -> Tuple[bool, str]:
        await self._socket.send(msg)
        raw = await self._socket.recv()
        resp = _deserialize_response(raw)
        result, message = resp["result"], resp.get("message", "")
        logger.debug(f"SafetyCheckerClient: response result={result}, message={message}")
        return result, message

    async def check_server_status(self) -> Tuple[bool, str]:
        msg = _serialize_request(SERVER_STATUS_REQ, [])
        return await self._send_request(msg)

    async def validate_waypoint(
        self, current: Coordinate, next_loc: Coordinate
    ) -> Tuple[bool, str]:
        logger.debug(
            f"SafetyCheckerClient: validate_waypoint current=({current.lat:.6f},{current.lon:.6f}) "
            f"next=({next_loc.lat:.6f},{next_loc.lon:.6f})"
        )
        msg = _serialize_request(
            VALIDATE_WAYPOINT_REQ,
            [current.to_json(), next_loc.to_json()],
        )
        return await self._send_request(msg)

    async def validate_change_speed(self, new_speed: float) -> Tuple[bool, str]:
        msg = _serialize_request(VALIDATE_CHANGE_SPEED_REQ, [new_speed])
        return await self._send_request(msg)

    async def validate_takeoff(
        self, takeoff_alt: float, current_lat: float, current_lon: float
    ) -> Tuple[bool, str]:
        msg = _serialize_request(
            VALIDATE_TAKEOFF_REQ, [takeoff_alt, current_lat, current_lon]
        )
        return await self._send_request(msg)

    async def validate_landing(
        self, current_lat: float, current_lon: float
    ) -> Tuple[bool, str]:
        msg = _serialize_request(VALIDATE_LANDING_REQ, [current_lat, current_lon])
        return await self._send_request(msg)
