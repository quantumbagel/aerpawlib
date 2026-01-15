"""
Connection handling for aerpawlib v2 API.

This module provides enhanced connection handling that aligns with AERPAW's
safety infrastructure. Key principles:

1. Connection drops are NOT recoverable in most cases:
   - Hardware/USB failures warrant experiment abort
   - MAVLink filter severing the connection means the experiment violated safety rules

2. When connection fails:
   - A message is sent to the OEO console
   - The experiment should be aborted
   - Safety pilots perform manual abort via RC control

3. Reconnection is generally NOT allowed because:
   - If hardware failed, the state is unsafe
   - If MAVLink filter severed connection, the script is deemed unsafe to continue
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Dict, List, Optional, TYPE_CHECKING

from .types import DisconnectReason
from ..aerpaw import OEOClient

if TYPE_CHECKING:
    from ..vehicle import Vehicle

logger = logging.getLogger("aerpawlib.connection")


class ConnectionState(Enum):
    """State of the vehicle connection."""

    DISCONNECTED = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    HEARTBEAT_LOST = auto()
    SEVERED = (
        auto()
    )  # Connection was forcibly severed (MAVLink filter or hardware failure)


@dataclass
class ConnectionEvent:
    """Event representing a connection state change."""

    previous_state: ConnectionState
    new_state: ConnectionState
    reason: DisconnectReason
    timestamp: float
    message: str
    recoverable: bool = False


class ConnectionHandler:
    """
    Handles vehicle connection lifecycle and failure scenarios.

    This handler implements AERPAW's connection handling policy:
    - Monitor heartbeats to detect connection loss
    - Distinguish between hardware failures and MAVLink filter disconnects
    - Notify OEO on connection failures
    - Prevent reconnection when safety requires experiment abort

    Example:
        handler = ConnectionHandler(vehicle, oeo_client)
        handler.on_disconnect(my_disconnect_callback)
        handler.start_monitoring()

        # Later, if connection is lost
        # - handler detects via missing heartbeats
        # - notifies OEO
        # - triggers disconnect callback
        # - does NOT attempt reconnection
    """

    def __init__(
        self,
        vehicle: "Vehicle",
        oeo_client: Optional[OEOClient] = None,
        heartbeat_timeout: float = 5.0,
    ):
        """
        Initialize the connection handler.

        Args:
            vehicle: The vehicle instance to monitor
            oeo_client: Optional OEO client for notifications
            heartbeat_timeout: Seconds without heartbeat before declaring connection lost
        """
        self._vehicle = vehicle
        self._oeo = oeo_client
        self._heartbeat_timeout = heartbeat_timeout

        self._state = ConnectionState.DISCONNECTED
        self._last_heartbeat: float = 0.0
        self._disconnect_reason: Optional[DisconnectReason] = None
        self._monitoring_task: Optional[asyncio.Task] = None
        self._running = False

        self._callbacks: Dict[ConnectionState, List[Callable]] = {}
        self._event_history: List[ConnectionEvent] = []

    @property
    def state(self) -> ConnectionState:
        """Current connection state."""
        return self._state

    @property
    def is_connected(self) -> bool:
        """Whether the vehicle is connected and healthy."""
        return self._state == ConnectionState.CONNECTED

    @property
    def is_recoverable(self) -> bool:
        """
        Whether the connection can be recovered.

        Per AERPAW policy, this is always False after any disconnection
        that was not user-initiated.
        """
        return False

    @property
    def disconnect_reason(self) -> Optional[DisconnectReason]:
        """Reason for the last disconnection."""
        return self._disconnect_reason

    @property
    def seconds_since_heartbeat(self) -> float:
        """Seconds since last heartbeat."""
        return (
            time.time() - self._last_heartbeat
            if self._last_heartbeat > 0
            else float("inf")
        )

    def on_state_change(
        self, state: ConnectionState, callback: Callable
    ) -> None:
        """Register a callback for a specific state change."""
        if state not in self._callbacks:
            self._callbacks[state] = []
        self._callbacks[state].append(callback)

    def on_disconnect(self, callback: Callable) -> None:
        """Register a callback for any disconnection event."""
        for state in (
            ConnectionState.DISCONNECTED,
            ConnectionState.SEVERED,
            ConnectionState.HEARTBEAT_LOST,
        ):
            self.on_state_change(state, callback)

    def update_heartbeat(self) -> None:
        """Update the last heartbeat timestamp. Called by telemetry handlers."""
        self._last_heartbeat = time.time()
        if self._state == ConnectionState.HEARTBEAT_LOST:
            self._transition_to(
                ConnectionState.CONNECTED,
                DisconnectReason.UNKNOWN,
                "Heartbeat restored",
            )

    def start_monitoring(self) -> None:
        """Start the connection monitoring task."""
        if self._running:
            return
        self._running = True
        self._monitoring_task = asyncio.create_task(self._monitor_loop())
        logger.info("Connection monitoring started")

    def stop_monitoring(self) -> None:
        """Stop the connection monitoring task."""
        self._running = False
        if self._monitoring_task:
            self._monitoring_task.cancel()
        logger.info("Connection monitoring stopped")

    async def _monitor_loop(self) -> None:
        """Main monitoring loop."""
        while self._running:
            try:
                if self._state == ConnectionState.CONNECTED:
                    if self.seconds_since_heartbeat > self._heartbeat_timeout:
                        await self._handle_heartbeat_lost()

                await asyncio.sleep(1.0)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Connection monitor error: {e}")
                await asyncio.sleep(1.0)

    async def _handle_heartbeat_lost(self) -> None:
        """Handle loss of heartbeat."""
        logger.warning(
            f"Heartbeat lost (last seen {self.seconds_since_heartbeat:.1f}s ago)"
        )

        # Determine if this is likely a MAVLink filter disconnect
        # In a real implementation, the MAVLink filter would send a specific message
        # For now, we assume heartbeat loss is hardware-related
        reason = DisconnectReason.HEARTBEAT_LOST

        self._transition_to(
            ConnectionState.HEARTBEAT_LOST,
            reason,
            "Lost heartbeat from vehicle",
        )

        # Notify OEO
        if self._oeo:
            await self._oeo.notify_connection_failure(
                reason="Lost heartbeat - possible hardware failure or MAVLink filter disconnect",
                is_mavlink_filter=False,  # Would be determined by actual disconnect message
                details={
                    "seconds_since_heartbeat": self.seconds_since_heartbeat,
                    "recoverable": self.is_recoverable,
                },
            )

    async def handle_mavlink_filter_disconnect(
        self, violation_message: str
    ) -> None:
        """
        Handle a disconnection caused by the MAVLink filter.

        This is called when the MAVLink filter severs the connection due to
        an illegal command. The connection is NOT recoverable.

        Args:
            violation_message: Description of the safety violation that triggered disconnect
        """
        logger.critical(
            f"MAVLink filter severed connection: {violation_message}"
        )

        self._transition_to(
            ConnectionState.SEVERED,
            DisconnectReason.MAVLINK_FILTER_SEVERED,
            f"Connection severed by MAVLink filter: {violation_message}",
        )

        # Notify OEO (critical - requires operator intervention)
        if self._oeo:
            await self._oeo.notify_connection_failure(
                reason=violation_message,
                is_mavlink_filter=True,
                details={
                    "action_required": "Manual abort via RC control",
                    "vehicle_state": "Should be in HOLD mode",
                    "recoverable": False,
                },
            )

    async def handle_hardware_failure(self, error_message: str) -> None:
        """
        Handle a hardware failure (USB disconnect, autopilot failure, etc.).

        Args:
            error_message: Description of the hardware failure
        """
        logger.critical(f"Hardware failure detected: {error_message}")

        self._transition_to(
            ConnectionState.SEVERED,
            DisconnectReason.HARDWARE_FAILURE,
            f"Hardware failure: {error_message}",
        )

        # Notify OEO
        if self._oeo:
            await self._oeo.notify_connection_failure(
                reason=error_message,
                is_mavlink_filter=False,
                details={
                    "action_required": "Safety pilot intervention required",
                    "recoverable": False,
                },
            )

    def _transition_to(
        self,
        new_state: ConnectionState,
        reason: DisconnectReason,
        message: str,
    ) -> None:
        """Transition to a new connection state."""
        old_state = self._state
        self._state = new_state
        self._disconnect_reason = reason

        event = ConnectionEvent(
            previous_state=old_state,
            new_state=new_state,
            reason=reason,
            timestamp=time.time(),
            message=message,
            recoverable=self.is_recoverable,
        )
        self._event_history.append(event)

        logger.info(
            f"Connection state: {old_state.name} -> {new_state.name} ({message})"
        )

        # Trigger callbacks
        for callback in self._callbacks.get(new_state, []):
            try:
                result = callback(event)
                if asyncio.iscoroutine(result):
                    asyncio.create_task(result)
            except Exception as e:
                logger.error(f"Error in connection state callback: {e}")

    def mark_connected(self) -> None:
        """Mark the connection as established."""
        self._last_heartbeat = time.time()
        self._disconnect_reason = None
        self._transition_to(
            ConnectionState.CONNECTED,
            DisconnectReason.UNKNOWN,
            "Connected to vehicle",
        )

    def mark_disconnected(
        self, reason: DisconnectReason = DisconnectReason.USER_DISCONNECT
    ) -> None:
        """Mark the connection as closed."""
        self._transition_to(
            ConnectionState.DISCONNECTED, reason, "Disconnected from vehicle"
        )

    @property
    def event_history(self) -> List[ConnectionEvent]:
        """Get the history of connection events."""
        return self._event_history.copy()


__all__ = ["ConnectionState", "ConnectionEvent", "ConnectionHandler"]
