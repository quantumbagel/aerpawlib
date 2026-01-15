"""
AERPAW platform integration for aerpawlib v2 API.

This module provides async-first integration with the AERPAW platform,
including OEO console logging and checkpoint functionality.

The AERPAW class can be used as an async context manager for automatic
connection handling.

Example:
    async with AERPAWPlatform() as platform:
        await platform.log_to_oeo("Starting mission")
        await platform.checkpoint_set("mission_started")
"""

from __future__ import annotations

import asyncio
import base64
import logging
import re
import json
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, Dict, Any, List, Callable

# Require aiohttp; no synchronous fallback
import aiohttp

# Use modular logging system
from .logging import get_logger, LogComponent
from .exceptions import (
    AERPAWConnectionError,
    AERPAWRequestError,
    AERPAWCheckpointError,
    AERPAWValidationError,
)

logger = get_logger(LogComponent.AERPAW)
oeo_logger = get_logger(LogComponent.OEO)


class NotificationSeverity(Enum):
    """Severity levels for OEO notifications."""

    INFO = auto()  # Informational message
    WARNING = auto()  # Something needs attention but not critical
    ERROR = auto()  # Something failed, may need intervention
    CRITICAL = auto()  # Requires immediate operator intervention
    EMERGENCY = auto()  # Safety-critical, abort likely needed


class NotificationType(Enum):
    """Types of notifications sent to OEO."""

    # Connection events
    CONNECTED = "connected"
    DISCONNECTED = "disconnected"
    HEARTBEAT_LOST = "heartbeat_lost"
    HARDWARE_FAILURE = "hardware_failure"
    MAVLINK_FILTER_SEVERED = "mavlink_filter_severed"

    # Safety events
    GEOFENCE_VIOLATION = "geofence_violation"
    SPEED_LIMIT_VIOLATION = "speed_limit_violation"
    ALTITUDE_VIOLATION = "altitude_violation"
    ILLEGAL_COMMAND_BLOCKED = "illegal_command_blocked"

    # Battery events
    BATTERY_LOW = "battery_low"
    BATTERY_CRITICAL = "battery_critical"

    # Flight events
    ARMED = "armed"
    DISARMED = "disarmed"
    TAKEOFF = "takeoff"
    LANDING = "landing"
    RTL_TRIGGERED = "rtl_triggered"
    HOLD_TRIGGERED = "hold_triggered"

    # Experiment events
    EXPERIMENT_START = "experiment_start"
    EXPERIMENT_END = "experiment_end"
    EXPERIMENT_ABORT = "experiment_abort"
    PREFLIGHT_FAILED = "preflight_failed"


@dataclass
class OEONotification:
    """
    A notification to be sent to the OEO system.

    Attributes:
        type: The type of notification
        severity: How serious is this notification
        message: Human-readable description
        timestamp: When the event occurred
        details: Additional context as key-value pairs
        requires_ack: Whether this notification requires operator acknowledgment
    """

    type: NotificationType
    severity: NotificationSeverity
    message: str
    timestamp: float = field(default_factory=time.time)
    details: Dict[str, Any] = field(default_factory=dict)
    requires_ack: bool = False

    def to_json(self) -> str:
        """Serialize notification to JSON for transmission."""
        return json.dumps(
            {
                "type": self.type.value,
                "severity": self.severity.name,
                "message": self.message,
                "timestamp": self.timestamp,
                "details": self.details,
                "requires_ack": self.requires_ack,
            }
        )

    @classmethod
    def from_json(cls, data: str) -> "OEONotification":
        """Deserialize notification from JSON."""
        d = json.loads(data)
        return cls(
            type=NotificationType(d["type"]),
            severity=NotificationSeverity[d["severity"]],
            message=d["message"],
            timestamp=d.get("timestamp", time.time()),
            details=d.get("details", {}),
            requires_ack=d.get("requires_ack", False),
        )


class OEOClient:
    """
    Client for communicating with the OEO (Operator Experience Operator) system.

    The OEO system is the bridge between the experimenter's container (E-VM)
    and the operators/safety pilots. It receives notifications about:
    - Safety violations that have been blocked by the MAVLink filter
    - Connection failures requiring operator intervention
    - Battery states and other telemetry warnings

    In a real AERPAW deployment, this communicates over the cellular link
    to the OEO console. For local/SITL testing, notifications are logged.

    Example:
        async with OEOClient() as oeo:
            await oeo.notify_experiment_start("My Mission")
            # ... run experiment ...
            await oeo.notify_experiment_end("completed successfully")

    Note:
        This is a stub implementation for local development.
        In actual AERPAW deployment, this connects to the OEO infrastructure.
    """

    def __init__(
        self,
        address: str = "localhost",
        port: int = 14590,
        experiment_id: Optional[str] = None,
    ):
        self._address = address
        self._port = port
        self._experiment_id = experiment_id
        self._connected = False
        self._notification_history: List[OEONotification] = []
        self._callbacks: Dict[NotificationType, List[Callable]] = {}
        self._socket = None  # Would be ZMQ socket in real implementation

    async def connect(self) -> bool:
        """
        Connect to the OEO system.

        Returns:
            True if connected successfully, False otherwise.

        Note:
            In local/SITL mode, this always succeeds but logs a warning
            that notifications are being logged locally only.
        """
        try:
            # In real implementation, this would connect via ZMQ
            # For now, we just log locally
            oeo_logger.info(
                "OEO client initialized (local mode - notifications logged only)"
            )
            self._connected = True
            return True
        except Exception as e:
            oeo_logger.warning(f"Failed to connect to OEO system: {e}")
            self._connected = False
            return False

    async def disconnect(self) -> None:
        """Disconnect from the OEO system."""
        self._connected = False
        oeo_logger.info("OEO client disconnected")

    async def __aenter__(self) -> "OEOClient":
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb) -> None:
        await self.disconnect()

    async def send_notification(self, notification: OEONotification) -> bool:
        """
        Send a notification to the OEO system.

        Args:
            notification: The notification to send

        Returns:
            True if sent successfully (or logged in local mode)
        """
        self._notification_history.append(notification)

        # Log based on severity
        log_msg = f"OEO [{notification.type.value}]: {notification.message}"
        if notification.details:
            log_msg += f" (details: {notification.details})"

        if notification.severity == NotificationSeverity.EMERGENCY:
            oeo_logger.critical(log_msg)
        elif notification.severity == NotificationSeverity.CRITICAL:
            oeo_logger.error(log_msg)
        elif notification.severity == NotificationSeverity.ERROR:
            oeo_logger.error(log_msg)
        elif notification.severity == NotificationSeverity.WARNING:
            oeo_logger.warning(log_msg)
        else:
            oeo_logger.info(log_msg)

        # Trigger callbacks
        for callback in self._callbacks.get(notification.type, []):
            try:
                result = callback(notification)
                if asyncio.iscoroutine(result):
                    await result
            except Exception as e:
                oeo_logger.error(f"Error in OEO notification callback: {e}")

        return True

    def on_notification(
        self, notification_type: NotificationType, callback: Callable
    ) -> None:
        """Register a callback for a specific notification type."""
        if notification_type not in self._callbacks:
            self._callbacks[notification_type] = []
        self._callbacks[notification_type].append(callback)

    # Convenience methods for common notifications

    async def notify_connection_failure(
        self,
        reason: str,
        is_mavlink_filter: bool = False,
        details: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """
        Notify OEO of a connection failure.

        This is sent when the MAVLink stream fails. Per AERPAW policy:
        - If the MAVLink filter severed the connection, the experiment must abort
        - If hardware failed, the experiment must abort
        - Reconnection is NOT an option in either case

        Args:
            reason: Description of why the connection failed
            is_mavlink_filter: True if the MAVLink filter severed the connection
            details: Additional context
        """
        notif_type = (
            NotificationType.MAVLINK_FILTER_SEVERED
            if is_mavlink_filter
            else NotificationType.HARDWARE_FAILURE
        )

        return await self.send_notification(
            OEONotification(
                type=notif_type,
                severity=NotificationSeverity.EMERGENCY,
                message=f"Connection severed: {reason}",
                details=details or {},
                requires_ack=True,  # Operators must acknowledge
            )
        )

    async def notify_illegal_command_blocked(
        self,
        command: str,
        violation_type: str,
        message: str,
        details: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """
        Notify OEO that an illegal command was blocked by the MAVLink filter.

        This notification is sent when the MAVLink filter intercepts and blocks
        a command that would violate safety constraints (geofence, speed, etc.).

        After this notification, the MAVLink filter severs the connection
        and puts the drone in HOLD mode. Safety pilots are alerted for manual abort.

        Args:
            command: The command that was blocked
            violation_type: Type of violation (e.g., "geofence", "speed_limit")
            message: Detailed explanation
            details: Additional context
        """
        return await self.send_notification(
            OEONotification(
                type=NotificationType.ILLEGAL_COMMAND_BLOCKED,
                severity=NotificationSeverity.CRITICAL,
                message=f"ILLEGAL COMMAND BLOCKED - {violation_type}: {message}",
                details={
                    "command": command,
                    "violation_type": violation_type,
                    **(details or {}),
                },
                requires_ack=True,
            )
        )

    async def notify_geofence_violation(
        self,
        position: Any,
        target: Any,
        message: str,
    ) -> bool:
        """Notify OEO of a geofence violation attempt."""
        return await self.send_notification(
            OEONotification(
                type=NotificationType.GEOFENCE_VIOLATION,
                severity=NotificationSeverity.CRITICAL,
                message=f"GEOFENCE VIOLATION: {message}",
                details={
                    "current_position": str(position),
                    "target_position": str(target),
                },
                requires_ack=True,
            )
        )

    async def notify_battery_critical(
        self,
        percentage: float,
        voltage: Optional[float] = None,
        rtl_triggered: bool = False,
    ) -> bool:
        """Notify OEO of critical battery level."""
        return await self.send_notification(
            OEONotification(
                type=NotificationType.BATTERY_CRITICAL,
                severity=NotificationSeverity.CRITICAL,
                message=f"CRITICAL BATTERY: {percentage:.1f}%"
                + (f" ({voltage:.2f}V)" if voltage else ""),
                details={
                    "percentage": percentage,
                    "voltage": voltage,
                    "rtl_triggered": rtl_triggered,
                },
                requires_ack=True,
            )
        )

    async def notify_battery_low(
        self,
        percentage: float,
        voltage: Optional[float] = None,
    ) -> bool:
        """Notify OEO of low battery warning."""
        return await self.send_notification(
            OEONotification(
                type=NotificationType.BATTERY_LOW,
                severity=NotificationSeverity.WARNING,
                message=f"Low battery warning: {percentage:.1f}%"
                + (f" ({voltage:.2f}V)" if voltage else ""),
                details={
                    "percentage": percentage,
                    "voltage": voltage,
                },
            )
        )

    async def notify_experiment_start(
        self,
        experiment_name: str,
        details: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """Notify OEO that an experiment is starting."""
        return await self.send_notification(
            OEONotification(
                type=NotificationType.EXPERIMENT_START,
                severity=NotificationSeverity.INFO,
                message=f"Experiment starting: {experiment_name}",
                details=details or {},
            )
        )

    async def notify_experiment_end(
        self,
        result: str,
        details: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """Notify OEO that an experiment has ended."""
        return await self.send_notification(
            OEONotification(
                type=NotificationType.EXPERIMENT_END,
                severity=NotificationSeverity.INFO,
                message=f"Experiment ended: {result}",
                details=details or {},
            )
        )

    async def notify_experiment_abort(
        self,
        reason: str,
        triggered_by: str = "script",
        details: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """
        Notify OEO that an experiment is being aborted.

        Args:
            reason: Why the experiment is aborting
            triggered_by: What triggered the abort (e.g., "script", "mavlink_filter", "battery")
            details: Additional context
        """
        return await self.send_notification(
            OEONotification(
                type=NotificationType.EXPERIMENT_ABORT,
                severity=NotificationSeverity.CRITICAL,
                message=f"EXPERIMENT ABORT ({triggered_by}): {reason}",
                details={
                    "triggered_by": triggered_by,
                    **(details or {}),
                },
                requires_ack=True,
            )
        )

    async def notify_preflight_failed(
        self,
        failed_checks: List[str],
        details: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """Notify OEO that preflight checks failed."""
        return await self.send_notification(
            OEONotification(
                type=NotificationType.PREFLIGHT_FAILED,
                severity=NotificationSeverity.ERROR,
                message=f"Preflight checks failed: {', '.join(failed_checks)}",
                details={
                    "failed_checks": failed_checks,
                    **(details or {}),
                },
            )
        )

    @property
    def notification_history(self) -> List[OEONotification]:
        """Get the history of all notifications sent this session."""
        return self._notification_history.copy()

    def clear_history(self) -> None:
        """Clear the notification history."""
        self._notification_history.clear()


class MessageSeverity(Enum):
    """Severity levels for OEO console messages."""

    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


# Constants
DEFAULT_CVM_IP = "192.168.32.25"
DEFAULT_CVM_PORT = 12435


# Validation pattern for checkpoint names (alphanumeric, underscores, hyphens)
_VALID_NAME_PATTERN = re.compile(r"^[a-zA-Z0-9_-]+$")


def _validate_name(name: str, param_name: str = "name") -> None:
    """Validate that a name contains only safe characters."""
    if not name:
        raise AERPAWValidationError(f"{param_name} cannot be empty")
    if not _VALID_NAME_PATTERN.match(name):
        raise AERPAWValidationError(
            f"{param_name} must contain only alphanumeric characters, "
            f"underscores, and hyphens. Got: '{name}'"
        )


@dataclass
class AERPAWConfig:
    """Configuration for AERPAW platform connection."""

    cvm_address: str = DEFAULT_CVM_IP
    cvm_port: int = DEFAULT_CVM_PORT
    timeout: float = 1.0
    suppress_stdout: bool = False
    retry_count: int = 3
    retry_delay: float = 0.5

    @property
    def base_url(self) -> str:
        """Get the base URL for the C-VM."""
        return f"http://{self.cvm_address}:{self.cvm_port}"


class AERPAWPlatform:
    """
    Async-first AERPAW platform integration.

    This class provides integration with the AERPAW platform for:
    - OEO console logging
    - Checkpoint management (flags, counters, key-value store)

    Can be used as an async context manager:

        async with AERPAWPlatform() as platform:
            await platform.log_to_oeo("Mission started")

    Or with explicit connect/disconnect:

        platform = AERPAWPlatform()
        await platform.connect()
        # ... use platform ...
        await platform.disconnect()

    Attributes:
        connected: Whether the platform is currently connected
        config: The platform configuration
    """

    def __init__(
        self,
        cvm_address: str = DEFAULT_CVM_IP,
        cvm_port: int = DEFAULT_CVM_PORT,
        timeout: float = 1.0,
        suppress_stdout: bool = False,
        retry_count: int = 3,
        retry_delay: float = 0.5,
    ):
        """
        Initialize the AERPAW platform interface.

        Args:
            cvm_address: IP address of the AERPAW C-VM
            cvm_port: Port number for the C-VM API
            timeout: Connection timeout in seconds
            suppress_stdout: If True, don't print messages to stdout
            retry_count: Number of retry attempts for failed requests
            retry_delay: Delay between retries in seconds
        """
        self.config = AERPAWConfig(
            cvm_address=cvm_address,
            cvm_port=cvm_port,
            timeout=timeout,
            suppress_stdout=suppress_stdout,
            retry_count=retry_count,
            retry_delay=retry_delay,
        )
        self._connected = False
        self._connection_warning_displayed = False
        # Use a forward-reference string for the ClientSession type so static
        # analyzers don't try to resolve attributes on the aiohttp module at
        # import time.
        self._session: Optional["aiohttp.ClientSession"] = None

    @property
    def connected(self) -> bool:
        """Whether the platform is currently connected."""
        return self._connected

    async def connect(self) -> bool:
        """
        Attempt to connect to the AERPAW platform.

        Returns:
            True if connection succeeded, False otherwise.
        """
        # Close any existing session first
        if self._session and not self._session.closed:
            await self._session.close()
            self._session = None

        # Always use aiohttp
        self._session = aiohttp.ClientSession(
            timeout=aiohttp.ClientTimeout(total=self.config.timeout)
        )
        try:
            async with self._session.post(
                f"{self.config.base_url}/ping"
            ) as response:
                self._connected = response.status == 200
        except (aiohttp.ClientError, asyncio.TimeoutError):
            self._connected = False

        # Clean up session if connection failed
        if not self._connected and self._session:
            await self._session.close()
            self._session = None

        # Reset warning flag on new connection attempt
        self._connection_warning_displayed = False
        return self._connected

    async def reconnect(self) -> bool:
        """
        Attempt to reconnect to the AERPAW platform.

        Closes any existing connection and establishes a new one.

        Returns:
            True if reconnection succeeded, False otherwise.
        """
        await self.disconnect()
        return await self.connect()

    async def health_check(self) -> bool:
        """
        Check if the connection to the AERPAW platform is still alive.

        Returns:
            True if connection is healthy, False otherwise.
        """
        if not self._connected:
            return False

        try:
            if self._session is None:
                # No active session -> not healthy
                self._connected = False
                return False

            async with self._session.post(
                f"{self.config.base_url}/ping",
                timeout=aiohttp.ClientTimeout(total=self.config.timeout),
            ) as response:
                return response.status == 200
        except Exception:
            self._connected = False
            return False

    async def disconnect(self):
        """Disconnect from the AERPAW platform and cleanup resources."""
        if self._session and not self._session.closed:
            await self._session.close()
        self._session = None
        self._connected = False

    async def __aenter__(self) -> AERPAWPlatform:
        """Async context manager entry."""
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.disconnect()
        return False

    def _log_stdout(self, msg: str, level: int = logging.INFO):
        """Log message and optionally print to stdout if not suppressed."""
        logger.log(level, msg)
        if not self.config.suppress_stdout:
            print(msg)

    def _display_connection_warning(self):
        """Display a warning about missing AERPAW connection."""
        if self._connection_warning_displayed:
            return
        logger.warning(
            "AERPAW platform functionality requested but not in AERPAW environment"
        )
        self._log_stdout(
            "[aerpawlib] INFO: the user script has attempted to use AERPAW "
            "platform functionality without being in the AERPAW environment"
        )
        self._connection_warning_displayed = True

    async def _request(
        self,
        method: str,
        path: str,
        timeout: Optional[float] = None,
        retry: bool = True,
    ) -> Optional[str]:
        """
        Make an HTTP request to the C-VM with automatic retry.

        Args:
            method: HTTP method ("GET" or "POST")
            path: URL path (will be appended to base URL)
            timeout: Override timeout for this request
            retry: Whether to retry on transient failures

        Returns:
            Response content as string, or None on failure.

        Raises:
            AERPAWConnectionError: If not connected to AERPAW platform.
            AERPAWRequestError: If the request fails after retries.
        """
        if not self._connected:
            self._display_connection_warning()
            raise AERPAWConnectionError(
                "AERPAW functionality only works in AERPAW environment"
            )

        url = f"{self.config.base_url}{path}"
        req_timeout = timeout or 3.0
        max_attempts = self.config.retry_count if retry else 1
        last_error: Optional[Exception] = None

        for attempt in range(max_attempts):
            try:
                return await self._do_request(method, url, req_timeout)
            except AERPAWRequestError as e:
                last_error = e
                if attempt < max_attempts - 1:
                    await asyncio.sleep(self.config.retry_delay)
                    logger.debug(f"Retry attempt {attempt + 1} for {path}")
            except AERPAWConnectionError:
                # Don't retry connection errors
                raise

        raise last_error or AERPAWRequestError("Request failed after retries")

    async def _do_request(self, method: str, url: str, timeout: float) -> str:
        """Execute a single HTTP request using aiohttp."""
        if self._session is None:
            raise AERPAWConnectionError("No active aiohttp session")

        try:
            if method.upper() == "GET":
                async with self._session.get(
                    url, timeout=aiohttp.ClientTimeout(total=timeout)
                ) as response:
                    if response.status != 200:
                        raise AERPAWRequestError(
                            f"Request failed with status {response.status}"
                        )
                    return await response.text()
            else:
                async with self._session.post(
                    url, timeout=aiohttp.ClientTimeout(total=timeout)
                ) as response:
                    if response.status != 200:
                        raise AERPAWRequestError(
                            f"Request failed with status {response.status}"
                        )
                    return await response.text()
        except (aiohttp.ClientError, asyncio.TimeoutError) as e:
            raise AERPAWConnectionError(f"Request failed: {e}")

    # OEO Console Logging

    async def log_to_oeo(
        self, message: str, severity: MessageSeverity = MessageSeverity.INFO
    ):
        """
        Send a message to the OEO console.

        The message is also printed to stdout (unless suppressed).
        If not connected to AERPAW, only prints locally.

        Args:
            message: The message to send
            severity: Message severity level

        Example:
            await platform.log_to_oeo("Starting landing sequence", MessageSeverity.WARNING)
        """
        self._log_stdout(message)

        if not self._connected:
            self._display_connection_warning()
            return

        encoded = base64.urlsafe_b64encode(message.encode("utf-8")).decode(
            "utf-8"
        )
        path = f"/oeo_msg/{severity.value}/{encoded}"

        try:
            await self._request("POST", path, retry=False)
        except (AERPAWConnectionError, AERPAWRequestError):
            self._log_stdout("Unable to send previous message to OEO.")

    # Checkpoint: Boolean Flags

    async def checkpoint_reset_server(self):
        """
        Reset the AERPAW checkpoint server.

        Should be called at the start of an experiment to ensure no stored
        state remains from previous runs.

        Raises:
            AERPAWConnectionError: If not connected to AERPAW platform.
            AERPAWCheckpointError: If the reset operation fails.
        """
        await self._request("POST", "/checkpoint/reset")

    async def checkpoint_set(self, checkpoint_name: str):
        """
        Set a boolean checkpoint flag.

        Args:
            checkpoint_name: Unique name for the checkpoint (alphanumeric, _, -)

        Raises:
            AERPAWValidationError: If the checkpoint name is invalid.
            AERPAWConnectionError: If not connected to AERPAW platform.
            AERPAWRequestError: If the operation fails.
        """
        _validate_name(checkpoint_name, "checkpoint_name")
        await self._request("POST", f"/checkpoint/bool/{checkpoint_name}")

    async def checkpoint_check(self, checkpoint_name: str) -> bool:
        """
        Check if a boolean checkpoint has been set.

        Args:
            checkpoint_name: Name of the checkpoint to check

        Returns:
            True if the checkpoint has been set, False otherwise.

        Raises:
            AERPAWValidationError: If the checkpoint name is invalid.
            AERPAWConnectionError: If not connected to AERPAW platform.
            AERPAWCheckpointError: If the operation fails.
        """
        _validate_name(checkpoint_name, "checkpoint_name")
        response = await self._request(
            "GET", f"/checkpoint/bool/{checkpoint_name}"
        )
        if response == "True":
            return True
        elif response == "False":
            return False
        raise AERPAWCheckpointError(
            f"Malformed response from server: {response}"
        )

    # Checkpoint: Counters

    async def checkpoint_increment_counter(self, counter_name: str):
        """
        Increment a counter in the checkpoint system.

        Args:
            counter_name: Name of the counter to increment (alphanumeric, _, -)

        Raises:
            AERPAWValidationError: If the counter name is invalid.
            AERPAWConnectionError: If not connected to AERPAW platform.
            AERPAWRequestError: If the operation fails.
        """
        _validate_name(counter_name, "counter_name")
        await self._request("POST", f"/checkpoint/int/{counter_name}")

    async def checkpoint_check_counter(self, counter_name: str) -> int:
        """
        Get the current value of a counter.

        An un-incremented counter will always return 0.

        Args:
            counter_name: Name of the counter to check (alphanumeric, _, -)

        Returns:
            The current counter value.

        Raises:
            AERPAWValidationError: If the counter name is invalid.
            AERPAWConnectionError: If not connected to AERPAW platform.
            AERPAWCheckpointError: If the operation fails.
        """
        _validate_name(counter_name, "counter_name")
        response = await self._request(
            "GET", f"/checkpoint/int/{counter_name}"
        )
        try:
            return int(response)
        except (TypeError, ValueError):
            raise AERPAWCheckpointError(
                f"Malformed response from server: {response}"
            )

    # Checkpoint: Key-Value Store

    async def checkpoint_set_string(self, key: str, value: str):
        """
        Set a string value in the checkpoint key-value store.

        Args:
            key: The key name (alphanumeric, _, -)
            value: The value to store (will be URL-encoded)

        Raises:
            AERPAWValidationError: If the key is invalid.
            AERPAWConnectionError: If not connected to AERPAW platform.
            AERPAWRequestError: If the operation fails.
        """
        _validate_name(key, "key")
        # URL-encode the value to handle special characters
        from urllib.parse import quote

        encoded_value = quote(value, safe="")
        await self._request(
            "POST", f"/checkpoint/string/{key}?val={encoded_value}"
        )

    async def checkpoint_check_string(self, key: str) -> str:
        """
        Get a string value from the checkpoint key-value store.

        Args:
            key: The key name (alphanumeric, _, -)

        Returns:
            The stored string value.

        Raises:
            AERPAWValidationError: If the key is invalid.
            AERPAWConnectionError: If not connected to AERPAW platform.
            AERPAWCheckpointError: If the operation fails.
        """
        _validate_name(key, "key")
        return await self._request("GET", f"/checkpoint/string/{key}")


__all__ = [
    # Main classes
    "AERPAWPlatform",
    "OEOClient",
    # Configuration
    "AERPAWConfig",
    "MessageSeverity",
    "NotificationSeverity",
    "NotificationType",
    "OEONotification",
    # Exceptions
    "AERPAWConnectionError",
    "AERPAWRequestError",
    "AERPAWCheckpointError",
    "AERPAWValidationError",
    # Constants
    "DEFAULT_CVM_IP",
    "DEFAULT_CVM_PORT",
]
