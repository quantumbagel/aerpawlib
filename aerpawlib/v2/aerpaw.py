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
from dataclasses import dataclass
from enum import Enum
from typing import Optional

# Require aiohttp; no synchronous fallback
import aiohttp

# Use modular logging system
from .logging import get_logger, LogComponent
logger = get_logger(LogComponent.AERPAW)


class MessageSeverity(Enum):
    """Severity levels for OEO console messages."""
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


# Constants
DEFAULT_CVM_IP = "192.168.32.25"
DEFAULT_CVM_PORT = 12435


class AERPAWConnectionError(Exception):
    """Raised when AERPAW platform connection fails or is unavailable."""
    pass


class AERPAWRequestError(Exception):
    """Raised when an HTTP request to the AERPAW platform fails."""
    pass


class AERPAWCheckpointError(Exception):
    """Raised when a checkpoint operation fails."""
    pass


class AERPAWValidationError(Exception):
    """Raised when input validation fails."""
    pass


# Validation pattern for checkpoint names (alphanumeric, underscores, hyphens)
_VALID_NAME_PATTERN = re.compile(r'^[a-zA-Z0-9_-]+$')


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
        retry_delay: float = 0.5
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
            retry_delay=retry_delay
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
                timeout=aiohttp.ClientTimeout(total=self.config.timeout)
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

    def _log_stdout(self, msg: str):
        """Print message to stdout if not suppressed."""
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
        retry: bool = True
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

    async def _do_request(
        self,
        method: str,
        url: str,
        timeout: float
    ) -> str:
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
        self,
        message: str,
        severity: MessageSeverity = MessageSeverity.INFO
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

        encoded = base64.urlsafe_b64encode(message.encode('utf-8')).decode('utf-8')
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
        response = await self._request("GET", f"/checkpoint/bool/{checkpoint_name}")
        if response == "True":
            return True
        elif response == "False":
            return False
        raise AERPAWCheckpointError(f"Malformed response from server: {response}")

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
        response = await self._request("GET", f"/checkpoint/int/{counter_name}")
        try:
            return int(response)
        except (TypeError, ValueError):
            raise AERPAWCheckpointError(f"Malformed response from server: {response}")

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
        encoded_value = quote(value, safe='')
        await self._request("POST", f"/checkpoint/string/{key}?val={encoded_value}")

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


# Synchronous compatibility wrapper
class AERPAW:
    """
    Synchronous wrapper for AERPAW platform integration.

    This class provides backward compatibility with the v1 API while
    using the async AERPAWPlatform internally.

    For new code, prefer using AERPAWPlatform directly with async/await.
    """

    def __init__(
        self,
        cvm_addr: str = DEFAULT_CVM_IP,
        cvm_port: int = DEFAULT_CVM_PORT
    ):
        self._platform = AERPAWPlatform(
            cvm_address=cvm_addr,
            cvm_port=cvm_port,
            suppress_stdout=True  # We handle stdout in the sync wrapper
        )
        self._no_stdout = False

        # Attempt synchronous connection check using the async connect
        try:
            self._run_async(self._platform.connect())
        except Exception:
            # If connect fails, platform.connect already sets _connected
            pass

    @property
    def _connected(self) -> bool:
        return self._platform._connected

    def _run_async(self, coro):
        """Run an async coroutine synchronously."""
        try:
            _ = asyncio.get_running_loop()
            # If there's already a running loop, run in a separate thread
            import threading
            result = []
            exception = []

            def run_in_thread():
                try:
                    result.append(asyncio.run(coro))
                except Exception as e:
                    exception.append(e)

            thread = threading.Thread(target=run_in_thread)
            thread.start()
            thread.join()

            if exception:
                raise exception[0]
            return result[0] if result else None
        except RuntimeError:
            # No running loop, safe to use asyncio.run
            return asyncio.run(coro)

    def attach_to_aerpaw_platform(self) -> bool:
        return self._platform._connected

    def _display_connection_warning(self):
        if not self._no_stdout:
            self._platform.config.suppress_stdout = False
        self._platform._display_connection_warning()
        self._platform.config.suppress_stdout = True

    def log_to_oeo(
        self,
        msg: str,
        severity: str = "INFO"
    ):
        sev = MessageSeverity(severity)
        # Print locally first
        if not self._no_stdout:
            print(msg)
        if self._connected:
            try:
                self._run_async(self._platform.log_to_oeo(msg, sev))
            except Exception:
                if not self._no_stdout:
                    print("unable to send previous message to OEO.")

    def checkpoint_reset_server(self):
        if not self._connected:
            self._display_connection_warning()
            raise Exception("AERPAW checkpoint functionality only works in AERPAW environment")
        self._run_async(self._platform.checkpoint_reset_server())

    def checkpoint_set(self, checkpoint_name: str):
        if not self._connected:
            self._display_connection_warning()
            raise Exception("AERPAW checkpoint functionality only works in AERPAW environment")
        self._run_async(self._platform.checkpoint_set(checkpoint_name))

    def checkpoint_check(self, checkpoint_name: str) -> bool:
        if not self._connected:
            self._display_connection_warning()
            raise Exception("AERPAW checkpoint functionality only works in AERPAW environment")
        return self._run_async(self._platform.checkpoint_check(checkpoint_name))

    def checkpoint_increment_counter(self, counter_name: str):
        if not self._connected:
            self._display_connection_warning()
            raise Exception("AERPAW checkpoint functionality only works in AERPAW environment")
        self._run_async(self._platform.checkpoint_increment_counter(counter_name))

    def checkpoint_check_counter(self, counter_name: str) -> int:
        if not self._connected:
            self._display_connection_warning()
            raise Exception("AERPAW checkpoint functionality only works in AERPAW environment")
        return self._run_async(self._platform.checkpoint_check_counter(counter_name))

    def checkpoint_set_string(self, string_name: str, value: str):
        if not self._connected:
            self._display_connection_warning()
            raise Exception("AERPAW checkpoint functionality only works in AERPAW environment")
        self._run_async(self._platform.checkpoint_set_string(string_name, value))

    def checkpoint_check_string(self, string_name: str) -> str:
        if not self._connected:
            self._display_connection_warning()
            raise Exception("AERPAW checkpoint functionality only works in AERPAW environment")
        return self._run_async(self._platform.checkpoint_check_string(string_name))


# Global singleton for backward compatibility
AERPAW_Platform = AERPAW()

# Backward compatibility exports
OEO_MSG_SEV_INFO = MessageSeverity.INFO.value
OEO_MSG_SEV_WARN = MessageSeverity.WARNING.value
OEO_MSG_SEV_ERR = MessageSeverity.ERROR.value
OEO_MSG_SEV_CRIT = MessageSeverity.CRITICAL.value


__all__ = [
    # Main classes
    "AERPAWPlatform",
    "AERPAW",
    "AERPAW_Platform",
    # Configuration
    "AERPAWConfig",
    "MessageSeverity",
    # Exceptions
    "AERPAWConnectionError",
    "AERPAWRequestError",
    "AERPAWCheckpointError",
    "AERPAWValidationError",
    # Constants
    "DEFAULT_CVM_IP",
    "DEFAULT_CVM_PORT",
    # Backward compatibility
    "OEO_MSG_SEV_INFO",
    "OEO_MSG_SEV_WARN",
    "OEO_MSG_SEV_ERR",
    "OEO_MSG_SEV_CRIT",
]
