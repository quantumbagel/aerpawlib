"""
ConnectionHandler for aerpawlib v2.

Single authority for connection state and heartbeat monitoring.
Starts monitoring after first telemetry or short delay.
"""

from __future__ import annotations

import asyncio
import signal
import sys
import time
from typing import Callable, Optional, Protocol

from ..constants import HEARTBEAT_START_DELAY_S, HEARTBEAT_TIMEOUT_S
from ..exceptions import HeartbeatLostError
from ..log import LogComponent, get_logger
from ..protocols import VehicleProtocol

logger = get_logger(LogComponent.VEHICLE)


class ConnectionHandler:
    """
    Single authority for connection state and heartbeat.

    Protocol-based: depends on VehicleProtocol (heartbeat_tick), not concrete Vehicle.
    Start monitoring only after first telemetry or short post-connect delay.
    On disconnect: notify OEO, trigger callbacks, exit.
    """

    def __init__(
        self,
        vehicle: VehicleProtocol,
        heartbeat_timeout: float = HEARTBEAT_TIMEOUT_S,
        start_delay: float = HEARTBEAT_START_DELAY_S,
        on_disconnect: Optional[Callable[[], None]] = None,
    ) -> None:
        self._vehicle = vehicle
        self._heartbeat_timeout = heartbeat_timeout
        self._start_delay = start_delay
        self._on_disconnect = on_disconnect
        self._last_tick: float = 0.0
        self._monitor_started = False
        self._monitor_task: Optional[asyncio.Task] = None
        self._disconnected = False
        self._disconnect_future: Optional[asyncio.Future] = None

    def heartbeat_tick(self) -> None:
        """Called by vehicle when telemetry received."""
        self._last_tick = time.monotonic()
        if not self._monitor_started:
            self._monitor_started = True

    def start(self) -> asyncio.Task:
        """Start heartbeat monitor. Returns the monitor task for awaiting."""
        logger.info(
            f"ConnectionHandler: starting heartbeat monitor "
            f"(timeout={self._heartbeat_timeout}s, start_delay={self._start_delay}s)"
        )
        self._disconnect_future = asyncio.get_running_loop().create_future()
        self._monitor_task = asyncio.create_task(self._monitor_loop())
        return self._monitor_task

    def get_disconnect_future(self) -> asyncio.Future:
        """Return a future that completes (with exception) when heartbeat is lost."""
        if self._disconnect_future is None:
            self._disconnect_future = asyncio.get_running_loop().create_future()
        return self._disconnect_future

    async def _monitor_loop(self) -> None:
        """Monitor heartbeat. Start checking after start_delay."""
        logger.debug(f"ConnectionHandler: monitor sleeping {self._start_delay}s before first check")
        await asyncio.sleep(self._start_delay)  # Justified: avoid false "heartbeat lost"
        logger.debug("ConnectionHandler: monitor active, checking heartbeat")
        while not self._disconnected:
            now = time.monotonic()
            age = now - self._last_tick
            if age > self._heartbeat_timeout:
                self._disconnected = True
                logger.error(f"Heartbeat lost (last {age:.1f}s ago)")
                if self._on_disconnect:
                    self._on_disconnect()
                err = HeartbeatLostError(last_heartbeat_age=age)
                if self._disconnect_future and not self._disconnect_future.done():
                    self._disconnect_future.set_exception(err)
                raise err
            await asyncio.sleep(1.0)  # Justified: heartbeat check interval

    def stop(self) -> None:
        """Stop monitor."""
        logger.debug("ConnectionHandler: stopping heartbeat monitor")
        self._disconnected = True
        if self._monitor_task:
            self._monitor_task.cancel()
            self._monitor_task = None


def setup_signal_handlers(
    loop: asyncio.AbstractEventLoop,
    on_sigint: Optional[Callable[[], None]] = None,
    on_sigterm: Optional[Callable[[], None]] = None,
) -> None:
    """
    Use loop.add_signal_handler for async-safe SIGINT/SIGTERM.
    Avoid raising from sync signal handlers.
    """
    if sys.platform == "win32":
        return
    try:
        if on_sigint:

            def _sigint():
                if on_sigint:
                    on_sigint()

            loop.add_signal_handler(signal.SIGINT, _sigint)
        if on_sigterm:

            def _sigterm():
                if on_sigterm:
                    on_sigterm()

            loop.add_signal_handler(signal.SIGTERM, _sigterm)
    except NotImplementedError:
        logger.debug("add_signal_handler not available on this platform")
    else:
        logger.debug("Signal handlers (SIGINT, SIGTERM) registered")
