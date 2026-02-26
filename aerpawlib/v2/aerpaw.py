"""
AERPAW platform for aerpawlib v2.

Minimal: connection loss and critical notifications via OEO.
"""

from __future__ import annotations

import base64
from typing import Optional

import requests

from .constants import DEFAULT_FORWARD_SERVER_IP, DEFAULT_FORWARD_SERVER_PORT
from .log import LogComponent, get_logger

logger = get_logger(LogComponent.AERPAW)

OEO_MSG_SEV_INFO = "INFO"
OEO_MSG_SEV_WARN = "WARNING"
OEO_MSG_SEV_ERR = "ERROR"
OEO_MSG_SEV_CRIT = "CRITICAL"


class AERPAW_Platform:
    """
    Minimal AERPAW interface for v2.

    Used for disconnect and critical notifications.
    """

    _instance: Optional["AERPAW_Platform"] = None
    _forw_addr: str = DEFAULT_FORWARD_SERVER_IP
    _forw_port: int = DEFAULT_FORWARD_SERVER_PORT
    _connected: bool = False
    _no_stdout: bool = False

    def __new__(cls) -> "AERPAW_Platform":
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self) -> None:
        if not hasattr(self, "_initialized") or not self._initialized:
            self._connected = self._attach()
            self._initialized = True

    def _attach(self) -> bool:
        try:
            requests.post(
                f"http://{self._forw_addr}:{self._forw_port}/ping",
                timeout=1,
            )
            logger.info(f"AERPAW platform: connected to forward server {self._forw_addr}:{self._forw_port}")
            return True
        except requests.exceptions.RequestException as e:
            logger.debug(f"AERPAW platform: not in AERPAW environment ({self._forw_addr}:{self._forw_port} unreachable: {e})")
            return False

    def _is_aerpaw_environment(self) -> bool:
        return self._connected

    def set_no_stdout(self, value: bool) -> None:
        """Suppress stdout when True."""
        self._no_stdout = value

    def log_to_oeo(
        self,
        msg: str,
        severity: str = OEO_MSG_SEV_INFO,
        agent_id: Optional[str] = None,
    ) -> None:
        """Send message to OEO console."""
        if not self._no_stdout:
            if severity == OEO_MSG_SEV_CRIT:
                logger.critical(msg)
            elif severity == OEO_MSG_SEV_ERR:
                logger.error(msg)
            elif severity == OEO_MSG_SEV_WARN:
                logger.warning(msg)
            else:
                logger.info(msg)
        if not self._connected:
            return
        encoded = base64.urlsafe_b64encode(msg.encode("utf-8")).decode("utf-8")
        try:
            url = f"http://{self._forw_addr}:{self._forw_port}/oeo_msg/{severity}/{encoded}"
            if agent_id:
                url += f"/{agent_id}"
            requests.post(url, timeout=3)
        except requests.exceptions.RequestException as e:
            if not self._no_stdout:
                logger.error(f"Failed to send message to OEO: {e}")
