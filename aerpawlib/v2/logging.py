"""
Simplified logging support for aerpawlib v2 API.

Uses Python's standard logging with minimal extensions.
For structured logging, use the structlog library.
"""
from __future__ import annotations

import logging
import sys
from enum import Enum
from functools import wraps
from typing import Any, Callable, Optional, TypeVar, Union

F = TypeVar("F", bound=Callable[..., Any])


class LogLevel(Enum):
    """Log level enumeration matching Python logging."""
    DEBUG = logging.DEBUG
    INFO = logging.INFO
    WARNING = logging.WARNING
    ERROR = logging.ERROR
    CRITICAL = logging.CRITICAL

    @classmethod
    def from_string(cls, level: str) -> "LogLevel":
        """Convert string to LogLevel."""
        level_upper = level.upper()
        if level_upper == "WARN":
            level_upper = "WARNING"
        return cls[level_upper]


class LogComponent(Enum):
    """Predefined logging components for categorized logging."""
    ROOT = "aerpawlib"
    VEHICLE = "aerpawlib.vehicle"
    DRONE = "aerpawlib.vehicle.drone"
    ROVER = "aerpawlib.vehicle.rover"
    SAFETY = "aerpawlib.safety"
    RUNNER = "aerpawlib.runner"
    TELEMETRY = "aerpawlib.telemetry"
    COMMAND = "aerpawlib.command"
    NAVIGATION = "aerpawlib.navigation"
    CONNECTION = "aerpawlib.connection"
    GEOFENCE = "aerpawlib.geofence"
    ZMQ = "aerpawlib.zmq"
    AERPAW = "aerpawlib.aerpaw"
    USER = "aerpawlib.user"


class ColoredFormatter(logging.Formatter):
    """Colored console formatter for better readability."""

    COLORS = {
        logging.DEBUG: "\033[36m",     # Cyan
        logging.INFO: "\033[32m",      # Green
        logging.WARNING: "\033[33m",   # Yellow
        logging.ERROR: "\033[31m",     # Red
        logging.CRITICAL: "\033[35m",  # Magenta
    }
    RESET = "\033[0m"
    BOLD = "\033[1m"

    def __init__(self, fmt: Optional[str] = None, datefmt: Optional[str] = None, use_colors: bool = True):
        super().__init__(fmt, datefmt)
        self.use_colors = use_colors and sys.stdout.isatty()

    def format(self, record: logging.LogRecord) -> str:
        if self.use_colors:
            color = self.COLORS.get(record.levelno, "")
            record.levelname = f"{color}{self.BOLD}{record.levelname}{self.RESET}"
            if record.levelno >= logging.WARNING:
                record.msg = f"{color}{record.msg}{self.RESET}"
        return super().format(record)


# Global state
_configured = False
_default_format = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"


def configure_logging(
    level: Union[LogLevel, str, int] = LogLevel.INFO,
    format_str: Optional[str] = None,
    use_colors: bool = True,
    log_file: Optional[str] = None,
) -> None:
    """
    Configure logging for aerpawlib.

    Args:
        level: Log level (LogLevel enum, string, or int)
        format_str: Custom format string (uses default if None)
        use_colors: Use colored output for console
        log_file: Optional path to log file
    """
    global _configured

    if isinstance(level, LogLevel):
        level_value = level.value
    elif isinstance(level, str):
        level_value = LogLevel.from_string(level).value
    else:
        level_value = level

    fmt = format_str or _default_format
    root_logger = logging.getLogger("aerpawlib")
    root_logger.setLevel(level_value)
    root_logger.handlers.clear()

    # Console handler
    console = logging.StreamHandler(sys.stdout)
    console.setLevel(level_value)
    console.setFormatter(ColoredFormatter(fmt, use_colors=use_colors))
    root_logger.addHandler(console)

    # File handler
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(level_value)
        file_handler.setFormatter(logging.Formatter(fmt))
        root_logger.addHandler(file_handler)

    _configured = True


def get_logger(component: Union[LogComponent, str] = LogComponent.ROOT) -> logging.Logger:
    """Get a logger for the specified component."""
    global _configured
    if not _configured:
        configure_logging()

    name = component.value if isinstance(component, LogComponent) else component
    return logging.getLogger(name)


def set_level(level: Union[LogLevel, str, int], component: Optional[LogComponent] = None) -> None:
    """Set the log level for a component or the root logger."""
    if isinstance(level, LogLevel):
        level_value = level.value
    elif isinstance(level, str):
        level_value = LogLevel.from_string(level).value
    else:
        level_value = level

    logger_name = component.value if component else "aerpawlib"
    logging.getLogger(logger_name).setLevel(level_value)


def log_call(level: LogLevel = LogLevel.DEBUG) -> Callable[[F], F]:
    """Decorator to log function calls and returns."""
    def decorator(func: F) -> F:
        @wraps(func)
        def wrapper(*args, **kwargs):
            logger = get_logger(LogComponent.ROOT)
            logger.log(level.value, f"Calling {func.__name__}")
            try:
                result = func(*args, **kwargs)
                logger.log(level.value, f"{func.__name__} completed")
                return result
            except Exception as e:
                logger.error(f"{func.__name__} failed: {e}")
                raise
        return wrapper  # type: ignore
    return decorator


def log_timing(level: LogLevel = LogLevel.DEBUG) -> Callable[[F], F]:
    """Decorator to log function execution time."""
    import time

    def decorator(func: F) -> F:
        @wraps(func)
        def wrapper(*args, **kwargs):
            logger = get_logger(LogComponent.ROOT)
            start = time.perf_counter()
            try:
                return func(*args, **kwargs)
            finally:
                elapsed = time.perf_counter() - start
                logger.log(level.value, f"{func.__name__} took {elapsed:.3f}s")
        return wrapper  # type: ignore
    return decorator


__all__ = [
    "LogLevel",
    "LogComponent",
    "ColoredFormatter",
    "configure_logging",
    "get_logger",
    "set_level",
    "log_call",
    "log_timing",
]

