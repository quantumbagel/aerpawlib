"""
Modularized logging support for aerpawlib v2 API.

This module provides a comprehensive, configurable logging system designed
specifically for drone/vehicle control applications. Features include:

- Component-specific loggers (vehicle, safety, runner, telemetry)
- Structured logging with JSON support
- Flight data recording and telemetry logging
- Configurable handlers (console, file, rotating file)
- Context managers for scoped logging
- Performance metrics logging
"""
from __future__ import annotations

import asyncio
import datetime
import json
import logging
import logging.handlers
import sys
import time
from contextlib import contextmanager
from dataclasses import dataclass, field, asdict
from enum import Enum
from functools import wraps
from pathlib import Path
from typing import (
    Any,
    Callable,
    Dict,
    IO,
    List,
    Optional,
    TypeVar,
    Union,
)

# Type variable for decorator typing
F = TypeVar("F", bound=Callable[..., Any])


# ============================================================================
# Log Level Enum
# ============================================================================

class LogLevel(Enum):
    """Log level enumeration with numeric values matching Python logging."""
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


# ============================================================================
# Log Component - Categorized loggers
# ============================================================================

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


# ============================================================================
# Structured Log Record
# ============================================================================

@dataclass
class StructuredLogRecord:
    """
    A structured log record for JSON serialization.

    This provides consistent structure for log entries, particularly useful
    for telemetry and flight data logging.
    """
    timestamp: str
    level: str
    component: str
    message: str
    context: Dict[str, Any] = field(default_factory=dict)

    # Optional fields
    vehicle_id: Optional[str] = None
    mission_id: Optional[str] = None
    state_name: Optional[str] = None
    position: Optional[Dict[str, float]] = None
    error: Optional[str] = None
    traceback: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary, excluding None values."""
        result = asdict(self)
        return {k: v for k, v in result.items() if v is not None}

    def to_json(self, indent: Optional[int] = None) -> str:
        """Convert to JSON string."""
        return json.dumps(self.to_dict(), indent=indent, default=str)


# ============================================================================
# Custom Formatters
# ============================================================================

class ColoredFormatter(logging.Formatter):
    """
    Colored console formatter for better readability.

    Colors are based on log level for quick visual identification.
    """

    # ANSI color codes
    COLORS = {
        logging.DEBUG: "\033[36m",     # Cyan
        logging.INFO: "\033[32m",      # Green
        logging.WARNING: "\033[33m",   # Yellow
        logging.ERROR: "\033[31m",     # Red
        logging.CRITICAL: "\033[35m",  # Magenta
    }
    RESET = "\033[0m"
    BOLD = "\033[1m"

    def __init__(
        self,
        fmt: Optional[str] = None,
        datefmt: Optional[str] = None,
        use_colors: bool = True,
    ):
        super().__init__(fmt, datefmt)
        self.use_colors = use_colors and sys.stdout.isatty()

    def format(self, record: logging.LogRecord) -> str:
        """Format the log record with optional colors."""
        if self.use_colors:
            color = self.COLORS.get(record.levelno, "")
            record.levelname = f"{color}{self.BOLD}{record.levelname}{self.RESET}"

            # Color the message based on level for errors/warnings
            if record.levelno >= logging.WARNING:
                record.msg = f"{color}{record.msg}{self.RESET}"

        return super().format(record)


class JSONFormatter(logging.Formatter):
    """
    JSON formatter for structured logging output.

    Produces one JSON object per log entry for easy parsing and analysis.
    """

    def __init__(
        self,
        include_extra: bool = True,
        include_traceback: bool = True,
        pretty: bool = False,
    ):
        super().__init__()
        self.include_extra = include_extra
        self.include_traceback = include_traceback
        self.indent = 2 if pretty else None

    def format(self, record: logging.LogRecord) -> str:
        """Format the log record as JSON."""
        log_dict: Dict[str, Any] = {
            "timestamp": datetime.datetime.fromtimestamp(record.created).isoformat(),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
        }

        # Add location info
        log_dict["location"] = {
            "file": record.filename,
            "line": record.lineno,
            "function": record.funcName,
        }

        # Add exception info if present
        if record.exc_info and self.include_traceback:
            log_dict["exception"] = self.formatException(record.exc_info)

        # Add extra fields from record
        if self.include_extra:
            extra_fields = {}
            for key, value in record.__dict__.items():
                if key not in (
                    "name", "msg", "args", "created", "filename", "funcName",
                    "levelname", "levelno", "lineno", "module", "msecs",
                    "pathname", "process", "processName", "relativeCreated",
                    "stack_info", "exc_info", "exc_text", "thread", "threadName",
                    "message", "asctime",
                ):
                    try:
                        json.dumps(value)  # Test if serializable
                        extra_fields[key] = value
                    except (TypeError, ValueError):
                        extra_fields[key] = str(value)

            if extra_fields:
                log_dict["extra"] = extra_fields

        return json.dumps(log_dict, indent=self.indent, default=str)


class TelemetryFormatter(logging.Formatter):
    """
    Specialized formatter for telemetry data.

    Produces compact, timestamped telemetry entries optimized for
    high-frequency logging.
    """

    def __init__(self, include_source: bool = False):
        super().__init__()
        self.include_source = include_source
        self._start_time = time.time()

    def format(self, record: logging.LogRecord) -> str:
        """Format telemetry record with relative timestamp."""
        elapsed = time.time() - self._start_time

        parts = [
            f"{elapsed:10.3f}",
            record.getMessage(),
        ]

        if self.include_source:
            parts.insert(1, f"[{record.name.split('.')[-1]}]")

        return " | ".join(parts)


# ============================================================================
# Custom Handlers
# ============================================================================

class TelemetryHandler(logging.Handler):
    """
    Handler that buffers telemetry data for batch writing.

    Designed for high-frequency telemetry logging with periodic flush.
    """

    def __init__(
        self,
        buffer_size: int = 100,
        flush_interval: float = 5.0,
    ):
        super().__init__()
        self.buffer: List[logging.LogRecord] = []
        self.buffer_size = buffer_size
        self.flush_interval = flush_interval
        self._last_flush = time.time()
        self._callbacks: List[Callable[[List[logging.LogRecord]], None]] = []

    def emit(self, record: logging.LogRecord) -> None:
        """Add record to buffer and flush if necessary."""
        self.buffer.append(record)

        should_flush = (
            len(self.buffer) >= self.buffer_size or
            time.time() - self._last_flush >= self.flush_interval
        )

        if should_flush:
            self.flush()

    def flush(self) -> None:
        """Flush buffered records to callbacks."""
        if not self.buffer:
            return

        for callback in self._callbacks:
            try:
                callback(self.buffer.copy())
            except Exception:
                pass  # Don't let callback errors affect logging

        self.buffer.clear()
        self._last_flush = time.time()

    def add_callback(self, callback: Callable[[List[logging.LogRecord]], None]) -> None:
        """Add a callback to receive flushed records."""
        self._callbacks.append(callback)

    def remove_callback(self, callback: Callable[[List[logging.LogRecord]], None]) -> None:
        """Remove a callback."""
        if callback in self._callbacks:
            self._callbacks.remove(callback)


class AsyncFileHandler(logging.Handler):
    """
    Async-friendly file handler that doesn't block the event loop.

    Uses a queue and background task for file I/O.
    """

    def __init__(
        self,
        filename: Union[str, Path],
        mode: str = "a",
        encoding: str = "utf-8",
    ):
        super().__init__()
        self.filename = Path(filename)
        self.mode = mode
        self.encoding = encoding
        self._queue: asyncio.Queue[Optional[str]] = asyncio.Queue()
        self._writer_task: Optional[asyncio.Task] = None
        self._file: Optional[IO[str]] = None

    async def start(self) -> None:
        """Start the async writer task."""
        self.filename.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.filename, self.mode, encoding=self.encoding)
        self._writer_task = asyncio.create_task(self._write_loop())

    async def _write_loop(self) -> None:
        """Background loop that writes queued messages to file."""
        while True:
            try:
                message = await self._queue.get()
                if message is None:  # Shutdown signal
                    break
                if self._file:
                    self._file.write(message + "\n")
                    self._file.flush()
            except asyncio.CancelledError:
                break
            except Exception:
                pass  # Don't let write errors crash the loop

    def emit(self, record: logging.LogRecord) -> None:
        """Queue a record for async writing."""
        try:
            msg = self.format(record)
            self._queue.put_nowait(msg)
        except asyncio.QueueFull:
            pass  # Drop message if queue is full
        except Exception:
            self.handleError(record)

    async def stop(self) -> None:
        """Stop the async writer and close the file."""
        await self._queue.put(None)  # Send shutdown signal
        if self._writer_task:
            await self._writer_task
        if self._file:
            self._file.close()
            self._file = None

    def close(self) -> None:
        """Synchronous close (for compatibility)."""
        if self._file:
            self._file.close()
            self._file = None
        super().close()


# ============================================================================
# Flight Data Recorder
# ============================================================================

@dataclass
class TelemetryPoint:
    """A single telemetry data point."""
    timestamp: float
    latitude: float
    longitude: float
    altitude: float
    heading: float
    groundspeed: float
    vertical_speed: float
    battery_voltage: float
    battery_percentage: float
    gps_satellites: int
    gps_fix_type: int
    flight_mode: str
    armed: bool
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return asdict(self)


@dataclass
class FlightLogMetadata:
    """Metadata for a flight log."""
    start_time: str
    end_time: Optional[str] = None
    vehicle_type: str = "unknown"
    vehicle_id: Optional[str] = None
    mission_name: Optional[str] = None
    pilot: Optional[str] = None
    notes: str = ""
    total_points: int = 0
    duration_seconds: float = 0.0
    max_altitude: float = 0.0
    max_speed: float = 0.0
    total_distance: float = 0.0


class FlightDataRecorder:
    """
    Records and manages flight telemetry data.

    Provides high-performance telemetry recording with configurable
    sample rates and multiple export formats.
    """

    def __init__(
        self,
        sample_interval: float = 0.1,
        auto_save_interval: Optional[float] = None,
        output_dir: Union[str, Path] = ".",
    ):
        """
        Initialize the flight data recorder.

        Args:
            sample_interval: Minimum time between samples in seconds
            auto_save_interval: If set, auto-save every N seconds
            output_dir: Directory for saving flight logs
        """
        self.sample_interval = sample_interval
        self.auto_save_interval = auto_save_interval
        self.output_dir = Path(output_dir)

        self._data: List[TelemetryPoint] = []
        self._metadata = FlightLogMetadata(
            start_time=datetime.datetime.now().isoformat()
        )
        self._recording = False
        self._last_sample_time = 0.0
        self._start_timestamp: Optional[float] = None
        self._auto_save_task: Optional[asyncio.Task] = None

        # Logging
        self._logger = get_logger(LogComponent.TELEMETRY)

    @property
    def is_recording(self) -> bool:
        """Check if recording is active."""
        return self._recording

    @property
    def point_count(self) -> int:
        """Get the number of recorded points."""
        return len(self._data)

    def start(
        self,
        vehicle_type: str = "unknown",
        vehicle_id: Optional[str] = None,
        mission_name: Optional[str] = None,
    ) -> None:
        """Start recording telemetry."""
        if self._recording:
            return

        self._data.clear()
        self._start_timestamp = time.time()
        self._last_sample_time = 0.0
        self._metadata = FlightLogMetadata(
            start_time=datetime.datetime.now().isoformat(),
            vehicle_type=vehicle_type,
            vehicle_id=vehicle_id,
            mission_name=mission_name,
        )
        self._recording = True

        self._logger.info(
            f"Started flight recording for {vehicle_type}",
            extra={"vehicle_id": vehicle_id, "mission": mission_name}
        )

        # Start auto-save task if configured
        if self.auto_save_interval:
            self._start_auto_save()

    def stop(self) -> int:
        """
        Stop recording and return the number of points recorded.

        Returns:
            Number of telemetry points recorded
        """
        if not self._recording:
            return 0

        self._recording = False
        self._metadata.end_time = datetime.datetime.now().isoformat()
        self._metadata.total_points = len(self._data)

        if self._start_timestamp:
            self._metadata.duration_seconds = time.time() - self._start_timestamp

        # Update statistics
        if self._data:
            self._metadata.max_altitude = max(p.altitude for p in self._data)
            self._metadata.max_speed = max(p.groundspeed for p in self._data)

        # Stop auto-save
        if self._auto_save_task:
            self._auto_save_task.cancel()
            self._auto_save_task = None

        self._logger.info(
            f"Stopped flight recording: {len(self._data)} points over "
            f"{self._metadata.duration_seconds:.1f}s"
        )

        return len(self._data)

    def record(
        self,
        latitude: float,
        longitude: float,
        altitude: float,
        heading: float = 0.0,
        groundspeed: float = 0.0,
        vertical_speed: float = 0.0,
        battery_voltage: float = 0.0,
        battery_percentage: float = 0.0,
        gps_satellites: int = 0,
        gps_fix_type: int = 0,
        flight_mode: str = "UNKNOWN",
        armed: bool = False,
        **extra: Any,
    ) -> bool:
        """
        Record a telemetry point.

        Returns True if point was recorded, False if skipped due to sample rate.
        """
        if not self._recording:
            return False

        current_time = time.time()
        elapsed = current_time - self._last_sample_time

        if elapsed < self.sample_interval:
            return False

        self._last_sample_time = current_time

        point = TelemetryPoint(
            timestamp=current_time,
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
            heading=heading,
            groundspeed=groundspeed,
            vertical_speed=vertical_speed,
            battery_voltage=battery_voltage,
            battery_percentage=battery_percentage,
            gps_satellites=gps_satellites,
            gps_fix_type=gps_fix_type,
            flight_mode=flight_mode,
            armed=armed,
            extra=extra,
        )

        self._data.append(point)
        return True

    def _start_auto_save(self) -> None:
        """Start the auto-save background task."""
        async def auto_save_loop():
            while self._recording:
                await asyncio.sleep(self.auto_save_interval)
                if self._recording:
                    filename = f"autosave_{datetime.datetime.now():%Y%m%d_%H%M%S}.json"
                    self.save(filename, format="json")

        try:
            loop = asyncio.get_running_loop()
            self._auto_save_task = loop.create_task(auto_save_loop())
        except RuntimeError:
            pass  # No running loop

    def save(
        self,
        filename: Optional[str] = None,
        format: str = "json",
    ) -> Path:
        """
        Save the flight log to a file.

        Args:
            filename: Output filename (auto-generated if not provided)
            format: Output format ('json' or 'csv')

        Returns:
            Path to the saved file
        """
        if filename is None:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"flight_log_{timestamp}.{format}"

        filepath = self.output_dir / filename
        filepath.parent.mkdir(parents=True, exist_ok=True)

        if format == "json":
            self._save_json(filepath)
        elif format == "csv":
            self._save_csv(filepath)
        else:
            raise ValueError(f"Unsupported format: {format}")

        self._logger.info(f"Saved flight log to {filepath}")
        return filepath

    def _save_json(self, filepath: Path) -> None:
        """Save as JSON."""
        data = {
            "metadata": asdict(self._metadata),
            "telemetry": [p.to_dict() for p in self._data],
        }
        with open(filepath, "w") as f:
            json.dump(data, f, indent=2, default=str)

    def _save_csv(self, filepath: Path) -> None:
        """Save as CSV."""
        import csv

        if not self._data:
            return

        fieldnames = list(self._data[0].to_dict().keys())
        # Flatten extra dict for CSV
        fieldnames.remove("extra")

        with open(filepath, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            for point in self._data:
                row = point.to_dict()
                row.pop("extra", None)
                writer.writerow(row)

    def get_data(self) -> List[TelemetryPoint]:
        """Get all recorded data points."""
        return self._data.copy()

    def clear(self) -> None:
        """Clear all recorded data."""
        self._data.clear()
        self._logger.debug("Cleared flight data recorder")


# ============================================================================
# Logger Configuration
# ============================================================================

@dataclass
class LoggingConfig:
    """Configuration for the logging system."""
    # Global settings
    level: LogLevel = LogLevel.INFO
    format: str = "%(asctime)s | %(levelname)-8s | %(name)s | %(message)s"
    date_format: str = "%Y-%m-%d %H:%M:%S"

    # Console settings
    console_enabled: bool = True
    console_level: Optional[LogLevel] = None
    console_colored: bool = True

    # File settings
    file_enabled: bool = False
    file_path: Optional[str] = None
    file_level: Optional[LogLevel] = None
    file_max_bytes: int = 10 * 1024 * 1024  # 10MB
    file_backup_count: int = 5

    # JSON file settings
    json_enabled: bool = False
    json_path: Optional[str] = None
    json_level: Optional[LogLevel] = None
    json_pretty: bool = False

    # Component-specific levels
    component_levels: Dict[str, LogLevel] = field(default_factory=dict)


class LoggingManager:
    """
    Central manager for all aerpawlib logging.

    Provides configuration, handler management, and component-specific logging.
    """

    _instance: Optional["LoggingManager"] = None
    _initialized: bool = False

    def __new__(cls) -> "LoggingManager":
        """Singleton pattern for global logging management."""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if self._initialized:
            return

        self._config = LoggingConfig()
        self._handlers: Dict[str, logging.Handler] = {}
        self._root_logger = logging.getLogger("aerpawlib")
        self._flight_recorder: Optional[FlightDataRecorder] = None
        self._context: Dict[str, Any] = {}
        self._initialized = True

    def configure(self, config: Optional[LoggingConfig] = None, **kwargs: Any) -> None:
        """
        Configure the logging system.

        Args:
            config: LoggingConfig object, or None to use kwargs
            **kwargs: Config options if config is None
        """
        if config is not None:
            self._config = config
        else:
            # Update config from kwargs
            for key, value in kwargs.items():
                if hasattr(self._config, key):
                    setattr(self._config, key, value)

        self._apply_config()

    def _apply_config(self) -> None:
        """Apply the current configuration."""
        cfg = self._config

        # Configure root logger
        self._root_logger.setLevel(cfg.level.value)

        # Remove existing handlers
        for handler in list(self._handlers.values()):
            self._root_logger.removeHandler(handler)
        self._handlers.clear()

        # Console handler
        if cfg.console_enabled:
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(
                cfg.console_level.value if cfg.console_level else cfg.level.value
            )

            if cfg.console_colored:
                formatter = ColoredFormatter(cfg.format, cfg.date_format)
            else:
                formatter = logging.Formatter(cfg.format, cfg.date_format)

            console_handler.setFormatter(formatter)
            self._root_logger.addHandler(console_handler)
            self._handlers["console"] = console_handler

        # File handler (rotating)
        if cfg.file_enabled and cfg.file_path:
            # Ensure directory exists
            Path(cfg.file_path).parent.mkdir(parents=True, exist_ok=True)

            file_handler = logging.handlers.RotatingFileHandler(
                cfg.file_path,
                maxBytes=cfg.file_max_bytes,
                backupCount=cfg.file_backup_count,
            )
            file_handler.setLevel(
                cfg.file_level.value if cfg.file_level else cfg.level.value
            )
            file_handler.setFormatter(
                logging.Formatter(cfg.format, cfg.date_format)
            )
            self._root_logger.addHandler(file_handler)
            self._handlers["file"] = file_handler

        # JSON file handler
        if cfg.json_enabled and cfg.json_path:
            Path(cfg.json_path).parent.mkdir(parents=True, exist_ok=True)

            json_handler = logging.handlers.RotatingFileHandler(
                cfg.json_path,
                maxBytes=cfg.file_max_bytes,
                backupCount=cfg.file_backup_count,
            )
            json_handler.setLevel(
                cfg.json_level.value if cfg.json_level else cfg.level.value
            )
            json_handler.setFormatter(JSONFormatter(pretty=cfg.json_pretty))
            self._root_logger.addHandler(json_handler)
            self._handlers["json"] = json_handler

        # Apply component-specific levels
        for component, level in cfg.component_levels.items():
            logging.getLogger(component).setLevel(level.value)

    def get_logger(
        self,
        component: Union[LogComponent, str],
        extra: Optional[Dict[str, Any]] = None,
    ) -> Union[logging.Logger, "LoggerAdapter"]:
        """
        Get a logger for a specific component.

        Args:
            component: LogComponent enum or string name
            extra: Extra context to attach to all log records

        Returns:
            Configured logger instance
        """
        if isinstance(component, LogComponent):
            name = component.value
        else:
            name = component

        logger = logging.getLogger(name)

        if extra:
            return LoggerAdapter(logger, {**self._context, **extra})

        if self._context:
            return LoggerAdapter(logger, self._context)

        return logger

    def set_context(self, **kwargs: Any) -> None:
        """Set global context that will be added to all log records."""
        self._context.update(kwargs)

    def clear_context(self) -> None:
        """Clear the global logging context."""
        self._context.clear()

    @contextmanager
    def context(self, **kwargs: Any):
        """Context manager for temporary logging context."""
        old_context = self._context.copy()
        self._context.update(kwargs)
        try:
            yield
        finally:
            self._context = old_context

    def get_flight_recorder(
        self,
        sample_interval: float = 0.1,
        output_dir: Union[str, Path] = ".",
    ) -> FlightDataRecorder:
        """
        Get or create the flight data recorder.

        Args:
            sample_interval: Time between samples in seconds
            output_dir: Directory for saving flight logs

        Returns:
            FlightDataRecorder instance
        """
        if self._flight_recorder is None:
            self._flight_recorder = FlightDataRecorder(
                sample_interval=sample_interval,
                output_dir=output_dir,
            )
        return self._flight_recorder

    def set_level(
        self,
        level: Union[LogLevel, str, int],
        component: Optional[Union[LogComponent, str]] = None,
    ) -> None:
        """
        Set logging level globally or for a specific component.

        Args:
            level: Log level
            component: If provided, set level only for this component
        """
        if isinstance(level, str):
            level = LogLevel.from_string(level)
        elif isinstance(level, int):
            level = LogLevel(level)

        if component is None:
            self._config.level = level
            self._root_logger.setLevel(level.value)
        else:
            name = component.value if isinstance(component, LogComponent) else component
            logging.getLogger(name).setLevel(level.value)
            self._config.component_levels[name] = level

    def add_handler(self, name: str, handler: logging.Handler) -> None:
        """Add a custom handler to the root logger."""
        if name in self._handlers:
            self._root_logger.removeHandler(self._handlers[name])
        self._root_logger.addHandler(handler)
        self._handlers[name] = handler

    def remove_handler(self, name: str) -> None:
        """Remove a handler by name."""
        if name in self._handlers:
            self._root_logger.removeHandler(self._handlers[name])
            del self._handlers[name]


class LoggerAdapter(logging.LoggerAdapter):
    """
    Logger adapter that adds context to all log messages.

    Extends the standard LoggerAdapter with additional convenience methods.
    """

    def process(
        self, msg: str, kwargs: Dict[str, Any]
    ) -> tuple[str, Dict[str, Any]]:
        """Add extra context to log record."""
        extra = kwargs.get("extra", {})
        extra.update(self.extra)
        kwargs["extra"] = extra
        return msg, kwargs

    def success(self, msg: str, *args: Any, **kwargs: Any) -> None:
        """Log a success message at INFO level with 'success' flag."""
        kwargs.setdefault("extra", {})["success"] = True
        self.info(msg, *args, **kwargs)

    def mission_event(
        self,
        event: str,
        details: Optional[Dict[str, Any]] = None,
        **kwargs: Any,
    ) -> None:
        """Log a mission-related event."""
        extra = kwargs.get("extra", {})
        extra["event_type"] = "mission"
        extra["event"] = event
        if details:
            extra["details"] = details
        kwargs["extra"] = extra
        self.info(f"Mission event: {event}", **kwargs)

    def telemetry(
        self,
        position: Optional[tuple[float, float, float]] = None,
        velocity: Optional[tuple[float, float, float]] = None,
        attitude: Optional[tuple[float, float, float]] = None,
        **kwargs: Any,
    ) -> None:
        """Log telemetry data at DEBUG level."""
        extra = kwargs.get("extra", {})
        extra["telemetry"] = True
        if position:
            extra["position"] = {"lat": position[0], "lon": position[1], "alt": position[2]}
        if velocity:
            extra["velocity"] = {"vn": velocity[0], "ve": velocity[1], "vd": velocity[2]}
        if attitude:
            extra["attitude"] = {"pitch": attitude[0], "roll": attitude[1], "yaw": attitude[2]}
        kwargs["extra"] = extra

        msg_parts = []
        if position:
            msg_parts.append(f"pos=({position[0]:.6f}, {position[1]:.6f}, {position[2]:.1f})")
        if velocity:
            msg_parts.append(f"vel=({velocity[0]:.1f}, {velocity[1]:.1f}, {velocity[2]:.1f})")
        if attitude:
            msg_parts.append(f"att=({attitude[0]:.1f}, {attitude[1]:.1f}, {attitude[2]:.1f})")

        self.debug(", ".join(msg_parts) if msg_parts else "telemetry", **kwargs)


# ============================================================================
# Module-level convenience functions
# ============================================================================

def get_manager() -> LoggingManager:
    """Get the global LoggingManager instance."""
    return LoggingManager()


def configure_logging(
    level: Union[LogLevel, str] = LogLevel.INFO,
    console: bool = True,
    colored: bool = True,
    file: Optional[str] = None,
    json_file: Optional[str] = None,
    **kwargs: Any,
) -> LoggingManager:
    """
    Configure the aerpawlib logging system.

    This is the main entry point for setting up logging.

    Args:
        level: Default log level
        console: Enable console logging
        colored: Use colored console output
        file: Path to log file (enables file logging)
        json_file: Path to JSON log file (enables JSON logging)
        **kwargs: Additional LoggingConfig options

    Returns:
        The LoggingManager instance

    Example:
        >>> from aerpawlib.v2.logging import configure_logging, LogLevel
        >>> configure_logging(
        ...     level=LogLevel.DEBUG,
        ...     file="flight.log",
        ...     json_file="flight.json",
        ... )
    """
    if isinstance(level, str):
        level = LogLevel.from_string(level)

    config = LoggingConfig(
        level=level,
        console_enabled=console,
        console_colored=colored,
        file_enabled=file is not None,
        file_path=file,
        json_enabled=json_file is not None,
        json_path=json_file,
        **kwargs,
    )

    manager = get_manager()
    manager.configure(config)
    return manager


def get_logger(
    component: Union[LogComponent, str] = LogComponent.USER,
    **extra: Any,
) -> Union[logging.Logger, LoggerAdapter]:
    """
    Get a logger for a component.

    Args:
        component: LogComponent enum or string name
        **extra: Extra context to add to all log records

    Returns:
        Logger instance

    Example:
        >>> from aerpawlib.v2.logging import get_logger, LogComponent
        >>> logger = get_logger(LogComponent.VEHICLE)
        >>> logger.info("Vehicle connected")
    """
    return get_manager().get_logger(component, extra if extra else None)


def set_level(
    level: Union[LogLevel, str, int],
    component: Optional[Union[LogComponent, str]] = None,
) -> None:
    """
    Set the logging level.

    Args:
        level: Log level
        component: Optional component to set level for
    """
    get_manager().set_level(level, component)


def get_flight_recorder(
    sample_interval: float = 0.1,
    output_dir: Union[str, Path] = ".",
) -> FlightDataRecorder:
    """
    Get the flight data recorder.

    Args:
        sample_interval: Time between samples in seconds
        output_dir: Directory for flight log files

    Returns:
        FlightDataRecorder instance
    """
    return get_manager().get_flight_recorder(sample_interval, output_dir)


# ============================================================================
# Decorators
# ============================================================================

def log_call(
    logger: Optional[logging.Logger] = None,
    level: int = logging.DEBUG,
    include_args: bool = True,
    include_result: bool = False,
) -> Callable[[F], F]:
    """
    Decorator to log function calls.

    Args:
        logger: Logger to use (defaults to function's module logger)
        level: Log level for the messages
        include_args: Include function arguments in log
        include_result: Include function result in log

    Example:
        >>> @log_call(level=logging.INFO)
        ... async def takeoff(altitude: float):
        ...     ...
    """
    def decorator(func: F) -> F:
        _logger = logger or logging.getLogger(getattr(func, "__module__", __name__))

        @wraps(func)
        async def async_wrapper(*args: Any, **kwargs: Any) -> Any:
            call_info = f"{func.__name__}"
            if include_args:
                args_str = ", ".join(
                    [repr(a) for a in args[1:]]  # Skip self
                    + [f"{k}={v!r}" for k, v in kwargs.items()]
                )
                call_info += f"({args_str})"

            _logger.log(level, f"Calling {call_info}")
            try:
                result = await func(*args, **kwargs)
                if include_result:
                    _logger.log(level, f"{func.__name__} returned {result!r}")
                return result
            except Exception as e:
                _logger.exception(f"{func.__name__} raised {type(e).__name__}: {e}")
                raise

        @wraps(func)
        def sync_wrapper(*args: Any, **kwargs: Any) -> Any:
            call_info = f"{func.__name__}"
            if include_args:
                args_str = ", ".join(
                    [repr(a) for a in args[1:]]
                    + [f"{k}={v!r}" for k, v in kwargs.items()]
                )
                call_info += f"({args_str})"

            _logger.log(level, f"Calling {call_info}")
            try:
                result = func(*args, **kwargs)
                if include_result:
                    _logger.log(level, f"{func.__name__} returned {result!r}")
                return result
            except Exception as e:
                _logger.exception(f"{func.__name__} raised {type(e).__name__}: {e}")
                raise

        if asyncio.iscoroutinefunction(func):
            return async_wrapper  # type: ignore
        return sync_wrapper  # type: ignore

    return decorator


def log_timing(
    logger: Optional[logging.Logger] = None,
    level: int = logging.DEBUG,
    threshold_ms: Optional[float] = None,
) -> Callable[[F], F]:
    """
    Decorator to log function execution time.

    Args:
        logger: Logger to use
        level: Log level
        threshold_ms: Only log if execution exceeds this time in milliseconds

    Example:
        >>> @log_timing(threshold_ms=100)
        ... async def complex_navigation():
        ...     ...
    """
    def decorator(func: F) -> F:
        _logger = logger or logging.getLogger(getattr(func, "__module__", __name__))

        @wraps(func)
        async def async_wrapper(*args: Any, **kwargs: Any) -> Any:
            start = time.perf_counter()
            try:
                return await func(*args, **kwargs)
            finally:
                elapsed_ms = (time.perf_counter() - start) * 1000
                if threshold_ms is None or elapsed_ms >= threshold_ms:
                    _logger.log(level, f"{func.__name__} took {elapsed_ms:.2f}ms")

        @wraps(func)
        def sync_wrapper(*args: Any, **kwargs: Any) -> Any:
            start = time.perf_counter()
            try:
                return func(*args, **kwargs)
            finally:
                elapsed_ms = (time.perf_counter() - start) * 1000
                if threshold_ms is None or elapsed_ms >= threshold_ms:
                    _logger.log(level, f"{func.__name__} took {elapsed_ms:.2f}ms")

        if asyncio.iscoroutinefunction(func):
            return async_wrapper  # type: ignore
        return sync_wrapper  # type: ignore

    return decorator


# ============================================================================
# Exports
# ============================================================================

__all__ = [
    # Enums
    "LogLevel",
    "LogComponent",
    # Configuration
    "LoggingConfig",
    "LoggingManager",
    # Formatters
    "ColoredFormatter",
    "JSONFormatter",
    "TelemetryFormatter",
    # Handlers
    "TelemetryHandler",
    "AsyncFileHandler",
    # Data structures
    "StructuredLogRecord",
    "TelemetryPoint",
    "FlightLogMetadata",
    # Flight recording
    "FlightDataRecorder",
    # Adapter
    "LoggerAdapter",
    # Module functions
    "get_manager",
    "configure_logging",
    "get_logger",
    "set_level",
    "get_flight_recorder",
    # Decorators
    "log_call",
    "log_timing",
]
