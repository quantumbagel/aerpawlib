# aerpawlib v2 Logging System

The aerpawlib v2 API includes a comprehensive, modularized logging system designed specifically for drone and vehicle control applications. This document covers all features and usage patterns.

## Quick Start

```python
from aerpawlib.v2 import configure_logging, get_logger, LogLevel, LogComponent

# Configure logging with defaults
configure_logging()

# Get a component-specific logger
logger = get_logger(LogComponent.VEHICLE)
logger.info("Vehicle connected")

# Or configure with more options
configure_logging(
    level=LogLevel.DEBUG,
    file="flight.log",
    json_file="flight.json",
    colored=True,
)
```

## Features

### 1. Component-Based Logging

The logging system provides predefined components for categorized logging:

```python
from aerpawlib.v2 import get_logger, LogComponent

# Available components
LogComponent.ROOT        # "aerpawlib" - root logger
LogComponent.VEHICLE     # "aerpawlib.vehicle" - vehicle operations
LogComponent.DRONE       # "aerpawlib.vehicle.drone" - drone-specific
LogComponent.ROVER       # "aerpawlib.vehicle.rover" - rover-specific  
LogComponent.SAFETY      # "aerpawlib.safety" - safety checks
LogComponent.RUNNER      # "aerpawlib.runner" - execution framework
LogComponent.TELEMETRY   # "aerpawlib.telemetry" - telemetry data
LogComponent.COMMAND     # "aerpawlib.command" - command execution
LogComponent.NAVIGATION  # "aerpawlib.navigation" - navigation
LogComponent.CONNECTION  # "aerpawlib.connection" - connections
LogComponent.GEOFENCE    # "aerpawlib.geofence" - geofencing
LogComponent.ZMQ         # "aerpawlib.zmq" - ZMQ messaging
LogComponent.AERPAW      # "aerpawlib.aerpaw" - AERPAW platform
LogComponent.USER        # "aerpawlib.user" - user scripts

# Get component-specific logger
vehicle_logger = get_logger(LogComponent.VEHICLE)
safety_logger = get_logger(LogComponent.SAFETY)
```

### 2. Colored Console Output

Console output uses colors for quick visual identification:

- **DEBUG**: Cyan
- **INFO**: Green  
- **WARNING**: Yellow
- **ERROR**: Red
- **CRITICAL**: Magenta

Disable colors if needed:

```python
configure_logging(colored=False)
```

### 3. JSON Structured Logging

Enable JSON logging for machine-parseable logs:

```python
configure_logging(
    json_file="flight.json",
    json_pretty=True,  # Pretty-print for readability
)
```

JSON output includes:
- Timestamp (ISO format)
- Log level
- Logger name
- Message
- Source location (file, line, function)
- Exception traceback (if any)
- Extra context fields

### 4. Rotating File Logs

File logs automatically rotate to prevent disk space issues:

```python
configure_logging(
    file="flight.log",
    file_max_bytes=10 * 1024 * 1024,  # 10MB
    file_backup_count=5,  # Keep 5 backup files
)
```

### 5. Component-Specific Log Levels

Set different log levels for different components:

```python
from aerpawlib.v2 import set_level, LogLevel, LogComponent

# Set global level
set_level(LogLevel.INFO)

# Set component-specific level (more verbose for debugging)
set_level(LogLevel.DEBUG, LogComponent.VEHICLE)

# Or via configuration
configure_logging(
    level=LogLevel.INFO,
    component_levels={
        "aerpawlib.vehicle": LogLevel.DEBUG,
        "aerpawlib.safety": LogLevel.WARNING,
    }
)
```

### 6. Logging Context

Add contextual information to all log messages:

```python
from aerpawlib.v2 import get_manager

manager = get_manager()

# Set global context
manager.set_context(mission_id="mission_001", pilot="john")

# Use context manager for temporary context
with manager.context(state="takeoff"):
    logger.info("Taking off")  # Includes state="takeoff"

# Clear context
manager.clear_context()
```

### 7. Logger Adapter with Extended Methods

The `LoggerAdapter` provides additional convenience methods:

```python
from aerpawlib.v2 import get_logger, LogComponent

logger = get_logger(LogComponent.USER, vehicle_id="drone1")

# Standard logging
logger.info("Mission started")

# Success message (INFO level with success flag)
logger.success("Takeoff complete")

# Mission event logging
logger.mission_event("waypoint_reached", {"waypoint": 1, "time": 5.2})

# Telemetry logging (DEBUG level)
logger.telemetry(
    position=(35.7749, -78.6419, 25.0),
    velocity=(5.0, 2.0, -0.5),
    attitude=(2.0, 1.0, 45.0),
)
```

## Flight Data Recording

The logging system includes a specialized flight data recorder for telemetry:

```python
from aerpawlib.v2 import get_flight_recorder, FlightDataRecorder

# Get the flight recorder
recorder = get_flight_recorder(
    sample_interval=0.1,  # Record every 100ms
    output_dir="./logs",
)

# Start recording
recorder.start(
    vehicle_type="drone",
    vehicle_id="drone1",
    mission_name="survey_mission",
)

# Record telemetry points
recorder.record(
    latitude=35.7749,
    longitude=-78.6419,
    altitude=25.0,
    heading=45.0,
    groundspeed=5.0,
    vertical_speed=0.0,
    battery_voltage=12.5,
    battery_percentage=85.0,
    gps_satellites=12,
    gps_fix_type=3,
    flight_mode="GUIDED",
    armed=True,
)

# Stop and get point count
points = recorder.stop()
print(f"Recorded {points} telemetry points")

# Save to file
recorder.save("flight_log.json", format="json")
recorder.save("flight_log.csv", format="csv")
```

### Auto-Save Feature

Enable automatic periodic saving:

```python
recorder = FlightDataRecorder(
    sample_interval=0.1,
    auto_save_interval=60.0,  # Save every 60 seconds
    output_dir="./logs",
)
```

## Decorators

### `@log_call` - Log Function Calls

```python
from aerpawlib.v2 import log_call
import logging

@log_call(level=logging.INFO, include_args=True, include_result=True)
async def takeoff(altitude: float):
    # Logs: "Calling takeoff(altitude=10.0)"
    await drone.takeoff(altitude)
    return True
    # Logs: "takeoff returned True"
```

### `@log_timing` - Log Execution Time

```python
from aerpawlib.v2 import log_timing

@log_timing(threshold_ms=100)  # Only log if > 100ms
async def complex_navigation():
    await calculate_path()
    # Logs: "complex_navigation took 250.32ms"
```

## Custom Handlers

### TelemetryHandler

Buffers telemetry data for batch processing:

```python
from aerpawlib.v2 import TelemetryHandler, get_manager

handler = TelemetryHandler(
    buffer_size=100,       # Flush after 100 records
    flush_interval=5.0,    # Or flush every 5 seconds
)

def process_batch(records):
    """Called when buffer is flushed."""
    for record in records:
        print(f"{record.getMessage()}")

handler.add_callback(process_batch)
get_manager().add_handler("telemetry", handler)
```

### AsyncFileHandler

Non-blocking file writes for async applications:

```python
from aerpawlib.v2 import AsyncFileHandler

handler = AsyncFileHandler("async_log.txt")

# Must start before using
await handler.start()

# Add to logger
get_manager().add_handler("async", handler)

# Clean shutdown
await handler.stop()
```

## Configuration Options

### LoggingConfig

Full configuration options:

```python
from aerpawlib.v2 import LoggingConfig, LogLevel, get_manager

config = LoggingConfig(
    # Global settings
    level=LogLevel.INFO,
    format="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
    date_format="%Y-%m-%d %H:%M:%S",
    
    # Console settings
    console_enabled=True,
    console_level=LogLevel.DEBUG,  # Different from global
    console_colored=True,
    
    # File settings
    file_enabled=True,
    file_path="flight.log",
    file_level=LogLevel.INFO,
    file_max_bytes=10 * 1024 * 1024,  # 10MB
    file_backup_count=5,
    
    # JSON file settings
    json_enabled=True,
    json_path="flight.json",
    json_level=LogLevel.DEBUG,
    json_pretty=False,
    
    # Component-specific levels
    component_levels={
        "aerpawlib.vehicle": LogLevel.DEBUG,
    },
)

get_manager().configure(config)
```

## Formatters

### ColoredFormatter

Console output with ANSI colors:

```python
from aerpawlib.v2 import ColoredFormatter

formatter = ColoredFormatter(
    fmt="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%H:%M:%S",
    use_colors=True,
)
```

### JSONFormatter

JSON output for log aggregation:

```python
from aerpawlib.v2 import JSONFormatter

formatter = JSONFormatter(
    include_extra=True,      # Include extra context
    include_traceback=True,  # Include exception traces
    pretty=False,            # Compact JSON
)
```

### TelemetryFormatter

Compact format for high-frequency telemetry:

```python
from aerpawlib.v2 import TelemetryFormatter

formatter = TelemetryFormatter(include_source=True)
# Output: "    1.234 | [vehicle] | Position: (35.77, -78.64, 25.0)"
```

## Integration with StateMachine

The logging system integrates naturally with the StateMachine runner:

```python
from aerpawlib.v2 import (
    StateMachine, state, background, at_init,
    configure_logging, get_logger, LogComponent, LogLevel,
    get_flight_recorder,
)

# Configure logging before running
configure_logging(
    level=LogLevel.INFO,
    file="mission.log",
    json_file="mission.json",
)

class SurveyMission(StateMachine):
    def __init__(self):
        super().__init__()
        self.logger = get_logger(LogComponent.USER, mission="survey")
        self.recorder = get_flight_recorder()
    
    @at_init
    async def setup(self, drone):
        self.logger.info("Setting up survey mission")
        self.recorder.start(vehicle_type="drone", mission_name="survey")
    
    @state("takeoff", first=True)
    async def takeoff(self, drone):
        self.logger.info("Taking off to 50m")
        await drone.takeoff(altitude=50)
        return "survey"
    
    @background
    async def log_telemetry(self, drone):
        """Record telemetry every 100ms."""
        pos = drone.position
        self.recorder.record(
            latitude=pos.latitude,
            longitude=pos.longitude,
            altitude=pos.altitude,
            heading=drone.heading,
            groundspeed=drone.groundspeed,
            battery_percentage=drone.battery.percentage,
            gps_satellites=drone.gps.satellites,
            gps_fix_type=drone.gps.fix_type,
            flight_mode=drone.flight_mode.name,
            armed=drone.armed,
        )
        await asyncio.sleep(0.1)
    
    def cleanup(self):
        count = self.recorder.stop()
        self.logger.info(f"Mission complete. Recorded {count} points.")
        self.recorder.save("survey_flight.json")
```

## Best Practices

1. **Use Component Loggers**: Use `LogComponent` enums for consistent categorization
2. **Set Appropriate Levels**: Use DEBUG for development, INFO for production
3. **Enable File Logging in Production**: Always log to file for post-flight analysis
4. **Use JSON for Analysis**: JSON logs are easier to parse programmatically
5. **Record Telemetry**: Use FlightDataRecorder for flight data analysis
6. **Add Context**: Use logging context for mission/vehicle identification
7. **Clean Up**: Call `recorder.stop()` to ensure data is saved

