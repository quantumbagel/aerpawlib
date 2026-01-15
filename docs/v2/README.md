# aerpawlib v2 API Documentation

The v2 API is the next-generation aerpawlib interface built on **MAVSDK**. It features a modern, Pythonic design with enhanced safety features, comprehensive exception handling, and improved observability.

> **Note**: v2 is currently in beta. Use the [v1 API](../v1/README.md) for new experiments.

## Requirements

- Python 3.9+
- MAVSDK-Python (`pip install mavsdk`)
- pyzmq for safety checker (`pip install pyzmq`)

## Quick Start

```python
from aerpawlib.v2 import Drone, Coordinate, BasicRunner, entrypoint

class MyMission(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        await drone.connect()
        
        # Take off to 10 meters
        await drone.takeoff(altitude=10)
        
        # Go to a coordinate
        target = Coordinate(35.7275, -78.6960, 10)
        await drone.goto(coordinates=target)
        
        # Land
        await drone.land()
```

## Connection

The v2 API uses MAVSDK connection strings:

```python
from aerpawlib.v2 import Drone

# Connection string examples
drone = Drone("udp://:14540")                 # SITL default
drone = Drone("serial:///dev/ttyUSB0:57600")  # Serial
drone = Drone("udp://192.168.1.100:14540")    # Remote UDP
drone = Drone("tcp://localhost:5760")         # TCP
```

> **Note**: Connection is handled automatically by the runner. You typically don't need to call `connect()` or `disconnect()` manually.

---

## API Reference

### Vehicle Properties

| Property             | Type         | Description                          |
|----------------------|--------------|--------------------------------------|
| `connected`          | `bool`       | True if receiving heartbeats         |
| `connection_healthy` | `bool`       | True if recent heartbeat received    |
| `position`           | `Coordinate` | Current position (lat, lon, alt)     |
| `altitude`           | `float`      | Current relative altitude in meters  |
| `heading`            | `float`      | Current heading in degrees           |
| `velocity`           | `VectorNED`  | Current velocity (north, east, down) |
| `is_in_air`          | `bool`       | True if vehicle is airborne          |
| `armed`              | `bool`       | True if vehicle is armed             |
| `is_armable`         | `bool`       | True if vehicle can be armed         |
| `home`               | `Coordinate` | Home/launch position                 |
| `throttle`           | `float`      | Current throttle value               |

### State Container

Access detailed telemetry via `drone.state`:

| Property            | Type          | Description                 |
|---------------------|---------------|-----------------------------|
| `heading`           | `float`       | Heading in degrees          |
| `velocity`          | `VectorNED`   | Velocity vector             |
| `attitude`          | `Attitude`    | Roll, pitch, yaw in radians |
| `altitude`          | `float`       | Absolute altitude           |
| `relative_altitude` | `float`       | Altitude above home         |
| `latitude`          | `float`       | Current latitude            |
| `longitude`         | `float`       | Current longitude           |
| `groundspeed`       | `float`       | Ground speed in m/s         |
| `airspeed`          | `float`       | Air speed in m/s            |
| `climb_rate`        | `float`       | Vertical speed in m/s       |
| `flight_mode`       | `FlightMode`  | Current flight mode         |
| `landed_state`      | `LandedState` | Current landed state        |

### GPS Container

Access GPS information via `drone.gps`:

| Property     | Type   | Description                      |
|--------------|--------|----------------------------------|
| `satellites` | `int`  | Number of visible satellites     |
| `fix_type`   | `int`  | GPS fix type (0-6)               |
| `quality`    | `str`  | Human-readable quality string    |
| `has_fix`    | `bool` | True if GPS has 2D or better fix |

### Battery Container

Access battery information via `drone.battery`:

| Property      | Type              | Description                    |
|---------------|-------------------|--------------------------------|
| `voltage`     | `float`           | Battery voltage in volts       |
| `current`     | `float`           | Current draw in amps           |
| `charge`      | `float`           | Charge level (0.0 - 1.0)       |
| `percentage`  | `float`           | Charge as percentage (0 - 100) |
| `is_low`      | `bool`            | True if charge < 20%           |
| `is_critical` | `bool`            | True if charge < 10%           |
| `temperature` | `Optional[float]` | Battery temperature in °C      |

### Info Container

Access vehicle information via `drone.info`:

| Property        | Type              | Description                  |
|-----------------|-------------------|------------------------------|
| `hardware_uuid` | `str`             | Hardware unique identifier   |
| `vendor_name`   | `str`             | Vehicle vendor name          |
| `product_name`  | `str`             | Vehicle product name         |
| `version`       | `str`             | Firmware version string      |
| `arm_time`      | `Optional[float]` | Unix timestamp when armed    |
| `takeoff_time`  | `Optional[float]` | Unix timestamp when took off |

---

### Drone Methods

#### `await connect(timeout: float = 30.0, retry_count: int = 3, retry_delay: float = 2.0) -> bool`
Connect to the vehicle.

| Parameter     | Type    | Description                             |
|---------------|---------|-----------------------------------------|
| `timeout`     | `float` | Connection timeout in seconds           |
| `retry_count` | `int`   | Number of connection attempts           |
| `retry_delay` | `float` | Delay between retries in seconds        |

```python
await drone.connect(timeout=30, retry_count=3)
```

#### `await disconnect()`
Disconnect from the vehicle and clean up resources.

```python
await drone.disconnect()
```

#### `await arm(force: bool = False, skip_preflight: bool = False) -> bool`
Arm the vehicle.

| Parameter        | Type   | Description                              |
|------------------|--------|------------------------------------------|
| `force`          | `bool` | Skip armability check                    |
| `skip_preflight` | `bool` | Skip preflight safety checks             |

```python
await drone.arm()
```

#### `await disarm(force: bool = False) -> bool`
Disarm the vehicle.

| Parameter | Type   | Description                    |
|-----------|--------|--------------------------------|
| `force`   | `bool` | Force disarm (emergency kill)  |

```python
await drone.disarm()
```

#### `await takeoff(altitude: float = 5.0, wait: bool = True, timeout: float = 60.0) -> Optional[CommandHandle]`
Take off to the specified altitude.

| Parameter  | Type    | Description                                |
|------------|---------|--------------------------------------------|
| `altitude` | `float` | Target altitude in meters                  |
| `wait`     | `bool`  | Wait for completion (True) or return handle |
| `timeout`  | `float` | Maximum time to wait                       |

```python
# Blocking takeoff
await drone.takeoff(altitude=10)

# Non-blocking with handle
handle = await drone.takeoff(altitude=10, wait=False)
print(f"Progress: {handle.progress}")
await handle.wait()
```

#### `await land(wait: bool = True, timeout: float = 120.0) -> Optional[CommandHandle]`
Land at the current position.

```python
await drone.land()
```

#### `await rtl(wait: bool = True, timeout: float = 300.0) -> Optional[CommandHandle]`
Return to launch position.

```python
await drone.rtl()
```

#### `await goto(...) -> Optional[CommandHandle]`
Navigate to specific coordinates.

| Parameter     | Type                   | Description                              |
|---------------|------------------------|------------------------------------------|
| `latitude`    | `Optional[float]`      | Target latitude                          |
| `longitude`   | `Optional[float]`      | Target longitude                         |
| `altitude`    | `Optional[float]`      | Target altitude                          |
| `coordinates` | `Optional[Coordinate]` | Target as Coordinate object              |
| `tolerance`   | `float`                | Acceptance radius in meters (default: 2) |
| `speed`       | `Optional[float]`      | Travel speed in m/s                      |
| `heading`     | `Optional[float]`      | Heading to maintain during travel        |
| `timeout`     | `float`                | Maximum time to wait (default: 300)      |
| `wait`        | `bool`                 | Wait for completion                      |

```python
# Using lat/lon
await drone.goto(latitude=35.7275, longitude=-78.6960, altitude=15)

# Using Coordinate
target = Coordinate(35.7275, -78.6960, 15)
await drone.goto(coordinates=target, tolerance=3, speed=8)

# Non-blocking
handle = await drone.goto(coordinates=target, wait=False)
while not handle.is_complete:
    print(f"Distance: {handle.progress['distance']:.1f}m")
    await asyncio.sleep(1)
```

#### `await set_heading(degrees: float, blocking: bool = True, timeout: float = 30.0) -> Optional[CommandHandle]`
Set the vehicle heading.

| Parameter  | Type    | Description                           |
|------------|---------|---------------------------------------|
| `degrees`  | `float` | Target heading in degrees (0 = North) |
| `blocking` | `bool`  | Wait for heading to be reached        |
| `timeout`  | `float` | Maximum time to wait                  |

```python
await drone.set_heading(90)  # Face east
```

#### `await set_velocity(velocity: VectorNED, heading: Optional[float] = None, duration: Optional[float] = None, wait: bool = True) -> Optional[CommandHandle]`
Set the vehicle velocity.

| Parameter  | Type               | Description                     |
|------------|--------------------|---------------------------------|
| `velocity` | `VectorNED`        | Velocity in m/s                 |
| `heading`  | `Optional[float]`  | Heading to maintain             |
| `duration` | `Optional[float]`  | Duration in seconds             |
| `wait`     | `bool`             | Wait for duration to complete   |

```python
from aerpawlib.v2 import VectorNED

# Move north at 5 m/s for 10 seconds
await drone.set_velocity(VectorNED(5, 0, 0), duration=10)

# Continuous velocity (no duration)
await drone.set_velocity(VectorNED(0, 3, 0))
```

#### `await set_groundspeed(speed: float)`
Set the cruise speed for goto operations.

```python
await drone.set_groundspeed(8)  # 8 m/s
```

#### `await set_altitude(altitude: float, tolerance: float = 0.5)`
Change to a specific altitude.

```python
await drone.set_altitude(20)  # Climb to 20m
```

#### `await hold()`
Hold current position.

```python
await drone.hold()
```

#### `await abort(rtl: bool = True)`
Abort current operation.

| Parameter | Type   | Description                              |
|-----------|--------|------------------------------------------|
| `rtl`     | `bool` | Return to launch (True) or hold (False)  |

```python
await drone.abort(rtl=True)
```

#### `await orbit(center: Coordinate, radius: float, speed: float = 5.0, clockwise: bool = True, revolutions: float = 1.0, wait: bool = True) -> Optional[CommandHandle]`
Fly in a circular orbit around a point.

| Parameter     | Type         | Description                        |
|---------------|--------------|------------------------------------|
| `center`      | `Coordinate` | Center point of the orbit          |
| `radius`      | `float`      | Orbit radius in meters             |
| `speed`       | `float`      | Orbit speed in m/s                 |
| `clockwise`   | `bool`       | Direction of orbit                 |
| `revolutions` | `float`      | Number of revolutions              |
| `wait`        | `bool`       | Wait for completion                |

```python
await drone.orbit(
    center=Coordinate(35.7275, -78.6960, 10),
    radius=20,
    speed=5,
    revolutions=2
)
```

---

## Command Handles

The v2 API introduces `CommandHandle` for non-blocking operations with progress tracking:

```python
from aerpawlib.v2 import Drone, Coordinate, BasicRunner, entrypoint
import asyncio

class NonBlockingMission(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        await drone.takeoff(altitude=10)
        
        # Start non-blocking goto
        handle = await drone.goto(
            coordinates=Coordinate(35.7275, -78.6960, 10),
            wait=False
        )

        # Monitor progress
        while not handle.is_complete:
            progress = handle.progress
            print(f"Distance: {progress['distance']:.1f}m")
            print(f"Elapsed: {handle.elapsed_time:.1f}s")
            await asyncio.sleep(1)

        # Check result
        if handle.succeeded:
            print("Navigation complete!")
        elif handle.was_cancelled:
            print("Navigation was cancelled")
        elif handle.timed_out:
            print("Navigation timed out")

        # Or wait for completion
        await handle.wait()

        # Cancel if needed
        await handle.cancel()
        
        await drone.land()
```

### CommandHandle Properties

| Property         | Type                  | Description                              |
|------------------|-----------------------|------------------------------------------|
| `command`        | `str`                 | Name of the command                      |
| `status`         | `CommandStatus`       | Current status                           |
| `is_pending`     | `bool`                | True if not yet started                  |
| `is_running`     | `bool`                | True if currently executing              |
| `is_complete`    | `bool`                | True if finished (any result)            |
| `succeeded`      | `bool`                | True if completed successfully           |
| `was_cancelled`  | `bool`                | True if cancelled                        |
| `timed_out`      | `bool`                | True if timed out                        |
| `error`          | `Optional[Exception]` | Exception if failed                      |
| `elapsed_time`   | `float`               | Time since command started               |
| `time_remaining` | `Optional[float]`     | Time until timeout (if applicable)       |
| `progress`       | `Dict[str, Any]`      | Command-specific progress information    |

---

## Safety Features

The v2 API includes comprehensive safety features:

### Safety Limits

Configure safety parameters at initialization:

```python
from aerpawlib.v2 import Drone, SafetyLimits

# Create custom safety limits
limits = SafetyLimits(
    max_altitude=120.0,         # Maximum altitude in meters
    min_altitude=2.0,           # Minimum altitude in meters
    max_speed=15.0,             # Maximum speed in m/s
    min_speed=0.0,              # Minimum speed
    max_distance_from_home=500, # Maximum range in meters
    enable_preflight_checks=True,
    enable_parameter_validation=True,
    auto_clamp_values=True,     # Auto-clamp out-of-range values
)

drone = Drone(safety_limits=limits)

# Or use AERPAW defaults
drone = Drone(safety_limits=SafetyLimits.aerpaw_default())
```

### Preflight Checks

Run preflight checks before arming:

```python
from aerpawlib.v2 import Drone, BasicRunner, entrypoint

class PreflightMission(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        # Manual preflight check
        result = await drone.preflight_check()
        if result:
            print("All checks passed!")
        else:
            print(f"Failed checks: {result.failed_checks}")

        # Preflight is automatic on arm() unless skipped
        await drone.arm()  # Runs preflight checks
        await drone.arm(skip_preflight=True)  # Skip checks
```

### SafetyCheckerServer

Run the safety checker server in a separate process:

```python
from aerpawlib.v2 import SafetyCheckerServer

# Create server with config file
server = SafetyCheckerServer("geofence_config.yaml", server_port=14580)

# Start serving (blocks)
server.start_server()
```

### SafetyCheckerClient

Connect to the safety checker from your script:

```python
from aerpawlib.v2 import Drone, SafetyCheckerClient

# Create client
checker = SafetyCheckerClient("127.0.0.1", 14580)

# Use with drone
drone = Drone(safety_checker=checker)

# Manual validation
result = checker.validateWaypoint(current_pos, target_pos)
if not result:
    print("Waypoint outside geofence!")

result = checker.validateSpeed(15.0)
if not result:
    print("Speed exceeds limit!")

result = checker.validateTakeoff(altitude=10, lat=35.7, lon=-78.6)
```

### Configuration File Format

```yaml
# geofence_config.yaml
vehicle_type: copter      # or "rover"
max_speed: 15             # m/s
min_speed: 0.5            # m/s
max_alt: 120              # meters (copter only)
min_alt: 5                # meters (copter only)
include_geofences:        # allowed flight areas
  - flight_area.kml
exclude_geofences:        # no-go zones
  - no_fly_zone.kml
```

---


## Flight Recording

Record telemetry for post-flight analysis:

```python
from aerpawlib.v2 import Drone, Coordinate, BasicRunner, entrypoint

class RecordedMission(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        # Start recording
        drone.start_recording(interval=0.1)  # Record every 100ms

        # Fly mission...
        await drone.takeoff(altitude=10)
        target = Coordinate(35.7275, -78.6960, 10)
        await drone.goto(coordinates=target)
        await drone.land()

        # Stop recording
        count = drone.stop_recording()
        print(f"Recorded {count} data points")

        # Get log data
        log = drone.get_flight_log()

        # Save to file
        drone.save_flight_log("flight.json", format="json")
        drone.save_flight_log("flight.csv", format="csv")

        # Clear log
        drone.clear_flight_log()
```

---

## Exception Handling

The v2 API uses a structured exception hierarchy:

```python
from aerpawlib.v2 import (
    Drone,
    # Base exception
    AerpawlibError,
    # Connection errors
    ConnectionError,
    ConnectionTimeoutError,
    HeartbeatLostError,
    # Command errors
    CommandError,
    ArmError,
    TakeoffError,
    LandingError,
    NavigationError,
    # Timeout errors
    TimeoutError,
    GotoTimeoutError,
    TakeoffTimeoutError,
    # Safety errors
    SafetyError,
    GeofenceViolationError,
    SpeedLimitExceededError,
    PreflightCheckError,
    # Abort errors
    AbortError,
    CommandCancelledError,
)

try:
    await drone.goto(coordinates=target)
except GotoTimeoutError as e:
    print(f"Navigation timed out after {e.details['timeout']}s")
    print(f"Distance remaining: {e.details['distance_remaining']}m")
except GeofenceViolationError as e:
    print(f"Target outside geofence: {e.message}")
except NavigationError as e:
    print(f"Navigation failed: {e.message}")
except AerpawlibError as e:
    print(f"Error [{e.code}]: {e.message}")
    print(f"Severity: {e.severity}")
    print(f"Recoverable: {e.recoverable}")
```

### Exception Properties

All exceptions inherit from `AerpawlibError`:

| Property      | Type            | Description                      |
|---------------|-----------------|----------------------------------|
| `message`     | `str`           | Human-readable description       |
| `code`        | `ErrorCode`     | Error code for programmatic use  |
| `severity`    | `ErrorSeverity` | WARNING, ERROR, CRITICAL, FATAL  |
| `recoverable` | `bool`          | Whether operation can be retried |
| `details`     | `Dict`          | Additional context               |

---

## Utility Types

### Coordinate

Represents a geographic position with enhanced operations:

```python
from aerpawlib.v2 import Coordinate, VectorNED

# Create a coordinate
pos = Coordinate(lat=35.7275, lon=-78.6960, alt=10)

# Create with a label
pos = Coordinate(lat=35.7275, lon=-78.6960, alt=10, label="Waypoint 1")

# Distance calculations
distance = pos.distance_to(other_pos)           # 3D distance in meters
ground_dist = pos.ground_distance_to(other_pos) # 2D distance

# Bearing
bearing = pos.bearing_to(other_pos)  # Degrees

# Vector between coordinates
vec = pos.vector_to(other_pos)  # Returns VectorNED

# Add offset
new_pos = pos + VectorNED(100, 50, 0)  # 100m north, 50m east
```

### VectorNED

Represents a 3D vector in North-East-Down frame:

```python
from aerpawlib.v2 import VectorNED

vec = VectorNED(north=5.0, east=3.0, down=0.0)

# Operations
magnitude = vec.magnitude()                      # 3D magnitude
ground_mag = vec.magnitude(ignore_vertical=True) # 2D magnitude
normalized = vec.normalize()                     # Unit vector
heading = vec.heading()                          # Heading in degrees
rotated = vec.rotate_by_angle(45)                # Rotate by degrees

# Vector math
cross = vec.cross_product(other_vec)
dot = vec.dot_product(other_vec)

# Arithmetic
sum_vec = vec1 + vec2
diff_vec = vec1 - vec2
scaled = vec * 2.0
```

### Waypoint

Represents a waypoint with additional parameters:

```python
from aerpawlib.v2 import Waypoint, read_waypoints_from_plan

# Create waypoint
wp = Waypoint(
    coordinate=Coordinate(35.7275, -78.6960, 10),
    speed=5.0,
    hold_time=2.0,
    acceptance_radius=3.0,
)

# Load from QGroundControl plan file
waypoints = read_waypoints_from_plan("mission.plan")
```

### Flight Enums

```python
from aerpawlib.v2 import FlightMode, LandedState

# Flight modes
FlightMode.MANUAL
FlightMode.STABILIZED
FlightMode.ALTITUDE
FlightMode.POSITION
FlightMode.OFFBOARD
FlightMode.HOLD
FlightMode.MISSION
FlightMode.RETURN_TO_LAUNCH
FlightMode.LAND
FlightMode.TAKEOFF

# Landed states
LandedState.UNKNOWN
LandedState.ON_GROUND
LandedState.IN_AIR
LandedState.TAKING_OFF
LandedState.LANDING
```

---

## Decorators

### `@entrypoint`
Mark a method as the entry point for `BasicRunner`.

```python
class MyMission(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        await drone.takeoff(altitude=10)
```

### `@state(name: str, first: bool = False)`
Define a state for `StateMachine`. Return the name of the next state.

```python
class MyMission(StateMachine):
    @state("takeoff", first=True)
    async def takeoff(self, drone: Drone):
        await drone.takeoff(altitude=10)
        return "navigate"
    
    @state("navigate")
    async def navigate(self, drone: Drone):
        await drone.goto(coordinates=target)
        return "land"
    
    @state("land")
    async def land_state(self, drone: Drone):
        await drone.land()
        return None  # End state machine
```

### `@timed_state(name: str, duration: float, loop: bool = False, first: bool = False)`
Define a state that runs for a specific duration.

```python
@timed_state("hover", duration=10.0, loop=True)
async def hover(self, drone: Drone):
    print(f"Altitude: {drone.altitude}")
    return "land"  # Transition after 10 seconds
```

### `@background`
Run a function in parallel with the state machine.

```python
@background
async def log_telemetry(self, drone: Drone):
    while True:
        print(f"Position: {drone.position}")
        await asyncio.sleep(1)
```

### `@at_init`
Run initialization code before the mission starts.

```python
@at_init
async def setup(self, drone: Drone):
    print("Initializing mission...")
```

---

## Runners

### BasicRunner

Simple entry-point based execution:

```python
from aerpawlib.v2 import BasicRunner, Drone, entrypoint

class MyMission(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        await drone.connect()
        await drone.takeoff(altitude=10)
        await drone.land()
```

### StateMachine

State-based execution with transitions:

```python
from aerpawlib.v2 import StateMachine, Drone, Coordinate, state, background
import asyncio

class PatrolMission(StateMachine):
    def __init__(self):
        self.patrol_count = 0
        self.patrol_points = [
            Coordinate(35.7275, -78.6960, 15),
            Coordinate(35.7280, -78.6955, 15),
        ]
    
    @state("start", first=True)
    async def start(self, drone: Drone):
        await drone.connect()
        await drone.takeoff(altitude=10)
        return "patrol"
    
    @state("patrol")
    async def patrol(self, drone: Drone):
        point = self.patrol_points[self.patrol_count % len(self.patrol_points)]
        await drone.goto(coordinates=point)
        self.patrol_count += 1
        
        if self.patrol_count >= 3:
            return "land"
        return "patrol"
    
    @state("land")
    async def land_state(self, drone: Drone):
        await drone.land()
        return None
    
    @background
    async def battery_monitor(self, drone: Drone):
        while True:
            if drone.battery.is_low:
                print("Low battery warning!")
            await asyncio.sleep(5)
```

---

## Logging

The v2 API includes structured logging:

```python
from aerpawlib.v2 import (
    configure_logging,
    get_logger,
    set_level,
    LogLevel,
    LogComponent,
)

# Configure logging
configure_logging(level=LogLevel.DEBUG, color=True)

# Get component-specific logger
logger = get_logger(LogComponent.USER)
logger.info("Mission started")
logger.debug("Target coordinates: %s", target)

# Set level for specific component
set_level(LogComponent.VEHICLE, LogLevel.WARNING)
```

### Log Components

| Component  | Description                     |
|------------|---------------------------------|
| `USER`     | User script logging             |
| `VEHICLE`  | Vehicle communication           |
| `RUNNER`   | Runner execution                |
| `SAFETY`   | Safety system                   |
| `AERPAW`   | AERPAW platform integration     |

---

## ZMQ Communication

The v2 API includes ZMQ utilities for inter-process communication:

### ZMQ Proxy

```python
from aerpawlib.v2 import run_zmq_proxy

# Start proxy (blocks)
run_zmq_proxy(in_port=5555, out_port=5556)
```

### ZMQ Publisher/Subscriber

```python
from aerpawlib.v2 import ZMQPublisher, ZMQSubscriber, MessageType

# Publisher
pub = ZMQPublisher(port=5555)
pub.publish(MessageType.STATUS, {"state": "flying"})
pub.publish(MessageType.TELEMETRY, {"altitude": 15.0})

# Subscriber
sub = ZMQSubscriber(address="tcp://127.0.0.1:5556")
msg = sub.receive()
if msg:
    print(f"Type: {msg.type}, Data: {msg.data}")
```

---

## Testing

The v2 API includes mock classes for testing:

```python
from aerpawlib.v2 import MockDrone, MockState, Coordinate

# Create mock drone
drone = MockDrone()

# Configure mock state
drone.mock_state.latitude = 35.7275
drone.mock_state.longitude = -78.6960
drone.mock_state.relative_altitude = 10.0
drone._armed = True

# Use in tests
async def test_goto():
    drone = MockDrone()
    await drone.connect()
    await drone.takeoff(altitude=10)
    await drone.goto(coordinates=Coordinate(35.7275, -78.6960, 10))
    await drone.land()
```

---

## Examples

### Basic Flight

```python
from aerpawlib.v2 import Drone, Coordinate, BasicRunner, entrypoint

class SimpleFlight(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        await drone.connect()
        await drone.takeoff(altitude=10)
        
        waypoints = [
            Coordinate(35.7275, -78.6960, 10),
            Coordinate(35.7280, -78.6955, 15),
        ]
        
        for wp in waypoints:
            await drone.goto(coordinates=wp)
        
        await drone.land()
```

### With Safety Checker

```python
from aerpawlib.v2 import Drone, Coordinate, BasicRunner, entrypoint

class SafeFlight(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        # Preflight checks run automatically on arm
        await drone.arm()
        await drone.takeoff(altitude=10)
        
        target = Coordinate(35.7275, -78.6960, 10)
        
        # Safety checker validates waypoint automatically
        await drone.goto(coordinates=target)
        
        await drone.land()
```

### Non-Blocking Operations

```python
from aerpawlib.v2 import Drone, Coordinate, BasicRunner, entrypoint
import asyncio

class NonBlockingMission(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        await drone.takeoff(altitude=10)
        
        # Start goto without waiting
        handle = await drone.goto(
            coordinates=Coordinate(35.7275, -78.6960, 10),
            wait=False
        )
        
        # Do other things while moving
        while handle.is_running:
            print(f"Distance: {handle.progress['distance']:.1f}m")
            print(f"Time: {handle.elapsed_time:.1f}s")
            
            # Check for timeout
            if handle.elapsed_time > 60:
                await handle.cancel()
                break
            
            await asyncio.sleep(1)
        
        if handle.succeeded:
            print("Arrived!")
        
        await drone.land()
```


---

## Migration from v1/Legacy

### Import Changes

```python
# Before (v1/legacy)
from aerpawlib.v1 import Drone, Coordinate, BasicRunner, entrypoint

# After (v2)
from aerpawlib.v2 import Drone, Coordinate, BasicRunner, entrypoint
```

### API Changes

| v1/Legacy                       | v2                                                   | Notes           |
|---------------------------------|------------------------------------------------------|-----------------|
| `drone.takeoff(target_alt)`     | `drone.takeoff(altitude=target_alt)`                 | Named parameter |
| `drone.goto_coordinates(coord)` | `drone.goto(coordinates=coord)`                      | Method rename   |
| `drone.position.alt`            | `drone.altitude` or `drone.state.relative_altitude`  | Property location |
| `drone.battery.level`           | `drone.battery.charge` or `drone.battery.percentage` | Property names  |
| -                               | `handle = await drone.goto(..., wait=False)`         | Command handles |

### Key Differences

1. **Named parameters**: Many methods use keyword arguments
2. **Command handles**: Non-blocking operations return handles for progress tracking
3. **Safety integration**: Built-in safety limits and checker integration
4. **Structured exceptions**: Rich exception hierarchy with error codes
