# aerpawlib v1 API Documentation

The v1 API provides backward compatibility with the legacy aerpawlib API while using **MAVSDK** as the backend instead of DroneKit. This allows existing scripts to work with modern autopilot firmware without code changes. It will also allow AERPAW to update the firmware on the vehicle without having to monkey patch `aerpawlib`.

> **Note**: v1 uses the same API as the legacy version. It has no additional features and is intended to become a stopgap until aerpawlib v2 is ready.
## Requirements

- Python 3.8+
- MAVSDK-Python (`pip install mavsdk`)
- pyzmq for safety checker (`pip install pyzmq`)

## Quick Start

```python
from aerpawlib.v1 import Drone, Coordinate, BasicRunner, entrypoint

class MyMission(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        # Take off to 10 meters
        await drone.takeoff(10)
        
        # Go to a coordinate
        target = Coordinate(35.7275, -78.6960, 10)
        await drone.goto_coordinates(target)
        
        # Land
        await drone.land()
```

## Connection

The v1 API uses MAVSDK connection strings:

```python
from aerpawlib.v1 import Drone

# Connect to SITL (default)
drone = Drone("udp://:14540")

# Connect to serial
drone = Drone("serial:///dev/ttyUSB0:57600")

# Connect to specific address
drone = Drone("udp://192.168.1.100:14540")

# Connect to TCP
drone = Drone("tcp://localhost:5760")
```

---

## API Reference

The v1 API is fully compatible with the [legacy API](../legacy/README.md). All classes, methods, and properties work identically.


### Vehicle Properties

| Property         | Type         | Description                               |
|------------------|--------------|-------------------------------------------|
| `connected`      | `bool`       | True if receiving heartbeats              |
| `position`       | `Coordinate` | Current position (lat, lon, alt)          |
| `battery`        | `object`     | Battery status (voltage, current, level)  |
| `gps`            | `object`     | GPS status (fix_type, satellites_visible) |
| `armed`          | `bool`       | True if vehicle is armed                  |
| `home_coords`    | `Coordinate` | Home/launch position                      |
| `heading`        | `float`      | Current heading in degrees                |
| `velocity`       | `VectorNED`  | Current velocity (north, east, down)      |
| `attitude`       | `object`     | Roll, pitch, yaw in radians               |
| `autopilot_info` | `object`     | Autopilot version info                    |

---

### Drone Methods

#### `await takeoff(target_alt: float, min_alt_tolerance: float = 0.95, wait_for_throttle: bool = False)`
Take off to the specified altitude.

| Parameter           | Type    | Description                                                 |
|---------------------|---------|-------------------------------------------------------------|
| `target_alt`        | `float` | Target altitude in meters                                   |
| `min_alt_tolerance` | `float` | Fraction of target alt considered "reached" (default: 0.95) |
| `wait_for_throttle` | `bool`  | Wait for RC throttle to center before takeoff               |

```python
await drone.takeoff(10)  # Take off to 10 meters
```

#### `await land()`
Land at the current position and wait for disarm.

```python
await drone.land()
```

#### `await goto_coordinates(coordinates: Coordinate, tolerance: float = 2, target_heading: float = None)`
Navigate to specific coordinates.

| Parameter        | Type         | Description                              |
|------------------|--------------|------------------------------------------|
| `coordinates`    | `Coordinate` | Target position                          |
| `tolerance`      | `float`      | Acceptance radius in meters (default: 2) |
| `target_heading` | `float`      | Optional heading to maintain (degrees)   |

```python
target = Coordinate(35.7275, -78.6960, 15)
await drone.goto_coordinates(target, tolerance=3)
```

#### `await set_heading(heading: float, blocking: bool = True, lock_in: bool = True)`
Set the vehicle heading.

| Parameter  | Type    | Description                                |
|------------|---------|--------------------------------------------|
| `heading`  | `float` | Target heading in degrees (0 = North)      |
| `blocking` | `bool`  | Wait for current command to complete first |
| `lock_in`  | `bool`  | Use this heading for subsequent commands   |

```python
await drone.set_heading(90)  # Face east
```

#### `await set_velocity(velocity_vector: VectorNED, global_relative: bool = True, duration: float = None)`
Set the vehicle velocity.

| Parameter         | Type        | Description                         |
|-------------------|-------------|-------------------------------------|
| `velocity_vector` | `VectorNED` | Velocity in m/s                     |
| `global_relative` | `bool`      | If True, velocity is world-relative |
| `duration`        | `float`     | Optional duration in seconds        |

```python
# Move north at 5 m/s for 10 seconds
await drone.set_velocity(VectorNED(5, 0, 0), duration=10)
```

#### `await set_groundspeed(velocity: float)`
Set the cruise speed for goto operations.

```python
await drone.set_groundspeed(8)  # 8 m/s
```

---

## Safety Checker

The v1 API includes the same safety checker as the legacy API for geofence validation. For detailed documentation, see the [Safety Checker Guide](safety_checker.md).

### SafetyCheckerServer

Run the safety checker server in a separate process:

```python
from aerpawlib.v1 import SafetyCheckerServer

# Create server with config file
server = SafetyCheckerServer("geofence_config.yaml", server_port=14580)

# Start serving (blocks)
server.start_server()
```

Or from command line:
```bash
python -c "from aerpawlib.v1.safetyChecker import SafetyCheckerServer; SafetyCheckerServer('config.yaml', 14580).start_server()"
```

### SafetyCheckerClient

Connect to the safety checker from your script:

```python
from aerpawlib.v1 import SafetyCheckerClient

client = SafetyCheckerClient("127.0.0.1", 14580)

# Check server status
status = client.check_server_status()
print(f"Server running: {status}")

# Validate a waypoint
result = client.validate_waypoint_command(current_pos, target_pos)
if not result:
    print("Waypoint outside geofence!")

# Validate speed
result = client.validate_change_speed_command(15.0)
if not result:
    print("Speed exceeds limit!")

# Validate takeoff
result = client.validate_takeoff_command(altitude=10, lat=35.7, lon=-78.6)
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

### KML Geofence Format

Create KML files with polygon boundaries:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <Placemark>
      <name>Flight Area</name>
      <Polygon>
        <outerBoundaryIs>
          <LinearRing>
            <coordinates>
              -78.7,35.7,0
              -78.6,35.7,0
              -78.6,35.8,0
              -78.7,35.8,0
              -78.7,35.7,0
            </coordinates>
          </LinearRing>
        </outerBoundaryIs>
      </Polygon>
    </Placemark>
  </Document>
</kml>
```

> **Note**: For more advanced safety features (pre-flight checks, battery failsafe, parameter validation, continuous monitoring), upgrade to the [v2 API](../v2/README.md).

---

## Utility Types

### Coordinate

Represents a geographic position.

```python
from aerpawlib.v1 import Coordinate

# Create a coordinate
pos = Coordinate(lat=35.7275, lon=-78.6960, alt=10)

# Create with a label
pos = Coordinate(lat=35.7275, lon=-78.6960, alt=10, label="Waypoint 1")

# Operations
distance = pos.distance(other_pos)      # 3D distance in meters
ground_dist = pos.ground_distance(other_pos)  # 2D distance
bearing = pos.bearing(other_pos)        # Degrees
new_pos = pos + VectorNED(100, 0, 0)    # Add offset
```

### VectorNED

Represents a 3D vector in North-East-Down frame.

```python
from aerpawlib.v1 import VectorNED

vec = VectorNED(north=5.0, east=3.0, down=0.0)

# Operations
magnitude = vec.hypot()                 # 3D magnitude
ground_mag = vec.hypot(ignore_down=True)  # 2D magnitude
normalized = vec.norm()                 # Unit vector
rotated = vec.rotate_by_angle(45)       # Rotate by degrees

# Arithmetic
sum_vec = vec1 + vec2
scaled = vec * 2.0
```

---

## Decorators

### `@entrypoint`
Mark a method as the entry point for `BasicRunner`.

```python
class MyMission(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        await drone.takeoff(10)
```

### `@state(name: str, first: bool = False)`
Define a state for `StateMachine`. Return the name of the next state.

```python
class MyMission(StateMachine):
    @state("takeoff", first=True)
    async def takeoff(self, drone: Drone):
        await drone.takeoff(10)
        return "navigate"
    
    @state("navigate")
    async def navigate(self, drone: Drone):
        await drone.goto_coordinates(target)
        return "land"
    
    @state("land")
    async def land_state(self, drone: Drone):
        await drone.land()
        return None
```

### `@timed_state(name: str, duration: float, loop: bool = False, first: bool = False)`
Define a timed state.

```python
@timed_state("hover", duration=10.0, loop=True)
async def hover(self, drone: Drone):
    print(f"Altitude: {drone.position.alt}")
    return "land"
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

---

## Runners

### BasicRunner

Simple entry-point based execution:

```python
from aerpawlib.v1 import BasicRunner, entrypoint

class MyMission(BasicRunner):
    @entrypoint
    async def run(self, drone):
        await drone.takeoff(10)
        await drone.land()
```

### StateMachine

State-based execution with transitions:

```python
from aerpawlib.v1 import StateMachine, state, background
import asyncio

class PatrolMission(StateMachine):
    def __init__(self):
        self.patrol_count = 0
        self.patrol_points = [
            Coordinate(35.7275, -78.6960, 15),
            Coordinate(35.7280, -78.6955, 15),
        ]
    
    @state("start", first=True)
    async def start(self, drone):
        await drone.takeoff(10)
        return "patrol"
    
    @state("patrol")
    async def patrol(self, drone):
        point = self.patrol_points[self.patrol_count % len(self.patrol_points)]
        await drone.goto_coordinates(point)
        self.patrol_count += 1
        
        if self.patrol_count >= 3:
            return "land"
        return "patrol"
    
    @state("land")
    async def land_state(self, drone):
        await drone.land()
        return None
    
    @background
    async def battery_monitor(self, drone):
        while True:
            if drone.battery.level < 20:
                print("Low battery warning!")
            await asyncio.sleep(5)
```

---

## ZMQ Communication

The v1 API includes ZMQ utilities for inter-process communication.

### ZMQ Proxy

```python
from aerpawlib.v1 import run_zmq_proxy

# Start proxy (blocks)
run_zmq_proxy(in_port=5555, out_port=5556)
```

### ZMQ Publisher/Subscriber

```python
from aerpawlib.v1 import ZMQPublisher, ZMQSubscriber

# Publisher
pub = ZMQPublisher(port=5555)
pub.publish("status", {"state": "flying"})

# Subscriber
sub = ZMQSubscriber(address="tcp://127.0.0.1:5556")
msg = sub.receive()
```

---

## Examples

### Basic Flight

```python
from aerpawlib.v1 import Drone, Coordinate, BasicRunner, entrypoint

class SimpleFlight(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        await drone.takeoff(10)
        
        waypoints = [
            Coordinate(35.7275, -78.6960, 10),
            Coordinate(35.7280, -78.6955, 15),
        ]
        
        for wp in waypoints:
            await drone.goto_coordinates(wp)
        
        await drone.land()
```

### With Safety Checker

```python
from aerpawlib.v1 import Drone, Coordinate, BasicRunner, entrypoint, SafetyCheckerClient

class SafeFlight(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        # Connect to safety checker
        checker = SafetyCheckerClient("127.0.0.1", 14580)
        
        if not checker.checkServerStatus():
            print("Safety checker not running!")
            return
        
        await drone.takeoff(10)
        
        target = Coordinate(35.7275, -78.6960, 10)
        
        # Validate before flying
        current = drone.position
        if checker.validateWaypoint(current, target):
            await drone.goto_coordinates(target)
        else:
            print("Target outside geofence, aborting!")
            await drone.land()
            return
        
        await drone.land()
```

### Velocity Control

```python
from aerpawlib.v1 import Drone, VectorNED, BasicRunner, entrypoint
import asyncio

class VelocityFlight(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        await drone.takeoff(10)
        
        # Move north for 5 seconds
        await drone.set_velocity(VectorNED(5, 0, 0), duration=5)
        
        # Move east for 5 seconds
        await drone.set_velocity(VectorNED(0, 5, 0), duration=5)
        
        # Hover for 3 seconds
        await asyncio.sleep(3)
        
        await drone.land()
```

### Heading Control

```python
from aerpawlib.v1 import Drone, BasicRunner, entrypoint
import asyncio

class HeadingDemo(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        await drone.takeoff(10)
        
        # Point north
        await drone.set_heading(0)
        await asyncio.sleep(2)
        
        # Point east
        await drone.set_heading(90)
        await asyncio.sleep(2)
        
        # Point south
        await drone.set_heading(180)
        await asyncio.sleep(2)
        
        # Turn 90 degrees from current heading
        await drone.set_heading(drone.heading + 90)
        await asyncio.sleep(2)
        
        await drone.land()
```

---

## Migration from Legacy

Migration from legacy to v1 requires minimal changes:

### Update imports

```python
# Before (legacy)
from aerpawlib import Drone, Coordinate, BasicRunner, entrypoint  # Note that this still works but is deprecated

# After (v1)
from aerpawlib.v1 import Drone, Coordinate, BasicRunner, entrypoint
```


> Note that compatible versions of datastructures (_GPSInfoCompat) do not inherit from their DroneKit counterparts, so type assertions will fail. If you were relying on type assertions, you're probably doing something wrong.



