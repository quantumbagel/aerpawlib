# aerpawlib Legacy API Documentation

> **⚠️ Deprecated**: The legacy API uses DroneKit which is no longer actively maintained and has not been for over 2 years. For new projects, please use the [v2 API (beta)](../v2/README.md) or [v1 API (preferred)](../v1/README.md).

The legacy aerpawlib API is the original DroneKit-based implementation for vehicle control. It is preserved for backward compatibility and reference. 

## Requirements

- Python 3.7+
- DroneKit (`pip install dronekit`)
- pymavlink (`pip install pymavlink`)

## Quick Start

```python
from aerpawlib.legacy import Drone, Coordinate, BasicRunner, entrypoint

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


### Connection

The legacy API connects to vehicles using DroneKit's connection strings:

```python
from aerpawlib.legacy import Drone

# Connect to SITL
drone = Drone("udp:127.0.0.1:14540")

# Connect to serial
drone = Drone("serial:///dev/ttyUSB0:57600")

# Connect to TCP
drone = Drone("tcp:127.0.0.1:5760")
```

### Runners

Runners are execution frameworks that manage script lifecycle:

| Runner | Description |
|--------|-------------|
| `BasicRunner` | Simple entry point execution with `@entrypoint` |
| `StateMachine` | State-based execution with `@state` and `@timed_state` |

### Vehicle Types

| Type | Description |
|------|-------------|
| `Vehicle` | Base class with common functionality |
| `Drone` | Multicopter with takeoff, landing, heading control |
| `Rover` | Ground vehicle with goto capabilities |
| `DummyVehicle` | For scripts that don't need a vehicle |

---

## API Reference

### Vehicle Properties

| Property | Type | Description |
|----------|------|-------------|
| `connected` | `bool` | True if receiving heartbeats |
| `position` | `Coordinate` | Current position (lat, lon, alt) |
| `battery` | `dronekit.Battery` | Battery status (voltage, current, level) |
| `gps` | `dronekit.GPSInfo` | GPS status (fix_type, satellites_visible) |
| `armed` | `bool` | True if vehicle is armed |
| `home_coords` | `Coordinate` | Home/launch position |
| `heading` | `float` | Current heading in degrees |
| `velocity` | `VectorNED` | Current velocity (north, east, down) |
| `attitude` | `dronekit.Attitude` | Roll, pitch, yaw in radians |
| `autopilot_info` | `dronekit.Version` | Autopilot version info |

### Vehicle Methods

#### `await set_armed(value: bool)`
Arm or disarm the vehicle. Blocks until the vehicle reaches the desired state.

```python
await drone.set_armed(True)   # Arm
await drone.set_armed(False)  # Disarm
```

#### `done_moving() -> bool`
Check if the vehicle has completed its current movement command.

```python
if drone.done_moving():
    print("Ready for next command")
```

#### `await await_ready_to_move()`
Block until the vehicle is ready for the next command.

```python
await drone.await_ready_to_move()
await drone.goto_coordinates(target)
```

#### `close()`
Clean up the vehicle connection and resources.

```python
drone.close()
```

---

### Drone-Specific Methods

#### `await takeoff(target_alt: float, min_alt_tolerance: float = 0.95, wait_for_throttle: bool = False)`
Take off to the specified altitude.

| Parameter | Type | Description |
|-----------|------|-------------|
| `target_alt` | `float` | Target altitude in meters |
| `min_alt_tolerance` | `float` | Fraction of target alt considered "reached" (default: 0.95) |
| `wait_for_throttle` | `bool` | Wait for RC throttle to center before takeoff |

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

| Parameter | Type | Description |
|-----------|------|-------------|
| `coordinates` | `Coordinate` | Target position |
| `tolerance` | `float` | Acceptance radius in meters (default: 2) |
| `target_heading` | `float` | Optional heading to maintain (degrees) |

```python
target = Coordinate(35.7275, -78.6960, 15)
await drone.goto_coordinates(target, tolerance=3)
```

#### `await set_heading(heading: float, blocking: bool = True, lock_in: bool = True)`
Set the vehicle heading.

| Parameter | Type | Description |
|-----------|------|-------------|
| `heading` | `float` | Target heading in degrees (0 = North) |
| `blocking` | `bool` | Wait for current command to complete first |
| `lock_in` | `bool` | Use this heading for subsequent commands |

```python
await drone.set_heading(90)  # Face east
await drone.set_heading(drone.heading + 45)  # Turn 45° clockwise
```

#### `await set_velocity(velocity_vector: VectorNED, global_relative: bool = True, duration: float = None)`
Set the vehicle velocity.

| Parameter | Type | Description |
|-----------|------|-------------|
| `velocity_vector` | `VectorNED` | Velocity in m/s |
| `global_relative` | `bool` | If True, velocity is world-relative; if False, body-relative |
| `duration` | `float` | Optional duration in seconds |

```python
from aerpawlib.legacy import VectorNED

# Move north at 5 m/s for 10 seconds
await drone.set_velocity(VectorNED(5, 0, 0), duration=10)

# Move forward (body-relative) at 3 m/s
await drone.set_velocity(VectorNED(3, 0, 0), global_relative=False)
```

#### `await set_groundspeed(velocity: float)`
Set the cruise speed for goto operations.

```python
await drone.set_groundspeed(8)  # 8 m/s
```

---

### Rover-Specific Methods

#### `await goto_coordinates(coordinates: Coordinate, tolerance: float = 2.1, target_heading: float = None)`
Navigate to specific coordinates (ground distance only).

```python
target = Coordinate(35.7275, -78.6960, 0)
await rover.goto_coordinates(target)
```

---

## Safety Checker

The legacy API includes a basic safety checker for geofence validation. For detailed documentation, see the [Safety Checker Guide](safety_checker.md). SafetyChecker uses a client-server model to validate waypoints against geofences.

### SafetyCheckerServer

Run the safety checker server to enforce geofences:

```python
from aerpawlib.legacy import SafetyCheckerServer

# Create server with config file
server = SafetyCheckerServer("geofence_config.yaml", server_port=14580)

# Start serving (blocks)
server.start_server()
```

This can also be run from the safetyChecker.py script in the aerpawlib/legacy/ directory. It is automatically started within the AERPAW environment.

### SafetyCheckerClient

> **Note**: The safety checker is managed by the runner in AERPAW. You typically do not need to connect to it manually.


Connect to the safety checker from your script:

```python
from aerpawlib.legacy import SafetyCheckerClient

client = SafetyCheckerClient("127.0.0.1", 14580)

# Check server status
status = client.checkServerStatus()

# Validate a waypoint
result = client.validateWaypoint(current_pos, target_pos)
if not result:
    print("Waypoint outside geofence!")
```


### Configuration File Format

```yaml
# geofence_config.yaml
vehicle_type: copter
max_speed: 15
min_speed: 0.5
max_alt: 120
min_alt: 5
include_geofences:
  - flight_area.kml
exclude_geofences:
  - no_fly_zone.kml
```

---

## Utility Types

### Coordinate

Represents a geographic position.

```python
from aerpawlib.legacy import Coordinate

# Create a coordinate
pos = Coordinate(lat=35.7275, lon=-78.6960, alt=10)

# Create with a label
pos = Coordinate(lat=35.7275, lon=-78.6960, alt=10, label="Waypoint 1")

# Get distance to another coordinate
distance = pos.distance(other_pos)  # 3D distance in meters
ground_dist = pos.ground_distance(other_pos)  # 2D distance

# Get bearing to another coordinate
bearing = pos.bearing(other_pos)  # Degrees

# Add/subtract VectorNED for relative positioning
new_pos = pos + VectorNED(100, 50, 0)  # 100m north, 50m east
```

### VectorNED

Represents a 3D vector in North-East-Down frame.

```python
from aerpawlib.legacy import VectorNED

vec = VectorNED(north=5.0, east=3.0, down=0.0)

# Operations
magnitude = vec.hypot()                    # 3D magnitude
ground_mag = vec.hypot(ignore_down=True)   # 2D magnitude
normalized = vec.norm()                    # Unit vector
rotated = vec.rotate_by_angle(45)          # Rotate by degrees

# Arithmetic
sum_vec = vec1 + vec2
diff_vec = vec1 - vec2
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
    @state("start", first=True)
    async def start(self, drone: Drone):
        await drone.takeoff(10)
        return "navigate"
    
    @state("navigate")
    async def navigate(self, drone: Drone):
        await drone.goto_coordinates(target)
        return "land"
    
    @state("land")
    async def land(self, drone: Drone):
        await drone.land()
        return None  # End state machine
```

### `@timed_state(name: str, duration: float, loop: bool = False, first: bool = False)`
Define a state that runs for a specific duration.

```python
@timed_state("hover", duration=10.0, loop=True)
async def hover(self, drone: Drone):
    print(f"Altitude: {drone.position.alt}")
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

---

## ZMQ Communication

The legacy API includes ZMQ utilities for IPC (inter-process communication).

### ZMQ Proxy

Run a ZMQ proxy for multi-vehicle coordination:

```python
from aerpawlib.legacy import run_zmq_proxy

# Start proxy (blocks)
run_zmq_proxy(in_port=5555, out_port=5556)
```

### ZMQ Publisher

Publish messages to other scripts:

```python
from aerpawlib.legacy import ZMQPublisher

pub = ZMQPublisher(port=5555)

# Publish state transitions
pub.publish("transition", {"from": "takeoff", "to": "navigate"})

# Publish custom data
pub.publish("telemetry", {"altitude": drone.position.alt})
```

### ZMQ Subscriber

Subscribe to messages from other scripts:

```python
from aerpawlib.legacy import ZMQSubscriber

sub = ZMQSubscriber(address="tcp://127.0.0.1:5556")

# Receive messages
while True:
    msg = sub.receive()
    if msg:
        print(f"Received: {msg}")
```

---

## Examples

### Basic Flight

```python
from aerpawlib.legacy import Drone, Coordinate, BasicRunner, entrypoint

class SimpleFlight(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        await drone.takeoff(10)
        
        waypoints = [
            Coordinate(35.7275, -78.6960, 10),
            Coordinate(35.7280, -78.6955, 15),
            Coordinate(35.7270, -78.6950, 10),
        ]
        
        for wp in waypoints:
            await drone.goto_coordinates(wp)
        
        await drone.land()
```

### State Machine

```python
from aerpawlib.legacy import Drone, StateMachine, state, background
import asyncio

class PatrolMission(StateMachine):
    def __init__(self):
        self.patrol_count = 0
        self.max_patrols = 3
        self.patrol_points = [
            Coordinate(35.7275, -78.6960, 15),
            Coordinate(35.7280, -78.6955, 15),
        ]
    
    @state("takeoff", first=True)
    async def takeoff(self, drone: Drone):
        await drone.takeoff(15)
        return "patrol"
    
    @state("patrol")
    async def patrol(self, drone: Drone):
        # Go to patrol points
        point = self.patrol_points[self.patrol_count % len(self.patrol_points)]
        await drone.goto_coordinates(point)
        self.patrol_count += 1
        
        if self.patrol_count >= self.max_patrols:
            return "rtl"
        return "patrol"
    
    @state("rtl")
    async def rtl(self, drone: Drone):
        await drone.goto_coordinates(drone.home_coords)
        await drone.land()
        return None
    
    @background
    async def battery_monitor(self, drone: Drone):
        while True:
            if drone.battery.level < 20:
                print("Low battery!")
            await asyncio.sleep(5)
```

### Multi-Vehicle Coordination

```python
from aerpawlib.legacy import Drone, BasicRunner, entrypoint, ZMQPublisher, ZMQSubscriber
import asyncio

class Leader(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        pub = ZMQPublisher(port=5555)
        
        await drone.takeoff(10)
        pub.publish("status", {"state": "airborne"})
        
        target = Coordinate(35.7275, -78.6960, 10)
        await drone.goto_coordinates(target)
        pub.publish("status", {"state": "at_target", "position": str(target)})
        
        await drone.land()
        pub.publish("status", {"state": "landed"})

class Follower(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        sub = ZMQSubscriber(address="tcp://127.0.0.1:5556")
        
        # Wait for leader to take off
        while True:
            msg = sub.receive()
            if msg and msg.get("state") == "airborne":
                break
            await asyncio.sleep(0.5)
        
        await drone.takeoff(10)
        
        # Follow leader
        while True:
            msg = sub.receive()
            if msg and msg.get("state") == "landed":
                break
            await asyncio.sleep(0.5)
        
        await drone.land()
```