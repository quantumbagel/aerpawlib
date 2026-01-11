# Safety Checker Guide (v1)

The v1 API includes a safety checker system for geofence validation using ZMQ communication. It is functionally identical to the legacy safety checker but uses MAVSDK internally.

## Overview

The safety checker system consists of:

| Component | Description |
|-----------|-------------|
| `SafetyCheckerServer` | Runs geofence validation, receives requests via ZMQ |
| `SafetyCheckerClient` | Sends validation requests to the server |

## Quick Start

### 1. Create a Geofence Configuration

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

### 2. Start the Server

```bash
# From command line
python -c "from aerpawlib.v1.safetyChecker import SafetyCheckerServer; SafetyCheckerServer('geofence_config.yaml', 14580).start_server()"
```

Or in Python:

```python
from aerpawlib.v1.safetyChecker import SafetyCheckerServer

server = SafetyCheckerServer("geofence_config.yaml", server_port=14580)
server.start_server()  # Blocks
```

### 3. Connect from Your Script

```python
from aerpawlib.v1.safetyChecker import SafetyCheckerClient

client = SafetyCheckerClient("127.0.0.1", 14580)

# Check server status
if client.checkServerStatus():
    print("Server is running")

# Validate waypoint before flying
result = client.validateWaypoint(current_pos, target_pos)
if result:
    print("Waypoint is valid")
else:
    print("Waypoint outside geofence!")
```

---

## SafetyCheckerServer

### Initialization

```python
from aerpawlib.v1.safetyChecker import SafetyCheckerServer

# Create server with YAML config
server = SafetyCheckerServer(
    vehicle_config="geofence_config.yaml",
    server_port=14580
)
```

### Configuration Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `vehicle_type` | `str` | "copter" or "rover" |
| `max_speed` | `float` | Maximum allowed speed (m/s) |
| `min_speed` | `float` | Minimum allowed speed (m/s) |
| `max_alt` | `float` | Maximum altitude for copters (m) |
| `min_alt` | `float` | Minimum altitude for copters (m) |
| `include_geofences` | `list` | KML files defining allowed areas |
| `exclude_geofences` | `list` | KML files defining no-go zones |

### Validation Logic

The server validates:

1. **Waypoint position**: Must be inside at least one include geofence
2. **No-go zones**: Must not be inside any exclude geofence
3. **Path validation**: Path must not cross geofence boundaries
4. **Path no-go**: Path must not enter any no-go zones
5. **Altitude (copter)**: Must be within min_alt and max_alt
6. **Speed**: Must be within min_speed and max_speed

---

## SafetyCheckerClient

### Initialization

```python
from aerpawlib.v1.safetyChecker import SafetyCheckerClient

client = SafetyCheckerClient(
    server_address="127.0.0.1",
    server_port=14580
)
```

### Methods

#### `checkServerStatus() -> bool`

```python
if client.checkServerStatus():
    print("Server is running")
```

#### `validateWaypoint(current: Coordinate, target: Coordinate) -> bool`

```python
from aerpawlib.v1 import Coordinate

current = Coordinate(35.7275, -78.6960, 10)
target = Coordinate(35.7280, -78.6955, 15)

if client.validateWaypoint(current, target):
    await drone.goto_coordinates(target)
else:
    print("Target is outside geofence!")
```

#### `validateSpeed(speed: float) -> bool`

```python
if client.validateSpeed(15.0):
    await drone.set_groundspeed(15.0)
```

#### `validateTakeoff(altitude: float, lat: float, lon: float) -> bool`

```python
if client.validateTakeoff(10, drone.position.lat, drone.position.lon):
    await drone.takeoff(10)
```

---

## Protocol Details

The safety checker uses ZMQ REQ/REP pattern with zlib-compressed JSON messages.

### Request Format

```json
{
  "request_function": "validate_waypoint_req",
  "params": [
    {"latitude": 35.7275, "longitude": -78.6960, "altitude": 10},
    {"latitude": 35.7280, "longitude": -78.6955, "altitude": 15}
  ]
}
```

### Response Format

```json
{
  "request_function": "validate_waypoint_req",
  "result": true,
  "message": ""
}
```

### Request Types

| Type | Description |
|------|-------------|
| `server_status_req` | Check server status |
| `validate_waypoint_req` | Validate waypoint |
| `validate_change_speed_req` | Validate speed change |
| `validate_takeoff_req` | Validate takeoff |
| `validate_landing_req` | Validate landing |

---

## Example: Full Integration

```python
from aerpawlib.v1 import Drone, Coordinate, BasicRunner, entrypoint
from aerpawlib.v1.safetyChecker import SafetyCheckerClient

class SafeMission(BasicRunner):
    def __init__(self):
        self.checker = SafetyCheckerClient("127.0.0.1", 14580)
    
    @entrypoint
    async def run(self, drone: Drone):
        # Verify safety checker is running
        if not self.checker.checkServerStatus():
            print("ERROR: Safety checker not running!")
            return
        
        # Validate takeoff
        pos = drone.position
        if not self.checker.validateTakeoff(10, pos.lat, pos.lon):
            print("Cannot take off at this location")
            return
        
        await drone.takeoff(10)
        
        # Define waypoints
        waypoints = [
            Coordinate(35.7275, -78.6960, 10),
            Coordinate(35.7280, -78.6955, 15),
        ]
        
        # Validate each waypoint before flying
        for wp in waypoints:
            if self.checker.validateWaypoint(drone.position, wp):
                await drone.goto_coordinates(wp)
            else:
                print(f"Waypoint {wp} is outside geofence!")
                break
        
        await drone.land()
```

---

## Running Server in Background

### Using Python's multiprocessing

```python
from multiprocessing import Process
from aerpawlib.v1.safetyChecker import SafetyCheckerServer

def run_server():
    server = SafetyCheckerServer("config.yaml", 14580)
    server.start_server()

# Start server in background
server_process = Process(target=run_server, daemon=True)
server_process.start()

# Your mission code here...
```

### Using screen (Linux)

```bash
screen -dmS safety_checker python -c "from aerpawlib.v1.safetyChecker import SafetyCheckerServer; SafetyCheckerServer('config.yaml', 14580).start_server()"
```

---

## Upgrading to v2

The v2 API offers a more comprehensive safety system with async support:

```python
from aerpawlib.v2 import (
    Drone, SafetyLimits, SafetyCheckerClient, SafetyConfig
)

# Async context manager for cleaner code
async with SafetyCheckerClient("localhost", 14580) as checker:
    drone = Drone(
        "udp://:14540",
        safety_limits=SafetyLimits.restrictive(),
        safety_checker=checker
    )
    
    await drone.connect()
    await drone.arm()  # Pre-flight checks run automatically
    await drone.takeoff(altitude=10)
    await drone.land()
```

### v2 Safety Advantages

| Feature | v1 | v2 |
|---------|-----|-----|
| Geofence validation | ✅ | ✅ |
| Pre-flight checks | ❌ | ✅ |
| Battery failsafe | ❌ | ✅ |
| Speed limits | ❌ | ✅ |
| Parameter validation | ❌ | ✅ |
| Async support | ❌ | ✅ |
| Context manager | ❌ | ✅ |
| Configurable limits | ❌ | ✅ |

See the [v2 Safety Guide](../v2/safety.md) for details.

