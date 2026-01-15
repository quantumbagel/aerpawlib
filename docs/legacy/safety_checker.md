# Safety Checker Guide (Legacy)

The legacy API includes a safety checker system for geofence validation using ZMQ communication.

> **Note**: There are several significant safety features missing from the legacy safety checker that are included in the v2 API.
## Overview

The safety checker system consists of:

| Component             | Description                                         |
|-----------------------|-----------------------------------------------------|
| `SafetyCheckerServer` | Runs geofence validation, receives requests via ZMQ |
| `SafetyCheckerClient` | Sends validation requests to the server             |

## Quick Start

### Create a Geofence Configuration

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

### Start the Server

```python
from aerpawlib.legacy.safetyChecker import SafetyCheckerServer

server = SafetyCheckerServer("geofence_config.yaml", server_port=14580)
server.start_server()  # Blocks
```

### Connect from Your Script

```python
from aerpawlib.legacy.safetyChecker import SafetyCheckerClient

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
from aerpawlib.legacy.safetyChecker import SafetyCheckerServer

# Create server with YAML config
server = SafetyCheckerServer(
    vehicle_config="geofence_config.yaml",
    server_port=14580
)
```

### Starting the Server

```python
# Blocking - runs until interrupted
server.start_server()

# Or specify port at runtime
server.start_server(port=14581)
```

### Configuration Parameters

| Parameter           | Type    | Description                      |
|---------------------|---------|----------------------------------|
| `vehicle_type`      | `str`   | "copter" or "rover"              |
| `max_speed`         | `float` | Maximum allowed speed (m/s)      |
| `min_speed`         | `float` | Minimum allowed speed (m/s)      |
| `max_alt`           | `float` | Maximum altitude for copters (m) |
| `min_alt`           | `float` | Minimum altitude for copters (m) |
| `include_geofences` | `list`  | KML files defining allowed areas |
| `exclude_geofences` | `list`  | KML files defining no-go zones   |

---

## SafetyCheckerClient

### Initialization

```python
from aerpawlib.legacy.safetyChecker import SafetyCheckerClient

client = SafetyCheckerClient(
    server_address="127.0.0.1",
    server_port=14580
)
```

### Methods

#### `checkServerStatus() -> bool`

Check if the server is running and responsive.

```python
if client.checkServerStatus():
    print("Server is running")
else:
    print("Server not responding")
```

#### `validateWaypoint(current: Coordinate, target: Coordinate) -> bool`

Validate that a waypoint is within geofences.

```python
from aerpawlib.legacy import Coordinate

current = Coordinate(35.7275, -78.6960, 10)
target = Coordinate(35.7280, -78.6955, 15)

if client.validateWaypoint(current, target):
    await drone.goto_coordinates(target)
else:
    print("Target is outside geofence!")
```

#### `validateSpeed(speed: float) -> bool`

Validate that a speed is within limits.

```python
if client.validateSpeed(15.0):
    await drone.set_groundspeed(15.0)
else:
    print("Speed exceeds limit!")
```

#### `validateTakeoff(altitude: float, lat: float, lon: float) -> bool`

Validate a takeoff command.

```python
if client.validateTakeoff(10, drone.position.lat, drone.position.lon):
    await drone.takeoff(10)
else:
    print("Cannot take off at this location!")
```

---

## Geofence Files

### KML Format

Create geofence polygons using KML files:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Flight Area</name>
    <Placemark>
      <name>Allowed Area</name>
      <Polygon>
        <outerBoundaryIs>
          <LinearRing>
            <coordinates>
              -78.70,35.70,0
              -78.60,35.70,0
              -78.60,35.80,0
              -78.70,35.80,0
              -78.70,35.70,0
            </coordinates>
          </LinearRing>
        </outerBoundaryIs>
      </Polygon>
    </Placemark>
  </Document>
</kml>
```

### Creating KML Files

You can create KML files using:
- Google Earth Pro
- QGIS
- Online KML generators
- Programmatically with Python

### Multiple Geofences

You can have multiple geofences:

```yaml
include_geofences:
  - area1.kml
  - area2.kml
  - area3.kml
exclude_geofences:
  - no_fly_zone1.kml
  - no_fly_zone2.kml
```

The drone must be inside at least one include geofence and outside all exclude geofences.