# aerpawlib Documentation

Welcome to the aerpawlib documentation. aerpawlib is a Python library for autonomous vehicle control within the
[AERPAW](https://aerpaw.org) environment. 
## API Versions

aerpawlib offers three API versions to accommodate different needs:


| Version                    | Backend  | Recommended For                          |
|----------------------------|----------|------------------------------------------|
| [v2](v2/README.md)         | MAVSDK   | Nothing currently as it is very buggy :D |
| [v1](v1/README.md)         | MAVSDK   | No API changes. Still has major bugs.    |
| [legacy](legacy/README.md) | DroneKit | Still use for most projects.             |


## Quick Comparison

| Feature               | Legacy                      | v1              | v2             |
|-----------------------|-----------------------------|-----------------|----------------|
| Backend               | DroneKit                    | MAVSDK          | MAVSDK         |
| Python                | 3.7-3.10 (patched to 3.10+) | 3.9+            | 3.9+           |
| Async Support         | Some components             | Some components | All components |


## Installation

```bash
pip install -e .
```

## Examples

See the [`examples/`](../examples/) directory for working examples:

- [`examples/v2/`](../examples/v2/) - v2 API examples
- [`examples/v1/`](../examples/v1/) - v1 API examples
- [`examples/legacy/`](../examples/legacy/) - Legacy API examples

## Connection Strings

All versions use similar connection strings:

```python
# SITL / UDP
"udp://:14540"
"udp://127.0.0.1:14540"

# Serial
"serial:///dev/ttyUSB0:57600"
"serial:///dev/ttyACM0:115200"

# TCP
"tcp://192.168.1.100:5760"
```

## Vehicle Types

| Type           | Description                              | Available In |
|----------------|------------------------------------------|--------------|
| `Drone`        | Multicopter (takeoff, land, 3D movement) | All versions |
| `Rover`        | Ground vehicle (2D movement)             | All versions |
| `DummyVehicle` | For scripts without vehicles             | All versions |