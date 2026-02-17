# aerpawlib Documentation

Welcome to the aerpawlib documentation. aerpawlib is a Python library for autonomous vehicle control within the [AERPAW](https://aerpaw.org) platform.

## Documentation Index

| Document | Audience | Description |
|----------|----------|-------------|
| [User Guide](USER_GUIDE.md) | All users | Overview of supported workflows, features, and how to run missions |
| [Development Guide](DEVELOPMENT.md) | Contributors | Project structure, testing, coding conventions |
| [v1 API Reference](v1/README.md) | v1 users | Full v1 API documentation (MAVSDK, DroneKit-compatible) |
| [v2 API Reference](v2/README.md) | v2 users | v2 API documentation (modern, async-first) |
| [v1 Safety Checker](v1/safety_checker.md) | v1 users | Geofence validation and safety server/client |
| [v1 Architecture](v1/compromises.md) | Developers | v1 design tradeoffs and dual-loop architecture |
| [Roadmap](ROADMAP.md) | All | Planned features and improvements |


## Quick Links

- [User Guide → Quick Start](USER_GUIDE.md#quick-start)
- [User Guide → Running Scripts](USER_GUIDE.md#running-scripts)
- [examples/](../examples/) directory
- [tests/README.md](../tests/README.md)

## Installation

```bash
pip install -e .
```

## Connection Strings

All versions use MAVSDK connection strings:

```python
# SITL / UDP
"udp://:14540"
"udp://127.0.0.1:14540"
"udpin://127.0.0.1:14550"

# Serial
"serial:///dev/ttyUSB0:57600"
"serial:///dev/ttyACM0:115200"

# TCP
"tcp://192.168.1.100:5760"
```

## Vehicle Types

| Type           | Description                              | Available In |
|----------------|------------------------------------------|--------------|
| `Drone`        | Multicopter (takeoff, land, 3D movement) | v1, v2       |
| `Rover`        | Ground vehicle (2D movement)             | v1, v2       |
| `DummyVehicle` | No-op for scripts without vehicles       | v1           |
| `MockDrone`    | Test double for unit tests               | v2           |
