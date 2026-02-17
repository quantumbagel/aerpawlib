# aerpawlib

![Python Version](https://img.shields.io/badge/python-3.9%2B-blue)
![License](https://img.shields.io/badge/license-MIT-green)

A Python library for controlling vehicles within the [AERPAW](https://aerpaw.org) platform. Provides a unified interface for vehicle control, telemetry, and mission execution with ArduPilot.

## Features

- **Unified vehicle control** – Drone and Rover support via MAVSDK
- **Scriptable missions** – BasicRunner, StateMachine, ZmqStateMachine
- **Multi-vehicle coordination** – ZMQ-based leader/follower and swarm patterns
- **Safety checker** – Geofence validation for waypoints, takeoff, and speed
- **AERPAW integration** – OEO logging, checkpoints, safety pilot arming

## Installation

```bash
pip install -e .
```

## Quick Start

```python
# my_mission.py
from aerpawlib.v1 import Drone, Coordinate, BasicRunner, entrypoint

class MyMission(BasicRunner):
    @entrypoint
    async def run(self, vehicle):
        await vehicle.takeoff(10)
        for wp in [Coordinate(35.7275, -78.6960, 10), Coordinate(35.7280, -78.6955, 10)]:
            await vehicle.goto_coordinates(wp)
        await vehicle.land()
```

```bash
python -m aerpawlib --script my_mission --conn udp://127.0.0.1:14550 --vehicle drone
```

## Documentation

| Document | Description |
|----------|-------------|
| [**docs/README.md**](docs/README.md) | Documentation index and API overview |
| [**docs/USER_GUIDE.md**](docs/USER_GUIDE.md) | Supported workflows, features, running scripts |
| [**docs/DEVELOPMENT.md**](docs/DEVELOPMENT.md) | Contributors: project structure, testing, conventions |
| [**docs/v1/README.md**](docs/v1/README.md) | v1 API reference (full) |
| [**docs/v2/README.md**](docs/v2/README.md) | v2 API reference (beta) |
| [**docs/ROADMAP.md**](docs/ROADMAP.md) | Planned features and improvements |

## Examples

```bash
# Basic square flight
python -m aerpawlib --script examples.v1.basic_example --conn udp://127.0.0.1:14550 --vehicle drone

# With config file
python -m aerpawlib --config tests/configs/testv1_basic_runner.json
```

See [examples/README.md](examples/README.md) for full list.

## API Versions

| Version | Status | Use Case |
|---------|--------|----------|
| **v1** | Stable | Production, legacy compatibility |
| **v2** | Beta | Future (has known bugs) |

## License

MIT License – see [LICENSE](LICENSE).
