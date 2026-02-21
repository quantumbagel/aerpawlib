# aerpawlib

![Python Version](https://img.shields.io/badge/python-3.9%2B-blue)
![License](https://img.shields.io/badge/license-MIT-green)

A Python library for controlling vehicles within the [AERPAW](https://aerpaw.org) platform. Provides a unified interface for vehicle control, telemetry, and mission execution with ArduPilot.

## Features

- Unified vehicle control
- Scriptable missions
- Multi-vehicle coordination
- Safety checker
- AERPAW Platform integration

## Installation

```bash
pip install -e .
```

### Development (with ArduPilot SITL)

For running integration tests and local SITL simulation:

```bash
pip install -e .[dev]
aerpawlib-setup-sitl
```

Or use the one-liner script:

```bash
./scripts/install_dev.sh
```
This installs dev dependencies (pytest, etc.), ArduPilot SITL, MAVProxy, and compiles Copter + Rover SITL.

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
aerpawlib --script my_mission --conn udpin://127.0.0.1:14550 --vehicle drone
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
aerpawlib --script examples.v1.basic_example --conn udp://127.0.0.1:14550 --vehicle drone

# With config file
aerpawlib --config tests/configs/testv1_basic_runner.json
```

See [examples/README.md](examples/README.md) for full list.

## License

MIT License – see [LICENSE](LICENSE).
