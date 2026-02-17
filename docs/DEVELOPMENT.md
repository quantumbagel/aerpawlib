# aerpawlib Development Guide

This document is for contributors and developers working on the aerpawlib codebase.

## Project Structure

```
aerpawlib-vehicle-control/
├── aerpawlib/
│   ├── __main__.py          # CLI entry point (python -m aerpawlib)
│   ├── v1/                  # v1 API (MAVSDK, DroneKit-compatible)
│   │   ├── __init__.py
│   │   ├── aerpaw.py        # AERPAW platform (OEO, checkpoints)
│   │   ├── constants.py     # Timing, limits, config
│   │   ├── exceptions.py    # Exception hierarchy
│   │   ├── external.py      # ExternalProcess
│   │   ├── helpers.py       # wait_for_condition, validate_*, ThreadSafeValue
│   │   ├── log.py           # Logging configuration
│   │   ├── runner.py        # Runner, BasicRunner, StateMachine, ZmqStateMachine
│   │   ├── safety.py        # SafetyCheckerServer, SafetyCheckerClient
│   │   ├── safetyChecker.py # Deprecation alias for safety
│   │   ├── util.py          # Coordinate, VectorNED, geofence, plan parsing
│   │   ├── vehicle.py       # Re-exports Vehicle, Drone, Rover
│   │   ├── zmqutil.py       # ZMQ proxy
│   │   └── vehicles/
│   │       ├── core_vehicle.py  # Vehicle base, telemetry, connection
│   │       ├── drone.py        # Drone implementation
│   │       └── rover.py        # Rover implementation
│   └── v2/                  # v2 API (modern, async-first)
│       ├── ...
├── docs/                    # Documentation
├── examples/                # Example scripts
├── tests/                   # Pytest tests
│   ├── conftest.py          # Fixtures, SITL manager
│   ├── unit/                # Unit tests (no SITL)
│   └── integration/         # Integration tests (SITL)
├── pyproject.toml
└── README.md
```

## Key Architectural Concepts

### v1 Dual-Loop Architecture

v1 maintains a background thread with its own asyncio event loop for MAVSDK. User code runs on the main thread/loop. All MAVSDK calls go through `_run_on_mavsdk_loop()`, which uses `run_coroutine_threadsafe` to bridge threads.

- Received on background thread, stored in `ThreadSafeValue` wrappers
- Commands: Scheduled on background loop, main thread awaits result
- Close: Must call `vehicle.close()` to stop background thread and cancel tasks

See [v1/compromises.md](v1/compromises.md) for design tradeoffs.

### v1 Runner Hierarchy

```
Runner (base)
├── BasicRunner     # Single @entrypoint
├── StateMachine    # @state, @timed_state, @background, @at_init
└── ZmqStateMachine  # StateMachine + ZMQ remote control
```

### Module Dependencies

- `v1/vehicle.py` aggregates `core_vehicle`, `drone`, `rover`
- `v1/__init__.py` uses `from .X import *` for flat API
- `safetyChecker` is a deprecated alias for `safety`

## Testing

### Unit Tests (No SITL)

```bash
pytest tests/unit/ -v
# or
pytest -m unit -v
```

Covers: util, helpers, exceptions, runner, external.

### Integration Tests (SITL)

Requires ArduPilot SITL. Pytest manages SITL lifecycle by default.

```bash
# Pytest starts SITL, runs tests, stops SITL
pytest tests/integration/ -v

# Use external SITL (start sim_vehicle.py manually first)
pytest tests/integration/ -v --no-sitl
```

**Prerequisites**:
- `ARDUPILOT_HOME` set, or run `./install_ardupilot.sh`
- `sim_vehicle.py` at `$ARDUPILOT_HOME/Tools/autotest/sim_vehicle.py`

See [tests/README.md](../tests/README.md) for full details.

## Code Conventions

### Python Version

- **Minimum**: Python 3.9 (pyproject.toml `requires-python = ">=3.9"`)
- Use `Optional[X]` not `X | None` for 3.9 compatibility

### Formatting

- Black: 79 char line length, py39–py312
- Config in `pyproject.toml`

### Imports

- Prefer absolute imports: `from aerpawlib.v1 import X`
- v1 re-exports from submodules in `__init__.py`

### Logging

- Use `get_logger(LogComponent.X)` from `aerpawlib.log`
- Components: VEHICLE, DRONE, ROVER, RUNNER, SAFETY, AERPAW, etc.

### Exceptions

- Inherit from `AerpawlibError` (or subclasses)
- Use `original_error=e` when wrapping

## Adding New Features

### New Vehicle Method (v1 Drone)

1. Add method to `aerpawlib/v1/vehicles/drone.py`
2. Use `await self._run_on_mavsdk_loop(coro)` for MAVSDK calls
3. Set `_ready_to_move` for blocking methods
4. Add integration test in `tests/integration/test_v1_drone.py`

### New Runner Decorator

1. Add decorator in `aerpawlib/v1/runner.py`
2. Set attribute on function (e.g., `_is_background`)
3. Handle in `StateMachine._build()` or equivalent
4. Add unit test in `tests/unit/test_v1_runner.py`

### New Safety Checker Request

1. Add constant in `aerpawlib/v1/constants.py`
2. Add handler in `SafetyCheckerServer`
3. Add client method in `SafetyCheckerClient`
4. Update [v1/safety_checker.md](v1/safety_checker.md)

## Debugging

### Verbose Vehicle State

```bash
python -m aerpawlib --script my_mission --conn ... --vehicle drone --debug-dump
```

Writes CSV of vehicle state to `aerpawlib_vehicle_dump_*.csv`.

### Log Level

```bash
python -m aerpawlib --script my_mission ... --debug   # DEBUG
python -m aerpawlib --script my_mission ... -v       # INFO
python -m aerpawlib --script my_mission ... -q       # WARNING only
```

### SITL Verbosity

```bash
SITL_VERBOSE=1 pytest tests/integration/ -v
```

### gRPC Fork Warning

When using `ExternalProcess` with fork:
```
Other threads are currently calling into gRPC, skipping fork() handlers
```
Set `GRPC_ENABLE_FORK_SUPPORT=false` to suppress (cosmetic only).

## Release Checklist

- [ ] Run full test suite: `pytest tests/ -v`
- [ ] Update version in `pyproject.toml`
- [ ] Update [ROADMAP.md](ROADMAP.md) if needed
- [ ] Ensure [docs/README.md](README.md) and [USER_GUIDE.md](USER_GUIDE.md) are current

## Related Documentation

- [User Guide](USER_GUIDE.md) – Workflows and features
- [v1 API Reference](v1/README.md) – Full v1 API
- [v1 Architecture](v1/compromises.md) – Design tradeoffs
- [v1 Safety Checker](v1/safety_checker.md) – Geofence validation
- [ROADMAP.md](ROADMAP.md) – Planned improvements
