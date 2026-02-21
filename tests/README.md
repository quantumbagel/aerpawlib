# aerpawlib Test Suite

Pytest-based tests for aerpawlib v1 API. See [docs/DEVELOPMENT.md](../docs/DEVELOPMENT.md) for contributor testing guidelines. **SITL is managed by pytest** for integration tests: it starts ArduPilot SITL before tests and stops it after. A full SITL reset (disarm, clear mission, battery reset) runs between each integration test.

## Structure

```
tests/
├── conftest.py           # Fixtures, SITL manager, markers
├── unit/                 # Unit tests (no SITL)
│   ├── test_v1_util.py       # Coordinate, VectorNED, plan, geofence
│   ├── test_v1_helpers.py    # wait_for_condition, validate_*
│   ├── test_v1_runner.py     # BasicRunner, StateMachine
│   ├── test_v1_external.py   # ExternalProcess
│   └── test_v1_exceptions.py # Exception hierarchy
└── integration/          # Integration tests (SITL)
    ├── test_v1_drone.py     # Drone connection, takeoff, nav, land
    ├── test_v1_rover.py     # Rover (requires Rover SITL)
    └── test_v1_vehicles.py  # DummyVehicle (no SITL)
```

## Prerequisites

1. Unit tests: `pip install -e .[dev]`
2. Integration tests: Run `aerpawlib-setup-sitl` (or `./scripts/install_dev.sh`) to install the modified ArduPilot SITL. Pytest then starts ArduCopter SITL for drone tests and ArduRover SITL for rover tests (separate ports).

## Running Tests

### Unit tests only (fast, no SITL)

```bash
pytest tests/unit/ -v
# or
pytest -m unit -v
```

### Integration tests (pytest manages SITL)

```bash
pytest tests/integration/ -v
# or
pytest -m integration -v
```

Pytest will:
1. Start ArduCopter SITL with MAVProxy on instance 0, UDP output to port 14550
2. Start Rover SITL with MAVProxy on instance 1, UDP output to port 14560
3. Run integration tests (only starts SITLs for the vehicle types being tested)
4. Perform full SITL reset between each test
5. Stop SITL when done

Different instance IDs (`-I 0` for drone, `-I 1` for rover) ensure the internal TCP ports don't conflict when running both concurrently.

### Use external SITL (pytest does not start/stop)

```bash
# Terminal 1: start drone SITL (instance 0)
sim_vehicle.py -v ArduCopter -I 0 --out=udp:127.0.0.1:14550 -w

# Terminal 2: start rover SITL (instance 1, different internal ports)
sim_vehicle.py -v Rover -I 1 --out=udp:127.0.0.1:14560 -w

# Terminal 3: run tests
pytest tests/integration/ -v --no-sitl
```

### Options

| Option                   | Description                                                                       |
|--------------------------|-----------------------------------------------------------------------------------|
| `--sitl-port PORT`       | Legacy: UDP port for drone SITL (default: 14550)                                  |
| `--sitl-port-drone PORT` | UDP port for ArduCopter SITL (default: 14550)                                     |
| `--sitl-port-rover PORT` | UDP port for ArduRover SITL (default: 14560)                                      |
| `--no-sitl`              | Do not start SITL; use externally running instance                                |
| `--no-sitl-manage`       | Pytest does not start/stop SITL; use external SITL (default: pytest manages SITL) |

### Log files

SITL output is captured to separate log files per vehicle type:
- `logs/sitl_drone_output.log` – sim_vehicle.py output (build, progress)
- `logs/sitl_rover_output.log` – sim_vehicle.py output (build, progress)
- `/tmp/ArduCopter.log` – SITL binary output (when run headless, no terminal window)
- `/tmp/Rover.log` – SITL binary output (when run headless, no terminal window)

Pytest unsets `DISPLAY` so sim_vehicle does not open a new Terminal window; the SITL process runs headless.

Integration tests disable pytest output capture (`-s` behavior) because MAVProxy blocks when stdout is a pipe.

### Environment variables

- `SITL_VERBOSE=1` – show SITL stdout/stderr
- `SIM_SPEEDUP=5` – simulation speed (default: 5)
- `ARDUPILOT_HOME` – path to ArduPilot (or use `./ardupilot`)

## Markers

- `unit` – unit tests (auto-applied to `tests/unit/`)
- `integration` – integration tests (auto-applied to `tests/integration/`)

## Troubleshooting

### "Mode change to GUIDED failed: requires position"

This occurs when starting an experiment before SITL has fully initialized. 
This is a inconsistency with the MavSDK library lying to us about whether the drone actually has a position or not.

There are two solutions:
- Wait for SITL to fully start – Give SITL 10–15 seconds after `sim_vehicle.py` reports "Ready to FLY" before running your script.
2. Use external SITL – Run SITL in a separate terminal first, then run your experiment with `--no-sitl` (for pytest) or after SITL is ready.
