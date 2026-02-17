# aerpawlib User Guide

This guide provides an overview of supported workflows, features, and how to run missions with aerpawlib.

## Quick Start

1. **Install aerpawlib**:
   ```bash
   pip install -e .
   ```

2. **Write a simple mission** (v1 API):
   ```python
   # my_mission.py
   from aerpawlib.v1 import Drone, Coordinate, BasicRunner, entrypoint

   class MyMission(BasicRunner):
       @entrypoint
       async def run(self, vehicle):
           await vehicle.takeoff(10)
           target = Coordinate(35.7275, -78.6960, 10)
           await vehicle.goto_coordinates(target)
           await vehicle.land()
   ```

3. **Run with aerpawlib**:
   ```bash
   python -m aerpawlib --script my_mission --conn udp://127.0.0.1:14550 --vehicle drone
   ```

## Supported Workflows

### 1. Basic Single-Vehicle Mission

The simplest workflow: one vehicle, one script, linear execution.

**Runner**: `BasicRunner` with `@entrypoint`  
**Use case**: Simple waypoint missions, surveys, demos

```python
from aerpawlib.v1 import Drone, Coordinate, BasicRunner, entrypoint

class SimpleMission(BasicRunner):
    @entrypoint
    async def run(self, vehicle):
        await vehicle.takeoff(10)
        for wp in waypoints:
            await vehicle.goto_coordinates(wp)
        await vehicle.land()
```

**Run**:
```bash
python -m aerpawlib --script my_mission --conn udp://127.0.0.1:14550 --vehicle drone
```

---

### 2. State Machine Mission

For missions with distinct phases, loops, or conditional transitions.

**Runner**: `StateMachine` with `@state` decorators  
**Use case**: Patrol patterns, multi-phase missions, dynamic routing

```python
from aerpawlib.v1 import Drone, StateMachine, state

class PatrolMission(StateMachine):
    @state("takeoff", first=True)
    async def takeoff_state(self, vehicle):
        await vehicle.takeoff(10)
        return "patrol"

    @state("patrol")
    async def patrol_state(self, vehicle):
        await vehicle.goto_coordinates(next_point)
        return "patrol" if more_points else "land"

    @state("land")
    async def land_state(self, vehicle):
        await vehicle.land()
        return None  # End
```

**Run**: Same as basic mission.

---

### 3. State Machine with Background Tasks

Run telemetry logging, monitoring, or other tasks in parallel with the state machine.

**Runner**: `StateMachine` with `@background`  
**Use case**: Logging, battery monitoring, external process coordination

```python
from aerpawlib.v1 import StateMachine, state, background
import asyncio

class LoggingMission(StateMachine):
    @state("fly", first=True)
    async def fly_state(self, vehicle):
        await vehicle.goto_coordinates(target)
        return None

    @background
    async def log_position(self, vehicle):
        while True:
            print(f"Position: {vehicle.position}")
            await asyncio.sleep(1)
```

---

### 4. Multi-Vehicle ZMQ Coordination

Coordinate multiple vehicles via ZMQ. One script per vehicle; a proxy relays messages.

**Runner**: `ZmqStateMachine` with `@expose_zmq`  
**Use case**: Leader/follower, swarm coordination, ground-air teams

**Setup**:
1. Start ZMQ proxy: `python -m aerpawlib --run-proxy`
2. Run each vehicle with `--zmq-identifier` and `--zmq-proxy-server`

**Leader**:
```bash
python -m aerpawlib --script examples.legacy.zmq_runner.leader \
  --conn udp://127.0.0.1:14550 --vehicle drone \
  --zmq-identifier leader --zmq-proxy-server 127.0.0.1
```

**Follower**:
```bash
python -m aerpawlib --script examples.legacy.zmq_runner.follower \
  --conn udp://127.0.0.1:14551 --vehicle drone \
  --zmq-identifier follower --zmq-proxy-server 127.0.0.1
```

See [examples/legacy/zmq_runner/README.md](../examples/legacy/zmq_runner/README.md) and [examples/legacy/zmq_preplanned_orbit/README.md](../examples/legacy/zmq_preplanned_orbit/README.md).

---

### 5. Preplanned Trajectory from QGroundControl

Load waypoints from a QGroundControl `.plan` file.

**Use case**: Pre-defined missions, survey grids from QGC

```python
from aerpawlib.v1 import read_from_plan, get_location_from_waypoint

waypoints = read_from_plan("mission.plan")
for cmd, x, y, z, wp_id, speed in waypoints:
    coord = get_location_from_waypoint((cmd, x, y, z, wp_id, speed))
    await vehicle.goto_coordinates(coord)
```

**Run**:
```bash
python -m aerpawlib --script examples.legacy.preplanned_trajectory \
  --conn udp://127.0.0.1:14550 --vehicle drone --plan mission.plan
```

---

### 6. Safety Checker Integration

Validate waypoints, takeoff, and speed against geofences before commanding the vehicle.

**Components**: `SafetyCheckerServer` (separate process), `SafetyCheckerClient` (in script)

**Setup**:
1. Create YAML config and KML geofences (see [v1 Safety Checker](v1/safety_checker.md))
2. Start server: `python -m aerpawlib.v1.safety --port 14580 --vehicle_config config.yaml`
3. In script: create `SafetyCheckerClient`, call `validate_waypoint_command` before `goto_coordinates`

See [v1/safety_checker.md](v1/safety_checker.md) for full details.

---

### 7. External Process Coordination

Spawn and interact with external processes (e.g., radio scripts, sensors) from the mission.

**Use case**: Radio experiments, sensor fusion, co-located tools

```python
from aerpawlib.v1 import ExternalProcess

proc = ExternalProcess("my_script", params=["--arg", "value"])
await proc.start()
line = await proc.read_line()
await proc.send_input("command\n")
await proc.wait_until_terminated()
```

---

### 8. AERPAW Platform Integration

When running on AERPAW infrastructure:

- **Safety pilot arming**: Script waits for safety pilot to arm; no auto-arm
- **OEO logging**: `AERPAW_Platform.log_to_oeo("[aerpawlib] message")`
- **Checkpoints**: `AERPAW_Platform.checkpoint_set("name")`, `checkpoint_check("name")`
- **Abort**: `vehicle._abort()` triggers OEO log and stops movement

Standalone/SITL: vehicle auto-arms after armable state is reached.

---

## Running Scripts

### Command-Line Interface

```bash
python -m aerpawlib --script <module> --conn <connection> --vehicle <type> [options]
```

**Required**:
- `--script`: Python module path (e.g., `examples.v1.basic_example` or `my_mission`)
- `--conn`: MAVLink connection string
- `--vehicle`: `drone`, `rover`, or `none` (for DummyVehicle)

**Common options**:
- `--api-version v1` or `v2` (default: v1)
- `--skip-init`: Skip pre-arm wait (use with caution)
- `--skip-rtl`: Do not RTL/land at script end
- `--debug-dump`: Enable verbose vehicle state logging
- `--zmq-identifier`, `--zmq-proxy-server`: For ZMQ scripts
- `-v` / `--verbose`, `--debug`, `-q` / `--quiet`: Logging level

### Config File (beta)

Use `--config` to load options from JSON:

```json
{
  "script": "examples.v1.basic_example",
  "conn": "udp://127.0.0.1:14550",
  "vehicle": "drone",
  "verbose": true
}
```

```bash
python -m aerpawlib --config mission.json
```

CLI arguments override config file values.


---

## Examples Directory

| Path | Description |
|------|-------------|
| `examples/v1/basic_example.py` | Simple square flight |
| `examples/v1/figure_eight.py` | Figure-8 pattern |
| `examples/legacy/basic_runner.py` | Minimal BasicRunner |
| `examples/legacy/squareoff_logging.py` | StateMachine + background logging |
| `examples/legacy/preplanned_trajectory.py` | Load .plan file |
| `examples/legacy/zmq_runner/` | Leader/follower ZMQ |
| `examples/legacy/zmq_preplanned_orbit/` | Multi-drone orbit mission |
| `examples/v2/` | v2 API examples |

---

## Troubleshooting

**Connection timeout**
- Ensure SITL/vehicle is running and MAVLink port is correct
- Try `udpin://` if `udp://` fails (OS-dependent)
- Increase `--conn-timeout`

**No Runner found**
- Script must define a class that subclasses `Runner` (or `BasicRunner`, `StateMachine`)
- For BasicRunner: exactly one method must have `@entrypoint`
- For StateMachine: exactly one state must have `first=True`

**ZMQ scripts fail**
- Start proxy first: `python -m aerpawlib --run-proxy`
- Pass `--zmq-identifier` and `--zmq-proxy-server` to each vehicle

**Vehicle not armable**
- Check GPS fix (3D required)
- In AERPAW: wait for safety pilot to arm
- In SITL: ensure home position is set (wait a few seconds after SITL start)
