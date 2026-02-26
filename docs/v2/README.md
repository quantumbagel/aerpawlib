# aerpawlib v2 API

v2 is a modern, async-first replacement for the v1 API. It eliminates the dual-loop architecture, ThreadSafeValue overhead, and function-attribute runners in favor of a single event loop, native async telemetry, descriptor-based decorators, and built-in safety/connection handling.

## Quick Start

```python
from aerpawlib.v2 import BasicRunner, Drone, VectorNED, entrypoint

class MyMission(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        await drone.takeoff(altitude=10)
        await drone.goto_coordinates(drone.position + VectorNED(20, 0))
        await drone.land()
```

Run with:
```bash
aerpawlib --api-version v2 --script my_script --vehicle drone --conn udpin://127.0.0.1:14550
```

## Architecture

- No background threads for MAVSDK. All commands use direct `await`.
- Subscriptions update plain attributes; no ThreadSafeValue.
- `@entrypoint`, `@state`, `@timed_state`, `@background`, `@at_init` use `__set_name__`.
- Non-blocking commands with `progress`, `cancel()`, and `wait_done()`.

## Key Types

| Type | Purpose |
|------|---------|
| `Coordinate` | WGS84 position (lat, lon, alt) |
| `VectorNED` | NED displacement (north, east, down) in meters |
| `Battery`, `GPSInfo`, `Attitude` | Telemetry dataclasses |
| `VehicleTask` | Non-blocking command with progress |

## Vehicle API

### Connection
```python
vehicle = await Drone.connect("udpin://127.0.0.1:14550")
```

### Commands (blocking)
```python
await drone.takeoff(altitude=10)
await drone.goto_coordinates(target, tolerance=2)
await drone.set_heading(90)
await drone.land()
await drone.return_to_launch()
```

### Non-blocking goto
```python
handle = await drone.goto_coordinates(target, blocking=False)
print(handle.progress)
await handle.wait_done()
```

## Runners

### BasicRunner
Single entry point:
```python
class MyScript(BasicRunner):
    @entrypoint
    async def run(self, drone: Drone):
        ...
```

### StateMachine
States with transitions:
```python
class MyMission(StateMachine):
    @state(name="start", first=True)
    async def start(self, drone: Drone):
        return "fly"

    @state(name="fly")
    async def fly(self, drone: Drone):
        return "land"

    @timed_state(name="hold", duration=5)
    async def hold(self, drone: Drone):
        return "land"

    @background
    async def monitor(self, drone: Drone):
        while True:
            print(drone.position)
            await asyncio.sleep(1)
```