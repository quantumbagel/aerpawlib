# aerpawlib v2 Safety Model

## Overview

v2 provides built-in safety components that warn and monitor but do not enforce. Enforcement stays in the autopilot and C-VM (Controller VM) on AERPAW.

## SafetyLimits

Dataclass for altitude, speed, battery, and geofence limits:

```python
from aerpawlib.v2.safety import SafetyLimits

limits = SafetyLimits(
    max_altitude_m=50,
    min_altitude_m=5,
    max_speed_m_s=10,
    min_battery_percent=20,
)
```

## SafetyMonitor

Emits events and logs when limits are exceeded. Does not block or enforce.

```python
from aerpawlib.v2.safety import SafetyMonitor, SafetyLimits

monitor = SafetyMonitor(limits)

async def on_violation(event_type: str, message: str):
    print(f"Safety: {event_type} - {message}")

monitor.on_violation(on_violation)
await monitor.check_altitude(vehicle.position)
await monitor.check_battery(vehicle.battery)
```

Callbacks are awaited always.

## SafetyCheckerClient

Optional async ZMQ client for external geofence validation (C-VM SafetyCheckerServer):

```python
from aerpawlib.v2.safety import SafetyCheckerClient

client = SafetyCheckerClient(addr="192.168.32.25", port=14580)
ok, msg = await client.validate_waypoint(current, next_loc)
```

## ConnectionHandler

Single authority for connection state and heartbeat. Protocol-based (depends on `VehicleProtocol`).

- Starts monitoring **after** first telemetry or short delay (avoids false "heartbeat lost").
- On disconnect: notify OEO, trigger callbacks, exit. No auto-reconnect in production.
- Uses `loop.add_signal_handler()` for async-safe SIGINT/SIGTERM where available.

## PreflightChecks

Integrated into vehicle lifecycle:

```python
from aerpawlib.v2.safety import PreflightChecks

ok = await PreflightChecks.run_all(vehicle)
# Checks: GPS 3D fix, minimum battery
```

## AERPAW-Specific Behavior

- Connection loss = non-recoverable. Abort experiment and notify OEO.
- MAVLink filter can sever connection on safety violations. Do not attempt reconnect.
- Battery failsafe, speed limits, geofence enforced by autopilot/C-VM, not by the library.
- OEO client used for disconnect and critical notifications only.
