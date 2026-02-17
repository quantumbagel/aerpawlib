# aerpawlib Examples

> **Documentation**: See [docs/USER_GUIDE.md](../docs/USER_GUIDE.md) for workflows and [docs/README.md](../docs/README.md) for the full documentation index.

## Squareoff with Logging

`squareoff_logging.py` is an example aerpawlib script that will fly a drone in
a 10m by 10m square, while using the `StateMachine`'s `background` utility to
continually log the drone's position to a file. This example is intended to
demonstrate how to write a dynamic state machine as well as use the
`background` tool.

This script can be run on either a drone or a rover. To run it, use aerpawlib:

```
python -m aerpawlib --conn ... --script squareoff_logging --vehicle drone
```

When run, it will continually output positional data to a log file that can be
specified using the `--output` param. The sample rate can be changed using
`--samplerate`. If no file is specified, it will default to outputting to a
file named `GPS_DATA_YYYY-MM-DD_HH:MM:SS.csv`. This output format is below:

```
line num,lon,lat,alt,battery voltage,timestamp,GPS fix type,# visible sats

timestamp format: YYYY-MM-DD HH:MM:SS.ssssss
```

A flowchart of the various states is below:

```
┌───────┐drone ┌──────────┐
│ start ├──────► take_off │
└───┬───┘      └─────┬────┘
    │                │
    ├────────────────┘
    │
┌───▼───────┐
│ leg_north ├───────────┐
│           │           │
│ leg_west  │           │
│           │       ┌───▼─────────┐
│ leg_south ◄───────┤ at_position │
│           │pick   └───┬──┬────▲─┘
│ leg_east  │based on   │  └────┘sleep 5s
└───────────┘current_leg│
                    ┌───▼────┐drone ┌──────┐
                    │ finish ├──────► land │
                    └────────┘      └──────┘
```

The centerpiece of this script, aside from the background logging, is the
dynamic state changing. By default, the only state stored by a `StateMachine`
is the current state *function* -- individually, each function can be
considered to be stateless (with side effects affecting the vehicle).

To introduce additional state into the state machine, all you have to do is add
method variables to your `StateMachine` derived object. This script uses
`_legs: List[str]` and `_current_leg: int` to do that. `_current_leg` is
altered and interpreted by the `at_position` state to then dynamically pick the
next state on the fly.

## Preplanned Trajectory

`preplanned_trajectory.py` is an example aerpawlib script that makes a vehicle
fly between different waypoints read from a `.plan` file generated using
`QGroundControl`. This example is a good starting point for experiments that
make use of non-dynamic flight plans.

This script can be run on either a drone or rover. To run it, use aerpawlib:

```
python -m aerpawlib --conn ... --script preplanned_trajectory --vehicle drone \
    --file <.plan file to use>
```

When run, it will load in the `.plan` file located at the path specified by
`--plan` and then send the drone to each waypoint specified sequentially in it.

A flowchart of the various states is below:

```
┌──────────┐
│ take_off │
└──────┬───┘
       │
       ├────────────────────────────────────────┐
       │                                        │
┌──────▼────────┐    ┌────────────┐     ┌───────┴─────┐
│ next_waypoint ├────► in_transit ├─────► at_waypoint │
└──────┬────────┘    └────────────┘     └─────────────┘
       │
    ┌──▼──┐
    │ rtl │
    └─────┘
```

This script includes several states that can be used as hooks to introduce
custom logic that runs during various parts of the flight plan.

`in_transit` is a function that will be called once after the script picks a
waypoint for the vehicle to go to, at which point it blocks until the vehicle
arrives. Custom logic can be added before the `await` statement, at which point
the script blocks while waiting for the drone to finish moving.

To add custom logic that waits for the drone to finish moving, you can
continually poll `drone.done_moving()`.

`at_waypoint` is a timed state that is called once the vehicle arrives at a
waypoint. As a timed state, it is guaranteed to be called repeatedly for at
least `duration` seconds.

---

## Directory Structure

```
examples/
├── v1/                      # v1 API examples (MAVSDK-based, DroneKit-compatible)
│   ├── basic_example.py         # Simple square flight pattern
│   └── figure_eight.py          # Figure-8 pattern with waypoints
├── v2/                      # v2 API examples (modern, async-first)
│   ├── basic_example.py         # Simple mission with telemetry
│   ├── command_handle_example.py # Non-blocking command execution
│   ├── enhanced_example.py      # Advanced features
│   ├── state_machine_example.py # StateMachine pattern
│   └── test_runner.py           # Test runner
└── legacy/                  # Legacy examples (compatible with v1 runner)
    ├── basic_runner.py          # Minimal BasicRunner
    ├── circle.py               # Circular flight
    ├── squareoff_logging.py    # StateMachine + background logging
    ├── preplanned_trajectory.py # Load .plan file
    ├── external_runner.py       # ExternalProcess usage
    ├── hide_rover.py
    ├── zmq_runner/              # Leader/follower ZMQ
    └── zmq_preplanned_orbit/    # Multi-drone orbit mission
```

## Running Examples

### v1 API Examples

```bash
# Basic square flight
python -m aerpawlib --api-version v1 --script examples.v1.basic_example \
    --vehicle drone --conn udp:127.0.0.1:14550

# Figure-8 pattern
python -m aerpawlib --api-version v1 --script examples.v1.figure_eight \
    --vehicle drone --conn udp:127.0.0.1:14550
```

### Legacy Examples (v1 runner)

```bash
# Basic runner
python -m aerpawlib --script examples.legacy.basic_runner \
    --vehicle drone --conn udp:127.0.0.1:14550

# Squareoff with logging
python -m aerpawlib --script examples.legacy.squareoff_logging \
    --vehicle drone --conn udp:127.0.0.1:14550

# Preplanned trajectory from .plan file
python -m aerpawlib --script examples.legacy.preplanned_trajectory \
    --vehicle drone --conn udp:127.0.0.1:14550 --plan mission.plan

# Circle flight
python -m aerpawlib --script examples.legacy.circle \
    --vehicle drone --conn udp:127.0.0.1:14550
```

### v2 API Examples

```bash
# Basic example
python -m aerpawlib --api-version v2 --script examples.v2.basic_example \
    --vehicle drone --conn udp:127.0.0.1:14550

# State machine example
python -m aerpawlib --api-version v2 --script examples.v2.state_machine_example \
    --vehicle drone --conn udp:127.0.0.1:14550

# Non-blocking command handles
python -m aerpawlib --api-version v2 --script examples.v2.command_handle_example \
    --vehicle drone --conn udp:127.0.0.1:14550
```


# Legacy Documentation

The following documentation is for legacy examples (deprecated, use v1 or v2 instead):

## Squareoff with Logging

`squareoff_logging.py` is an example aerpawlib script that will fly a drone in
a 10m by 10m square, while using the `StateMachine`'s `background` utility to
continually log the drone's position to a file. This example is intended to
demonstrate how to write a dynamic state machine as well as use the
`background` tool.

This script can be run on either a drone or a rover. To run it, use aerpawlib:

```
python -m aerpawlib --conn ... --script squareoff_logging --vehicle drone
```

When run, it will continually output positional data to a log file that can be
specified using the `--output` param. The sample rate can be changed using
`--samplerate`. If no file is specified, it will default to outputting to a
file named `GPS_DATA_YYYY-MM-DD_HH:MM:SS.csv`. This output format is below:

```
line num,lon,lat,alt,battery voltage,timestamp,GPS fix type,# visible sats

timestamp format: YYYY-MM-DD HH:MM:SS.ssssss
```

A flowchart of the various states is below:

```
┌───────┐drone ┌──────────┐
│ start ├──────► take_off │
└───┬───┘      └─────┬────┘
    │                │
    ├────────────────┘
    │
┌───▼───────┐
│ leg_north ├───────────┐
│           │           │
│ leg_west  │           │
│           │       ┌───▼─────────┐
│ leg_south ◄───────┤ at_position │
│           │pick   └───┬──┬────▲─┘
│ leg_east  │based on   │  └────┘sleep 5s
└───────────┘current_leg│
                    ┌───▼────┐drone ┌──────┐
                    │ finish ├──────► land │
                    └────────┘      └──────┘
```

The centerpiece of this script, aside from the background logging, is the
dynamic state changing. By default, the only state stored by a `StateMachine`
is the current state *function* -- individually, each function can be
considered to be stateless (with side effects affecting the vehicle).

To introduce additional state into the state machine, all you have to do is add
method variables to your `StateMachine` derived object. This script uses
`_legs: List[str]` and `_current_leg: int` to do that. `_current_leg` is
altered and interpreted by the `at_position` state to then dynamically pick the
next state on the fly.

## Preplanned Trajectory

`preplanned_trajectory.py` is an example aerpawlib script that makes a vehicle
fly between different waypoints read from a `.plan` file generated using
`QGroundControl`. This example is a good starting point for experiments that
make use of non-dynamic flight plans.

This script can be run on either a drone or rover. To run it, use aerpawlib:

```
python -m aerpawlib --conn ... --script preplanned_trajectory --vehicle drone \
    --file <.plan file to use>
```

When run, it will load in the `.plan` file located at the path specified by
`--plan` and then send the drone to each waypoint specified sequentially in it.

A flowchart of the various states is below:

```
┌──────────┐
│ take_off │
└──────┬───┘
       │
       ├────────────────────────────────────────┐
       │                                        │
┌──────▼────────┐    ┌────────────┐     ┌───────┴─────┐
│ next_waypoint ├────► in_transit ├─────► at_waypoint │
└──────┬────────┘    └────────────┘     └─────────────┘
       │
    ┌──▼──┐
    │ rtl │
    └─────┘
```

This script includes several states that can be used as hooks to introduce
custom logic that runs during various parts of the flight plan.

`in_transit` is a function that will be called once after the script picks a
waypoint for the vehicle to go to, at which point it blocks until the vehicle
arrives. Custom logic can be added before the `await` statement, at which point
the script blocks while waiting for the drone to finish moving.

To add custom logic that waits for the drone to finish moving, you can
continually poll `drone.done_moving()`.

`at_waypoint` is a timed state that is called once the vehicle arrives at a
waypoint. As a timed state, it is guaranteed to be called repeatedly for at
least `duration` seconds.
