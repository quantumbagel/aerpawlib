"""
aerpawlib v2 API - async-first vehicle control.

Modern replacement for v1 with single event loop, native async telemetry,
descriptor-based runners, VehicleTask for progress/cancellation, and
built-in safety/connection handling.

Usage:
    from aerpawlib.v2 import Drone, Coordinate, BasicRunner, entrypoint

    class MyMission(BasicRunner):
        @entrypoint
        async def run(self, drone: Drone):
            await drone.takeoff(altitude=10)
            await drone.goto_coordinates(drone.position + VectorNED(20, 0))
            await drone.land()
"""

from . import constants
from .exceptions import (
    AerpawlibError,
    ArmError,
    CommandError,
    ConnectionTimeoutError,
    HeartbeatLostError,
    InvalidStateError,
    LandingError,
    MultipleInitialStatesError,
    NavigationError,
    NoEntrypointError,
    NoInitialStateError,
    NotArmableError,
    RTLError,
    RunnerError,
    TakeoffError,
)
from .geofence import do_intersect, inside, read_geofence
from .protocols import GPSProtocol, VehicleProtocol
from .types import Attitude, Battery, Coordinate, GPSInfo, VectorNED
from .zmqutil import check_zmq_proxy_reachable, run_zmq_proxy

# Lazy imports for vehicle, runner, safety, aerpaw, external
def __getattr__(name: str):
    if name == "Drone":
        from .vehicle import Drone
        return Drone
    if name == "Rover":
        from .vehicle import Rover
        return Rover
    if name == "Vehicle":
        from .vehicle import Vehicle
        return Vehicle
    if name == "DummyVehicle":
        from .vehicle import DummyVehicle
        return DummyVehicle
    if name in ("Runner", "BasicRunner", "StateMachine", "entrypoint", "state", "timed_state", "background", "at_init"):
        from . import runner
        return getattr(runner, name)
    if name == "AERPAW_Platform":
        from . import aerpaw
        return getattr(aerpaw, "AERPAW_Platform", None)
    if name == "ExternalProcess":
        from . import external
        return getattr(external, "ExternalProcess")
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "AerpawlibError",
    "ArmError",
    "Attitude",
    "Battery",
    "BasicRunner",
    "Runner",
    "check_zmq_proxy_reachable",
    "CommandError",
    "constants",
    "Coordinate",
    "ConnectionTimeoutError",
    "DummyVehicle",
    "Drone",
    "do_intersect",
    "entrypoint",
    "ExternalProcess",
    "GPSInfo",
    "GPSProtocol",
    "HeartbeatLostError",
    "inside",
    "InvalidStateError",
    "LandingError",
    "MultipleInitialStatesError",
    "NavigationError",
    "NoEntrypointError",
    "NoInitialStateError",
    "NotArmableError",
    "read_geofence",
    "RTLError",
    "RunnerError",
    "Rover",
    "run_zmq_proxy",
    "StateMachine",
    "state",
    "timed_state",
    "background",
    "at_init",
    "TakeoffError",
    "Vehicle",
    "VehicleProtocol",
    "VectorNED",
]
