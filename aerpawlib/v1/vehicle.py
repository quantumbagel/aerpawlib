"""
Vehicle module with MAVSDK-based implementation.

This module now aggregates the split implementations:
- core_vehicle.py: Base Vehicle class and helpers
- drone.py: Drone class
- rover.py: Rover class
"""

# Re-export from split files
from aerpawlib.v1.vehicles.core_vehicle import (
    Vehicle,
    DummyVehicle,
    _BatteryCompat,
    _GPSInfoCompat,
    _AttitudeCompat,
    _VersionCompat,
)
from aerpawlib.v1.vehicles.drone import Drone
from aerpawlib.v1.vehicles.rover import Rover

__all__ = [
    "Vehicle",
    "DummyVehicle",
    "Drone",
    "Rover",
    "_BatteryCompat",
    "_GPSInfoCompat",
    "_AttitudeCompat",
    "_VersionCompat",
]
