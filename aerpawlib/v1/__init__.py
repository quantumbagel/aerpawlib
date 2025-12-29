"""
aerpawlib v1 API - MAVSDK-based vehicle control (API compatible)

This module provides backward compatibility with the original aerpawlib API
while using MAVSDK as the backend instead of DroneKit.

The v1 API provides:
- Vehicle, Drone, Rover classes for vehicle control
- Runner, BasicRunner, StateMachine for script execution
- Coordinate, VectorNED for position/movement handling
- Utility functions for geofencing and waypoint handling

Usage:
    from aerpawlib.v1 import Drone, Coordinate, BasicRunner, entrypoint

    # All existing v1 code works unchanged

@author: Julian Reder (quantumbagel) - MAVSDK adaptation
@author: John Kesler (morzack) - some code from original aerpawlib
"""

from .external import *
from .aerpaw import *
from .zmqutil import *
from .safetyChecker import *
from .util import *
from .vehicle import *
from .runner import *

