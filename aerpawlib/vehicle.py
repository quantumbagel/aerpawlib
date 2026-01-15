"""
Vehicle module for aerpawlib.

This module has been moved to version-specific directories:
- aerpawlib.legacy.vehicle: Original implementation
- aerpawlib.v1.vehicle: V1 API compatible implementation
- aerpawlib.v2.vehicle: V2 API implementation

This file provides backward compatibility by re-exporting from legacy.
For new code, consider using aerpawlib.legacy.vehicle, aerpawlib.v1.vehicle or aerpawlib.v2.vehicle.
"""

import warnings

# Issue a deprecation warning for direct imports
warnings.warn(
    "Importing from aerpawlib.vehicle is deprecated. "
    "Use 'from aerpawlib.legacy.vehicle import ...', "
    "'from aerpawlib.v1.vehicle import ...' or "
    "'from aerpawlib.v2.vehicle import ...' instead.",
    DeprecationWarning,
    stacklevel=2,
)

# Re-export everything from legacy for backward compatibility
from .legacy.vehicle import *
