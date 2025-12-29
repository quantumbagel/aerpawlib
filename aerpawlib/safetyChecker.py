"""
Safety checker module for aerpawlib.

This module has been moved to version-specific directories:
- aerpawlib.legacy.safetyChecker: Original DroneKit-based implementation
- aerpawlib.v1.safetyChecker: MAVSDK-based implementation (v1 API compatible)
- aerpawlib.v2.safety_checker: Modernized async-first implementation

This file provides backward compatibility by re-exporting from v1.
For new code, consider using aerpawlib.v2.safety_checker for async support.
"""

import warnings

# Issue a deprecation warning for direct imports
warnings.warn(
    "Importing from aerpawlib.safetyChecker is deprecated. "
    "Use 'from aerpawlib.v1.safetyChecker import ...' or "
    "'from aerpawlib.v2.safety_checker import ...' instead.",
    DeprecationWarning,
    stacklevel=2
)

# Re-export everything from v1 for backward compatibility
from v1.safetyChecker import *