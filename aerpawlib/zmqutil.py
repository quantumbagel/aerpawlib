"""
ZMQ utilities module for aerpawlib.

This module has been moved to version-specific directories:
- aerpawlib.legacy.zmqutil: Original implementation
- aerpawlib.v1.zmqutil: V1 API compatible implementation
- aerpawlib.v2.zmqutil: Modernized async-first implementation

This file provides backward compatibility by re-exporting from legacy.
For new code, consider using aerpawlib.v2.zmqutil for async support.
"""

import warnings

# Issue a deprecation warning for direct imports
warnings.warn(
    "Importing from aerpawlib.zmqutil is deprecated. "
    "Use 'from aerpawlib.legacy.zmqutil import ...', "
    "'from aerpawlib.v1.zmqutil import ...' or "
    "'from aerpawlib.v2.zmqutil import ...' instead.",
    DeprecationWarning,
    stacklevel=2,
)

# Re-export everything from legacy for backward compatibility
from .legacy.zmqutil import *
