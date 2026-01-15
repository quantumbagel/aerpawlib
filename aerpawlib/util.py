"""
Utility module for aerpawlib.

This module has been moved to version-specific directories:
- aerpawlib.legacy.util: Original implementation
- aerpawlib.v1.util: V1 API compatible implementation
- aerpawlib.v2.types: V2 API implementation (types and utilities)

This file provides backward compatibility by re-exporting from legacy.
For new code, consider using aerpawlib.legacy.util, aerpawlib.v1.util or aerpawlib.v2.types.
"""

import warnings

# Issue a deprecation warning for direct imports
warnings.warn(
    "Importing from aerpawlib.util is deprecated. "
    "Use 'from aerpawlib.legacy.util import ...', "
    "'from aerpawlib.v1.util import ...' or "
    "'from aerpawlib.v2.types import ...' instead.",
    DeprecationWarning,
    stacklevel=2,
)

# Re-export everything from legacy for backward compatibility
from .legacy.util import *
