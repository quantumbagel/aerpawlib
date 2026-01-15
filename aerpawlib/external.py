"""
External process utilities module for aerpawlib.

This module has been moved to version-specific directories:
- aerpawlib.legacy.external: Original implementation
- aerpawlib.v1.external: V1 API compatible implementation
- aerpawlib.v2.external: V2 API implementation

This file provides backward compatibility by re-exporting from legacy.
For new code, consider using aerpawlib.legacy.external, aerpawlib.v1.external or aerpawlib.v2.external.
"""

import warnings

# Issue a deprecation warning for direct imports
warnings.warn(
    "Importing from aerpawlib.external is deprecated. "
    "Use 'from aerpawlib.legacy.external import ...', "
    "'from aerpawlib.v1.external import ...' or "
    "'from aerpawlib.v2.external import ...' instead.",
    DeprecationWarning,
    stacklevel=2,
)

# Re-export everything from legacy for backward compatibility
from .legacy.external import *
