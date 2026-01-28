"""
AERPAW platform integration module for aerpawlib.

This module has been moved to version-specific directories:
- aerpawlib.v1.aerpaw: V1 API compatible implementation
- aerpawlib.v2.aerpaw: Modernized async-first implementation

This file provides backward compatibility by re-exporting from v1.
For new code, consider using aerpawlib.v2.aerpaw for async support.
"""

import warnings

# Issue a deprecation warning for direct imports
warnings.warn(
    "Importing from aerpawlib.aerpaw is deprecated. "
    "Use 'from aerpawlib.v1.aerpaw import ...' or "
    "'from aerpawlib.v2.aerpaw import ...' instead.",
    DeprecationWarning,
    stacklevel=2,
)

# Re-export everything from v1 for backward compatibility
from .v1.aerpaw import *
