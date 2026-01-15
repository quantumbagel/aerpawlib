"""
Runner module for aerpawlib.

This module has been moved to version-specific directories:
- aerpawlib.legacy.runner: Original implementation
- aerpawlib.v1.runner: V1 API compatible implementation
- aerpawlib.v2.runner: V2 API implementation

This file provides backward compatibility by re-exporting from legacy.
For new code, consider using aerpawlib.legacy.runner, aerpawlib.v1.runner or aerpawlib.v2.runner.
"""

import warnings

# Issue a deprecation warning for direct imports
warnings.warn(
    "Importing from aerpawlib.runner is deprecated. "
    "Use 'from aerpawlib.legacy.runner import ...', "
    "'from aerpawlib.v1.runner import ...' or "
    "'from aerpawlib.v2.runner import ...' instead.",
    DeprecationWarning,
    stacklevel=2,
)

# Re-export everything from legacy for backward compatibility
from .legacy.runner import *
