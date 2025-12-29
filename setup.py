"""
DEPRECATED: This file is kept for backward compatibility with older pip versions.
Please use pyproject.toml for all package configuration.
"""
import warnings

warnings.warn(
    "setup.py is deprecated. Package configuration has moved to pyproject.toml. "
    "Consider upgrading pip: pip install --upgrade pip",
    DeprecationWarning,
)