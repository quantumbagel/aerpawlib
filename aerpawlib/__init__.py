"""
.. include:: ./documentation.md
"""

__version__ = "2.0.0"
__author__ = "John Kesler and Julian Reder"


# Lazy load legacy API only when accessed
# In the legacy API, you imported from aerpawlib.*
# Now that it is aerpawlib.v1.* or aerpawlib.v2.*, we only load the legacy module
# when something from it is accessed.
_legacy_loaded = False

def __getattr__(name):
    global _legacy_loaded
    if not _legacy_loaded:
        from . import legacy
        # Import all from legacy into globals
        for attr in dir(legacy):
            if not attr.startswith('_'):
                globals()[attr] = getattr(legacy, attr)
        _legacy_loaded = True
    return globals().get(name)
