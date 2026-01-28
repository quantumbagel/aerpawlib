"""
.. include:: ./documentation.md
"""

__version__ = "2.0.0"
__author__ = "John Kesler and Julian Reder"


# Lazy load v1 API only when accessed
# New code should be written using aerpawlib.v1.*, but legacy code uses aerpawlib.*.
# We then lazy load the v1 module on first access.
_v1_loaded = False
_loading = False

def __getattr__(name):
    global _v1_loaded, _loading
    if _loading:
        raise AttributeError(f"module '{__name__}' has no attribute '{name}'")
    if not _v1_loaded:
        _loading = True
        try:
            from . import v1
            # Import all from v1 into globals
            for attr in dir(v1):
                if not attr.startswith('_'):
                    globals()[attr] = getattr(v1, attr)
            _v1_loaded = True
        finally:
            _loading = False
    return globals().get(name)
