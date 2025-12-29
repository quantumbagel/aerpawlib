"""
Safety checker for aerpawlib v2 API.

NOTE: This module is now a re-export wrapper. All safety features have been
consolidated into safety.py. Import from there instead:

    from aerpawlib.v2.safety import SafetyCheckerClient, SafetyCheckerServer, ...

Or use the main package exports:

    from aerpawlib.v2 import SafetyCheckerClient, SafetyCheckerServer, ...

This module is kept for backward compatibility.
"""
from __future__ import annotations

import warnings

# Re-export everything from safety.py for backward compatibility
from .safety import (RequestType, SERVER_STATUS_REQ, SafetyCheckerClient, SafetyCheckerServer, SafetyConfig,
                     VALIDATE_CHANGE_SPEED_REQ, VALIDATE_LANDING_REQ, VALIDATE_TAKEOFF_REQ, VALIDATE_WAYPOINT_REQ,
                     ValidationResult, VehicleType, deserialize_msg, serialize_msg, serialize_request,
                     serialize_response)  # Enums; Configuration; Result types; Client/Server; Backward compatibility constants; Backward compatibility functions


def _warn_deprecated():
    warnings.warn(
        "Importing from aerpawlib.v2.safety_checker is deprecated. "
        "Import from aerpawlib.v2.safety or aerpawlib.v2 instead.",
        DeprecationWarning,
        stacklevel=3
    )


__all__ = [
    # Enums
    "RequestType",
    "VehicleType",
    # Configuration
    "SafetyConfig",
    # Result types
    "ValidationResult",
    # Client/Server
    "SafetyCheckerClient",
    "SafetyCheckerServer",
    # Backward compatibility
    "SERVER_STATUS_REQ",
    "VALIDATE_WAYPOINT_REQ",
    "VALIDATE_CHANGE_SPEED_REQ",
    "VALIDATE_TAKEOFF_REQ",
    "VALIDATE_LANDING_REQ",
    "serialize_request",
    "serialize_response",
    "serialize_msg",
    "deserialize_msg",
]


# Main entry point for running as a module
if __name__ == "__main__":
    from argparse import ArgumentParser

    parser = ArgumentParser(
        description="safetyChecker - Launch a safety checker server"
    )
    parser.add_argument(
        "--port",
        help="Port for communication between client and server",
        required=True,
        type=int
    )
    parser.add_argument(
        "--vehicle_config",
        help="Path to YAML file containing geofences and vehicle constraints",
        required=True,
    )
    args, _ = parser.parse_known_args()

    # This call blocks
    config = SafetyConfig.from_yaml(args.vehicle_config)
    server = SafetyCheckerServer(config, port=args.port)
    server.start_server()

