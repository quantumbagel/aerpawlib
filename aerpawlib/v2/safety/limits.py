"""
Safety limits configuration for aerpawlib v2 API.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, TYPE_CHECKING

import yaml

from .types import VehicleType

if TYPE_CHECKING:
    from ..geofence import Polygon


@dataclass
class SafetyLimits:
    """
    Safety limits configuration for vehicle operations.

    These limits are primarily for LOCAL DEVELOPMENT and SITL testing.
    In the AERPAW environment, safety enforcement is handled externally:

    - Battery failsafe: Handled by the AUTOPILOT (RTL/LAND on low battery)
    - Speed/geofence limits: Enforced by the MAVLink FILTER in C-VM
    - Preflight checks: Handled by the PARAMETER CHECKER OEO script

    The SafetyMonitor uses these limits for logging warnings, but does NOT
    enforce RTL or other actions. The limits are also used by validation
    functions to clamp values and provide helpful error messages.
    """
    # Speed limits (for local validation/clamping)
    max_speed: float = 15.0           # m/s horizontal
    max_vertical_speed: float = 5.0   # m/s vertical

    # Battery limits (for warnings only - autopilot handles actual failsafe)
    min_battery_percent: float = 20.0      # warning threshold
    critical_battery_percent: float = 10.0  # critical warning threshold

    # GPS requirements
    require_gps_fix: bool = True
    min_satellites: int = 6

    # Feature toggles
    enable_speed_limits: bool = True
    enable_battery_failsafe: bool = True
    enable_parameter_validation: bool = True
    enable_preflight_checks: bool = True
    auto_clamp_values: bool = True

    def validate(self) -> List[str]:
        """Validate the safety limits configuration."""
        errors = []
        if self.max_speed <= 0:
            errors.append("max_speed must be positive")
        if self.max_vertical_speed <= 0:
            errors.append("max_vertical_speed must be positive")
        if not 0 <= self.min_battery_percent <= 100:
            errors.append("min_battery_percent must be between 0 and 100")
        if not 0 <= self.critical_battery_percent <= 100:
            errors.append("critical_battery_percent must be between 0 and 100")
        if self.critical_battery_percent >= self.min_battery_percent:
            errors.append("critical_battery_percent must be less than min_battery_percent")
        if self.min_satellites < 0:
            errors.append("min_satellites cannot be negative")
        return errors

    @classmethod
    def permissive(cls) -> "SafetyLimits":
        """Create permissive safety limits for advanced users."""
        return cls(
            max_speed=30.0,
            max_vertical_speed=10.0,
            enable_battery_failsafe=False,
            enable_preflight_checks=False,
            auto_clamp_values=False,
        )

    @classmethod
    def restrictive(cls) -> "SafetyLimits":
        """Create restrictive safety limits for beginners or indoor use."""
        return cls(
            max_speed=5.0,
            max_vertical_speed=2.0,
            min_battery_percent=30.0,
            critical_battery_percent=20.0,
            min_satellites=8,
        )

    @classmethod
    def disabled(cls) -> "SafetyLimits":
        """Create safety limits with all checks disabled. Use with caution!"""
        return cls(
            max_speed=100.0,
            max_vertical_speed=50.0,
            enable_speed_limits=False,
            enable_battery_failsafe=False,
            enable_parameter_validation=False,
            enable_preflight_checks=False,
            auto_clamp_values=False,
        )

    @classmethod
    def from_safety_config(cls, config: "SafetyConfig") -> "SafetyLimits":
        """Create SafetyLimits from a SafetyConfig (YAML-based configuration)."""
        return cls(
            max_speed=config.max_speed,
            max_vertical_speed=config.max_speed,
        )


@dataclass
class SafetyConfig:
    """Safety configuration loaded from YAML for the safety checker server."""
    vehicle_type: VehicleType
    max_speed: float
    min_speed: float
    include_geofences: List["Polygon"] = field(default_factory=list)
    exclude_geofences: List["Polygon"] = field(default_factory=list)
    max_altitude: Optional[float] = None
    min_altitude: Optional[float] = None

    @classmethod
    def from_yaml(cls, config_path: str | Path) -> "SafetyConfig":
        """Load safety configuration from a YAML file."""
        from ..geofence import read_geofence

        config_path = Path(config_path)
        config_dir = config_path.parent

        with config_path.open("r") as f:
            config = yaml.safe_load(f)

        required = ["vehicle_type", "max_speed", "min_speed", "include_geofences", "exclude_geofences"]
        for param in required:
            if param not in config:
                raise ValueError(f"Required parameter '{param}' not found in {config_path}")

        try:
            vehicle_type = VehicleType(config["vehicle_type"])
        except ValueError:
            valid_types = [t.value for t in VehicleType]
            raise ValueError(f"Invalid vehicle_type '{config['vehicle_type']}'. Must be one of {valid_types}")

        max_alt, min_alt = None, None
        if vehicle_type == VehicleType.COPTER:
            if "max_alt" not in config:
                raise ValueError("Copter requires 'max_alt' parameter")
            if "min_alt" not in config:
                raise ValueError("Copter requires 'min_alt' parameter")
            max_alt = float(config["max_alt"])
            min_alt = float(config["min_alt"])

        include_geofences = [read_geofence(config_dir / f) for f in config["include_geofences"]]
        exclude_geofences = [read_geofence(config_dir / f) for f in config["exclude_geofences"]]

        return cls(
            vehicle_type=vehicle_type,
            max_speed=float(config["max_speed"]),
            min_speed=float(config["min_speed"]),
            include_geofences=include_geofences,
            exclude_geofences=exclude_geofences,
            max_altitude=max_alt,
            min_altitude=min_alt,
        )


__all__ = ["SafetyLimits", "SafetyConfig"]

