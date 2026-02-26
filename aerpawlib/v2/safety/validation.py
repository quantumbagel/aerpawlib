"""
Preflight checks for aerpawlib v2.

Integrated into vehicle lifecycle.
"""

from __future__ import annotations

from ..log import LogComponent, get_logger
from ..protocols import VehicleProtocol

logger = get_logger(LogComponent.SAFETY)


class PreflightChecks:
    """Preflight checks run before arm/takeoff."""

    @staticmethod
    async def check_gps_fix(vehicle: VehicleProtocol) -> bool:
        """Require 3D GPS fix."""
        if vehicle.gps.fix_type >= 3:
            logger.debug(f"Preflight: GPS OK (fix_type={vehicle.gps.fix_type}, sats={vehicle.gps.satellites_visible})")
            return True
        logger.warning(f"Preflight: No 3D GPS fix (fix_type={vehicle.gps.fix_type}, sats={vehicle.gps.satellites_visible})")
        return False

    @staticmethod
    async def check_battery(vehicle: VehicleProtocol, min_percent: float = 10.0) -> bool:
        """Require minimum battery."""
        if vehicle.battery.level >= min_percent:
            logger.debug(f"Preflight: Battery OK ({vehicle.battery.level}% >= {min_percent}%)")
            return True
        logger.warning(f"Preflight: Battery {vehicle.battery.level}% below {min_percent}%")
        return False

    @staticmethod
    async def run_all(vehicle: VehicleProtocol) -> bool:
        """Run all preflight checks. Returns True if all pass."""
        logger.info("PreflightChecks: running all checks")
        gps_ok = await PreflightChecks.check_gps_fix(vehicle)
        bat_ok = await PreflightChecks.check_battery(vehicle)
        passed = gps_ok and bat_ok
        logger.info(f"PreflightChecks: gps={gps_ok}, battery={bat_ok}, all_pass={passed}")
        return passed
