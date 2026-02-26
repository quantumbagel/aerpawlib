"""
Enhanced v2 Example - SafetyMonitor, PreflightChecks, ConnectionHandler

Run with:
    aerpawlib --api-version v2 --script examples.v2.enhanced_example \
        --vehicle drone --conn udpin://127.0.0.1:14550
"""

import asyncio

from aerpawlib.v2 import BasicRunner, Drone, VectorNED, entrypoint
from aerpawlib.v2.safety import PreflightChecks, SafetyLimits, SafetyMonitor


class EnhancedMission(BasicRunner):
    """Mission with safety monitor and preflight checks."""

    @entrypoint
    async def run(self, drone: Drone):
        # Preflight
        ok = await PreflightChecks.run_all(drone)
        if not ok:
            print("[example] Preflight checks failed")
            return

        # Safety monitor (warnings only)
        limits = SafetyLimits(
            max_altitude_m=50,
            min_battery_percent=20,
        )
        monitor = SafetyMonitor(limits)

        async def on_violation(event_type: str, message: str):
            print(f"[safety] {event_type}: {message}")

        monitor.on_violation(on_violation)

        await drone.takeoff(altitude=10)
        await drone.goto_coordinates(drone.position + VectorNED(20, 0))
        await drone.land()
        print("[example] Mission complete")
