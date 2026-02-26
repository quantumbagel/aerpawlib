"""
DummyVehicle v2 Example - BasicRunner with DummyVehicle for dry-run / CI.

Run with (no SITL required):
    aerpawlib --api-version v2 --script examples.v2.dummy_vehicle_example \
        --vehicle none --conn ""
"""

from aerpawlib.v2 import BasicRunner, DummyVehicle, VectorNED, entrypoint


class DummyMission(BasicRunner):
    """Dry-run mission using DummyVehicle (no hardware)."""

    @entrypoint
    async def run(self, vehicle: DummyVehicle):
        print(f"[example] Position: {vehicle.position}")
        print(f"[example] Home: {vehicle.home_coords}")
        print(f"[example] Battery: {vehicle.battery.level}%")
        # DummyVehicle no-ops: goto_coordinates, takeoff, land do nothing
        target = vehicle.position + VectorNED(20, 10, 0)
        print(f"[example] Would goto {target}")
        await vehicle.goto_coordinates(target)
        print("[example] Dummy mission complete (no actual movement)")
