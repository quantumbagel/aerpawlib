"""
Geofence v2 Example - Validate waypoints with read_geofence, inside, do_intersect.

Run with:
    aerpawlib --api-version v2 --script examples.v2.geofence_example \
        --vehicle drone --conn udpin://127.0.0.1:14550
"""

import os

from aerpawlib.v2 import BasicRunner, Drone, VectorNED, entrypoint
from aerpawlib.v2.geofence import do_intersect, inside, polygon_edges, read_geofence


def path_inside_geofence(lon1: float, lat1: float, lon2: float, lat2: float, geofence: list) -> bool:
    """Check if both endpoints are inside and path does not cross polygon boundary."""
    if not inside(lon1, lat1, geofence) or not inside(lon2, lat2, geofence):
        return False
    for p1, p2 in polygon_edges(geofence):
        l1, o1 = p1["lon"], p1["lat"]
        l2, o2 = p2["lon"], p2["lat"]
        if do_intersect(lon1, lat1, lon2, lat2, l1, o1, l2, o2):
            return False
    return True


class GeofenceMission(BasicRunner):
    """Mission that validates waypoints against geofence before goto."""

    @entrypoint
    async def run(self, drone: Drone):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        geofence_path = os.path.join(script_dir, "geofence.kml")
        geofence = read_geofence(geofence_path)

        pos = drone.position
        if not inside(pos.lon, pos.lat, geofence):
            print("[example] Current position outside geofence, aborting")
            return

        await drone.takeoff(altitude=10)
        current = drone.position
        target = current + VectorNED(25, 15, 0)

        if not inside(target.lon, target.lat, geofence):
            print("[example] Target outside geofence, skipping goto")
        elif not path_inside_geofence(
            current.lon, current.lat, target.lon, target.lat, geofence
        ):
            print("[example] Path would cross geofence boundary, skipping")
        else:
            await drone.goto_coordinates(target, tolerance=3)

        await drone.land()
        print("[example] Mission complete")
