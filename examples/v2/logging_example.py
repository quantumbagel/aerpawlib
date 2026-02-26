"""
Logging v2 Example - Structured logging with get_logger and LogComponent.

Run with:
    aerpawlib --api-version v2 --script examples.v2.logging_example \
        --vehicle drone --conn udpin://127.0.0.1:14550
"""

from aerpawlib.v2 import BasicRunner, Drone, VectorNED, entrypoint
from aerpawlib.v2.log import LogComponent, get_logger

logger = get_logger(LogComponent.ROOT)


class LoggingMission(BasicRunner):
    """Mission with structured logging."""

    @entrypoint
    async def run(self, drone: Drone):
        logger.info("Starting logging example mission")
        logger.debug("Position: %s", drone.position)
        await drone.takeoff(altitude=10)
        logger.info("Takeoff complete")
        target = drone.position + VectorNED(15, 0, 0)
        logger.debug("Goto target: %s", target)
        await drone.goto_coordinates(target)
        logger.info("Goto complete")
        await drone.land()
        logger.info("Mission complete")
