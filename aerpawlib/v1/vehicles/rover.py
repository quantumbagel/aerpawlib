"""
Rover vehicle implementation.
"""

import logging
import time
from typing import Optional

try:
    from mavsdk.action import ActionError
except ImportError:
    ActionError = Exception

from aerpawlib.v1 import util
from aerpawlib.v1.constants import (
    POLLING_DELAY_S,
    DEFAULT_ROVER_POSITION_TOLERANCE_M,
)
from aerpawlib.v1.exceptions import (
    NavigationError,
)
from aerpawlib.v1.helpers import (
    wait_for_condition,
    validate_tolerance,
)
from aerpawlib.v1.vehicles.core_vehicle import Vehicle

logger = logging.getLogger(__name__)


class Rover(Vehicle):
    """
    Rover vehicle type. Implements all functionality that AERPAW's rovers
    expose to user scripts, which includes basic movement control (going to
    coords).

    `target_heading` is ignored for rovers, as they can't strafe.
    """

    async def goto_coordinates(
        self,
        coordinates: util.Coordinate,
        tolerance: float = DEFAULT_ROVER_POSITION_TOLERANCE_M,
        target_heading: Optional[float] = None,
    ) -> None:
        """
        Make the vehicle go to provided coordinates.

        Args:
            coordinates: Target position
            tolerance: Distance in meters to consider destination reached
            target_heading: Ignored for rovers (they can't strafe)

        Raises:
            ValueError: If tolerance is out of acceptable range
            NavigationError: If navigation command fails
        """
        validate_tolerance(tolerance, "tolerance")

        logger.debug(
            f"goto_coordinates(lat={coordinates.lat}, lon={coordinates.lon}, "
            f"tolerance={tolerance}, target_heading={target_heading}) called"
        )
        await self.await_ready_to_move()
        self._ready_to_move = lambda self: False

        if self._mission_start_time is None:
            self._mission_start_time = time.time()

        try:
            logger.debug(
                f"Navigating to: lat={coordinates.lat}, lon={coordinates.lon}"
            )
            await self._run_on_mavsdk_loop(
                self._system.action.goto_location(
                    coordinates.lat,
                    coordinates.lon,
                    self.home_amsl,  # Rovers use home altitude
                    0,  # Heading
                )
            )

            def at_coords(self) -> bool:
                return coordinates.ground_distance(self.position) <= tolerance

            self._ready_to_move = at_coords

            logger.debug(
                f"Waiting to reach destination (tolerance={tolerance}m)..."
            )
            await wait_for_condition(
                lambda: at_coords(self),
                poll_interval=POLLING_DELAY_S,
                timeout=300,
                timeout_message=f"Rover failed to reach destination {coordinates} within 300s",
            )
            logger.debug(
                f"Arrived at destination, distance: {coordinates.ground_distance(self.position)}m"
            )
        except ActionError as e:
            logger.error(f"Goto failed: {e}")
            raise NavigationError(str(e), original_error=e)
