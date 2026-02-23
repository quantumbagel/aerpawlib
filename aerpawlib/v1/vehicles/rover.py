"""
Rover vehicle implementation.
"""

import asyncio

from aerpawlib.log import get_logger, LogComponent
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
    DEFAULT_GOTO_TIMEOUT_S,
)
from aerpawlib.v1.exceptions import (
    NavigationError,
)
from aerpawlib.v1.helpers import (
    wait_for_condition,
    validate_tolerance,
)
from aerpawlib.v1.vehicles.core_vehicle import Vehicle

logger = get_logger(LogComponent.ROVER)


class Rover(Vehicle):
    """
    Rover implementation for ground vehicles.

    Focuses on 2D ground navigation using MAVSDK's action.goto_location.

    Note:
        `target_heading` is currently ignored during movement as the
        MAVLink mission item for navigation usually handles steering.
    """

    def __init__(self, connection_string: str, mavsdk_server_port: int = 50051):
        """
        Initialize the rover.

        Args:
            connection_string (str): MAVLink connection string.
            mavsdk_server_port (int): Port for the embedded mavsdk_server gRPC interface.
                Each Vehicle instance should use a unique port to avoid conflicts.
                Defaults to 50051.
        """
        super().__init__(connection_string, mavsdk_server_port=mavsdk_server_port)

    async def goto_coordinates(
        self,
        coordinates: util.Coordinate,
        tolerance: float = DEFAULT_ROVER_POSITION_TOLERANCE_M,
        target_heading: Optional[float] = None,
        timeout: Optional[float] = DEFAULT_GOTO_TIMEOUT_S,
    ) -> None:
        """
        Make the vehicle go to provided coordinates.

        Args:
            coordinates: Target position
            tolerance: Distance in meters to consider destination reached
            target_heading: Ignored for rovers (they can't strafe)
            timeout: Timeout in seconds for mavsdk action to complete (default: DEFAULT_GOTO_TIMEOUT_S)

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

        try:
            await self._run_on_mavsdk_loop(self._system.action.hold())
            await wait_for_condition(
                lambda: self.mode == "HOLD",
                timeout=5.0,
                poll_interval=POLLING_DELAY_S,
                timeout_message="HOLD mode did not engage within 5s",
            )
        except (ActionError, TimeoutError) as e:
            logger.warning(f"Could not set HOLD mode: {e}")

        self._ready_to_move = lambda _: False

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

            self._ready_to_move = (
                lambda s: coordinates.ground_distance(s.position) <= tolerance
            )

            logger.debug(
                f"Waiting to reach destination (tolerance={tolerance}m)..."
            )
            await wait_for_condition(
                lambda: self._ready_to_move(self),
                poll_interval=POLLING_DELAY_S,
                timeout=timeout,
                timeout_message=f"Rover failed to reach destination {coordinates} within {timeout}s",
            )
            logger.debug(
                f"Arrived at destination, distance: {coordinates.ground_distance(self.position)}m"
            )
        except ActionError as e:
            logger.error(f"Goto failed: {e}")
            raise NavigationError(str(e), original_error=e)
        except TimeoutError as e:
            logger.error(f"Goto timed out: {e}")
            raise NavigationError(str(e), original_error=e)
