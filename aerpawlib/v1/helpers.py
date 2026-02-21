"""
Helper utilities for aerpawlib v1.

Common patterns extracted to reduce code duplication and improve maintainability.

@author: Julian Reder
"""

import asyncio
from typing import Callable, Optional, TypeVar, Any

from .constants import POLLING_DELAY_S, MIN_POSITION_TOLERANCE_M, MAX_POSITION_TOLERANCE_M



T = TypeVar("T")


async def wait_for_condition(
    condition: Callable[[], bool],
    timeout: Optional[float] = None,
    poll_interval: float = POLLING_DELAY_S,
    timeout_message: str = "Operation timed out",
) -> bool:
    """
    Wait for a condition to become true.

    This replaces the pattern:
        while not condition():
            await asyncio.sleep(POLLING_DELAY)

    Args:
        condition: Callable that returns True when condition is met
        timeout: Maximum time to wait in seconds, None for no timeout
        poll_interval: Time between condition checks
        timeout_message: Message for TimeoutError if timeout occurs

    Returns:
        True if condition was met

    Raises:
        TimeoutError: If timeout is specified and exceeded
    """
    import time

    start_time = time.time()

    while not condition():
        if timeout is not None and (time.time() - start_time) > timeout:
            raise TimeoutError(timeout_message)
        await asyncio.sleep(poll_interval)

    return True


async def wait_for_value_change(
    getter: Callable[[], T],
    target_value: T,
    timeout: Optional[float] = None,
    poll_interval: float = POLLING_DELAY_S,
    timeout_message: str = "Timeout waiting for value change",
) -> T:
    """
    Wait for a value to change to a specific target.

    Args:
        getter: Callable that returns the current value
        target_value: The value to wait for
        timeout: Maximum time to wait in seconds
        poll_interval: Time between checks
        timeout_message: Message for TimeoutError

    Returns:
        The final value (should equal target_value)

    Raises:
        TimeoutError: If timeout is exceeded
    """
    await wait_for_condition(
        lambda: getter() == target_value,
        timeout=timeout,
        poll_interval=poll_interval,
        timeout_message=timeout_message,
    )
    return getter()


def validate_tolerance(
    tolerance: float, param_name: str = "tolerance"
) -> float:
    """
    Validate a tolerance value is within acceptable bounds.

    This is a client-side sanity check for navigation tolerances (e.g. how
    close the vehicle must get to a waypoint before considering it reached).
    It is *not* related to mission safety constraints — altitude and speed
    limits are the responsibility of the SafetyCheckerServer, which enforces
    mission-specific bounds loaded from the vehicle YAML config.

    Args:
        tolerance: The tolerance value to validate
        param_name: Name of parameter for error message

    Returns:
        The validated tolerance value

    Raises:
        ValueError: If tolerance is out of acceptable range
    """

    if tolerance < MIN_POSITION_TOLERANCE_M:
        raise ValueError(
            f"{param_name} must be at least {MIN_POSITION_TOLERANCE_M}m, got {tolerance}m"
        )
    if tolerance > MAX_POSITION_TOLERANCE_M:
        raise ValueError(
            f"{param_name} must be at most {MAX_POSITION_TOLERANCE_M}m, got {tolerance}m"
        )
    return tolerance



def normalize_heading(heading: float) -> float:
    """
    Normalize a heading to 0-360 range.

    Args:
        heading: Heading in degrees (can be any value)

    Returns:
        Heading normalized to [0, 360) range
    """
    return heading % 360


def heading_difference(heading1: float, heading2: float) -> float:
    """
    Calculate the minimum angular difference between two headings.

    Args:
        heading1: First heading in degrees
        heading2: Second heading in degrees

    Returns:
        The minimum angular difference (0-180 degrees)
    """
    diff = abs(normalize_heading(heading1) - normalize_heading(heading2))
    return min(diff, 360 - diff)


class ThreadSafeValue:
    """
    A simple thread-safe wrapper for values that may be accessed from multiple threads.

    Uses a lock to ensure atomic read/write operations.
    """

    def __init__(self, initial_value: Any = None):
        import threading

        self._value = initial_value
        self._lock = threading.Lock()

    def get(self) -> Any:
        """Get the current value."""
        with self._lock:
            return self._value

    def set(self, value: Any) -> None:
        """Set a new value."""
        with self._lock:
            self._value = value

    def compare_and_set(self, expected: Any, new_value: Any) -> bool:
        """
        Atomically set value if current value equals expected.

        Args:
            expected: The expected current value
            new_value: The new value to set if expected matches

        Returns:
            True if the value was updated, False otherwise
        """
        with self._lock:
            if self._value == expected:
                self._value = new_value
                return True
            return False
