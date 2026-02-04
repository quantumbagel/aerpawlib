"""
Pytest configuration and fixtures for aerpawlib tests.
"""

from __future__ import annotations

import asyncio
import time
from typing import AsyncGenerator

import pytest
import pytest_asyncio

from aerpawlib.v1.vehicle import Drone, Rover
from aerpawlib.v1.util import Coordinate, VectorNED

# Constants
DEFAULT_SITL_PORT = 14550
SITL_GPS_TIMEOUT = 120  # Increased for stability

# AERPAW Lake Wheeler site coordinates
LAKE_WHEELER_LAT = 35.727436
LAKE_WHEELER_LON = -78.696587

def pytest_addoption(parser: pytest.Parser) -> None:
    """Add custom command-line options."""
    parser.addoption(
        "--sitl-port",
        action="store",
        default=str(DEFAULT_SITL_PORT),
        help=f"UDP port for SITL connection (default: {DEFAULT_SITL_PORT})",
    )

def pytest_configure(config: pytest.Config) -> None:
    """Additional pytest configuration."""
    pass

def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    """Auto-apply markers based on test directory."""
    for item in items:
        path_str = str(item.path)
        if "/unit/" in path_str:
            item.add_marker(pytest.mark.unit)
        elif "/integration/" in path_str:
            item.add_marker(pytest.mark.integration)

# Utility Fixtures

@pytest.fixture
def origin_coordinate():
    return Coordinate(LAKE_WHEELER_LAT, LAKE_WHEELER_LON, 0)

@pytest.fixture
def nearby_coordinate():
    return Coordinate(LAKE_WHEELER_LAT + 0.0009, LAKE_WHEELER_LON, 0)

@pytest.fixture
def zero_vector():
    return VectorNED(0, 0, 0)

# SITL Connection Fixtures

@pytest.fixture(scope="session")
def sitl_connection_string(request: pytest.FixtureRequest) -> str:
    port = request.config.getoption("--sitl-port")
    return f"udp://127.0.0.1:{port}"

@pytest_asyncio.fixture(scope="function")
async def connected_drone(
    sitl_connection_string: str,
) -> AsyncGenerator[Drone, None]:
    """
    Provide a connected Drone instance for integration testing.

    This fixture:
    1. Connects to SITL in a background thread to avoid hanging the event loop.
    2. Waits for a valid GPS fix.
    3. Resets (reboots) the vehicle after the test.
    """
    # Create drone in a thread because its constructor is blocking
    drone = await asyncio.to_thread(Drone, sitl_connection_string)

    # Wait for GPS fix (fix_type >= 3 = 3D fix)
    start = time.monotonic()
    while time.monotonic() - start < SITL_GPS_TIMEOUT:
        if drone.gps.fix_type >= 3:
            break
        await asyncio.sleep(0.5)
    else:
        drone.close()
        pytest.fail(f"GPS fix not acquired within {SITL_GPS_TIMEOUT}s. Check if SITL is running and has a clear sky view.")

    yield drone

    # Reset SITL between tests by rebooting
    try:
        # We use the internal MAVSDK system to send a reboot command
        # This ensures parameters/missions etc are reset
        await drone._run_on_mavsdk_loop(drone._system.action.reboot())
        # Wait for the command to be sent and vehicle to start rebooting
        await asyncio.sleep(2)
    except Exception as e:
        print(f"Warning: Failed to reboot drone: {e}")
    finally:
        drone.close()

@pytest_asyncio.fixture(scope="function")
async def connected_rover(
    sitl_connection_string: str,
) -> AsyncGenerator[Rover, None]:
    """
    Provide a connected Rover instance for integration testing.
    """
    rover = await asyncio.to_thread(Rover, sitl_connection_string)
    yield rover
    try:
        await rover._run_on_mavsdk_loop(rover._system.action.reboot())
        await asyncio.sleep(2)
    except Exception:
        pass
    finally:
        rover.close()

