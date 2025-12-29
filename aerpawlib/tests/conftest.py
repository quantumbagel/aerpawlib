"""
Pytest configuration and shared fixtures for aerpawlib tests.

This file provides common fixtures and configuration for all test modules.
"""
import pytest
import asyncio
import sys
import os

# Ensure the project root is in the path
project_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)


@pytest.fixture(scope="session")
def event_loop():
    """Create an instance of the default event loop for the test session."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    yield loop
    loop.close()


@pytest.fixture
def sample_coordinate():
    """Provide a sample coordinate for testing."""
    from aerpawlib.v2.types import Coordinate
    return Coordinate(35.7275, -78.6960, 100.0, "Test Location")


@pytest.fixture
def sample_vector():
    """Provide a sample VectorNED for testing."""
    from aerpawlib.v2.types import VectorNED
    return VectorNED(100.0, 50.0, -10.0)


@pytest.fixture
def default_safety_limits():
    """Provide default safety limits for testing."""
    from aerpawlib.v2.safety import SafetyLimits
    return SafetyLimits()


@pytest.fixture
def restrictive_safety_limits():
    """Provide restrictive safety limits for testing."""
    from aerpawlib.v2.safety import SafetyLimits
    return SafetyLimits.restrictive()


@pytest.fixture
def permissive_safety_limits():
    """Provide permissive safety limits for testing."""
    from aerpawlib.v2.safety import SafetyLimits
    return SafetyLimits.permissive()


# Mark tests that require MAVSDK/SITL
def pytest_configure(config):
    """Configure custom pytest markers."""
    config.addinivalue_line(
        "markers", "requires_mavsdk: mark test as requiring MAVSDK installation"
    )
    config.addinivalue_line(
        "markers", "requires_sitl: mark test as requiring SITL simulator"
    )
    config.addinivalue_line(
        "markers", "integration: mark test as an integration test"
    )
    config.addinivalue_line(
        "markers", "slow: mark test as slow running"
    )

