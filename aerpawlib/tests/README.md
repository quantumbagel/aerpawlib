# aerpawlib Tests

This directory contains comprehensive tests for all API versions of aerpawlib.

## Directory Structure

```
tests/
├── __init__.py
├── conftest.py          # Shared pytest fixtures and configuration
├── test_api_compatibility.py  # Cross-version compatibility tests
├── legacy/              # Tests for the original DroneKit-based API
│   ├── __init__.py
│   ├── test_util.py     # VectorNED, Coordinate tests
│   └── test_runner.py   # Runner, StateMachine, decorators tests
├── v1/                  # Tests for MAVSDK-based v1 API
│   ├── __init__.py
│   ├── test_util.py     # VectorNED, Coordinate tests
│   └── test_runner.py   # Runner, StateMachine, decorators tests
└── v2/                  # Tests for modern v2 API
    ├── __init__.py
    ├── test_types.py    # VectorNED, Coordinate, Attitude, etc.
    ├── test_runner.py   # Runner, StateMachine, decorators tests
    ├── test_safety.py   # SafetyLimits, validation functions tests
    ├── test_exceptions.py  # Exception hierarchy tests
    └── test_vehicle.py  # CommandHandle, CommandStatus tests
```

## Running Tests

### Prerequisites

Install the development dependencies:

```bash
pip install -e ".[dev]"
```

Or install pytest directly:

```bash
pip install pytest pytest-asyncio
```

### Run All Tests

```bash
# From the project root
pytest aerpawlib/tests/ -v
```

### Run Tests for a Specific API Version

```bash
# Legacy API tests
pytest aerpawlib/tests/legacy/ -v

# v1 API tests
pytest aerpawlib/tests/v1/ -v

# v2 API tests
pytest aerpawlib/tests/v2/ -v
```

### Run Specific Test Files

```bash
# Run utility tests for v2
pytest aerpawlib/tests/v2/test_types.py -v

# Run safety tests
pytest aerpawlib/tests/v2/test_safety.py -v
```

### Run with Coverage

```bash
pytest aerpawlib/tests/ --cov=aerpawlib --cov-report=html
```

## Test Categories

### Unit Tests

Most tests in this directory are unit tests that:
- Don't require MAVSDK or SITL
- Test individual classes and functions in isolation
- Run quickly and reliably

### Integration Tests (Marked)

Some tests require a simulator or real hardware. These are marked:

```python
@pytest.mark.requires_sitl
async def test_actual_flight():
    ...
```

To skip these tests:

```bash
pytest aerpawlib/tests/ -v -m "not requires_sitl"
```

## Adding New Tests

1. Choose the appropriate directory based on API version
2. Follow the existing naming conventions (`test_*.py`)
3. Use pytest fixtures from `conftest.py` when appropriate
4. Add docstrings explaining what each test verifies

### Example Test

```python
import pytest
from aerpawlib.v2.types import Coordinate, VectorNED


class TestCoordinateOperations:
    """Tests for Coordinate arithmetic operations."""

    def test_coordinate_add_vector(self):
        """Test that adding a VectorNED to a Coordinate works correctly."""
        coord = Coordinate(35.0, -78.0, 100)
        vector = VectorNED(100, 50, -10)
        result = coord + vector
        
        assert isinstance(result, Coordinate)
        assert result.latitude > coord.latitude  # Moved north
```

## Test Markers

The following pytest markers are available:

- `@pytest.mark.requires_mavsdk` - Requires MAVSDK installation
- `@pytest.mark.requires_sitl` - Requires SITL simulator running
- `@pytest.mark.integration` - Integration test (may be slow)
- `@pytest.mark.slow` - Slow-running test

