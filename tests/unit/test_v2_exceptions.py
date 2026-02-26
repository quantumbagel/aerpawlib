"""Unit tests for aerpawlib v2 exception types."""

import pytest

from aerpawlib.v2.exceptions import (
    AerpawlibError,
    ConnectionTimeoutError,
    HeartbeatLostError,
    TakeoffError,
    NavigationError,
    LandingError,
    RTLError,
    ArmError,
    NotArmableError,
    NoEntrypointError,
    InvalidStateError,
    NoInitialStateError,
    MultipleInitialStatesError,
)


class TestAerpawlibError:
    """Base exception."""

    def test_message(self):
        e = AerpawlibError("test message")
        assert str(e) == "test message"
        assert e.message == "test message"

    def test_with_original_error(self):
        inner = ValueError("inner")
        e = AerpawlibError("outer", original_error=inner)
        assert "outer" in str(e)
        assert "inner" in str(e)
        assert e.original_error is inner


class TestConnectionExceptions:
    """Connection-related exceptions."""

    def test_connection_timeout(self):
        e = ConnectionTimeoutError(30.0)
        assert "30" in str(e)
        assert e.timeout_seconds == 30.0

    def test_heartbeat_lost(self):
        e = HeartbeatLostError(5.2)
        assert "5.2" in str(e)


class TestCommandExceptions:
    """Command execution exceptions."""

    def test_takeoff_error(self):
        e = TakeoffError("GPS not ready")
        assert "Takeoff" in str(e)
        assert "GPS" in str(e)

    def test_navigation_error(self):
        e = NavigationError("timeout")
        assert "Navigation" in str(e)

    def test_landing_error(self):
        e = LandingError("obstacle")
        assert "Landing" in str(e)

    def test_rtl_error(self):
        e = RTLError("disarmed")
        assert "RTL" in str(e)

    def test_arm_error(self):
        e = ArmError("prearm failed")
        assert "arm" in str(e).lower()


class TestRunnerExceptions:
    """Runner/state machine exceptions."""

    def test_no_entrypoint(self):
        e = NoEntrypointError()
        assert "entrypoint" in str(e).lower()

    def test_invalid_state(self):
        e = InvalidStateError("bad", ["a", "b"])
        assert "bad" in str(e)
        assert "a" in str(e) or "b" in str(e)
        assert e.state_name == "bad"
        assert e.available_states == ["a", "b"]

    def test_no_initial_state(self):
        e = NoInitialStateError()
        assert "initial" in str(e).lower()

    def test_multiple_initial_states(self):
        e = MultipleInitialStatesError()
        assert "Multiple" in str(e)
