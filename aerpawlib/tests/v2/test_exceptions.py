"""
Tests for the v2 aerpawlib exceptions module.

These tests verify the exception hierarchy and error handling.
"""
import pytest
import sys
import os

# Add the parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))

from aerpawlib.v2.exceptions import (
    ErrorSeverity,
    AerpawlibError,
    ConnectionError,
    ConnectionTimeoutError,
    HeartbeatLostError,
    ReconnectionError,
    CommandError,
    ArmError,
    DisarmError,
    TakeoffError,
    TakeoffTimeoutError,
    LandingError,
    LandingTimeoutError,
    NavigationError,
    GotoTimeoutError,
    ModeChangeError,
    OffboardError,
    TimeoutError,
    AbortError,
    UserAbortError,
    NotArmableError,
    CommandCancelledError,
    SafetyError,
    GeofenceViolationError,
    SpeedLimitExceededError,
    ParameterValidationError,
    PreflightCheckError,
)


class TestErrorSeverity:
    """Tests for ErrorSeverity enum."""

    def test_severity_levels_exist(self):
        """Test that all severity levels exist."""
        assert ErrorSeverity.WARNING
        assert ErrorSeverity.ERROR
        assert ErrorSeverity.CRITICAL
        assert ErrorSeverity.FATAL


class TestAerpawlibError:
    """Tests for the base AerpawlibError class."""

    def test_basic_error(self):
        """Test creating a basic error."""
        error = AerpawlibError("Test error message")
        assert str(error) == "[ERROR] Test error message"
        assert error.message == "Test error message"
        assert error.severity == ErrorSeverity.ERROR
        assert error.recoverable is True
        assert error.details == {}

    def test_error_with_severity(self):
        """Test creating an error with custom severity."""
        error = AerpawlibError("Critical issue", severity=ErrorSeverity.CRITICAL)
        assert error.severity == ErrorSeverity.CRITICAL
        assert "CRITICAL" in str(error)

    def test_error_with_details(self):
        """Test creating an error with details."""
        error = AerpawlibError(
            "Test error",
            details={"code": 42, "component": "test"}
        )
        assert error.details["code"] == 42
        assert error.details["component"] == "test"
        assert "code=42" in str(error)

    def test_error_not_recoverable(self):
        """Test creating a non-recoverable error."""
        error = AerpawlibError("Fatal error", recoverable=False)
        assert error.recoverable is False


class TestConnectionErrors:
    """Tests for connection-related exceptions."""

    def test_connection_error(self):
        """Test ConnectionError."""
        error = ConnectionError("Failed to connect", address="localhost:14540")
        assert isinstance(error, AerpawlibError)
        assert error.address == "localhost:14540"
        assert "address" in error.details

    def test_connection_timeout_error(self):
        """Test ConnectionTimeoutError."""
        error = ConnectionTimeoutError("Connection timed out", timeout=30)
        assert isinstance(error, ConnectionError)
        assert error.timeout == 30

    def test_heartbeat_lost_error(self):
        """Test HeartbeatLostError."""
        error = HeartbeatLostError("Lost heartbeat")
        assert isinstance(error, ConnectionError)

    def test_reconnection_error(self):
        """Test ReconnectionError."""
        error = ReconnectionError("Reconnection failed", attempt=3, max_attempts=5)
        assert isinstance(error, ConnectionError)
        assert error.attempt == 3
        assert error.max_attempts == 5


class TestCommandErrors:
    """Tests for command-related exceptions."""

    def test_command_error(self):
        """Test CommandError."""
        error = CommandError("Command failed")
        assert isinstance(error, AerpawlibError)

    def test_arm_error(self):
        """Test ArmError."""
        error = ArmError("Failed to arm")
        assert isinstance(error, CommandError)

    def test_disarm_error(self):
        """Test DisarmError."""
        error = DisarmError("Failed to disarm")
        assert isinstance(error, CommandError)

    def test_takeoff_error(self):
        """Test TakeoffError."""
        error = TakeoffError("Takeoff failed")
        assert isinstance(error, CommandError)

    def test_takeoff_timeout_error(self):
        """Test TakeoffTimeoutError."""
        error = TakeoffTimeoutError("Takeoff timed out")
        assert isinstance(error, TakeoffError)

    def test_landing_error(self):
        """Test LandingError."""
        error = LandingError("Landing failed")
        assert isinstance(error, CommandError)

    def test_landing_timeout_error(self):
        """Test LandingTimeoutError."""
        error = LandingTimeoutError("Landing timed out")
        assert isinstance(error, LandingError)


class TestNavigationErrors:
    """Tests for navigation-related exceptions."""

    def test_navigation_error(self):
        """Test NavigationError."""
        error = NavigationError("Navigation failed")
        assert isinstance(error, CommandError)

    def test_goto_timeout_error(self):
        """Test GotoTimeoutError."""
        error = GotoTimeoutError("Goto timed out")
        assert isinstance(error, NavigationError)

    def test_mode_change_error(self):
        """Test ModeChangeError."""
        error = ModeChangeError("Mode change failed")
        assert isinstance(error, CommandError)

    def test_offboard_error(self):
        """Test OffboardError."""
        error = OffboardError("Offboard mode failed")
        assert isinstance(error, CommandError)


class TestTimeoutAndAbortErrors:
    """Tests for timeout and abort exceptions."""

    def test_timeout_error(self):
        """Test TimeoutError."""
        error = TimeoutError("Operation timed out")
        assert isinstance(error, AerpawlibError)

    def test_abort_error(self):
        """Test AbortError."""
        error = AbortError("Mission aborted")
        assert isinstance(error, AerpawlibError)

    def test_user_abort_error(self):
        """Test UserAbortError."""
        error = UserAbortError("User aborted mission")
        assert isinstance(error, AbortError)

    def test_command_cancelled_error(self):
        """Test CommandCancelledError."""
        error = CommandCancelledError("Command was cancelled")
        assert isinstance(error, AbortError)


class TestArmingErrors:
    """Tests for arming-related exceptions."""

    def test_not_armable_error(self):
        """Test NotArmableError."""
        error = NotArmableError("Vehicle is not armable")
        assert isinstance(error, ArmError)


class TestSafetyErrors:
    """Tests for safety-related exceptions."""

    def test_safety_error(self):
        """Test SafetyError."""
        error = SafetyError("Safety violation")
        assert isinstance(error, AerpawlibError)

    def test_geofence_violation_error(self):
        """Test GeofenceViolationError."""
        error = GeofenceViolationError("Geofence boundary exceeded")
        assert isinstance(error, SafetyError)

    def test_speed_limit_exceeded_error(self):
        """Test SpeedLimitExceededError."""
        error = SpeedLimitExceededError("Speed limit exceeded")
        assert isinstance(error, SafetyError)

    def test_parameter_validation_error(self):
        """Test ParameterValidationError."""
        error = ParameterValidationError("Invalid parameter value")
        assert isinstance(error, SafetyError)

    def test_preflight_check_error(self):
        """Test PreflightCheckError."""
        error = PreflightCheckError("Preflight check failed")
        assert isinstance(error, SafetyError)


class TestExceptionHierarchy:
    """Tests for the exception hierarchy structure."""

    def test_all_errors_inherit_from_base(self):
        """Test that all errors inherit from AerpawlibError."""
        error_classes = [
            ConnectionError,
            ConnectionTimeoutError,
            HeartbeatLostError,
            ReconnectionError,
            CommandError,
            ArmError,
            DisarmError,
            TakeoffError,
            TakeoffTimeoutError,
            LandingError,
            LandingTimeoutError,
            NavigationError,
            GotoTimeoutError,
            ModeChangeError,
            OffboardError,
            TimeoutError,
            AbortError,
            UserAbortError,
            NotArmableError,
            CommandCancelledError,
            SafetyError,
            GeofenceViolationError,
            SpeedLimitExceededError,
            ParameterValidationError,
            PreflightCheckError,
        ]

        for error_class in error_classes:
            assert issubclass(error_class, AerpawlibError)

    def test_can_catch_by_base_class(self):
        """Test that errors can be caught by base class."""
        try:
            raise TakeoffTimeoutError("Timeout")
        except CommandError as e:
            assert isinstance(e, TakeoffTimeoutError)
        except:
            pytest.fail("Should have been caught by CommandError")

        try:
            raise GeofenceViolationError("Violation")
        except SafetyError as e:
            assert isinstance(e, GeofenceViolationError)
        except:
            pytest.fail("Should have been caught by SafetyError")


class TestExceptionUsagePatterns:
    """Tests for common exception usage patterns."""

    def test_catching_connection_errors(self):
        """Test catching connection errors."""
        def simulate_connection():
            raise ConnectionTimeoutError("Timeout", address="localhost", timeout=30)

        with pytest.raises(ConnectionError):
            simulate_connection()

    def test_catching_command_errors(self):
        """Test catching command errors."""
        def simulate_command():
            raise TakeoffError("Failed to take off")

        with pytest.raises(CommandError):
            simulate_command()

    def test_catching_all_aerpawlib_errors(self):
        """Test catching all aerpawlib errors with base class."""
        errors_to_test = [
            ConnectionTimeoutError("test"),
            TakeoffError("test"),
            GeofenceViolationError("test"),
            UserAbortError("test"),
        ]

        for error in errors_to_test:
            with pytest.raises(AerpawlibError):
                raise error


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

