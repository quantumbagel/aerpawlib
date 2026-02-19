"""Unit tests for aerpawlib v1 exception hierarchy."""

import pytest

from aerpawlib.v1.exceptions import (
    AerpawConnectionError,
    AerpawlibError,
    ConnectionTimeoutError,
    ArmError,
    DisarmError,
    NotArmableError,
    TakeoffError,
    NavigationError,
    StateMachineError,
    HeartbeatLostError,
    MAVSDKNotInstalledError,
    AERPAWPlatformError,
    NotInAERPAWEnvironmentError,
    LandingError,
    RTLError,
    VelocityError,
    HeadingError,
    NotConnectedError,
    AbortedError,
    InvalidToleranceError,
    InvalidAltitudeError,
    InvalidSpeedError,
    NotImplementedForVehicleError,
    NoEntrypointError,
    InvalidStateError,
    NoInitialStateError,
    MultipleInitialStatesError,
    InvalidStateNameError,
    CommandError,
    StateError,
    ValidationError,
)


class TestAerpawlibError:
    """Base exception and hierarchy."""

    def test_base_message(self):
        e = AerpawlibError("test message")
        assert str(e) == "test message"
        assert e.message == "test message"

    def test_with_original_error(self):
        e = AerpawlibError("outer", original_error=ValueError("inner"))
        assert "caused by" in str(e)
        assert "inner" in str(e)


class TestConnectionErrors:
    """Connection-related exceptions."""

    def test_connection_timeout(self):
        e = ConnectionTimeoutError(30.0)
        assert "30" in str(e)
        assert e.timeout_seconds == 30.0

    def test_aerpaw_connection_inherits(self):
        assert issubclass(ConnectionTimeoutError, AerpawConnectionError)
        assert issubclass(ConnectionTimeoutError, AerpawlibError)


class TestActionErrors:
    """Arm, disarm, takeoff errors."""

    def test_arm_error(self):
        e = ArmError("failed")
        assert "failed" in str(e)

    def test_disarm_error(self):
        e = DisarmError("failed")
        assert "failed" in str(e)

    def test_not_armable_error(self):
        e = NotArmableError("not ready")
        assert "not ready" in str(e)

    def test_takeoff_error(self):
        e = TakeoffError("rejected")
        assert "rejected" in str(e)
        assert issubclass(TakeoffError, AerpawlibError)

    def test_navigation_error(self):
        e = NavigationError("timeout")
        assert "timeout" in str(e)


class TestStateMachineError:
    """StateMachineError."""

    def test_state_machine_error(self):
        e = StateMachineError("invalid state")
        assert "invalid state" in str(e)


class TestHeartbeatLostError:
    def test_default_message(self):
        e = HeartbeatLostError()
        assert "heartbeat" in str(e).lower()
        assert e.last_heartbeat_age == 0.0

    def test_custom_age(self):
        e = HeartbeatLostError(last_heartbeat_age=5.5)
        assert "5.5" in str(e)
        assert e.last_heartbeat_age == 5.5

    def test_custom_message_overrides(self):
        e = HeartbeatLostError(message="custom msg")
        assert "custom msg" in str(e)

    def test_inherits_connection_error(self):
        assert issubclass(HeartbeatLostError, AerpawConnectionError)
        assert issubclass(HeartbeatLostError, AerpawlibError)


class TestMAVSDKNotInstalledError:
    def test_message_content(self):
        e = MAVSDKNotInstalledError()
        assert "mavsdk" in str(e).lower()

    def test_inherits_connection_error(self):
        assert issubclass(MAVSDKNotInstalledError, AerpawConnectionError)


class TestAERPAWPlatformError:
    def test_basic_message(self):
        e = AERPAWPlatformError("platform down")
        assert "platform down" in str(e)
        assert issubclass(AERPAWPlatformError, AerpawlibError)

    def test_not_in_aerpaw_environment(self):
        e = NotInAERPAWEnvironmentError("zmq_control")
        assert "zmq_control" in str(e)
        assert issubclass(NotInAERPAWEnvironmentError, AERPAWPlatformError)
        assert e.feature == "zmq_control"


class TestCommandSubclasses:
    """LandingError, RTLError, VelocityError, HeadingError."""

    def test_landing_error(self):
        e = LandingError("vehicle stuck")
        assert "vehicle stuck" in str(e)
        assert issubclass(LandingError, CommandError)
        assert issubclass(LandingError, AerpawlibError)

    def test_landing_error_with_cause(self):
        cause = RuntimeError("hw fault")
        e = LandingError("refused", original_error=cause)
        assert "hw fault" in str(e)
        assert e.original_error is cause

    def test_rtl_error(self):
        e = RTLError("no GPS")
        assert "no GPS" in str(e)
        assert issubclass(RTLError, CommandError)

    def test_rtl_error_default_reason(self):
        e = RTLError()
        assert "Unknown" in str(e) or len(str(e)) > 0

    def test_velocity_error(self):
        e = VelocityError("offboard rejected")
        assert "offboard rejected" in str(e)
        assert issubclass(VelocityError, CommandError)

    def test_heading_error(self):
        e = HeadingError("setpoint out of range")
        assert "setpoint out of range" in str(e)
        assert issubclass(HeadingError, CommandError)


class TestStateErrors:
    def test_not_connected_error_default(self):
        e = NotConnectedError()
        assert "not connected" in str(e).lower()
        assert issubclass(NotConnectedError, StateError)
        assert issubclass(NotConnectedError, AerpawlibError)

    def test_not_connected_custom_message(self):
        e = NotConnectedError("serial port unavailable")
        assert "serial port" in str(e)

    def test_aborted_error_default(self):
        e = AbortedError()
        assert "aborted" in str(e).lower()
        assert issubclass(AbortedError, StateError)

    def test_aborted_custom_message(self):
        e = AbortedError("user cancelled")
        assert "user cancelled" in str(e)


class TestValidationErrors:
    def test_invalid_tolerance_error(self):
        e = InvalidToleranceError(0.001, 0.1, 100.0)
        assert "0.001" in str(e)
        assert e.value == 0.001
        assert e.min_val == 0.1
        assert e.max_val == 100.0
        assert issubclass(InvalidToleranceError, ValidationError)
        assert issubclass(InvalidToleranceError, AerpawlibError)

    def test_invalid_altitude_error(self):
        e = InvalidAltitudeError(500.0, 1.0, 400.0)
        assert "500.0" in str(e)
        assert e.value == 500.0
        assert issubclass(InvalidAltitudeError, ValidationError)

    def test_invalid_speed_error(self):
        e = InvalidSpeedError(99.0, 0.0, 30.0)
        assert "99.0" in str(e)
        assert e.value == 99.0
        assert issubclass(InvalidSpeedError, ValidationError)


class TestNotImplementedForVehicleError:
    def test_message_includes_feature_and_type(self):
        e = NotImplementedForVehicleError("set_velocity", "Rover")
        assert "set_velocity" in str(e)
        assert "Rover" in str(e)
        assert e.feature == "set_velocity"
        assert e.vehicle_type == "Rover"
        assert issubclass(NotImplementedForVehicleError, AerpawlibError)


class TestStateMachineExceptions:
    def test_no_entrypoint_error(self):
        e = NoEntrypointError()
        assert "entrypoint" in str(e).lower()
        assert issubclass(NoEntrypointError, StateMachineError)

    def test_invalid_state_error(self):
        e = InvalidStateError("unknown_state", ["init", "fly", "land"])
        assert "unknown_state" in str(e)
        assert e.state_name == "unknown_state"
        assert e.available_states == ["init", "fly", "land"]
        assert issubclass(InvalidStateError, StateMachineError)

    def test_no_initial_state_error(self):
        e = NoInitialStateError()
        assert "initial" in str(e).lower()
        assert issubclass(NoInitialStateError, StateMachineError)

    def test_multiple_initial_states_error(self):
        e = MultipleInitialStatesError()
        assert "one" in str(e).lower() or "multiple" in str(e).lower() or "initial" in str(e).lower()
        assert issubclass(MultipleInitialStatesError, StateMachineError)

    def test_invalid_state_name_error(self):
        e = InvalidStateNameError()
        assert "empty" in str(e).lower() or "name" in str(e).lower()
        assert issubclass(InvalidStateNameError, StateMachineError)


class TestExceptionCatchability:
    """Verify catch hierarchy works as expected."""

    def test_catch_all_as_aerpawlib_error(self):
        errors = [
            HeartbeatLostError(),
            LandingError(),
            RTLError(),
            VelocityError(),
            HeadingError(),
            NotConnectedError(),
            AbortedError(),
            InvalidToleranceError(1, 2, 3),
            NotImplementedForVehicleError("f", "Drone"),
            NoEntrypointError(),
            InvalidStateError("s", []),
        ]
        for err in errors:
            try:
                raise err
            except AerpawlibError:
                pass  # should always be caught

    def test_catch_command_error_subgroup(self):
        for cls in [LandingError, RTLError, VelocityError, HeadingError]:
            try:
                raise cls()
            except CommandError:
                pass

    def test_catch_state_error_subgroup(self):
        for cls in [NotConnectedError, AbortedError]:
            try:
                raise cls()
            except StateError:
                pass

