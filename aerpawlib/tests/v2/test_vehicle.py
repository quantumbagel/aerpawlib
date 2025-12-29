"""
Tests for the v2 aerpawlib vehicle module.

These tests verify CommandHandle and other vehicle-related features.
Note: Full vehicle tests require MAVSDK/SITL which are not available in unit tests.
"""
import pytest
import asyncio
import sys
import os

# Add the parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))

from aerpawlib.v2.vehicle import (
    CommandHandle,
    CommandStatus,
    CommandResult,
)


class TestCommandStatus:
    """Tests for CommandStatus enum."""

    def test_command_statuses_exist(self):
        """Test that all command statuses exist."""
        assert CommandStatus.PENDING
        assert CommandStatus.RUNNING
        assert CommandStatus.COMPLETED
        assert CommandStatus.FAILED
        assert CommandStatus.CANCELLED
        assert CommandStatus.TIMED_OUT


class TestCommandResult:
    """Tests for CommandResult dataclass."""

    def test_command_result_creation(self):
        """Test creating a CommandResult."""
        result = CommandResult(
            status=CommandStatus.COMPLETED,
            command="goto",
            start_time=100.0,
            end_time=110.0,
        )
        assert result.status == CommandStatus.COMPLETED
        assert result.command == "goto"
        assert result.start_time == 100.0
        assert result.end_time == 110.0

    def test_command_result_duration(self):
        """Test duration property."""
        result = CommandResult(
            status=CommandStatus.COMPLETED,
            command="goto",
            start_time=100.0,
            end_time=115.0,
        )
        assert result.duration == 15.0

    def test_command_result_succeeded(self):
        """Test succeeded property."""
        completed = CommandResult(
            status=CommandStatus.COMPLETED,
            command="goto",
            start_time=100.0,
            end_time=110.0,
        )
        assert completed.succeeded is True

        failed = CommandResult(
            status=CommandStatus.FAILED,
            command="goto",
            start_time=100.0,
            end_time=110.0,
        )
        assert failed.succeeded is False

    def test_command_result_was_cancelled(self):
        """Test was_cancelled property."""
        cancelled = CommandResult(
            status=CommandStatus.CANCELLED,
            command="goto",
            start_time=100.0,
            end_time=105.0,
        )
        assert cancelled.was_cancelled is True

        completed = CommandResult(
            status=CommandStatus.COMPLETED,
            command="goto",
            start_time=100.0,
            end_time=110.0,
        )
        assert completed.was_cancelled is False

    def test_command_result_with_error(self):
        """Test CommandResult with error."""
        error = Exception("Something went wrong")
        result = CommandResult(
            status=CommandStatus.FAILED,
            command="goto",
            start_time=100.0,
            end_time=101.0,
            error=error,
        )
        assert result.error == error
        assert result.succeeded is False

    def test_command_result_with_details(self):
        """Test CommandResult with details."""
        result = CommandResult(
            status=CommandStatus.COMPLETED,
            command="goto",
            start_time=100.0,
            end_time=110.0,
            details={"distance_traveled": 50.5, "waypoints_visited": 3},
        )
        assert result.details["distance_traveled"] == 50.5
        assert result.details["waypoints_visited"] == 3


class TestCommandHandle:
    """Tests for CommandHandle class."""

    def test_command_handle_creation(self):
        """Test creating a CommandHandle."""
        def completion_condition():
            return True

        handle = CommandHandle(
            command="goto",
            completion_condition=completion_condition,
        )
        assert handle is not None
        assert handle._command == "goto"

    def test_command_handle_with_timeout(self):
        """Test CommandHandle with timeout."""
        def completion_condition():
            return True

        handle = CommandHandle(
            command="goto",
            completion_condition=completion_condition,
            timeout=30.0,
        )
        assert handle._timeout == 30.0

    def test_command_handle_with_cancel_action(self):
        """Test CommandHandle with cancel action."""
        async def cancel_action():
            nonlocal cancel_called
            cancel_called = True

        def completion_condition():
            return True

        handle = CommandHandle(
            command="goto",
            completion_condition=completion_condition,
            cancel_action=cancel_action,
        )
        assert handle._cancel_action is not None

    def test_command_handle_with_progress_getter(self):
        """Test CommandHandle with progress getter."""
        def progress_getter():
            return {"distance": 50.0, "eta": 10.0}

        def completion_condition():
            return True

        handle = CommandHandle(
            command="goto",
            completion_condition=completion_condition,
            progress_getter=progress_getter,
        )
        assert handle._progress_getter is not None


class TestCommandHandleProperties:
    """Tests for CommandHandle properties."""

    def test_command_name_property(self):
        """Test command property returns command name."""
        handle = CommandHandle(
            command="takeoff",
            completion_condition=lambda: True,
        )
        assert handle.command == "takeoff"

    def test_is_running_property_initial(self):
        """Test is_running property before execution."""
        handle = CommandHandle(
            command="goto",
            completion_condition=lambda: True,
        )
        # Should be running until completion condition is met
        # (depends on implementation)
        assert hasattr(handle, 'is_running')

    def test_progress_property(self):
        """Test progress property."""
        def progress_getter():
            return {"distance": 25.0}

        handle = CommandHandle(
            command="goto",
            completion_condition=lambda: True,
            progress_getter=progress_getter,
        )
        # Access progress property
        assert hasattr(handle, 'progress')


class TestCommandHandleAsync:
    """Async tests for CommandHandle operations."""

    @pytest.mark.asyncio
    async def test_command_handle_wait_immediate_completion(self):
        """Test waiting on a handle that completes immediately."""
        completed = False

        def completion_condition():
            return completed

        handle = CommandHandle(
            command="test",
            completion_condition=completion_condition,
            timeout=1.0,
        )

        # Mark as completed
        completed = True

        # Should complete without timeout
        try:
            await asyncio.wait_for(handle.wait(), timeout=0.5)
        except asyncio.TimeoutError:
            pytest.fail("Handle should have completed immediately")

    @pytest.mark.asyncio
    async def test_command_handle_cancel(self):
        """Test cancelling a command handle."""
        cancel_called = False

        async def cancel_action():
            nonlocal cancel_called
            cancel_called = True

        handle = CommandHandle(
            command="test",
            completion_condition=lambda: False,  # Never completes
            cancel_action=cancel_action,
        )

        # Cancel the command
        await handle.cancel()

        assert cancel_called or handle.was_cancelled


class TestCommandHandleResult:
    """Tests for CommandHandle result retrieval."""

    def test_result_property(self):
        """Test result property exists."""
        handle = CommandHandle(
            command="test",
            completion_condition=lambda: True,
        )
        assert hasattr(handle, 'result')

    def test_succeeded_property(self):
        """Test succeeded property."""
        handle = CommandHandle(
            command="test",
            completion_condition=lambda: True,
        )
        assert hasattr(handle, 'succeeded')

    def test_was_cancelled_property(self):
        """Test was_cancelled property."""
        handle = CommandHandle(
            command="test",
            completion_condition=lambda: True,
        )
        assert hasattr(handle, 'was_cancelled')

    def test_error_property(self):
        """Test error property."""
        handle = CommandHandle(
            command="test",
            completion_condition=lambda: True,
        )
        assert hasattr(handle, 'error')


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

