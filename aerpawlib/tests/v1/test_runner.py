"""
Tests for the v1 aerpawlib runner framework.

These tests verify the v1 runner decorators and state machine functionality
matches the legacy API while using MAVSDK internally.
"""
import asyncio
import os
import sys

import pytest

# Add the parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))

from aerpawlib.v1.runner import (
    Runner,
    BasicRunner,
    StateMachine,
    entrypoint,
    state,
    timed_state,
    background,
)


class TestEntrypointDecorator:
    """Tests for the @entrypoint decorator in v1 API."""

    def test_entrypoint_marks_function(self):
        """Test that @entrypoint marks a function correctly."""
        @entrypoint
        async def my_func():
            pass

        assert hasattr(my_func, '_entrypoint')
        assert my_func._entrypoint is True

    def test_entrypoint_preserves_function(self):
        """Test that @entrypoint preserves the original function."""
        @entrypoint
        async def my_func():
            return 42

        result = asyncio.get_event_loop().run_until_complete(my_func())
        assert result == 42


class TestStateDecorator:
    """Tests for the @state decorator in v1 API."""

    def test_state_decorator_basic(self):
        """Test basic @state decorator usage."""
        @state("my_state")
        async def my_state_func():
            pass

        assert hasattr(my_state_func, '_is_state')
        assert my_state_func._is_state is True
        assert my_state_func._state_name == "my_state"

    def test_state_decorator_first(self):
        """Test @state decorator with first=True."""
        @state("initial", first=True)
        async def initial_state():
            pass

        assert initial_state._state_first is True

    def test_state_decorator_empty_name_raises(self):
        """Test that @state with empty name raises exception."""
        with pytest.raises(Exception, match="can't be"):
            @state("")
            async def bad_state():
                pass


class TestTimedStateDecorator:
    """Tests for the @timed_state decorator in v1 API."""

    def test_timed_state_basic(self):
        """Test basic @timed_state decorator usage."""
        @timed_state("timed", duration=5.0)
        async def my_timed_state():
            pass

        assert hasattr(my_timed_state, '_is_state')
        assert my_timed_state._state_duration == 5.0

    def test_timed_state_with_loop(self):
        """Test @timed_state decorator with loop=True."""
        @timed_state("looping", duration=2.0, loop=True)
        async def looping_state():
            pass

        assert looping_state._state_loop is True


class TestBackgroundDecorator:
    """Tests for the @background decorator in v1 API."""

    def test_background_marks_function(self):
        """Test that @background marks a function correctly."""
        @background
        async def bg_task():
            pass

        assert hasattr(bg_task, '_is_background')
        assert bg_task._is_background is True


class TestRunner:
    """Tests for the base Runner class in v1 API."""

    def test_runner_instantiation(self):
        """Test that Runner can be instantiated."""
        runner = Runner()
        assert runner is not None

    def test_runner_run_is_async(self):
        """Test that Runner.run is async."""
        runner = Runner()
        result = runner.run(None)
        assert asyncio.iscoroutine(result)
        asyncio.get_event_loop().run_until_complete(result)


class TestBasicRunner:
    """Tests for the BasicRunner class in v1 API."""

    def test_basic_runner_instantiation(self):
        """Test that BasicRunner can be instantiated."""
        runner = BasicRunner()
        assert runner is not None

    def test_basic_runner_with_entrypoint(self):
        """Test BasicRunner with an entrypoint method."""
        class MyRunner(BasicRunner):
            def __init__(self):
                super().__init__()
                self.ran = False

            @entrypoint
            async def my_entry(self):
                self.ran = True

        runner = MyRunner()
        assert hasattr(runner, 'my_entry')

    def test_basic_runner_without_entrypoint_raises(self):
        """Test that BasicRunner without entrypoint raises exception."""
        class NoEntryRunner(BasicRunner):
            async def some_method(self, vehicle):
                pass

        runner = NoEntryRunner()
        with pytest.raises(Exception, match="No @entrypoint"):
            asyncio.get_event_loop().run_until_complete(runner.run(None))


class TestStateMachine:
    """Tests for the StateMachine class in v1 API."""

    def test_state_machine_instantiation(self):
        """Test that StateMachine can be instantiated."""
        class MyMachine(StateMachine):
            @state("test", first=True)
            async def test_state(self):
                return None

        machine = MyMachine()
        assert machine is not None

    def test_state_machine_multiple_states(self):
        """Test StateMachine with multiple states."""
        class MultiStateMachine(StateMachine):
            def __init__(self):
                super().__init__()
                self.state_history = []

            @state("first", first=True)
            async def first_state(self):
                self.state_history.append("first")
                return "second"

            @state("second")
            async def second_state(self):
                self.state_history.append("second")
                return None

        machine = MultiStateMachine()
        assert hasattr(machine, 'first_state')
        assert hasattr(machine, 'second_state')


class TestV1ApiCompatibility:
    """Tests to verify v1 API is compatible with legacy runner API."""

    def test_entrypoint_same_as_legacy(self):
        """Test that entrypoint decorator works the same as legacy."""
        @entrypoint
        async def test_entry():
            return "test"

        assert test_entry._entrypoint is True
        result = asyncio.get_event_loop().run_until_complete(test_entry())
        assert result == "test"

    def test_state_same_as_legacy(self):
        """Test that state decorator works the same as legacy."""
        @state("test_state", first=True)
        async def test_state_func():
            return "next_state"

        assert test_state_func._is_state is True
        assert test_state_func._state_name == "test_state"
        assert test_state_func._state_first is True


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

