"""
Tests for the legacy aerpawlib runner framework.

These tests verify the runner decorators and state machine functionality.
"""
import pytest
import asyncio
import sys
import os

# Add the parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))

from aerpawlib.legacy.runner import (
    Runner,
    BasicRunner,
    StateMachine,
    entrypoint,
    state,
    timed_state,
    background,
    expose_field_zmq,
    expose_zmq,
)


class TestEntrypointDecorator:
    """Tests for the @entrypoint decorator."""

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
    """Tests for the @state decorator."""

    def test_state_decorator_basic(self):
        """Test basic @state decorator usage."""
        @state("my_state")
        async def my_state_func():
            pass

        assert hasattr(my_state_func, '_is_state')
        assert my_state_func._is_state is True
        assert my_state_func._state_name == "my_state"
        assert my_state_func._state_first is False

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
    """Tests for the @timed_state decorator."""

    def test_timed_state_basic(self):
        """Test basic @timed_state decorator usage."""
        @timed_state("timed", duration=5.0)
        async def my_timed_state():
            pass

        assert hasattr(my_timed_state, '_is_state')
        assert my_timed_state._state_name == "timed"
        assert my_timed_state._state_duration == 5.0
        assert my_timed_state._state_loop is False

    def test_timed_state_with_loop(self):
        """Test @timed_state decorator with loop=True."""
        @timed_state("looping", duration=2.0, loop=True)
        async def looping_state():
            pass

        assert looping_state._state_loop is True

    def test_timed_state_first(self):
        """Test @timed_state decorator with first=True."""
        @timed_state("initial_timed", duration=1.0, first=True)
        async def initial_timed():
            pass

        assert initial_timed._state_first is True


class TestBackgroundDecorator:
    """Tests for the @background decorator."""

    def test_background_marks_function(self):
        """Test that @background marks a function correctly."""
        @background
        async def bg_task():
            pass

        assert hasattr(bg_task, '_is_background')
        assert bg_task._is_background is True


class TestExposeZmqDecorators:
    """Tests for ZMQ exposure decorators."""

    def test_expose_zmq(self):
        """Test @expose_zmq decorator."""
        @expose_zmq("my_binding")
        async def zmq_func():
            pass

        assert hasattr(zmq_func, '_is_exposed_zmq')
        assert zmq_func._is_exposed_zmq is True
        assert zmq_func._zmq_name == "my_binding"

    def test_expose_zmq_empty_name_raises(self):
        """Test that @expose_zmq with empty name raises exception."""
        with pytest.raises(Exception, match="must be exported"):
            @expose_zmq("")
            async def bad_zmq():
                pass

    def test_expose_field_zmq(self):
        """Test @expose_field_zmq decorator."""
        @expose_field_zmq("my_field")
        def field_func():
            return 42

        assert hasattr(field_func, '_is_exposed_field_zmq')
        assert field_func._is_exposed_field_zmq is True
        assert field_func._zmq_name == "my_field"

    def test_expose_field_zmq_empty_name_raises(self):
        """Test that @expose_field_zmq with empty name raises exception."""
        with pytest.raises(Exception, match="must be exposed"):
            @expose_field_zmq("")
            def bad_field():
                pass


class TestRunner:
    """Tests for the base Runner class."""

    def test_runner_instantiation(self):
        """Test that Runner can be instantiated."""
        runner = Runner()
        assert runner is not None

    def test_runner_run_is_async(self):
        """Test that Runner.run is async."""
        runner = Runner()
        result = runner.run(None)
        assert asyncio.iscoroutine(result)
        # Clean up the coroutine
        asyncio.get_event_loop().run_until_complete(result)

    def test_runner_cleanup(self):
        """Test that Runner.cleanup can be called."""
        runner = Runner()
        runner.cleanup()  # Should not raise

    def test_runner_initialize_args(self):
        """Test that Runner.initialize_args can be called."""
        runner = Runner()
        runner.initialize_args(["arg1", "arg2"])  # Should not raise


class TestBasicRunner:
    """Tests for the BasicRunner class."""

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
        # We need a mock vehicle to test this fully
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
    """Tests for the StateMachine class."""

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
                return None  # End state machine

        machine = MultiStateMachine()
        assert hasattr(machine, 'first_state')
        assert hasattr(machine, 'second_state')


class TestStateMachineWithBackground:
    """Tests for StateMachine with background tasks."""

    def test_state_machine_with_background(self):
        """Test StateMachine with a background task."""
        class BackgroundMachine(StateMachine):
            def __init__(self):
                super().__init__()
                self.bg_ran = False

            @state("main", first=True)
            async def main_state(self):
                await asyncio.sleep(0.1)
                return None

            @background
            async def bg_task(self):
                self.bg_ran = True
                await asyncio.sleep(0.01)

        machine = BackgroundMachine()
        assert hasattr(machine, 'main_state')
        assert hasattr(machine, 'bg_task')


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

