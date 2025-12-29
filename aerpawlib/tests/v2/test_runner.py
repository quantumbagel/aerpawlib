"""
Tests for the v2 aerpawlib runner framework.

These tests verify the v2 runner decorators and state machine functionality.
"""
import pytest
import asyncio
import sys
import os

# Add the parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))

from aerpawlib.v2.runner import (
    Runner,
    BasicRunner,
    StateMachine,
    entrypoint,
    state,
    timed_state,
    background,
    at_init,
    StateConfig,
    StateType,
    DecoratorType,
    MethodDescriptor,
    StateWrapper,
)


class TestEntrypointDecorator:
    """Tests for the @entrypoint decorator in v2 API."""

    def test_entrypoint_creates_descriptor(self):
        """Test that @entrypoint creates a MethodDescriptor."""
        @entrypoint
        async def my_func():
            pass

        assert isinstance(my_func, MethodDescriptor)
        assert my_func.decorator_type == DecoratorType.ENTRYPOINT

    def test_entrypoint_preserves_function_behavior(self):
        """Test that @entrypoint preserves async function behavior."""
        @entrypoint
        async def my_func():
            return 42

        # Direct call should work
        result = asyncio.get_event_loop().run_until_complete(my_func.func())
        assert result == 42


class TestStateDecorator:
    """Tests for the @state decorator in v2 API."""

    def test_state_decorator_basic(self):
        """Test basic @state decorator usage."""
        @state("my_state")
        async def my_state_func():
            pass

        assert isinstance(my_state_func, MethodDescriptor)
        assert my_state_func.decorator_type == DecoratorType.STATE
        assert my_state_func.state_config.name == "my_state"
        assert my_state_func.state_config.is_initial is False

    def test_state_decorator_first(self):
        """Test @state decorator with first=True."""
        @state("initial", first=True)
        async def initial_state():
            pass

        assert initial_state.state_config.is_initial is True

    def test_state_config_immutable(self):
        """Test that StateConfig is immutable (frozen)."""
        config = StateConfig(name="test")
        with pytest.raises(Exception):  # FrozenInstanceError
            config.name = "changed"


class TestTimedStateDecorator:
    """Tests for the @timed_state decorator in v2 API."""

    def test_timed_state_basic(self):
        """Test basic @timed_state decorator usage."""
        @timed_state("timed", duration=5.0)
        async def my_timed_state():
            pass

        assert my_timed_state.state_config.state_type == StateType.TIMED
        assert my_timed_state.state_config.duration == 5.0
        assert my_timed_state.state_config.loop is False

    def test_timed_state_with_loop(self):
        """Test @timed_state decorator with loop=True."""
        @timed_state("looping", duration=2.0, loop=True)
        async def looping_state():
            pass

        assert looping_state.state_config.loop is True


class TestBackgroundDecorator:
    """Tests for the @background decorator in v2 API."""

    def test_background_creates_descriptor(self):
        """Test that @background creates a MethodDescriptor."""
        @background
        async def bg_task():
            pass

        assert isinstance(bg_task, MethodDescriptor)
        assert bg_task.decorator_type == DecoratorType.BACKGROUND


class TestInitDecorator:
    """Tests for the @at_init decorator in v2 API."""

    def test_at_init_creates_descriptor(self):
        """Test that @at_init creates a MethodDescriptor."""
        @at_init
        async def init_func():
            pass

        assert isinstance(init_func, MethodDescriptor)
        assert init_func.decorator_type == DecoratorType.INIT


class TestMethodDescriptor:
    """Tests for the MethodDescriptor class."""

    def test_method_descriptor_set_name(self):
        """Test that __set_name__ is called when assigned to class."""
        class TestClass:
            @entrypoint
            async def my_method(self, vehicle):
                pass

        # Check that the descriptor was properly set up
        descriptor = TestClass.__dict__["my_method"]
        assert descriptor.name == "my_method"
        assert descriptor.owner == TestClass

    def test_method_descriptor_get_from_class(self):
        """Test getting descriptor from class returns descriptor."""
        class TestClass:
            @entrypoint
            async def my_method(self, vehicle):
                pass

        descriptor = TestClass.my_method
        assert isinstance(descriptor, MethodDescriptor)

    def test_method_descriptor_get_from_instance(self):
        """Test getting descriptor from instance returns bound method."""
        class TestClass:
            @entrypoint
            async def my_method(self):
                return "test"

        obj = TestClass()
        method = obj.my_method
        # Should be callable
        assert callable(method)


class TestStateWrapper:
    """Tests for the StateWrapper class."""

    def test_state_wrapper_standard(self):
        """Test StateWrapper for standard states."""
        async def test_func():
            return "next_state"

        config = StateConfig(name="test", state_type=StateType.STANDARD)
        wrapper = StateWrapper(test_func, config)

        assert wrapper.name == "test"

    def test_state_wrapper_execute_standard(self):
        """Test executing a standard state."""
        async def test_func():
            return "next_state"

        config = StateConfig(name="test", state_type=StateType.STANDARD)
        wrapper = StateWrapper(test_func, config)

        result = asyncio.get_event_loop().run_until_complete(wrapper.execute(None))
        assert result == "next_state"


class TestStateType:
    """Tests for StateType enum."""

    def test_state_types_exist(self):
        """Test that expected state types exist."""
        assert StateType.STANDARD
        assert StateType.TIMED


class TestDecoratorType:
    """Tests for DecoratorType enum."""

    def test_decorator_types_exist(self):
        """Test that expected decorator types exist."""
        assert DecoratorType.ENTRYPOINT
        assert DecoratorType.STATE
        assert DecoratorType.BACKGROUND
        assert DecoratorType.INIT


class TestRunner:
    """Tests for the base Runner class in v2 API."""

    def test_runner_instantiation(self):
        """Test that Runner can be instantiated."""
        runner = Runner()
        assert runner is not None

    def test_runner_has_run_method(self):
        """Test that Runner has a run method."""
        runner = Runner()
        assert hasattr(runner, 'run')
        assert asyncio.iscoroutinefunction(runner.run) or callable(runner.run)


class TestBasicRunner:
    """Tests for the BasicRunner class in v2 API."""

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


class TestStateMachine:
    """Tests for the StateMachine class in v2 API."""

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

    def test_state_machine_with_background(self):
        """Test StateMachine with background tasks."""
        class BackgroundMachine(StateMachine):
            def __init__(self):
                super().__init__()
                self.bg_ran = False

            @state("main", first=True)
            async def main_state(self):
                return None

            @background
            async def bg_task(self):
                self.bg_ran = True

        machine = BackgroundMachine()
        assert hasattr(machine, 'main_state')
        assert hasattr(machine, 'bg_task')

    def test_state_machine_with_init(self):
        """Test StateMachine with init handler."""
        class InitMachine(StateMachine):
            def __init__(self):
                super().__init__()
                self.initialized = False

            @at_init
            async def setup(self):
                self.initialized = True

            @state("main", first=True)
            async def main_state(self):
                return None

        machine = InitMachine()
        assert hasattr(machine, 'setup')


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

