import asyncio
import pytest

from aerpawlib.v1.runner import (
    BasicRunner,
    Runner,
    StateMachine,
    ZmqStateMachine,
    background,
    at_init,
    entrypoint,
    expose_field_zmq,
    expose_zmq,
    state,
    timed_state,
)
from aerpawlib.v1.vehicle import DummyVehicle
from aerpawlib.v1.exceptions import (
    InvalidStateError,
    MultipleInitialStatesError,
    NoInitialStateError,
    InvalidStateNameError,
    StateMachineError,
)


class TestBasicRunner:
    """BasicRunner and @entrypoint."""

    def test_entrypoint_decorator(self):
        class R(BasicRunner):
            @entrypoint
            async def run_mission(self, vehicle):
                pass

        assert hasattr(R().run_mission, "_entrypoint")

    @pytest.mark.asyncio
    async def test_executes_entrypoint(self):
        ran = []

        class R(BasicRunner):
            @entrypoint
            async def run_mission(self, vehicle):
                ran.append(1)

        await R().run(DummyVehicle())
        assert ran == [1]

    @pytest.mark.asyncio
    async def test_no_entrypoint_raises(self):
        class R(BasicRunner):
            async def run_mission(self, vehicle):
                pass

        with pytest.raises(Exception, match="No @entrypoint"):
            await R().run(DummyVehicle())

    @pytest.mark.asyncio
    async def test_receives_vehicle(self):
        received = []

        class R(BasicRunner):
            @entrypoint
            async def run_mission(self, vehicle):
                received.append(vehicle)

        v = DummyVehicle()
        await R().run(v)
        assert received[0] is v


class TestStateMachine:
    """StateMachine and @state."""

    def test_state_decorator(self):
        class R(StateMachine):
            @state("start", first=True)
            async def start_state(self, vehicle):
                return None

        r = R()
        assert hasattr(r.start_state, "_is_state")
        assert r.start_state._state_name == "start"
        assert r.start_state._state_first is True

    @pytest.mark.asyncio
    async def test_single_state_exits(self):
        ran = []

        class R(StateMachine):
            @state("only", first=True)
            async def only_state(self, vehicle):
                ran.append(1)
                return None

        await R().run(DummyVehicle())
        assert ran == [1]

    @pytest.mark.asyncio
    async def test_transitions(self):
        ran = []

        class R(StateMachine):
            @state("first", first=True)
            async def first_state(self, vehicle):
                ran.append("first")
                return "second"

            @state("second")
            async def second_state(self, vehicle):
                ran.append("second")
                return "third"

            @state("third")
            async def third_state(self, vehicle):
                ran.append("third")
                return None

        await R().run(DummyVehicle())
        assert ran == ["first", "second", "third"]

    @pytest.mark.asyncio
    async def test_loop_back(self):
        count = [0]

        class R(StateMachine):
            @state("loop", first=True)
            async def loop_state(self, vehicle):
                count[0] += 1
                return "loop" if count[0] < 3 else None

        await R().run(DummyVehicle())
        assert count[0] == 3


class TestTimedState:
    """@timed_state decorator."""

    def test_timed_state_marks(self):
        class R(StateMachine):
            @timed_state("wait", duration=1.0, first=True)
            async def wait_state(self, vehicle):
                return None

        assert R().wait_state._state_duration == 1.0

    @pytest.mark.asyncio
    async def test_timed_state_waits(self):
        import time

        class R(StateMachine):
            @timed_state("wait", duration=0.3, first=True)
            async def wait_state(self, vehicle):
                return None

        start = time.time()
        await R().run(DummyVehicle())
        assert time.time() - start >= 0.25


class TestRunnerInit:
    """Runner.initialize_args and cleanup."""

    def test_initialize_args_default(self):
        Runner().initialize_args(["--foo"])

    def test_cleanup_default(self):
        Runner().cleanup()

    def test_custom_initialize_args(self):
        args = []

        class R(BasicRunner):
            def initialize_args(self, a):
                args.extend(a)

            @entrypoint
            async def run_mission(self, vehicle):
                pass

        R().initialize_args(["--x", "1"])
        assert args == ["--x", "1"]


class TestBasicRunnerExtended:
    @pytest.mark.asyncio
    async def test_multiple_entrypoint_raises(self):
        """Having two @entrypoint methods should raise StateMachineError during run."""
        class R(BasicRunner):
            @entrypoint
            async def entry_a(self, vehicle):
                pass

            @entrypoint
            async def entry_b(self, vehicle):
                pass

        with pytest.raises(StateMachineError, match="Multiple @entrypoint"):
            await R().run(DummyVehicle())

    @pytest.mark.asyncio
    async def test_entrypoint_exception_propagates(self):
        """Exceptions raised inside @entrypoint propagate to the caller."""
        class R(BasicRunner):
            @entrypoint
            async def run_mission(self, vehicle):
                raise ValueError("mission failed")

        with pytest.raises(ValueError, match="mission failed"):
            await R().run(DummyVehicle())


class TestStateDecoratorEdgeCases:
    def test_empty_name_raises_at_decoration_time(self):
        with pytest.raises(InvalidStateNameError):
            @state("")
            async def bad_state(self, vehicle):
                return None

    def test_timed_state_empty_name_raises(self):
        with pytest.raises(InvalidStateNameError):
            @timed_state("", duration=1.0)
            async def bad_state(self, vehicle):
                return None

    def test_expose_zmq_empty_name_raises(self):
        with pytest.raises(InvalidStateNameError):
            @expose_zmq("")
            async def bad_func(self, vehicle):
                return None

    def test_expose_field_zmq_marks_function(self):
        @expose_field_zmq("altitude")
        async def get_alt(self, vehicle):
            return 42.0

        assert hasattr(get_alt, "_is_exposed_field_zmq")
        assert get_alt._zmq_name == "altitude"

    def test_expose_zmq_marks_function(self):
        @expose_zmq("fly")
        async def fly_state(self, vehicle):
            return None

        assert hasattr(fly_state, "_is_exposed_zmq")
        assert fly_state._zmq_name == "fly"


class TestStateMachineLifecycle:
    @pytest.mark.asyncio
    async def test_stop_exits_after_current_state(self):
        """Calling stop() from inside a state causes the machine to exit."""
        ran = []

        class R(StateMachine):
            @state("first", first=True)
            async def first_state(self, vehicle):
                ran.append("first")
                self.stop()
                return "second"  # should be ignored

            @state("second")
            async def second_state(self, vehicle):
                ran.append("second")
                return None

        await R().run(DummyVehicle())
        # "second" should never execute because stop() was called
        assert "second" not in ran

    @pytest.mark.asyncio
    async def test_invalid_transition_raises(self):
        """Returning a non-existent state name raises InvalidStateError."""
        class R(StateMachine):
            @state("start", first=True)
            async def start_state(self, vehicle):
                return "nonexistent_state"

        with pytest.raises(InvalidStateError):
            await R().run(DummyVehicle())

    @pytest.mark.asyncio
    async def test_no_initial_state_raises(self):
        """State machine with no first=True state raises NoInitialStateError."""
        class R(StateMachine):
            @state("orphan")
            async def orphan_state(self, vehicle):
                return None

        with pytest.raises(NoInitialStateError):
            await R().run(DummyVehicle())

    @pytest.mark.asyncio
    async def test_multiple_initial_states_raises(self):
        """Two first=True states raises MultipleInitialStatesError."""
        class R(StateMachine):
            @state("a", first=True)
            async def state_a(self, vehicle):
                return None

            @state("b", first=True)
            async def state_b(self, vehicle):
                return None

        with pytest.raises(MultipleInitialStatesError):
            await R().run(DummyVehicle())


class TestBackgroundTasks:
    @pytest.mark.asyncio
    async def test_background_runs_during_state(self):
        """@background task increments a counter while the state machine runs."""
        ticks = [0]

        class R(StateMachine):
            @background
            async def ticker(self, vehicle):
                ticks[0] += 1
                await asyncio.sleep(0.02)

            @state("wait", first=True)
            async def wait_state(self, vehicle):
                # Let the background task run for a bit
                await asyncio.sleep(0.15)
                return None

        await R().run(DummyVehicle())
        # Background task should have fired multiple times
        assert ticks[0] >= 2

    @pytest.mark.asyncio
    async def test_background_cancelled_after_state_machine_stops(self):
        """Background tasks should be cancelled when the machine stops."""
        class R(StateMachine):
            @background
            async def monitor(self, vehicle):
                await asyncio.sleep(1000)  # would run forever without cancel

            @state("only", first=True)
            async def only_state(self, vehicle):
                return None

        # Should complete quickly (not hang on background task)
        await asyncio.wait_for(R().run(DummyVehicle()), timeout=2.0)


class TestAtInitTasks:
    @pytest.mark.asyncio
    async def test_at_init_runs_before_first_state(self):
        """@at_init function executes before the first state is entered."""
        log = []

        class R(StateMachine):
            @at_init
            async def setup(self, vehicle):
                log.append("init")

            @state("start", first=True)
            async def start_state(self, vehicle):
                log.append("start")
                return None

        await R().run(DummyVehicle())
        assert log.index("init") < log.index("start")

    @pytest.mark.asyncio
    async def test_multiple_at_init_all_run(self):
        """Multiple @at_init functions all execute."""
        ran = []

        class R(StateMachine):
            @at_init
            async def init_a(self, vehicle):
                ran.append("a")

            @at_init
            async def init_b(self, vehicle):
                ran.append("b")

            @state("start", first=True)
            async def start_state(self, vehicle):
                return None

        await R().run(DummyVehicle())
        assert "a" in ran and "b" in ran


class TestTimedStateLoop:
    @pytest.mark.asyncio
    async def test_timed_state_loop_calls_function_multiple_times(self):
        """With loop=True, the function should be called more than once."""
        call_count = [0]

        class R(StateMachine):
            @timed_state("loop", duration=0.3, loop=True, first=True)
            async def looping_state(self, vehicle):
                call_count[0] += 1
                return None  # next state doesn't matter during duration

        await R().run(DummyVehicle())
        # Should have been called multiple times during 0.3 s
        assert call_count[0] >= 2

    @pytest.mark.asyncio
    async def test_timed_state_no_loop_calls_once(self):
        """With loop=False (default), the function is called exactly once."""
        call_count = [0]

        class R(StateMachine):
            @timed_state("once", duration=0.2, loop=False, first=True)
            async def once_state(self, vehicle):
                call_count[0] += 1
                return None

        await R().run(DummyVehicle())
        assert call_count[0] == 1


class TestZmqStateMachine:
    @pytest.mark.asyncio
    async def test_run_without_bindings_raises(self):
        """ZmqStateMachine.run() should raise StateMachineError if not initialized."""
        class Z(ZmqStateMachine):
            @state("start", first=True)
            async def start(self, vehicle):
                return None

        with pytest.raises(StateMachineError, match="ZMQ bindings not initialized"):
            await Z().run(DummyVehicle())

    def test_initialize_zmq_bindings_sets_attrs(self):
        class Z(ZmqStateMachine):
            @state("start", first=True)
            async def start(self, vehicle):
                return None

        z = Z()
        z._initialize_zmq_bindings("myid", "127.0.0.1")
        assert z._zmq_identifier == "myid"
        assert z._zmq_proxy_server == "127.0.0.1"

    def test_expose_zmq_registered_in_build(self):
        """States decorated with @expose_zmq should appear in _exported_states."""
        class Z(ZmqStateMachine):
            @state("start", first=True)
            async def start(self, vehicle):
                return None

            @expose_zmq("remote_fly")
            @state("fly")
            async def fly_state(self, vehicle):
                return None

        z = Z()
        z._initialize_zmq_bindings("test", "127.0.0.1")
        z._build()
        assert "remote_fly" in z._exported_states

    def test_expose_field_zmq_registered_in_build(self):
        class Z(ZmqStateMachine):
            @state("start", first=True)
            async def start(self, vehicle):
                return None

            @expose_field_zmq("battery")
            async def get_battery(self, vehicle):
                return 100

        z = Z()
        z._initialize_zmq_bindings("test", "127.0.0.1")
        z._build()
        assert "battery" in z._exported_fields

