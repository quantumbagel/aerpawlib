"""Unit tests for aerpawlib v2 BasicRunner and StateMachine."""

import pytest

from aerpawlib.v2 import (
    BasicRunner,
    StateMachine,
    entrypoint,
    state,
    timed_state,
    background,
    at_init,
)
from aerpawlib.v2.exceptions import (
    NoEntrypointError,
    NoInitialStateError,
    MultipleInitialStatesError,
    RunnerError,
)
from aerpawlib.v2.testing import MockVehicle


class TestBasicRunner:
    """BasicRunner with MockVehicle."""

    @pytest.mark.asyncio
    async def test_entrypoint_runs(self):
        ran = []

        class MinimalRunner(BasicRunner):
            @entrypoint
            async def run(self, vehicle):
                ran.append(1)

        runner = MinimalRunner()
        await runner.run(MockVehicle())
        assert ran == [1]

    @pytest.mark.asyncio
    async def test_no_entrypoint_raises(self):
        class NoEntrypoint(BasicRunner):
            pass

        with pytest.raises(NoEntrypointError):
            await NoEntrypoint().run(MockVehicle())

    @pytest.mark.asyncio
    async def test_multiple_entrypoints_raises(self):
        class MultiEntry(BasicRunner):
            @entrypoint
            async def run1(self, vehicle):
                pass

            @entrypoint
            async def run2(self, vehicle):
                pass

        with pytest.raises(RunnerError, match="Multiple @entrypoint"):
            await MultiEntry().run(MockVehicle())


class TestStateMachine:
    """StateMachine with MockVehicle."""

    @pytest.mark.asyncio
    async def test_state_transitions(self):
        order = []

        class SM(StateMachine):
            @state(name="a", first=True)
            async def a(self, vehicle):
                order.append("a")
                return "b"

            @state(name="b")
            async def b(self, vehicle):
                order.append("b")
                return None

        await SM().run(MockVehicle())
        assert order == ["a", "b"]

    @pytest.mark.asyncio
    async def test_timed_state_duration(self):
        order = []

        class SM(StateMachine):
            @timed_state(name="t", duration=0.1, first=True)
            async def t(self, vehicle):
                order.append("t")
                return None

        await SM().run(MockVehicle())
        assert "t" in order

    @pytest.mark.asyncio
    async def test_background_task_starts(self):
        started = []

        class SM(StateMachine):
            @background
            async def bg(self, vehicle):
                started.append(1)
                import asyncio
                while True:
                    await asyncio.sleep(0.1)

            @state(name="s", first=True)
            async def s(self, vehicle):
                import asyncio
                await asyncio.sleep(0.15)
                return None

        await SM().run(MockVehicle())
        assert len(started) == 1

    @pytest.mark.asyncio
    async def test_at_init_runs_before_states(self):
        order = []

        class SM(StateMachine):
            @at_init
            async def init_task(self, vehicle):
                order.append("init")

            @state(name="s", first=True)
            async def s(self, vehicle):
                order.append("s")
                return None

        await SM().run(MockVehicle())
        assert order == ["init", "s"]

    @pytest.mark.asyncio
    async def test_no_initial_state_raises(self):
        class SM(StateMachine):
            @state(name="a")
            async def a(self, vehicle):
                return None

        with pytest.raises(NoInitialStateError):
            await SM().run(MockVehicle())

    @pytest.mark.asyncio
    async def test_multiple_initial_states_raises(self):
        class SM(StateMachine):
            @state(name="a", first=True)
            async def a(self, vehicle):
                return None

            @state(name="b", first=True)
            async def b(self, vehicle):
                return None

        with pytest.raises(MultipleInitialStatesError):
            await SM().run(MockVehicle())
