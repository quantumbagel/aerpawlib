"""
Runner for aerpawlib v2.

Descriptor-based decorators: @entrypoint, @state, @timed_state, @background, @at_init.
"""

from __future__ import annotations

import asyncio
import inspect
import types
from enum import Enum, auto
from typing import Any, Callable, Dict, List, Optional, TypeVar

from .constants import STATE_MACHINE_DELAY_S
from .exceptions import (
    InvalidStateError,
    MultipleInitialStatesError,
    NoEntrypointError,
    NoInitialStateError,
    InvalidStateNameError,
)
from .log import LogComponent, get_logger

logger = get_logger(LogComponent.RUNNER)

V = TypeVar("V")


class Runner:
    """Base execution framework for aerpawlib v2 scripts."""

    async def run(self, vehicle: V) -> None:
        """Core logic. Override in subclasses."""
        pass

    def initialize_args(self, args: List[str]) -> None:
        """Parse additional CLI args."""
        pass

    def cleanup(self) -> None:
        """Cleanup on exit."""
        pass


# --- Descriptor-based decorators ---


class _EntrypointDescriptor:
    """Descriptor for @entrypoint. Uses __set_name__ to register."""

    def __init__(self, func: Callable) -> None:
        self.func = func
        self.name: Optional[str] = None

    def __set_name__(self, owner: type, name: str) -> None:
        self.name = name
        if not hasattr(owner, "_v2_entrypoints"):
            owner._v2_entrypoints = []
        owner._v2_entrypoints.append((name, self))

    def __get__(self, obj: Any, objtype: Optional[type] = None) -> Any:
        if obj is None:
            return self
        return types.MethodType(self.func, obj)


def entrypoint(func: Callable) -> _EntrypointDescriptor:
    """Mark method as BasicRunner entry point."""
    return _EntrypointDescriptor(func)


class _StateType(Enum):
    STANDARD = auto()
    TIMED = auto()


class _StateDescriptor:
    """Descriptor for @state and @timed_state."""

    def __init__(
        self,
        name: str,
        first: bool = False,
        state_type: _StateType = _StateType.STANDARD,
        duration: float = 0.0,
        loop: bool = False,
    ) -> None:
        if not name:
            raise InvalidStateNameError()
        self.name = name
        self.first = first
        self.state_type = state_type
        self.duration = duration
        self.loop = loop
        self.func: Optional[Callable] = None

    def __call__(self, func: Callable) -> "_StateDescriptor":
        self.func = func
        return self

    def __set_name__(self, owner: type, attr_name: str) -> None:
        if "_v2_states" not in owner.__dict__:
            owner._v2_states = {}
        owner._v2_states[self.name] = self
        if self.first:
            if "_v2_initial_states" not in owner.__dict__:
                owner._v2_initial_states = []
            owner._v2_initial_states.append(self.name)

    def __get__(self, obj: Any, objtype: Optional[type] = None) -> Any:
        if obj is None:
            return self
        if self.func is None:
            raise RuntimeError("State decorator not applied to function")
        return types.MethodType(self.func, obj)


def state(name: str, first: bool = False) -> Callable[[Callable], _StateDescriptor]:
    """Decorator for StateMachine state."""

    def decorator(func: Callable) -> _StateDescriptor:
        desc = _StateDescriptor(name, first=first, state_type=_StateType.STANDARD)
        desc.func = func
        return desc

    return decorator


def timed_state(
    name: str,
    duration: float,
    loop: bool = False,
    first: bool = False,
) -> Callable[[Callable], _StateDescriptor]:
    """Decorator for timed state."""

    def decorator(func: Callable) -> _StateDescriptor:
        desc = _StateDescriptor(
            name,
            first=first,
            state_type=_StateType.TIMED,
            duration=duration,
            loop=loop,
        )
        desc.func = func
        return desc

    return decorator


class _BackgroundDescriptor:
    """Descriptor for @background."""

    def __init__(self, func: Callable) -> None:
        self.func = func
        self.name: Optional[str] = None

    def __set_name__(self, owner: type, name: str) -> None:
        self.name = name
        if not hasattr(owner, "_v2_backgrounds"):
            owner._v2_backgrounds = []
        owner._v2_backgrounds.append((name, self))

    def __get__(self, obj: Any, objtype: Optional[type] = None) -> Any:
        if obj is None:
            return self
        return types.MethodType(self.func, obj)


def background(func: Callable) -> _BackgroundDescriptor:
    """Mark method as background task."""
    return _BackgroundDescriptor(func)


class _AtInitDescriptor:
    """Descriptor for @at_init."""

    def __init__(self, func: Callable) -> None:
        self.func = func
        self.name: Optional[str] = None

    def __set_name__(self, owner: type, name: str) -> None:
        self.name = name
        if not hasattr(owner, "_v2_at_init"):
            owner._v2_at_init = []
        owner._v2_at_init.append((name, self))

    def __get__(self, obj: Any, objtype: Optional[type] = None) -> Any:
        if obj is None:
            return self
        return types.MethodType(self.func, obj)


def at_init(func: Callable) -> _AtInitDescriptor:
    """Mark method to run at init (before arm)."""
    return _AtInitDescriptor(func)


# --- Runner implementations ---


class BasicRunner(Runner):
    """Single entry point runner."""

    async def run(self, vehicle: Any) -> None:
        entrypoints = getattr(self.__class__, "_v2_entrypoints", [])
        if not entrypoints:
            raise NoEntrypointError()
        if len(entrypoints) > 1:
            from .exceptions import RunnerError

            raise RunnerError("Multiple @entrypoint decorators found")
        name, desc = entrypoints[0]
        method = getattr(self, name)
        logger.info(f"BasicRunner: starting entrypoint '{name}'")
        try:
            await method(vehicle)
            logger.info(f"BasicRunner: entrypoint '{name}' completed")
        except Exception as e:
            logger.error(f"BasicRunner: entrypoint '{name}' failed: {e}")
            raise


class StateMachine(Runner):
    """State-based mission runner."""

    def __init__(self) -> None:
        self._current_state: Optional[str] = None
        self._running = False
        self._background_futures: List[asyncio.Future] = []

    def _get_states(self) -> Dict[str, _StateDescriptor]:
        states = getattr(self.__class__, "_v2_states", None)
        return dict(states) if states else {}

    def _get_initial_state(self) -> str:
        initial_states = getattr(self.__class__, "_v2_initial_states", None) or []
        if len(initial_states) > 1:
            raise MultipleInitialStatesError()
        if not initial_states:
            raise NoInitialStateError()
        return initial_states[0]

    def _get_backgrounds(self) -> List[tuple]:
        return getattr(self.__class__, "_v2_backgrounds", [])

    def _get_at_init(self) -> List[tuple]:
        return getattr(self.__class__, "_v2_at_init", [])

    async def _run_state(
        self, state_desc: _StateDescriptor, vehicle: Any
    ) -> str:
        logger.debug(f"StateMachine: entering state '{state_desc.name}'")
        if state_desc.state_type == _StateType.STANDARD:
            next_state = await state_desc.__get__(self, type(self))(vehicle)
            logger.debug(f"StateMachine: state '{state_desc.name}' -> '{next_state}'")
            return next_state
        # Timed state
        running = True
        last_state = ""

        async def _bg() -> str:
            nonlocal running, last_state
            while running:
                last_state = await state_desc.__get__(self, type(self))(vehicle)
                if not running:
                    break
                if not state_desc.loop:
                    running = False
                    break
                await asyncio.sleep(STATE_MACHINE_DELAY_S)
            return last_state

        task = asyncio.create_task(_bg())
        logger.debug(
            f"StateMachine: timed_state '{state_desc.name}' "
            f"(duration={state_desc.duration}s, loop={state_desc.loop})"
        )
        await asyncio.sleep(state_desc.duration)  # Justified: timed_state duration
        running = False
        next_state = await task
        logger.debug(f"StateMachine: timed_state '{state_desc.name}' -> '{next_state}'")
        return next_state

    async def run(self, vehicle: Any) -> None:
        states = self._get_states()
        self._current_state = self._get_initial_state()
        self._running = True
        logger.info(
            f"StateMachine: starting with initial state '{self._current_state}' "
            f"(states: {list(states.keys())})"
        )

        # Run at_init tasks
        at_init_list = self._get_at_init()
        if at_init_list:
            logger.debug(f"StateMachine: running {len(at_init_list)} at_init task(s)")
        for name, desc in at_init_list:
            logger.debug(f"StateMachine: at_init '{name}'")
            method = getattr(self, name)
            await method(vehicle)

        # Start background tasks
        backgrounds = self._get_backgrounds()
        if backgrounds:
            logger.info(f"StateMachine: starting {len(backgrounds)} background task(s)")
        for name, desc in backgrounds:
            method = getattr(self, name)

            async def _bg_task(task):
                while self._running:
                    try:
                        await task(vehicle)
                    except asyncio.CancelledError:
                        return
                    except Exception as e:
                        logger.error(
                            f"Background task '{name}' failed: {e}",
                            exc_info=True,
                        )
                        await asyncio.sleep(0.5)

            fut = asyncio.create_task(_bg_task(method))
            self._background_futures.append(fut)

        # Main state loop
        while self._running:
            if self._current_state not in states:
                logger.error(
                    f"StateMachine: invalid state '{self._current_state}' "
                    f"(valid: {list(states.keys())})"
                )
                raise InvalidStateError(
                    self._current_state, list(states.keys())
                )
            state_desc = states[self._current_state]
            next_state = await self._run_state(state_desc, vehicle)
            self._current_state = next_state
            if next_state is None:
                logger.info(f"StateMachine: completed (final state returned None)")
                break
            await asyncio.sleep(STATE_MACHINE_DELAY_S)

        self._running = False
        logger.info("StateMachine: stopping")

        for fut in self._background_futures:
            fut.cancel()
        if self._background_futures:
            await asyncio.gather(
                *self._background_futures, return_exceptions=True
            )
        self.cleanup()

    def stop(self) -> None:
        """Stop the state machine after current state."""
        logger.debug(f"StateMachine: stop() called (current state: {self._current_state})")
        self._running = False
