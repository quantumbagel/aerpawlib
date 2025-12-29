"""
Execution frameworks for aerpawlib v2 API.

This module provides runners and decorators for building drone control scripts
with clean async/await patterns.

Decorators use `__set_name__` and descriptor protocols instead of attaching
attributes directly to functions, providing cleaner introspection and type safety.
"""
from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import (Any, Awaitable, Callable, ClassVar, Dict, Generic, List, Optional, Type, TypeVar, overload)

from .vehicle import Vehicle
from .logging import get_logger, LogComponent

# Configure logging using the modular logging system
logger = get_logger(LogComponent.RUNNER)

# Re-export asyncio helpers for convenience
sleep = asyncio.sleep
in_background = asyncio.ensure_future


# Type aliases
F = TypeVar("F", bound=Callable[..., Any])
T = TypeVar("T")
StateFunction = Callable[..., Awaitable[Optional[str]]]
BackgroundFunction = Callable[..., Awaitable[None]]
InitFunction = Callable[..., Awaitable[None]]


class StateType(Enum):
    """Type of state execution."""
    STANDARD = auto()
    TIMED = auto()


@dataclass(frozen=True, slots=True)
class StateConfig:
    """Immutable configuration for a state machine state."""
    name: str
    state_type: StateType = StateType.STANDARD
    is_initial: bool = False
    duration: float = 0.0
    loop: bool = False


class DecoratorType(Enum):
    """Types of decorators that can be applied to methods."""
    ENTRYPOINT = auto()
    STATE = auto()
    BACKGROUND = auto()
    INIT = auto()


@dataclass(slots=True)
class MethodDescriptor(Generic[T]):
    """
    Descriptor that wraps decorated methods and stores metadata.

    This uses Python's descriptor protocol instead of the function attribute hack.
    The descriptor stores metadata and provides clean access patterns.
    """
    func: Callable[..., T]
    decorator_type: DecoratorType
    state_config: Optional[StateConfig] = None

    # Set by __set_name__
    owner: Optional[Type] = field(default=None, repr=False)
    name: str = ""

    def __set_name__(self, owner: Type, name: str) -> None:
        """Called when the descriptor is assigned to a class attribute."""
        self.owner = owner
        self.name = name

        # Register this descriptor with the class for discovery
        if not hasattr(owner, "_aerpaw_descriptors"):
            owner._aerpaw_descriptors = []
        owner._aerpaw_descriptors.append(self)

    @overload
    def __get__(self, obj: None, objtype: Type) -> "MethodDescriptor[T]": ...

    @overload
    def __get__(self, obj: object, objtype: Type) -> Callable[..., T]: ...

    def __get__(self, obj: object | None, objtype: Type | None = None):
        """Get the bound method or the descriptor itself."""
        if obj is None:
            # Accessed from class, return descriptor for introspection
            return self
        # Accessed from instance, return bound method
        return self.func.__get__(obj, objtype)

    def __call__(self, *args, **kwargs) -> T:
        """Allow direct calling of the descriptor."""
        return self.func(*args, **kwargs)


class StateWrapper:
    """
    Wrapper for state functions that holds configuration and execution logic.

    This provides a clean interface for executing states without modifying
    the original function.
    """

    __slots__ = ("func", "config", "name")

    def __init__(self, func: StateFunction, config: StateConfig):
        self.func = func
        self.config = config
        self.name = config.name

    async def execute(self, vehicle: Vehicle) -> Optional[str]:
        """Execute the state and return the next state name."""
        if self.config.state_type == StateType.STANDARD:
            return await self.func(vehicle)
        elif self.config.state_type == StateType.TIMED:
            return await self._execute_timed(vehicle)
        return None

    async def _execute_timed(self, vehicle: Vehicle) -> Optional[str]:
        """Execute a timed state with optional looping."""
        last_result: Optional[str] = None

        async def run_state_loop() -> None:
            nonlocal last_result
            if self.config.loop:
                while True:
                    last_result = await self.func(vehicle)
                    # Yield to event loop for cancellation check
                    await asyncio.sleep(0)
            else:
                last_result = await self.func(vehicle)

        try:
            await asyncio.wait_for(
                run_state_loop(),
                timeout=self.config.duration
            )
        except asyncio.TimeoutError:
            # Expected when duration expires
            pass

        return last_result


# Decorators

def entrypoint(func: F) -> F:
    """
    Mark the entry point for a BasicRunner script.

    The decorated function will be called when the runner executes.
    Must be async.

    Example:
        class MyScript(BasicRunner):
            @entrypoint
            async def run_mission(self, drone: Drone):
                await drone.takeoff(altitude=5)
                await drone.goto(latitude=51.5, longitude=-0.1)
                await drone.land()
    """
    descriptor: MethodDescriptor = MethodDescriptor(
        func=func,
        decorator_type=DecoratorType.ENTRYPOINT,
    )
    return descriptor  # type: ignore


def state(name: str, *, first: bool = False) -> Callable[[F], F]:
    """
    Define a state in a StateMachine.

    Args:
        name: Unique name for this state
        first: If True, this is the initial state (keyword-only)

    The decorated function should be async and return the name of the
    next state, or None to stop the state machine.

    Example:
        class MyMission(StateMachine):
            @state("takeoff", first=True)
            async def takeoff_state(self, drone: Drone):
                await drone.takeoff(altitude=10)
                return "navigate"

            @state("navigate")
            async def navigate_state(self, drone: Drone):
                await drone.goto(latitude=51.5, longitude=-0.1)
                return "land"

            @state("land")
            async def land_state(self, drone: Drone):
                await drone.land()
                return None
    """
    if not name:
        raise ValueError("State name cannot be empty")

    def decorator(func: F) -> F:
        config = StateConfig(
            name=name,
            state_type=StateType.STANDARD,
            is_initial=first,
        )
        descriptor: MethodDescriptor = MethodDescriptor(
            func=func,
            decorator_type=DecoratorType.STATE,
            state_config=config,
        )
        return descriptor  # type: ignore

    return decorator


def timed_state(
    name: str,
    *,
    duration: float,
    loop: bool = False,
    first: bool = False,
) -> Callable[[F], F]:
    """
    Define a timed state that runs for a specific duration.

    Args:
        name: Unique name for this state
        duration: How long the state runs in seconds (keyword-only)
        loop: If True, repeatedly call the function until duration expires
        first: If True, this is the initial state

    Example:
        class MyMission(StateMachine):
            @timed_state("orbit", duration=30, loop=True)
            async def orbit_state(self, drone: Drone):
                await drone.move_in_current_direction(distance=5)
                return "next_state"
    """
    if not name:
        raise ValueError("State name cannot be empty")
    if duration <= 0:
        raise ValueError("Duration must be positive")

    def decorator(func: F) -> F:
        config = StateConfig(
            name=name,
            state_type=StateType.TIMED,
            is_initial=first,
            duration=duration,
            loop=loop,
        )
        descriptor: MethodDescriptor = MethodDescriptor(
            func=func,
            decorator_type=DecoratorType.STATE,
            state_config=config,
        )
        return descriptor  # type: ignore

    return decorator


def background(func: F) -> F:
    """
    Run a function in the background alongside the main state machine.

    The decorated function should be async and will run continuously until
    the state machine stops. Include delays in your function to prevent CPU hogging.

    Example:
        class MyMission(StateMachine):
            @background
            async def log_position(self, drone: Drone):
                print(f"Position: {drone.position}")
                await asyncio.sleep(1)
    """
    descriptor: MethodDescriptor = MethodDescriptor(
        func=func,
        decorator_type=DecoratorType.BACKGROUND,
    )
    return descriptor  # type: ignore


def at_init(func: F) -> F:
    """
    Run a function during initialization, before arming.

    The decorated function should be async and accept a Vehicle parameter.
    Multiple @at_init functions will run concurrently.

    Example:
        class MyMission(StateMachine):
            @at_init
            async def setup(self, drone: Drone):
                print("Initializing mission...")
                self.waypoints = load_waypoints("mission.plan")
    """
    descriptor: MethodDescriptor = MethodDescriptor(
        func=func,
        decorator_type=DecoratorType.INIT,
    )
    return descriptor  # type: ignore


# Runners

class Runner:
    """
    Base execution framework for vehicle control scripts.

    This is the base class that all runners extend. It provides the
    basic interface for script execution and lifecycle management.
    """

    # Class variable to hold discovered descriptors
    _aerpaw_descriptors: ClassVar[List[MethodDescriptor]] = []

    async def run(self, vehicle: Vehicle) -> None:
        """
        Run the script.

        Override in subclasses to implement core execution logic.

        Args:
            vehicle: The connected vehicle to control
        """
        pass

    def initialize_args(self, args: List[str]) -> None:
        """
        Parse and handle command line arguments.

        Override to add custom argument handling using argparse or similar.

        Args:
            args: List of command line arguments
        """
        pass

    def cleanup(self) -> None:
        """
        Perform cleanup when the script exits.

        Override to add custom cleanup logic.
        """
        pass

    @classmethod
    def _get_descriptors(cls) -> List[MethodDescriptor]:
        """Get all registered descriptors for this class."""
        descriptors = []
        for klass in cls.__mro__:
            if hasattr(klass, "_aerpaw_descriptors"):
                descriptors.extend(klass._aerpaw_descriptors)
        return descriptors


class BasicRunner(Runner):
    """
    Simple runner with a single entry point.

    Use the @entrypoint decorator to mark the function that should
    run when the script executes.

    Example:
        class MyScript(BasicRunner):
            @entrypoint
            async def main(self, drone: Drone):
                await drone.takeoff(altitude=5)
                await drone.land()
    """

    def _find_entrypoint(self) -> Optional[Callable]:
        """Find the entry point method using descriptors."""
        for descriptor in self._get_descriptors():
            if descriptor.decorator_type == DecoratorType.ENTRYPOINT:
                # Return the bound method
                return descriptor.__get__(self, type(self))
        return None

    async def run(self, vehicle: Vehicle) -> None:
        """Execute the entry point."""
        entry = self._find_entrypoint()
        if entry is None:
            raise RuntimeError(
                "No @entrypoint declared. Decorate a method with @entrypoint."
            )
        await entry(vehicle)


@dataclass
class _ExecutionContext:
    """Internal execution context for the state machine."""
    current_state: str
    running: bool = True
    override_next: bool = False
    next_override: str = ""
    background_tasks: List[asyncio.Task] = field(default_factory=list)


class StateMachine(Runner):
    """
    State machine runner for complex mission logic.

    States are defined using @state or @timed_state decorators. Each state
    returns the name of the next state to transition to.

    Background tasks can run in parallel using @background.
    Initialization tasks use @at_init to run before the mission starts.

    Example:
        class SurveyMission(StateMachine):
            @at_init
            async def setup(self, drone: Drone):
                self.waypoints = [...]
                self.current_wp = 0

            @state("takeoff", first=True)
            async def takeoff(self, drone: Drone):
                await drone.takeoff(altitude=10)
                return "survey"

            @state("survey")
            async def survey(self, drone: Drone):
                if self.current_wp >= len(self.waypoints):
                    return "rtl"
                await drone.goto(coordinates=self.waypoints[self.current_wp])
                self.current_wp += 1
                return "survey"

            @state("rtl")
            async def return_home(self, drone: Drone):
                await drone.rtl()
                return None

            @background
            async def telemetry_logger(self, drone: Drone):
                print(f"Alt: {drone.altitude}m, Bat: {drone.battery.percentage}%")
                await asyncio.sleep(1)
    """

    def __init__(self) -> None:
        super().__init__()
        self._states: Dict[str, StateWrapper] = {}
        self._background_methods: List[Callable] = []
        self._init_methods: List[Callable] = []
        self._initial_state: Optional[str] = None
        self._ctx: Optional[_ExecutionContext] = None

    def _discover_methods(self) -> None:
        """Discover all decorated methods and register them."""
        self._states.clear()
        self._background_methods.clear()
        self._init_methods.clear()
        self._initial_state = None

        initial_states: List[str] = []

        for descriptor in self._get_descriptors():
            # Get bound method for this instance
            bound_method = descriptor.__get__(self, type(self))

            if descriptor.decorator_type == DecoratorType.STATE:
                config = descriptor.state_config
                if config is None:
                    continue

                wrapper = StateWrapper(bound_method, config)
                self._states[config.name] = wrapper

                if config.is_initial:
                    initial_states.append(config.name)

            elif descriptor.decorator_type == DecoratorType.BACKGROUND:
                self._background_methods.append(bound_method)

            elif descriptor.decorator_type == DecoratorType.INIT:
                self._init_methods.append(bound_method)

        # Validate initial state
        if len(initial_states) == 0:
            raise RuntimeError(
                "No initial state defined. Use @state('name', first=True) "
                "to mark the starting state."
            )
        if len(initial_states) > 1:
            raise RuntimeError(
                f"Multiple initial states defined: {initial_states}. "
                "Only one state can have first=True."
            )

        self._initial_state = initial_states[0]

    async def _run_init_tasks(self, vehicle: Vehicle) -> None:
        """Run all initialization tasks concurrently."""
        if self._init_methods:
            await asyncio.gather(
                *(method(vehicle) for method in self._init_methods)
            )

    async def _start_background_tasks(self, vehicle: Vehicle) -> None:
        """Start all background tasks and track them."""
        if self._ctx is None:
            return

        for method in self._background_methods:
            async def task_runner(m: Callable = method) -> None:
                while self._ctx is not None and self._ctx.running:
                    try:
                        await m(vehicle)
                    except asyncio.CancelledError:
                        break
                    except Exception as e:
                        logger.exception(f"Background task error: {e}")
                        # Small delay before retry on error
                        await asyncio.sleep(0.1)

            task = asyncio.create_task(task_runner())
            self._ctx.background_tasks.append(task)

    async def _cancel_background_tasks(self) -> None:
        """Cancel all running background tasks."""
        if self._ctx is None:
            return

        for task in self._ctx.background_tasks:
            if not task.done():
                task.cancel()

        if self._ctx.background_tasks:
            await asyncio.gather(
                *self._ctx.background_tasks,
                return_exceptions=True
            )

        self._ctx.background_tasks.clear()

    async def run(self, vehicle: Vehicle, *, discover: bool = True) -> None:
        """
        Execute the state machine.

        Args:
            vehicle: The vehicle to control
            discover: If True, discover decorated methods before running
        """
        if discover:
            self._discover_methods()

        if self._initial_state is None:
            raise RuntimeError("No initial state configured")

        self._ctx = _ExecutionContext(current_state=self._initial_state)

        try:
            await self._run_init_tasks(vehicle)
            await self._start_background_tasks(vehicle)

            # Main state machine loop - no artificial delays
            while self._ctx.running:
                current = self._ctx.current_state

                if current not in self._states:
                    raise RuntimeError(
                        f"Invalid state: '{current}'. "
                        f"Available: {list(self._states.keys())}"
                    )

                state_wrapper = self._states[current]
                next_state = await state_wrapper.execute(vehicle)

                # Handle state transition
                if self._ctx.override_next:
                    self._ctx.override_next = False
                    self._ctx.current_state = self._ctx.next_override
                elif next_state is None:
                    self.stop()
                else:
                    self._ctx.current_state = next_state

        finally:
            await self._cancel_background_tasks()
            self.cleanup()

    def stop(self) -> None:
        """Stop the state machine after the current state completes."""
        if self._ctx is not None:
            self._ctx.running = False

    def transition_to(self, state_name: str) -> None:
        """
        Force transition to a specific state.

        This overrides the normal state transition after the current
        state completes.

        Args:
            state_name: Name of the state to transition to

        Raises:
            RuntimeError: If state machine is not running
            ValueError: If state_name is not a valid state
        """
        if self._ctx is None:
            raise RuntimeError("State machine is not running")

        if state_name not in self._states:
            raise ValueError(
                f"Unknown state: '{state_name}'. "
                f"Available: {list(self._states.keys())}"
            )

        self._ctx.override_next = True
        self._ctx.next_override = state_name

    @property
    def current_state_name(self) -> Optional[str]:
        """Get the name of the current state, or None if not running."""
        return self._ctx.current_state if self._ctx else None

    @property
    def is_running(self) -> bool:
        """Check if the state machine is currently running."""
        return self._ctx.running if self._ctx else False

    @property
    def available_states(self) -> List[str]:
        """Get a list of all available state names."""
        return list(self._states.keys())


__all__ = [
    # Runners
    "Runner",
    "BasicRunner",
    "StateMachine",
    # Decorators
    "entrypoint",
    "state",
    "timed_state",
    "background",
    "at_init",
    # Types (for type hints)
    "StateConfig",
    "StateType",
    "DecoratorType",
    "MethodDescriptor",
    # Helpers
    "sleep",
    "in_background",
]

