"""
Collection of execution frameworks that can be extended to make scripts
runnable using aerpawlib. The most basic framework is `Runner` -- any custom
frameworks *must* extend it to be executable.

This is the v1 API runner module, now using MAVSDK internally.

@author: Julian Reder (quantumbagel)
"""

import asyncio
import inspect
from enum import Enum, auto
from typing import Callable, Dict, List

import zmq
import zmq.asyncio

from .vehicle import Vehicle
from .constants import (
    STATE_MACHINE_DELAY_S,
    ZMQ_PROXY_IN_PORT,
    ZMQ_PROXY_OUT_PORT,
    ZMQ_TYPE_TRANSITION,
    ZMQ_TYPE_FIELD_REQUEST,
    ZMQ_TYPE_FIELD_CALLBACK,
)
from .exceptions import (
    NoEntrypointError,
    InvalidStateError,
    NoInitialStateError,
    MultipleInitialStatesError,
    InvalidStateNameError,
)


class Runner:
    """
    Base execution framework for aerpawlib scripts.

    All custom execution frameworks must extend this class to be executable
    by the aerpawlib infrastructure.
    """

    async def run(self, _: Vehicle):
        """
        Core logic of the script.

        This method is called by the launch script after initializations.
        It should be overridden by subclasses to implement specific execution models.

        Args:
            _: The vehicle object initialized for this script.
        """
        pass

    def initialize_args(self, _: List[str]):
        """
        Parse and handle additional command-line arguments.

        Args:
            _: List of command-line arguments as strings.
        """
        pass

    def cleanup(self):
        """
        Perform cleanup tasks when the script exits.
        """
        pass


_Runnable = Callable[[Runner, Vehicle], str]


def entrypoint(func):
    """
    Decorator used to identify the entry point used by `BasicRunner` driven
    scripts.

    The function decorated by this is expected to be `async`
    """
    func._entrypoint = True
    return func


class BasicRunner(Runner):
    """
    BasicRunners have a single entry point (specified by `entrypoint`) that is
    executed when the script is run. The function provided can be anything, as
    it will be run in parallel to background services used by aerpawlib.

    For an example of a minimum viable `BasicRunner`, check out
    `examples/basic_runner.py`
    """

    def _build(self):
        for _, method in inspect.getmembers(self):
            if not inspect.ismethod(method):
                continue
            if hasattr(method, "_entrypoint"):
                self._entry = method

    async def run(self, vehicle: Vehicle):
        self._build()
        if hasattr(self, "_entry"):
            await self._entry.__func__(self, vehicle)
        else:
            raise NoEntrypointError()


class _StateType(Enum):
    STANDARD = auto()
    TIMED = auto()


class _State:
    """
    Internal representation of a state in a StateMachine.

    Attributes:
        _name (str): The name of the state.
        _func (_Runnable): The function to be executed for this state.
    """

    _name: str
    _func: _Runnable

    def __init__(self, func: _Runnable, name: str):
        self._name = name
        self._func = func

    async def run(self, runner: Runner, vehicle: Vehicle) -> str:
        """
        Run the function associated with this state.

        Args:
            runner: The Runner instance executing this state.
            vehicle: The Vehicle instance to be controlled.

        Returns:
            str: The name of the next state to transition to.
        """
        if self._func._state_type == _StateType.STANDARD:
            return await self._func.__func__(runner, vehicle)
        elif self._func._state_type == _StateType.TIMED:
            running = True

            async def _bg():
                nonlocal running
                last_state = ""
                while running:
                    last_state = await self._func.__func__(runner, vehicle)
                    if not self._func._state_loop:
                        running = False
                    await asyncio.sleep(STATE_MACHINE_DELAY_S)
                return last_state

            r = asyncio.ensure_future(_bg())
            # order here is important and stops a race condition
            await asyncio.sleep(self._func._state_duration)
            running = False
            next_state = await r
            return next_state
        return ""


def state(name: str, first: bool = False):
    """
    Decorator to specify a state in a StateMachine.

    Args:
        name (str): The name of the state.
        first (bool, optional): Whether this is the initial state of the machine.
            Defaults to False.

    Returns:
        Callable: The decorated function.

    Raises:
        InvalidStateNameError: If the name is empty.
    """
    if name == "":
        raise InvalidStateNameError()

    def decorator(func):
        func._is_state = True
        func._state_name = name
        func._state_first = first
        func._state_type = _StateType.STANDARD
        return func

    return decorator


def timed_state(name: str, duration: float, loop=False, first: bool = False):
    """
    Decorator for a state that runs for a fixed duration.

    Args:
        name (str): The name of the state.
        duration (float): Minimum duration in seconds for this state.
        loop (bool, optional): Whether to repeatedly call the decorated function
            during the duration. Defaults to False.
        first (bool, optional): Whether this is the initial state. Defaults to False.

    Returns:
        Callable: The decorated function.
    """

    def decorator(func):
        func._is_state = True
        func._state_name = name
        func._state_first = first
        func._state_type = _StateType.TIMED
        func._state_duration = duration
        func._state_loop = loop
        return func

    return decorator


def expose_zmq(name: str):
    """
    Decorator to expose a state for remote control via ZMQ.

    Args:
        name (str): The name of the state to expose.

    Returns:
        Callable: The decorated function.

    Raises:
        InvalidStateNameError: If the name is empty.
    """
    if name == "":
        raise InvalidStateNameError()

    def decorator(func):
        func._is_exposed_zmq = True
        func._zmq_name = name
        return func

    return decorator


def expose_field_zmq(name: str):
    """
    Decorator to make a field requestable via ZMQ.

    Args:
        name (str): The name of the field to expose.

    Returns:
        Callable: The decorated function.

    Raises:
        InvalidStateNameError: If the name is empty.
    """
    if name == "":
        raise InvalidStateNameError()

    def decorator(func):
        func._is_exposed_field_zmq = True
        func._zmq_name = name
        return func

    return decorator


_BackgroundTask = Callable[[Runner, Vehicle], None]


def background(func):
    """
    Designate a function to be run in parallel to a StateMachine.

    Args:
        func: The asynchronous function to run in the background.

    Returns:
        Callable: The decorated function.
    """
    func._is_background = True
    return func


_InitializationTask = Callable[[Runner, Vehicle], None]


def at_init(func):
    """
    Designate a function to be run during vehicle initialization.

    The function will run before the vehicle is armed.

    Args:
        func: The asynchronous function to run.

    Returns:
        Callable: The decorated function.
    """
    func._run_at_init = True
    return func


class StateMachine(Runner):
    """
    A runner that executes states in a sequence.

    Each state returns the name of the next state to transition to.
    Supports background tasks and initialization tasks.

    Attributes:
        _states (Dict[str, _State]): Mapping of state names to _State objects.
        _background_tasks (List[_BackgroundTask]): List of background functions to run.
        _initialization_tasks (List[_InitializationTask]): Functions to run at init.
        _entrypoint (str): The name of the initial state.
        _current_state (str): The name of the state currently being executed.
        _running (bool): Whether the state machine is active.
    """

    _states: Dict[str, _State]
    _background_tasks: List[_BackgroundTask]
    _initialization_tasks: List[_InitializationTask]
    _entrypoint: str
    _current_state: str
    _override_next_state_transition: bool
    _running: bool

    def _build(self):
        """
        Introspect the class to identify states, background tasks, and init tasks.

        Raises:
            MultipleInitialStatesError: If more than one state is marked 'first'.
            NoInitialStateError: If no initial state is found.
        """
        self._states = {}
        self._background_tasks = []
        self._initialization_tasks = []
        for _, method in inspect.getmembers(self):
            if not inspect.ismethod(method):
                continue
            if hasattr(method, "_is_state"):
                self._states[method._state_name] = _State(
                    method, method._state_name
                )
                if method._state_first and not hasattr(self, "_entrypoint"):
                    self._entrypoint = method._state_name
                elif method._state_first and hasattr(self, "_entrypoint"):
                    raise MultipleInitialStatesError()
            if hasattr(method, "_is_background"):
                self._background_tasks.append(method)
            if hasattr(method, "_run_at_init"):
                self._initialization_tasks.append(method)
        if not self._entrypoint:
            raise NoInitialStateError()

    async def _start_background_tasks(self, vehicle: Vehicle):
        """
        Start all background tasks in the asyncio event loop.

        Args:
            vehicle: The vehicle instance.
        """
        for task in self._background_tasks:

            async def _task_runner(t=task):
                while self._running:
                    await t.__func__(self, vehicle)

            asyncio.ensure_future(_task_runner())

    async def run(self, vehicle: Vehicle, build_before_running=True):
        """
        Execute the state machine logic.

        Args:
            vehicle (Vehicle): The vehicle instance.
            build_before_running (bool): Whether to call _build() first.
                Defaults to True.

        Raises:
            InvalidStateError: If the machine transitions to an unregistered state.
        """
        if build_before_running:
            self._build()
        assert self._entrypoint
        self._current_state = self._entrypoint
        self._override_next_state_transition = False
        self._next_state_overr = ""
        self._running = True

        if len(self._initialization_tasks) != 0:
            await asyncio.wait(
                {f(vehicle) for f in self._initialization_tasks}
            )

        await self._start_background_tasks(vehicle)

        while self._running:
            if self._current_state not in self._states:
                raise InvalidStateError(
                    self._current_state, list(self._states.keys())
                )

            next_state = await self._states[self._current_state].run(
                self, vehicle
            )
            if self._override_next_state_transition:
                self._override_next_state_transition = False
                self._current_state = self._next_state_overr
            else:
                self._current_state = next_state

            if self._current_state is None:
                self.stop()
            await asyncio.sleep(STATE_MACHINE_DELAY_S)
        self.cleanup()

    def stop(self):
        """
        Call `stop` to stop the execution of the `StateMachine` after
        completion of the current state. This is equivalent to returning `None`
        at the end of a state's execution.
        """
        self._running = False


class ZmqStateMachine(StateMachine):
    """
    A StateMachine that can be controlled remotely via ZMQ.

    Attributes:
        _exported_states (Dict[str, _State]): States exposed for ZMQ transitions.
        _exported_fields (Dict[str, Callable]): Fields exposed for ZMQ queries.
        _zmq_identifier (str): Unique name for this machine in the ZMQ network.
        _zmq_proxy_server (str): Address of the ZMQ proxy.
    """

    _exported_states: Dict[str, _State]

    def _build(self):
        super()._build()
        self._exported_states = {}
        self._exported_fields = {}
        for _, method in inspect.getmembers(self):
            if not inspect.ismethod(method):
                continue
            if hasattr(method, "_is_exposed_zmq"):
                self._exported_states[method._zmq_name] = _State(
                    method, method._zmq_name
                )
            elif hasattr(method, "_is_exposed_field_zmq"):
                self._exported_fields[method._zmq_name] = method

    _zmq_identifier: str
    _zmq_proxy_server: str

    def _initialize_zmq_bindings(
        self, vehicle_identifier: str, proxy_server_addr: str
    ):
        """
        Configure ZMQ connection parameters.

        Args:
            vehicle_identifier (str): The identifier for this vehicle.
            proxy_server_addr (str): The address of the ZMQ proxy server.
        """
        self._zmq_identifier = vehicle_identifier
        self._zmq_proxy_server = proxy_server_addr
        self._zmq_context = zmq.asyncio.Context()

    @background
    async def _zmg_bg_sub(self, vehicle: Vehicle):
        """
        Background task to subscribe to ZMQ messages.

        Args:
            vehicle: The vehicle instance.
        """
        socket = zmq.asyncio.Socket(
            context=self._zmq_context,
            io_loop=asyncio.get_event_loop(),
            socket_type=zmq.SUB,
        )
        socket.connect(f"tcp://{self._zmq_proxy_server}:{ZMQ_PROXY_OUT_PORT}")

        socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self._zmq_messages_handling = asyncio.Queue()
        self._zmq_received_fields = {}  # indexed by [identifier][field]

        while self._running:
            message = await socket.recv_pyobj()
            if message["identifier"] != self._zmq_identifier:
                continue
            asyncio.ensure_future(self._zmq_handle_request(vehicle, message))

    async def _zmq_handle_request(self, vehicle: Vehicle, message):
        """
        Handle an incoming ZMQ request.

        Args:
            vehicle: The vehicle instance.
            message (dict): The received message object.
        """
        if message["msg_type"] == ZMQ_TYPE_TRANSITION:
            next_state = message["next_state"]
            self._next_state_overr = next_state
            self._override_next_state_transition = True
        elif message["msg_type"] == ZMQ_TYPE_FIELD_REQUEST:
            field = message["field"]
            return_val = None
            if field in self._exported_fields:
                return_val = await self._exported_fields[field](vehicle)
            await self._reply_queried_field(message["from"], field, return_val)
        elif message["msg_type"] == ZMQ_TYPE_FIELD_CALLBACK:
            field = message["field"]
            value = message["value"]
            msg_from = message["from"]
            self._zmq_received_fields[msg_from][field] = value

    @background
    async def _zmq_bg_pub(self, _: Vehicle):
        """
        Background task to publish ZMQ messages.

        Args:
            _: The vehicle instance.
        """
        self._zmq_messages_sending = asyncio.Queue()
        socket = zmq.asyncio.Socket(
            context=self._zmq_context,
            io_loop=asyncio.get_event_loop(),
            socket_type=zmq.PUB,
        )
        socket.connect(f"tcp://{self._zmq_proxy_server}:{ZMQ_PROXY_IN_PORT}")
        while self._running:
            msg_sending = await self._zmq_messages_sending.get()
            await socket.send_pyobj(msg_sending)

    async def run(self, vehicle: Vehicle, zmq_proxy=False):
        """
        Execute the ZMQ-enabled state machine.

        Args:
            vehicle (Vehicle): The vehicle instance.
            zmq_proxy (bool): Unused, for compatibility.

        Raises:
            StateMachineError: If ZMQ bindings were not initialized.
        """
        self._build()

        if None in [self._zmq_identifier, self._zmq_proxy_server]:
            from .exceptions import StateMachineError

            raise StateMachineError(
                "initialize_zmq_bindings must be used with a zmq runner"
            )

        await super().run(vehicle, build_before_running=False)

    async def transition_runner(self, identifier: str, state: str):
        """
        Use zmq to transition a runner within the network specified by `identifier`.
        The state to be transitioned to is specified with `state`
        """
        transition_obj = {
            "msg_type": ZMQ_TYPE_TRANSITION,
            "from": self._zmq_identifier,
            "identifier": identifier,
            "next_state": state,
        }
        await self._zmq_messages_sending.put(transition_obj)

    async def query_field(self, identifier: str, field: str):
        """
        Query a field from another ZMQ runner.

        Args:
            identifier (str): Identifier of the target runner.
            field (str): Name of the field to query.

        Returns:
            Any: The value returned by the target runner.
        """
        if identifier not in self._zmq_received_fields:
            self._zmq_received_fields[identifier] = {}
        self._zmq_received_fields[identifier][field] = None
        query_obj = {
            "msg_type": ZMQ_TYPE_FIELD_REQUEST,
            "from": self._zmq_identifier,
            "identifier": identifier,
            "field": field,
        }
        await self._zmq_messages_sending.put(query_obj)
        while self._zmq_received_fields[identifier][field] == None:
            await asyncio.sleep(0.01)
        return self._zmq_received_fields[identifier][field]

    async def _reply_queried_field(self, identifier: str, field: str, value):
        """
        Send a reply to a field query.

        Args:
            identifier (str): Identifier of the runner that requested the field.
            field (str): Name of the field.
            value (Any): Value of the field.
        """
        reply_obj = {
            "msg_type": ZMQ_TYPE_FIELD_CALLBACK,
            "from": self._zmq_identifier,
            "identifier": identifier,
            "field": field,
            "value": value,
        }
        await self._zmq_messages_sending.put(reply_obj)


# helper functions for working with asyncio code
in_background = asyncio.ensure_future
sleep = asyncio.sleep
