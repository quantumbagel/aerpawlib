"""
Tool used to run aerpawlib scripts that make use of a Runner class.

usage:
    python -m aerpawlib --script <script import path> --conn <connection string> \
            --vehicle <vehicle type>

example:
    python -m aerpawlib --script experimenter_script --conn /dev/ttyACM0 \
            --vehicle drone

    python -m aerpawlib --script experimenter_script --conn udp:127.0.0.1:14550 \
            --vehicle drone
"""

import asyncio
import concurrent.futures
import importlib
import inspect
import json
import logging
import os
import signal
import sys
import time
import warnings
from argparse import ArgumentParser, Namespace
from dataclasses import dataclass
from typing import Optional, Any, List, Type


# =============================================================================
# Logging Configuration
# =============================================================================

def setup_logging(verbose: bool = False, debug: bool = False, quiet: bool = False,
                  log_file: Optional[str] = None) -> logging.Logger:
    """
    Configure logging for aerpawlib and user scripts.

    Configures the root logger so that logs from all modules (aerpawlib,
    user scripts, and libraries) are captured and formatted consistently.

    Args:
        verbose: Enable verbose (INFO level) logging
        debug: Enable debug (DEBUG level) logging
        quiet: Suppress most output (WARNING level only)
        log_file: Optional path to write logs to file

    Returns:
        Configured logger instance
    """
    root_logger = logging.getLogger()

    if debug:
        level = logging.DEBUG
    elif verbose:
        level = logging.INFO
    elif quiet:
        level = logging.WARNING
    else:
        level = logging.INFO

    root_logger.setLevel(level)
    root_logger.handlers.clear()

    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)

    class AerpawFormatter(logging.Formatter):
        """Custom formatter with colors for different log levels."""

        COLORS = {
            logging.DEBUG: '\033[36m',     # Cyan
            logging.INFO: '\033[32m',      # Green
            logging.WARNING: '\033[33m',   # Yellow
            logging.ERROR: '\033[31m',     # Red
            logging.CRITICAL: '\033[35m',  # Magenta
        }
        RESET = '\033[0m'
        BOLD = '\033[1m'

        def format(self, record):
            color = self.COLORS.get(record.levelno, self.RESET)
            timestamp = time.strftime('%H:%M:%S', time.localtime(record.created))

            name = record.name
            if name == "root":
                name = "aerpawlib"
            elif name.startswith("aerpawlib."):
                name = name[10:]
            elif "." in name:
                parts = name.split(".")
                name = ".".join(parts[-2:]) if len(parts) >= 2 else parts[-1]

            level_name = record.levelname[0]

            if record.levelno >= logging.WARNING:
                prefix = f"{color}{self.BOLD}[{name} {level_name}]{self.RESET}"
            else:
                prefix = f"{color}[{name}]{self.RESET}"

            if record.levelno == logging.DEBUG:
                return f"{prefix} {color}{timestamp}{self.RESET} {record.getMessage()}"
            else:
                return f"{prefix} {record.getMessage()}"

    console_handler.setFormatter(AerpawFormatter())
    root_logger.addHandler(console_handler)

    if log_file:
        file_handler = logging.FileHandler(log_file, mode='a')
        file_handler.setLevel(logging.DEBUG)
        file_formatter = logging.Formatter(
            '%(asctime)s [%(levelname)s] %(name)s: %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        file_handler.setFormatter(file_formatter)
        root_logger.addHandler(file_handler)

    return logging.getLogger("aerpawlib")


# Global logger instance
logger: Optional[logging.Logger] = None


# =============================================================================
# Shared Data Structures
# =============================================================================

@dataclass
class RunnerContext:
    """Context object passed to API-specific runners."""
    args: Namespace
    unknown_args: List[str]
    api_module: Any
    experimenter_script: Any
    runner: Any
    vehicle: Any
    vehicle_type: Type
    aerpaw_platform: Any
    flag_zmq_runner: bool


# =============================================================================
# Argument Parsing
# =============================================================================

def setup_project_path():
    """Set up project root path for imports."""
    current_file = os.path.abspath(__file__)
    project_root = os.path.dirname(os.path.dirname(current_file))
    os.chdir(project_root)
    sys.path.insert(0, os.getcwd())


def load_config_file(config_path: str) -> List[str]:
    """Load configuration from JSON file and return as CLI args."""
    if not os.path.exists(config_path):
        print(f"Config file not found: {config_path}")
        sys.exit(1)

    try:
        with open(config_path, 'r') as f:
            config_data = json.load(f)

        config_cli_args = []
        for key, value in config_data.items():
            if isinstance(value, bool):
                if value:
                    config_cli_args.append(f"--{key}")
            elif isinstance(value, list):
                for item in value:
                    config_cli_args.append(f"--{key}")
                    config_cli_args.append(str(item))
            else:
                config_cli_args.append(f"--{key}")
                config_cli_args.append(str(value))

        return config_cli_args
    except Exception as e:
        print(f"Error loading config file: {e}")
        sys.exit(1)


def parse_arguments() -> tuple[Namespace, List[str]]:
    """Parse command line arguments."""
    conf_parser = ArgumentParser(add_help=False)
    conf_parser.add_argument("--config", help="path to JSON configuration file")
    conf_args, _ = conf_parser.parse_known_args()

    cli_args = sys.argv[1:]

    if conf_args.config:
        config_cli_args = load_config_file(conf_args.config)
        cli_args = config_cli_args + cli_args

    proxy_mode = "--run-proxy" in cli_args

    parser = ArgumentParser(description="aerpawlib - wrap and run aerpaw experimenter scripts")
    parser.add_argument("--config", help="path to JSON configuration file")

    # Core Arguments
    core_grp = parser.add_argument_group("Core Arguments")
    core_grp.add_argument("--script", help="experimenter script", required=not proxy_mode)
    core_grp.add_argument("--conn", help="connection string", required=not proxy_mode)
    core_grp.add_argument("--vehicle", help="vehicle type",
                          choices=["generic", "drone", "rover", "none"], required=not proxy_mode)
    core_grp.add_argument("--api-version", help="which API version to use",
                          choices=["v1", "v2", "legacy"], default="v1")

    # Execution Flags
    exec_grp = parser.add_argument_group("Execution Flags")
    exec_grp.add_argument("--skip-init", help="skip initialization",
                          action="store_false", dest="initialize")
    exec_grp.add_argument("--skip-rtl", help="don't rtl and land at the end",
                          action="store_false", dest="rtl_at_end")
    exec_grp.add_argument("--debug-dump", help="run aerpawlib's internal debug dump",
                          action="store_true", dest="debug_dump")
    exec_grp.add_argument("--no-aerpawlib-stdout", help="prevent aerpawlib from printing to stdout",
                          action="store_true", dest="no_stdout")

    # ZMQ Proxy Arguments
    zmq_grp = parser.add_argument_group("ZMQ Proxy")
    zmq_grp.add_argument("--run-proxy", help="run zmq proxy",
                         action="store_true", dest="run_zmq_proxy")
    zmq_grp.add_argument("--zmq-identifier", help="zmq identifier", dest="zmq_identifier")
    zmq_grp.add_argument("--zmq-proxy-server", help="zmq proxy server addr", dest="zmq_server_addr")

    # Logging Arguments
    log_grp = parser.add_argument_group("Logging")
    log_grp.add_argument("-v", "--verbose", help="enable verbose logging", action="store_true")
    log_grp.add_argument("--debug", help="enable debug logging", action="store_true")
    log_grp.add_argument("-q", "--quiet", help="suppress most output", action="store_true")
    log_grp.add_argument("--log-file", help="write logs to file", dest="log_file")

    # Connection Handling Arguments
    conn_grp = parser.add_argument_group("Connection Tuning")
    conn_grp.add_argument("--conn-timeout", help="connection timeout in seconds",
                          type=float, default=30.0, dest="conn_timeout")
    conn_grp.add_argument("--conn-retries", help="number of connection retry attempts",
                          type=int, default=3, dest="conn_retries")
    conn_grp.add_argument("--conn-retry-delay", help="delay between retries",
                          type=float, default=5.0, dest="conn_retry_delay")
    conn_grp.add_argument("--heartbeat-timeout", help="max seconds without heartbeat",
                          type=float, default=5.0, dest="heartbeat_timeout")

    return parser.parse_known_args(args=cli_args)


def load_api_module(api_version: str) -> Any:
    """Load the API module for the specified version."""
    logger.debug(f"Loading API version: {api_version}")
    try:
        api_module = importlib.import_module(f"aerpawlib.{api_version}")
        return api_module
    except Exception as e:
        logger.error(f"Failed to import aerpawlib {api_version}: {e}")
        raise


def load_experimenter_script(script_path: str) -> Any:
    """Load the experimenter script module."""
    logger.debug(f"Loading experimenter script: {script_path}")
    try:
        script = importlib.import_module(script_path)
        logger.debug(f"Script loaded successfully from: {script.__file__}")
        return script
    except Exception as e:
        logger.error(f"Failed to import script '{script_path}': {e}")
        raise


def find_runner_class(script: Any, api_module: Any, additional_runner_bases: List[Type] = None):
    """
    Find the Runner class in an experimenter script.

    Args:
        script: The loaded experimenter script module
        api_module: The API module to get Runner classes from
        additional_runner_bases: Additional base classes to check for (e.g., v1 classes for legacy compat)

    Returns:
        Tuple of (runner_instance, is_zmq_runner)
    """
    Runner = getattr(api_module, 'Runner')
    StateMachine = getattr(api_module, 'StateMachine')
    BasicRunner = getattr(api_module, 'BasicRunner')
    ZmqStateMachine = getattr(api_module, 'ZmqStateMachine', None)

    # Build list of base classes to skip
    base_classes = [Runner, StateMachine, BasicRunner]
    if ZmqStateMachine is not None:
        base_classes.append(ZmqStateMachine)

    # Add additional runner bases (for legacy compatibility)
    all_runner_bases = [Runner]
    if additional_runner_bases:
        base_classes.extend(additional_runner_bases)
        all_runner_bases.extend([c for c in additional_runner_bases if 'Runner' in c.__name__])

    def is_runner_subclass(cls):
        """Check if cls is a subclass of any Runner base."""
        for base in all_runner_bases:
            try:
                if issubclass(cls, base):
                    return True
            except TypeError:
                pass
        return False

    runner = None
    flag_zmq_runner = False

    logger.debug("Searching for Runner class in script...")
    for name, val in inspect.getmembers(script):
        if not inspect.isclass(val):
            continue
        if not is_runner_subclass(val):
            continue
        if val in base_classes:
            continue

        # Check for ZmqStateMachine
        if ZmqStateMachine and issubclass(val, ZmqStateMachine):
            flag_zmq_runner = True
            logger.debug(f"Found ZmqStateMachine: {name}")

        if runner:
            logger.error("Multiple Runner classes found in script")
            raise Exception("You can only define one runner")

        logger.info(f"Found runner class: {name}")
        runner = val()

    if runner is None:
        logger.error("No Runner class found in script")
        raise Exception("No Runner class found in script")

    return runner, flag_zmq_runner


def setup_signal_handlers(vehicle: Any, has_mavsdk: bool = False):
    """Set up graceful shutdown signal handlers."""
    shutdown_requested = False

    def handle_shutdown_signal(signum, frame):
        nonlocal shutdown_requested
        sig_name = signal.Signals(signum).name

        if shutdown_requested:
            logger.warning(f"Received {sig_name} again, forcing exit...")
            sys.exit(1)

        shutdown_requested = True
        logger.warning(f"Received {sig_name}, initiating graceful shutdown...")

        if vehicle and hasattr(vehicle, 'armed') and vehicle.armed:
            logger.warning("Vehicle is armed, attempting emergency landing...")
            try:
                if has_mavsdk and hasattr(vehicle, '_mavsdk_loop') and vehicle._mavsdk_loop:
                    future = asyncio.run_coroutine_threadsafe(
                        vehicle._system.action.land(),
                        vehicle._mavsdk_loop
                    )
                    try:
                        future.result(timeout=10)
                        logger.info("Emergency landing command sent")
                    except concurrent.futures.TimeoutError:
                        logger.warning("Emergency landing command timed out")
            except Exception as e:
                logger.error(f"Emergency landing failed: {e}")

        if vehicle:
            try:
                vehicle.close()
                logger.debug("Vehicle connection closed")
            except Exception as e:
                logger.error(f"Error closing vehicle: {e}")

        sys.exit(0)

    signal.signal(signal.SIGINT, handle_shutdown_signal)
    signal.signal(signal.SIGTERM, handle_shutdown_signal)


def run_experiment(runner: Any, vehicle: Any) -> tuple[bool, bool]:
    """
    Run the experiment and handle errors.

    Returns:
        Tuple of (success, connection_lost)
    """
    logger.info("=" * 50)
    logger.info("Starting experiment execution")
    logger.info("=" * 50)

    experiment_success = False
    connection_lost = False

    try:
        asyncio.run(runner.run(vehicle))
        experiment_success = True
        logger.info("Experiment completed successfully")
    except KeyboardInterrupt:
        logger.warning("Experiment interrupted by user")
    except ConnectionResetError as e:
        connection_lost = True
        logger.error(f"Connection to vehicle was reset: {e}")
    except BrokenPipeError as e:
        connection_lost = True
        logger.error(f"Connection to vehicle broken: {e}")
    except TimeoutError as e:
        connection_lost = True
        logger.error(f"Connection timed out: {e}")
    except OSError as e:
        if e.errno in (104, 111, 32):
            connection_lost = True
            logger.error(f"Connection error: {e}")
        else:
            logger.error(f"OS error during experiment: {e}")
            logger.debug("Exception details:", exc_info=True)
    except Exception as e:
        logger.error(f"Experiment failed with error: {e}")
        logger.debug("Exception details:", exc_info=True)

    if connection_lost:
        logger.warning("CONNECTION LOST - Cannot communicate with vehicle!")
        logger.warning("If vehicle is airborne, it should RTL automatically (failsafe)")

    return experiment_success, connection_lost


def cleanup_and_exit(vehicle: Any, vehicle_type: Type, args: Namespace,
                     aerpaw_platform: Any, experiment_success: bool,
                     connection_lost: bool, is_async_platform: bool = False,
                     drone_class: Type = None):
    """Handle RTL cleanup and exit."""

    if drone_class and vehicle_type == drone_class and not connection_lost:
        try:
            if vehicle.armed and args.rtl_at_end:
                logger.warning("Vehicle still armed after experiment! RTLing and LANDing automatically.")

                if aerpaw_platform:
                    try:
                        msg = "[aerpawlib] Vehicle still armed after experiment! RTLing and LANDing automatically."
                        if is_async_platform:
                            asyncio.run(aerpaw_platform.log_to_oeo(msg))
                        else:
                            aerpaw_platform.log_to_oeo(msg)
                    except Exception:
                        pass

                async def _rtl():
                    await vehicle.return_to_launch()

                asyncio.run(_rtl())
        except Exception as e:
            logger.error(f"Post-experiment RTL failed: {e}")

        try:
            if hasattr(vehicle, '_mission_start_time') and vehicle._mission_start_time:
                seconds = int(time.time() - vehicle._mission_start_time)
                duration = f"{(seconds // 60):02d}:{(seconds % 60):02d}"
                logger.info(f"Mission duration: {duration} (mm:ss)")

                if aerpaw_platform:
                    try:
                        msg = f"[aerpawlib] Mission took {duration} mm:ss"
                        if is_async_platform:
                            asyncio.run(aerpaw_platform.log_to_oeo(msg))
                        else:
                            aerpaw_platform.log_to_oeo(msg)
                    except Exception:
                        pass
        except Exception as e:
            logger.debug(f"Could not calculate mission duration: {e}")

    # Close vehicle connection
    logger.debug("Closing vehicle connection...")
    try:
        vehicle.close()
        logger.debug("Vehicle connection closed")
    except Exception as e:
        logger.debug(f"Error closing vehicle connection: {e}")

    # Exit with appropriate code
    if connection_lost:
        logger.info("aerpawlib shutdown complete (connection was lost)")
        sys.exit(1)
    elif experiment_success:
        logger.info("aerpawlib shutdown complete")
        sys.exit(0)
    else:
        logger.info("aerpawlib shutdown complete (experiment had errors)")
        sys.exit(1)


# =============================================================================
# Legacy API Main Function (DroneKit-based)
# =============================================================================

def run_legacy(args: Namespace, unknown_args: List[str]):
    """
    Main function for legacy API (DroneKit-based).

    The legacy API uses DroneKit which is fully synchronous.
    Connection is handled in a thread pool to avoid blocking issues.
    """
    warnings.warn(
        "'legacy' API is deprecated and will be removed in a future release. "
        "Please use 'v1' or 'v2' instead.",
        DeprecationWarning
    )

    # Load API module
    api_module = load_api_module("legacy")

    # Handle ZMQ proxy mode
    if args.run_zmq_proxy:
        logger.info("Starting ZMQ proxy mode")
        api_module.run_zmq_proxy()
        return

    # Load experimenter script
    script = load_experimenter_script(args.script)

    # For legacy compatibility: scripts may import from aerpawlib.runner (which exports v1)
    # So we need to also check against v1 Runner classes
    v1_runner_classes = []
    try:
        v1_module = importlib.import_module("aerpawlib.v1")
        v1_Runner = getattr(v1_module, 'Runner', None)
        v1_StateMachine = getattr(v1_module, 'StateMachine', None)
        v1_BasicRunner = getattr(v1_module, 'BasicRunner', None)
        v1_ZmqStateMachine = getattr(v1_module, 'ZmqStateMachine', None)
        v1_runner_classes = [c for c in [v1_Runner, v1_StateMachine, v1_BasicRunner, v1_ZmqStateMachine] if c]
        logger.debug("Also checking against v1 Runner classes for legacy compatibility")
    except Exception as e:
        logger.debug(f"Could not import v1 module for compatibility check: {e}")

    # Find runner class
    runner, flag_zmq_runner = find_runner_class(script, api_module, v1_runner_classes)

    # Get vehicle classes
    Vehicle = api_module.Vehicle
    Drone = api_module.Drone
    Rover = api_module.Rover
    DummyVehicle = getattr(api_module, 'DummyVehicle', None)
    AERPAW_Platform = getattr(api_module, 'AERPAW_Platform', None)

    vehicle_type = {
        "generic": Vehicle,
        "drone": Drone,
        "rover": Rover,
        "none": DummyVehicle
    }.get(args.vehicle)

    if vehicle_type is None:
        logger.error(f"Invalid vehicle type: {args.vehicle}")
        raise Exception("Please specify a valid vehicle type")

    # Connect to vehicle using ThreadPoolExecutor (DroneKit is synchronous)
    vehicle = None
    last_error = None

    for attempt in range(1, args.conn_retries + 1):
        logger.info(f"Connecting to vehicle (attempt {attempt}/{args.conn_retries})...")
        logger.debug(f"Connection string: {args.conn}")
        logger.debug(f"Timeout: {args.conn_timeout}s")

        try:
            def create_vehicle():
                return vehicle_type(args.conn)

            with concurrent.futures.ThreadPoolExecutor() as executor:
                future = executor.submit(create_vehicle)
                try:
                    vehicle = future.result(timeout=args.conn_timeout)
                except concurrent.futures.TimeoutError:
                    raise TimeoutError(f"Connection timeout after {args.conn_timeout}s")

            logger.info("Vehicle connected successfully")
            break

        except TimeoutError as e:
            last_error = e
            logger.warning(f"Connection attempt {attempt} timed out: {e}")
        except ConnectionRefusedError as e:
            last_error = e
            logger.warning(f"Connection attempt {attempt} refused: {e}")
        except OSError as e:
            last_error = e
            logger.warning(f"Connection attempt {attempt} failed (OS error): {e}")
        except Exception as e:
            last_error = e
            logger.warning(f"Connection attempt {attempt} failed: {e}")

        if attempt < args.conn_retries:
            logger.info(f"Retrying in {args.conn_retry_delay}s...")
            time.sleep(args.conn_retry_delay)

    if vehicle is None:
        logger.error(f"Failed to connect after {args.conn_retries} attempts")
        logger.error(f"Last error: {last_error}")
        raise ConnectionError(f"Could not connect to vehicle: {last_error}")

    # Set up signal handlers (no MAVSDK in legacy)
    setup_signal_handlers(vehicle, has_mavsdk=False)

    # Enable debug dump if requested
    if args.debug_dump and hasattr(vehicle, "_verbose_logging"):
        vehicle._verbose_logging = True
        logger.debug("Verbose vehicle logging enabled")

    # Configure AERPAW platform
    aerpaw_platform = None
    if AERPAW_Platform is not None:
        AERPAW_Platform._no_stdout = args.no_stdout
        aerpaw_platform = AERPAW_Platform

    # Initialize runner
    logger.debug("Initializing runner arguments...")
    runner.initialize_args(unknown_args)

    # Pre-arm initialization
    if vehicle_type in [Drone, Rover] and args.initialize:
        logger.debug("Running pre-arm initialization...")
        vehicle._initialize_prearm(args.initialize)

    # ZMQ bindings
    if flag_zmq_runner:
        if None in [args.zmq_identifier, args.zmq_server_addr]:
            logger.error("ZMQ runner requires --zmq-identifier and --zmq-proxy-server")
            raise Exception("you must declare an identifier and server address for a zmq enabled state machine")
        logger.info(f"Initializing ZMQ bindings (ID: {args.zmq_identifier}, Server: {args.zmq_server_addr})")
        runner._initialize_zmq_bindings(args.zmq_identifier, args.zmq_server_addr)

    # Run experiment
    success, connection_lost = run_experiment(runner, vehicle)

    # Cleanup and exit
    cleanup_and_exit(vehicle, vehicle_type, args, aerpaw_platform,
                     success, connection_lost, is_async_platform=False, drone_class=Drone)



def run_v1(args: Namespace, unknown_args: List[str]):
    """
    Main function for v1 API (MAVSDK-based, API compatible with legacy).

    The v1 API uses MAVSDK with a synchronous-looking interface.
    Connection is handled synchronously but internally uses async in a background thread.
    """
    # Load API module
    api_module = load_api_module("v1")

    # Handle ZMQ proxy mode
    if args.run_zmq_proxy:
        logger.info("Starting ZMQ proxy mode")
        api_module.run_zmq_proxy()
        return

    # Load experimenter script
    script = load_experimenter_script(args.script)

    # Find runner class
    runner, flag_zmq_runner = find_runner_class(script, api_module)

    # Get vehicle classes
    Vehicle = api_module.Vehicle
    Drone = api_module.Drone
    Rover = api_module.Rover
    DummyVehicle = getattr(api_module, 'DummyVehicle', None)
    AERPAW_Platform = getattr(api_module, 'AERPAW_Platform', None)

    vehicle_type = {
        "generic": Vehicle,
        "drone": Drone,
        "rover": Rover,
        "none": DummyVehicle
    }.get(args.vehicle)

    if vehicle_type is None:
        logger.error(f"Invalid vehicle type: {args.vehicle}")
        raise Exception("Please specify a valid vehicle type")

    # Connect to vehicle (v1 constructor is synchronous but uses internal async)
    vehicle = None
    last_error = None

    for attempt in range(1, args.conn_retries + 1):
        logger.info(f"Connecting to vehicle (attempt {attempt}/{args.conn_retries})...")
        logger.debug(f"Connection string: {args.conn}")
        logger.debug(f"Timeout: {args.conn_timeout}s")

        try:
            # v1 Vehicle constructor handles connection synchronously
            vehicle = vehicle_type(args.conn)
            logger.info("Vehicle connected successfully")
            break

        except TimeoutError as e:
            last_error = e
            logger.warning(f"Connection attempt {attempt} timed out: {e}")
        except ConnectionRefusedError as e:
            last_error = e
            logger.warning(f"Connection attempt {attempt} refused: {e}")
        except OSError as e:
            last_error = e
            logger.warning(f"Connection attempt {attempt} failed (OS error): {e}")
        except Exception as e:
            last_error = e
            logger.warning(f"Connection attempt {attempt} failed: {e}")

        if attempt < args.conn_retries:
            logger.info(f"Retrying in {args.conn_retry_delay}s...")
            time.sleep(args.conn_retry_delay)

    if vehicle is None:
        logger.error(f"Failed to connect after {args.conn_retries} attempts")
        logger.error(f"Last error: {last_error}")
        raise ConnectionError(f"Could not connect to vehicle: {last_error}")

    # Set up signal handlers (v1 uses MAVSDK)
    setup_signal_handlers(vehicle, has_mavsdk=True)

    # Enable debug dump if requested
    if args.debug_dump and hasattr(vehicle, "_verbose_logging"):
        vehicle._verbose_logging = True
        logger.debug("Verbose vehicle logging enabled")

    # Configure AERPAW platform
    aerpaw_platform = None
    if AERPAW_Platform is not None:
        AERPAW_Platform._no_stdout = args.no_stdout
        aerpaw_platform = AERPAW_Platform

    # Initialize runner
    logger.debug("Initializing runner arguments...")
    runner.initialize_args(unknown_args)

    # Pre-arm initialization
    if vehicle_type in [Drone, Rover] and args.initialize:
        logger.debug("Running pre-arm initialization...")
        vehicle._initialize_prearm(args.initialize)

    # ZMQ bindings
    if flag_zmq_runner:
        if None in [args.zmq_identifier, args.zmq_server_addr]:
            logger.error("ZMQ runner requires --zmq-identifier and --zmq-proxy-server")
            raise Exception("you must declare an identifier and server address for a zmq enabled state machine")
        logger.info(f"Initializing ZMQ bindings (ID: {args.zmq_identifier}, Server: {args.zmq_server_addr})")
        runner._initialize_zmq_bindings(args.zmq_identifier, args.zmq_server_addr)

    # Run experiment
    success, connection_lost = run_experiment(runner, vehicle)

    # Cleanup and exit
    cleanup_and_exit(vehicle, vehicle_type, args, aerpaw_platform,
                     success, connection_lost, is_async_platform=False, drone_class=Drone)



def run_v2(args: Namespace, unknown_args: List[str]):
    """
    Main function for v2 API (async-first MAVSDK).

    The v2 API is fully async and requires explicit connect() calls.
    """
    # Load API module
    api_module = load_api_module("v2")

    # Handle ZMQ proxy mode (v2 may not have this)
    if args.run_zmq_proxy:
        if hasattr(api_module, 'run_zmq_proxy'):
            logger.info("Starting ZMQ proxy mode")
            api_module.run_zmq_proxy()
        else:
            logger.error("ZMQ proxy mode is not supported in v2 API")
            raise Exception("ZMQ proxy not available in v2")
        return

    # Load experimenter script
    script = load_experimenter_script(args.script)

    # Find runner class (v2 doesn't have ZmqStateMachine)
    runner, _ = find_runner_class(script, api_module)

    # Get vehicle classes (v2 uses different naming)
    Vehicle = api_module.Vehicle
    Drone = api_module.Drone
    Rover = api_module.Rover
    MockDrone = getattr(api_module, 'MockDrone', None)
    AERPAWPlatform = getattr(api_module, 'AERPAWPlatform', None)

    vehicle_type = {
        "generic": Vehicle,
        "drone": Drone,
        "rover": Rover,
        "none": MockDrone
    }.get(args.vehicle)

    if vehicle_type is None:
        logger.error(f"Invalid vehicle type: {args.vehicle}")
        raise Exception("Please specify a valid vehicle type")

    # Connect to vehicle (v2 is fully async)
    vehicle = None

    async def connect_v2():
        v = vehicle_type(args.conn)
        await v.connect(
            timeout=args.conn_timeout,
            retry_count=args.conn_retries,
            retry_delay=args.conn_retry_delay
        )
        return v

    logger.info("Connecting to vehicle...")
    logger.debug(f"Connection string: {args.conn}")
    logger.debug(f"Timeout: {args.conn_timeout}s, Retries: {args.conn_retries}")

    try:
        vehicle = asyncio.run(
            asyncio.wait_for(
                connect_v2(),
                timeout=args.conn_timeout * args.conn_retries
            )
        )
        logger.info("Vehicle connected successfully")
    except asyncio.TimeoutError:
        logger.error(f"Connection timed out after {args.conn_timeout * args.conn_retries}s")
        raise ConnectionError("Connection timeout")
    except Exception as e:
        logger.error(f"Failed to connect: {e}")
        raise ConnectionError(f"Could not connect to vehicle: {e}")

    # Set up signal handlers (v2 uses MAVSDK)
    setup_signal_handlers(vehicle, has_mavsdk=True)

    # Enable debug dump if requested
    if args.debug_dump and hasattr(vehicle, "_verbose_logging"):
        vehicle._verbose_logging = True
        logger.debug("Verbose vehicle logging enabled")

    # Configure AERPAW platform (v2 uses class instance)
    aerpaw_platform = None
    if AERPAWPlatform is not None:
        aerpaw_platform = AERPAWPlatform(suppress_stdout=args.no_stdout)

    # Initialize runner
    logger.debug("Initializing runner arguments...")
    runner.initialize_args(unknown_args)

    # v2 doesn't support pre-arm initialization
    if args.initialize:
        logger.debug("API v2 does not support pre-arm initialization, skipping...")

    # Run experiment
    success, connection_lost = run_experiment(runner, vehicle)

    # Cleanup and exit
    cleanup_and_exit(vehicle, vehicle_type, args, aerpaw_platform,
                     success, connection_lost, is_async_platform=True, drone_class=Drone)


# =============================================================================
# Main Entry Point
# =============================================================================

def main():
    """Main entry point for aerpawlib CLI."""
    global logger

    # Set up project path
    setup_project_path()

    # Parse arguments
    args, unknown_args = parse_arguments()

    # Initialize logging
    logger = setup_logging(
        verbose=args.verbose,
        debug=args.debug,
        quiet=args.quiet,
        log_file=args.log_file
    )

    # Log startup information
    logger.info("aerpawlib - AERPAW Vehicle Control Library")
    logger.info("By John Kesler and Julian Reder")
    logger.debug(f"Python version: {sys.version}")
    logger.debug(f"Working directory: {os.getcwd()}")
    logger.debug(f"API version: {args.api_version}")
    logger.debug(f"Script: {args.script}")
    logger.debug(f"Vehicle type: {args.vehicle}")
    logger.debug(f"Connection string: {args.conn}")
    logger.debug(f"Additional arguments: {unknown_args}")

    # Dispatch to API-specific main function
    api_version = args.api_version

    if api_version == "legacy":
        run_legacy(args, unknown_args)
    elif api_version == "v1":
        run_v1(args, unknown_args)
    elif api_version == "v2":
        run_v2(args, unknown_args)
    else:
        logger.error(f"Unknown API version: {api_version}")
        raise Exception(f"Unknown API version: {api_version}")


if __name__ == "__main__":
    main()

