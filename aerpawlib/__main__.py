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

# Removed static imports so we can select API version at runtime
# (the appropriate modules will be imported after parsing CLI args)

import asyncio
import importlib
import inspect
import json
import logging
import os
import signal
import sys
import time
from argparse import ArgumentParser
from typing import Optional


# Configure logging
def setup_logging(
    verbose: bool = False,
    debug: bool = False,
    quiet: bool = False,
    log_file: Optional[str] = None,
) -> logging.Logger:
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
    # Configure the ROOT logger to capture logs from all modules
    root_logger = logging.getLogger()

    # Determine log level
    if debug:
        level = logging.DEBUG
    elif verbose:
        level = logging.INFO
    elif quiet:
        level = logging.WARNING
    else:
        level = logging.INFO

    root_logger.setLevel(level)

    # Remove existing handlers to prevent duplicates
    root_logger.handlers.clear()

    # Console handler with colored output
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)

    # Custom formatter with colors and aerpawlib prefix
    class AerpawFormatter(logging.Formatter):
        """Custom formatter with colors for different log levels."""

        COLORS = {
            logging.DEBUG: "\033[36m",  # Cyan
            logging.INFO: "\033[32m",  # Green
            logging.WARNING: "\033[33m",  # Yellow
            logging.ERROR: "\033[31m",  # Red
            logging.CRITICAL: "\033[35m",  # Magenta
        }
        RESET = "\033[0m"
        BOLD = "\033[1m"

        def format(self, record):
            # Add color based on level
            color = self.COLORS.get(record.levelno, self.RESET)

            # Format timestamp
            timestamp = time.strftime(
                "%H:%M:%S", time.localtime(record.created)
            )

            # Determine logger name prefix
            name = record.name
            if name == "root":
                name = "aerpawlib"
            elif name.startswith("aerpawlib."):
                # Shorten aerpawlib.v1.vehicle -> v1.vehicle
                name = name[10:]  # Remove "aerpawlib."
            elif "." in name:
                # For user scripts, show last two parts: examples.legacy.basic_runner -> legacy.basic_runner
                parts = name.split(".")
                name = ".".join(parts[-2:]) if len(parts) >= 2 else parts[-1]

            level_name = record.levelname[0]  # Single letter: D, I, W, E, C

            if record.levelno >= logging.WARNING:
                prefix = f"{color}{self.BOLD}[{name} {level_name}]{self.RESET}"
            else:
                prefix = f"{color}[{name}]{self.RESET}"

            # Include timestamp for debug level
            if record.levelno == logging.DEBUG:
                return f"{prefix} {color}{timestamp}{self.RESET} {record.getMessage()}"
            else:
                return f"{prefix} {record.getMessage()}"

    console_handler.setFormatter(AerpawFormatter())
    root_logger.addHandler(console_handler)

    # File handler (if specified)
    if log_file:
        file_handler = logging.FileHandler(log_file, mode="a")
        file_handler.setLevel(logging.DEBUG)  # Always log everything to file
        file_formatter = logging.Formatter(
            "%(asctime)s [%(levelname)s] %(name)s: %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )
        file_handler.setFormatter(file_formatter)
        root_logger.addHandler(file_handler)

    # Return the aerpawlib-specific logger for the main module
    return logging.getLogger("aerpawlib")


# Global logger instance (configured in main())
logger: Optional[logging.Logger] = None


def discover_runner(api_module, experimenter_script):
    """Search for a Runner class in the experimenter script."""
    Runner = getattr(api_module, "Runner")
    StateMachine = getattr(api_module, "StateMachine")
    BasicRunner = getattr(api_module, "BasicRunner")
    # ZmqStateMachine only exists in v1/legacy
    ZmqStateMachine = getattr(api_module, "ZmqStateMachine", None)

    runner = None
    flag_zmq_runner = False

    logger.debug("Searching for Runner class in script...")
    for name, val in inspect.getmembers(experimenter_script):
        if not inspect.isclass(val):
            continue
        if not issubclass(val, Runner):
            continue
        if val in [StateMachine, BasicRunner, ZmqStateMachine]:
            continue
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


def run_v2_experiment(args, unknown_args, api_module, experimenter_script):
    """Run an experiment using the v2 API."""
    runner, flag_zmq_runner = discover_runner(api_module, experimenter_script)

    Vehicle = getattr(api_module, "Vehicle")
    Drone = getattr(api_module, "Drone")
    Rover = getattr(api_module, "Rover")
    MockDrone = getattr(api_module, "MockDrone", None)
    AERPAWPlatform = getattr(api_module, "AERPAWPlatform", None)

    vehicle_type = {
        "generic": Vehicle,
        "drone": Drone,
        "rover": Rover,
        "none": MockDrone,
    }.get(args.vehicle, None)

    if vehicle_type is None:
        logger.error(f"Invalid vehicle type: {args.vehicle}")
        raise Exception("Please specify a valid vehicle type")

    # Connection
    logger.info(f"Connecting to vehicle...")
    logger.debug(f"Connection string: {args.conn}")

    async def create_and_connect():
        v = vehicle_type(args.conn)
        await v.connect(
            timeout=args.conn_timeout,
            retry_count=args.conn_retries,
            retry_delay=args.conn_retry_delay,
        )
        return v

    try:
        vehicle: Vehicle = asyncio.run(
            asyncio.wait_for(
                create_and_connect(),
                timeout=args.conn_timeout * args.conn_retries + 10,
            )
        )
        logger.info("Vehicle connected successfully")
    except Exception as e:
        logger.error(f"Failed to connect to vehicle: {e}")
        raise ConnectionError(f"Could not connect to vehicle: {e}")

    # Shutdown handler
    def handle_shutdown(signum, frame):
        sig_name = signal.Signals(signum).name
        logger.warning(f"Received {sig_name}, initiating graceful shutdown...")
        if vehicle:
            if hasattr(vehicle, "armed") and vehicle.armed:
                logger.warning(
                    "Vehicle is armed, attempting emergency landing..."
                )
                try:
                    import concurrent.futures

                    future = asyncio.run_coroutine_threadsafe(
                        vehicle._system.action.land(), vehicle._mavsdk_loop
                    )
                    future.result(timeout=10)
                    logger.info("Emergency landing command sent")
                except Exception as e:
                    logger.error(f"Emergency landing failed: {e}")
            asyncio.run(vehicle.disconnect())
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    if args.debug_dump and hasattr(vehicle, "_verbose_logging"):
        vehicle._verbose_logging = True

    aerpaw_platform = (
        AERPAWPlatform(suppress_stdout=args.no_stdout)
        if AERPAWPlatform
        else None
    )

    logger.debug("Initializing runner arguments...")
    runner.initialize_args(unknown_args)

    if flag_zmq_runner:
        runner._initialize_zmq_bindings(
            args.zmq_identifier, args.zmq_server_addr
        )

    logger.info("=" * 50)
    logger.info("Starting experiment execution (v2)")
    logger.info("=" * 50)

    experiment_success = False
    connection_lost = False
    try:
        asyncio.run(runner.run(vehicle))
        experiment_success = True
        logger.info("Experiment completed successfully")
    except Exception as e:
        logger.error(f"Experiment failed: {e}")
        if isinstance(
            e, (ConnectionResetError, BrokenPipeError, TimeoutError)
        ):
            connection_lost = True

    # Cleanup
    if vehicle and not connection_lost:
        if vehicle.armed and args.rtl_at_end:
            logger.warning("Vehicle still armed! RTLing...")
            if aerpaw_platform:
                try:
                    asyncio.run(
                        aerpaw_platform.log_to_oeo(
                            "[aerpawlib] Vehicle still armed! RTLing..."
                        )
                    )
                except:
                    pass
            asyncio.run(vehicle.rtl())

        # Duration
        try:
            if (
                hasattr(vehicle, "_mission_start_time")
                and vehicle._mission_start_time
            ):
                duration = int(time.time() - vehicle._mission_start_time)
                msg = f"Mission took {duration // 60:02d}:{duration % 60:02d}"
                logger.info(msg)
                if aerpaw_platform:
                    try:
                        asyncio.run(
                            aerpaw_platform.log_to_oeo(f"[aerpawlib] {msg}")
                        )
                    except:
                        pass
        except:
            pass
        asyncio.run(vehicle.disconnect())


def run_v1_experiment(
    args, unknown_args, api_module, experimenter_script, version_name="v1"
):
    """Run an experiment using the v1/legacy API."""
    runner, flag_zmq_runner = discover_runner(api_module, experimenter_script)

    Vehicle = getattr(api_module, "Vehicle")
    Drone = getattr(api_module, "Drone")
    Rover = getattr(api_module, "Rover")
    DummyVehicle = getattr(api_module, "DummyVehicle", None)
    AERPAW_Platform = getattr(api_module, "AERPAW_Platform", None)

    vehicle_type = {
        "generic": Vehicle,
        "drone": Drone,
        "rover": Rover,
        "none": DummyVehicle,
    }.get(args.vehicle, None)

    if vehicle_type is None:
        logger.error(f"Invalid vehicle type: {args.vehicle}")
        raise Exception("Please specify a valid vehicle type")

    # Connection with retry
    vehicle = None
    last_error = None
    for attempt in range(1, args.conn_retries + 1):
        logger.info(
            f"Connecting to vehicle (attempt {attempt}/{args.conn_retries})..."
        )
        try:

            async def create_vehicle():
                v = vehicle_type(args.conn)
                if hasattr(v, "_connected"):
                    start = time.time()
                    while (
                        not v._connected
                        and (time.time() - start) < args.conn_timeout
                    ):
                        await asyncio.sleep(0.1)
                    if not v._connected:
                        raise TimeoutError("Connection timeout")
                return v

            vehicle = asyncio.run(
                asyncio.wait_for(create_vehicle(), timeout=args.conn_timeout)
            )
            break
        except Exception as e:
            last_error = e
            logger.warning(f"Attempt {attempt} failed: {e}")
            if attempt < args.conn_retries:
                time.sleep(args.conn_retry_delay)

    if not vehicle:
        raise ConnectionError(f"Could not connect: {last_error}")

    # Shutdown
    def handle_shutdown(signum, frame):
        logger.warning("Initiating graceful shutdown...")
        if vehicle:
            vehicle.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    if AERPAW_Platform:
        AERPAW_Platform._no_stdout = args.no_stdout

    runner.initialize_args(unknown_args)
    if args.initialize and hasattr(vehicle, "_initialize_prearm"):
        vehicle._initialize_prearm(args.initialize)

    if flag_zmq_runner:
        runner._initialize_zmq_bindings(
            args.zmq_identifier, args.zmq_server_addr
        )

    logger.info("=" * 50)
    logger.info(f"Starting experiment execution ({version_name})")
    logger.info("=" * 50)

    experiment_success = False
    try:
        asyncio.run(runner.run(vehicle))
        experiment_success = True
    except Exception as e:
        logger.error(f"Experiment failed: {e}")

    # RTL/Cleanup
    if vehicle:
        if vehicle.armed and args.rtl_at_end:
            logger.warning("Vehicle still armed! RTLing...")
            try:
                if args.vehicle == "drone":
                    asyncio.run(vehicle.return_to_launch())
                elif args.vehicle == "rover" and vehicle.home_coords:
                    asyncio.run(vehicle.goto_coordinates(vehicle.home_coords))
            except Exception as e:
                logger.error(f"RTL failed: {e}")
        vehicle.close()

    sys.exit(0 if experiment_success else 1)


def main():
    """Main entry point for aerpawlib CLI."""

    # Import trick to allow things to work when run as "aerpawlib"

    current_file = os.path.abspath(__file__)

    #  Go up two levels to find the Project Root
    #    Level 1: /.../aerpawlib-vehicle-control/aerpawlib
    #    Level 2: /.../aerpawlib-vehicle-control (Where 'examples' lives)
    project_root = os.path.dirname(os.path.dirname(current_file))

    # Set the Process CWD to the Project Root
    #    (This does NOT change the terminal's path, only this running process)
    os.chdir(project_root)
    sys.path.insert(0, os.getcwd())

    # Pre-parse for config file
    conf_parser = ArgumentParser(add_help=False)
    conf_parser.add_argument(
        "--config", help="path to JSON configuration file"
    )
    conf_args, _ = conf_parser.parse_known_args()

    cli_args = sys.argv[1:]

    # If config file is provided, load it and merge arguments
    if conf_args.config:
        if not os.path.exists(conf_args.config):
            print(f"Config file not found: {conf_args.config}")
            sys.exit(1)

        try:
            with open(conf_args.config, "r") as f:
                config_data = json.load(f)

            config_cli_args = []
            for key, value in config_data.items():
                if isinstance(value, bool):
                    if value:
                        config_cli_args.append(f"--{key}")
                    # for store_false args, if user puts "skip-init": false in json,
                    # it means they DON'T want to skip init (which is default).
                    # So we don't add --skip-init flag.
                    # if user puts "skip-init": true, we add --skip-init.
                elif isinstance(value, list):
                    for item in value:
                        config_cli_args.append(f"--{key}")
                        config_cli_args.append(str(item))
                else:
                    config_cli_args.append(f"--{key}")
                    config_cli_args.append(str(value))

            # Prepend config args to CLI args so CLI overrides config
            cli_args = config_cli_args + cli_args

        except Exception as e:
            print(f"Error loading config file: {e}")
            sys.exit(1)

    proxy_mode = "--run-proxy" in cli_args

    parser = ArgumentParser(
        description="aerpawlib - wrap and run aerpaw experimenter scripts"
    )
    parser.add_argument(
        "--config",
        help="path to JSON configuration file. Keys are other arguments.\n"
        "Providing arguments to aerpawlib will override the config file.",
    )

    # Core Arguments
    core_grp = parser.add_argument_group("Core Arguments")
    core_grp.add_argument(
        "--script", help="experimenter script", required=not proxy_mode
    )
    core_grp.add_argument(
        "--conn", help="connection string", required=not proxy_mode
    )
    core_grp.add_argument(
        "--vehicle",
        help="vehicle type",
        choices=["generic", "drone", "rover", "none"],
        required=not proxy_mode,
    )
    core_grp.add_argument(
        "--api-version",
        help="which API version to use (legacy, v1 or v2)",
        choices=["legacy", "v1", "v2"],
        default="v1",
    )

    # Execution Flags
    exec_grp = parser.add_argument_group("Execution Flags")
    exec_grp.add_argument(
        "--skip-init",
        help="skip initialization",
        action="store_false",
        dest="initialize",
    )
    exec_grp.add_argument(
        "--skip-rtl",
        help="don't rtl and land at the end of an experiment automatically",
        action="store_false",
        dest="rtl_at_end",
    )
    exec_grp.add_argument(
        "--debug-dump",
        help="run aerpawlib's internal debug dump on vehicle object",
        action="store_true",
        dest="debug_dump",
    )
    exec_grp.add_argument(
        "--no-aerpawlib-stdout",
        help="prevent aerpawlib from printing to stdout",
        action="store_true",
        dest="no_stdout",
    )

    # ZMQ Proxy Arguments
    zmq_grp = parser.add_argument_group("ZMQ Proxy")
    zmq_grp.add_argument(
        "--run-proxy",
        help="run zmq proxy",
        action="store_true",
        dest="run_zmq_proxy",
    )
    zmq_grp.add_argument(
        "--zmq-identifier", help="zmq identifier", dest="zmq_identifier"
    )
    zmq_grp.add_argument(
        "--zmq-proxy-server",
        help="zmq proxy server addr",
        dest="zmq_server_addr",
    )

    # Logging Arguments
    log_grp = parser.add_argument_group("Logging")
    log_grp.add_argument(
        "-v",
        "--verbose",
        help="enable verbose logging (INFO level)",
        action="store_true",
    )
    log_grp.add_argument(
        "--debug",
        help="enable debug logging (DEBUG level, very verbose)",
        action="store_true",
    )
    log_grp.add_argument(
        "-q",
        "--quiet",
        help="suppress most output (WARNING level only)",
        action="store_true",
    )
    log_grp.add_argument(
        "--log-file",
        help="write logs to file in addition to console",
        dest="log_file",
    )

    # Connection Handling Arguments
    conn_grp = parser.add_argument_group("Connection Tuning")
    conn_grp.add_argument(
        "--conn-timeout",
        help="connection timeout in seconds (default: 30)",
        type=float,
        default=30.0,
        dest="conn_timeout",
    )
    conn_grp.add_argument(
        "--conn-retries",
        help="number of connection retry attempts (default: 3)",
        type=int,
        default=3,
        dest="conn_retries",
    )
    conn_grp.add_argument(
        "--conn-retry-delay",
        help="delay between connection retries in seconds (default: 5)",
        type=float,
        default=5.0,
        dest="conn_retry_delay",
    )
    conn_grp.add_argument(
        "--heartbeat-timeout",
        help="max seconds without heartbeat before considered disconnected (default: 5)",
        type=float,
        default=5.0,
        dest="heartbeat_timeout",
    )

    args, unknown_args = parser.parse_known_args(
        args=cli_args
    )  # we'll pass other args to the script

    # Initialize logging based on command line arguments
    global logger
    logger = setup_logging(
        verbose=args.verbose,
        debug=args.debug,
        quiet=args.quiet,
        log_file=args.log_file,
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

    # Dynamically import API module
    api_version = args.api_version
    logger.debug(f"Loading API version: {api_version}")
    try:
        api_module = importlib.import_module(f"aerpawlib.{api_version}")
        # Inject into globals for backward compatibility in some scripts if needed
        for name in dir(api_module):
            if not name.startswith("_"):
                globals()[name] = getattr(api_module, name)
    except Exception as e:
        logger.error(f"Failed to import aerpawlib {api_version}: {e}")
        sys.exit(1)

    if args.run_zmq_proxy:
        logger.info("Starting ZMQ proxy mode")
        if hasattr(api_module, "run_zmq_proxy"):
            api_module.run_zmq_proxy()
        else:
            logger.error(f"API {api_version} does not support ZMQ proxy")
        sys.exit(0)

    # import script
    logger.debug(f"Loading experimenter script: {args.script}")
    try:
        experimenter_script = importlib.import_module(args.script)
    except Exception as e:
        logger.error(f"Failed to import script '{args.script}': {e}")
        sys.exit(1)

    # Dispatch to version-specific runner
    if api_version == "v2":
        run_v2_experiment(args, unknown_args, api_module, experimenter_script)
    elif api_version == "v1":
        run_v1_experiment(
            args,
            unknown_args,
            api_module,
            experimenter_script,
            version_name="v1",
        )
    elif api_version == "legacy":
        run_v1_experiment(
            args,
            unknown_args,
            api_module,
            experimenter_script,
            version_name="legacy",
        )


if __name__ == "__main__":
    main()
