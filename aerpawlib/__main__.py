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
import logging
import os
import sys
import time
from argparse import ArgumentParser
from typing import Optional


# Configure logging
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
            logging.DEBUG: '\033[36m',     # Cyan
            logging.INFO: '\033[32m',      # Green
            logging.WARNING: '\033[33m',   # Yellow
            logging.ERROR: '\033[31m',     # Red
            logging.CRITICAL: '\033[35m',  # Magenta
        }
        RESET = '\033[0m'
        BOLD = '\033[1m'

        def format(self, record):
            # Add color based on level
            color = self.COLORS.get(record.levelno, self.RESET)

            # Format timestamp
            timestamp = time.strftime('%H:%M:%S', time.localtime(record.created))

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
        file_handler = logging.FileHandler(log_file, mode='a')
        file_handler.setLevel(logging.DEBUG)  # Always log everything to file
        file_formatter = logging.Formatter(
            '%(asctime)s [%(levelname)s] %(name)s: %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        file_handler.setFormatter(file_formatter)
        root_logger.addHandler(file_handler)

    # Return the aerpawlib-specific logger for the main module
    return logging.getLogger("aerpawlib")


# Global logger instance (configured in main())
logger: Optional[logging.Logger] = None


def main():
    """Main entry point for aerpawlib CLI."""

    # Import trick to allow things to work when run as "aerpawlib"

    current_file = os.path.abspath(__file__)

    # 2. Go up two levels to find the Project Root
    #    Level 1: /.../aerpawlib-vehicle-control/aerpawlib
    #    Level 2: /.../aerpawlib-vehicle-control (Where 'examples' lives)
    project_root = os.path.dirname(os.path.dirname(current_file))

    # 3. Set the Process CWD to the Project Root
    #    (This does NOT change your terminal's path, only this running process)
    os.chdir(project_root)
    sys.path.insert(0, os.getcwd())

    proxy_mode = "--run-proxy" in sys.argv

    parser = ArgumentParser(description="aerpawlib - wrap and run aerpaw scripts")
    parser.add_argument("--script", help="experimenter script", required=not proxy_mode)
    parser.add_argument("--conn", help="connection string",
                        required=not proxy_mode)
    parser.add_argument("--vehicle", help="vehicle type [generic, drone, rover, none]", required=not proxy_mode)
    parser.add_argument("--api-version", help="which API version to use (v1 or v2)",
                        choices=["v1", "v2"], default="v1")
    parser.add_argument("--skip-init", help="skip initialization", required=False,
            const=False, default=True, action="store_const", dest="initialize")
    parser.add_argument("--run-proxy", help="run zmq proxy", required=False,
            const=True, default=False, action="store_const", dest="run_zmq_proxy")
    parser.add_argument("--zmq-identifier", help="zmq identifier", required=False, dest="zmq_identifier")
    parser.add_argument("--zmq-proxy-server", help="zmq proxy server addr", required=False, dest="zmq_server_addr")
    parser.add_argument("--skip-rtl", help="don't rtl and land at the end of an experiment automatically",
            const=False, default=True, action="store_const", dest="rtl_at_end")
    parser.add_argument("--debug-dump", help="run aerpawlib's internal debug dump on vehicle object", required=False,
            const=True, default=False, action="store_const", dest="debug_dump")
    parser.add_argument("--no-aerpawlib-stdout", help="prevent aerpawlib from printing to stdout", required=False,
            const=True, default=False, action="store_const", dest="no_stdout")

    # Logging arguments
    parser.add_argument("-v", "--verbose", help="enable verbose logging (INFO level)",
            action="store_true", default=False)
    parser.add_argument("--debug", help="enable debug logging (DEBUG level, very verbose)",
            action="store_true", default=False)
    parser.add_argument("-q", "--quiet", help="suppress most output (WARNING level only)",
            action="store_true", default=False)
    parser.add_argument("--log-file", help="write logs to file in addition to console",
            default=None, dest="log_file")


    args, unknown_args = parser.parse_known_args() # we'll pass other args to the script

    # Initialize logging based on command line arguments
    global logger
    logger = setup_logging(
        verbose=args.verbose,
        debug=args.debug,
        quiet=args.quiet,
        log_file=args.log_file
    )

    # Log startup information
    logger.info("=" * 50)
    logger.info("aerpawlib - AERPAW Vehicle Control Library")
    logger.info("=" * 50)
    logger.debug(f"Python version: {sys.version}")
    logger.debug(f"Working directory: {os.getcwd()}")
    logger.debug(f"API version: {args.api_version}")
    if args.script:
        logger.debug(f"Script: {args.script}")
    if args.vehicle:
        logger.debug(f"Vehicle type: {args.vehicle}")
    if args.conn:
        logger.debug(f"Connection string: {args.conn}")
    if unknown_args:
        logger.debug(f"Additional arguments: {unknown_args}")


    # Dynamically import all symbols from the selected API version into global namespace
    api_version = args.api_version
    logger.debug(f"Loading API version: {api_version}")
    try:
        api_module = importlib.import_module(f"aerpawlib.{api_version}")
        # Import all public symbols from the API module into globals
        imported_symbols = []
        if hasattr(api_module, '__all__'):
            for name in api_module.__all__:
                globals()[name] = getattr(api_module, name)
                imported_symbols.append(name)
        else:
            # If no __all__ is defined, import all non-private symbols
            for name in dir(api_module):
                if not name.startswith('_'):
                    globals()[name] = getattr(api_module, name)
                    imported_symbols.append(name)
        logger.debug(f"Imported {len(imported_symbols)} symbols from aerpawlib.{api_version}")
    except Exception as e:
        logger.error(f"Failed to import aerpawlib {api_version}: {e}")
        raise Exception(f"Failed to import aerpawlib {api_version}: {e}")

    if args.run_zmq_proxy:
        # don't even bother running the script, just the proxy
        logger.info("Starting ZMQ proxy mode")
        globals()['run_zmq_proxy']()
        exit()

    # import script and use reflection to get StateMachine
    logger.debug(f"Loading experimenter script: {args.script}")
    try:
        experimenter_script = importlib.import_module(args.script)
        logger.debug(f"Script loaded successfully from: {experimenter_script.__file__}")
    except Exception as e:
        logger.error(f"Failed to import script '{args.script}': {e}")
        raise

    runner = None
    flag_zmq_runner = False
    Runner = globals()['Runner']
    StateMachine = globals()['StateMachine']
    BasicRunner = globals()['BasicRunner']
    ZmqStateMachine = globals()['ZmqStateMachine']

    logger.debug("Searching for Runner class in script...")
    for name, val in inspect.getmembers(experimenter_script):
        if not inspect.isclass(val):
            continue
        if not issubclass(val, Runner):
            continue
        if val in [StateMachine, BasicRunner, ZmqStateMachine]:
            continue
        if issubclass(val, ZmqStateMachine):
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

    Vehicle = globals()['Vehicle']
    Drone = globals()['Drone']
    Rover = globals()['Rover']
    DummyVehicle = globals()['DummyVehicle']
    AERPAW_Platform = globals()['AERPAW_Platform']

    vehicle_type = {
            "generic": Vehicle,
            "drone": Drone,
            "rover": Rover,
            "none": DummyVehicle
            }.get(args.vehicle, None)
    if vehicle_type is None:
        logger.error(f"Invalid vehicle type: {args.vehicle}")
        raise Exception("Please specify a valid vehicle type")

    logger.info(f"Creating {args.vehicle} vehicle...")
    logger.debug(f"Connection string: {args.conn}")
    try:
        vehicle = vehicle_type(args.conn)
        logger.info(f"Vehicle created successfully")
    except Exception as e:
        logger.error(f"Failed to create vehicle: {e}")
        raise

    if args.debug_dump and hasattr(vehicle, "_verbose_logging"):
        vehicle._verbose_logging = True
        logger.debug("Verbose vehicle logging enabled")

    AERPAW_Platform._no_stdout = args.no_stdout

    # everything after this point is user script dependent. avoid adding extra logic below here

    logger.debug("Initializing runner arguments...")
    runner.initialize_args(unknown_args)
    
    if vehicle_type in [Drone, Rover] and args.initialize:
        logger.debug("Running pre-arm initialization...")
        vehicle._initialize_prearm(args.initialize)

    if flag_zmq_runner:
        if None in [args.zmq_identifier, args.zmq_server_addr]:
            logger.error("ZMQ runner requires --zmq-identifier and --zmq-proxy-server")
            raise Exception("you must declare an identifier and server address for a zmq enabled state machine")
        logger.info(f"Initializing ZMQ bindings (ID: {args.zmq_identifier}, Server: {args.zmq_server_addr})")
        runner._initialize_zmq_bindings(args.zmq_identifier, args.zmq_server_addr)

    logger.info("=" * 50)
    logger.info("Starting experiment execution")
    logger.info("=" * 50)

    start_time = time.time()
    try:
        asyncio.run(runner.run(vehicle))
        logger.info("Experiment completed successfully")
    except KeyboardInterrupt:
        logger.warning("Experiment interrupted by user")
    except Exception as e:
        logger.error(f"Experiment failed with error: {e}")
        logger.debug("Exception details:", exc_info=True)

    # rtl / land if not already done
    async def _rtl_cleanup(vehicle):
        await vehicle.goto_coordinates(vehicle._home_location)
        if vehicle_type in [Drone]:
            await vehicle.land()

    if vehicle_type in [Drone, Rover]:
        if vehicle.armed and args.rtl_at_end:
            logger.warning("Vehicle still armed after experiment! RTLing and LANDing automatically.")
            AERPAW_Platform.log_to_oeo("[aerpawlib] Vehicle still armed after experiment! RTLing and LANDing automatically.")
            asyncio.run(_rtl_cleanup(vehicle))

        stop_time = time.time()
        seconds_to_complete = int(stop_time - vehicle._mission_start_time)
        time_to_complete = f"{(seconds_to_complete // 60):02d}:{(seconds_to_complete % 60):02d}"
        logger.info(f"Mission duration: {time_to_complete} (mm:ss)")
        AERPAW_Platform.log_to_oeo(f"[aerpawlib] Mission took {time_to_complete} mm:ss")

    # clean up
    logger.debug("Closing vehicle connection...")
    vehicle.close()
    logger.debug("Vehicle connection closed")

    logger.info("aerpawlib shutdown complete")


if __name__ == "__main__":
    main()

