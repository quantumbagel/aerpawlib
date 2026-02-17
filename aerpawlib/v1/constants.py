"""
Central configuration constants for aerpawlib v1.

All timing, tolerance, and configuration values should be defined here
for easy tuning and documentation.
"""

import os

# Connection Constants

# Maximum time to wait for initial connection to vehicle (seconds)
CONNECTION_TIMEOUT_S = 30.0

# Maximum time to wait for vehicle to become armable (seconds)
ARMABLE_TIMEOUT_S = 60.0

# Maximum time to wait for GPS/position ready (seconds)
# SITL can report armable before autopilot has valid position for GUIDED mode
POSITION_READY_TIMEOUT_S = 60.0

# Interval for checking connection state (seconds)
POLLING_DELAY_S = 0.01

# Interval between heartbeat timeout checks (seconds)
HEARTBEAT_CHECK_INTERVAL_S = 1.0

# Maximum time since last heartbeat before considering vehicle disconnected (seconds)
HEARTBEAT_TIMEOUT_S = 1.0


# Safety Initialization Constants


# Whether to wait for external arming by default (True = safe, False = SITL-friendly)
DEFAULT_WAIT_FOR_EXTERNAL_ARM = True

# Log interval while waiting for arm (seconds)
WAITING_FOR_ARM_LOG_INTERVAL_S = 5.0

# Delay after entering GUIDED mode before proceeding (seconds)
POST_GUIDED_DELAY_S = 1.0

# Minimum time to wait after arming before attempting takeoff (seconds)
MIN_ARM_TO_TAKEOFF_DELAY_S = 2.0

# Default RTL on script end behavior (True = safe, returns home)
DEFAULT_RTL_ON_END = True

# Default skip initialization behavior (False = perform all safety checks)
DEFAULT_SKIP_INIT = False


# Movement Constants

# Default tolerance for position reached checks (meters)
DEFAULT_POSITION_TOLERANCE_M = 2.0

# Tolerance for heading reached checks (degrees)
HEADING_TOLERANCE_DEG = 5.0

# Default minimum altitude tolerance for takeoff (percentage, 0.0-1.0)
# Vehicle must reach this fraction of target altitude to consider takeoff complete
DEFAULT_TAKEOFF_ALTITUDE_TOLERANCE = 0.95

# Default tolerance for rover position checks (meters)
# Slightly larger than drone default due to rover GPS accuracy
DEFAULT_ROVER_POSITION_TOLERANCE_M = 2.1

# Default timeout for goto / navigation commands (seconds)
# Used by both drones and rovers when waiting to reach a target coordinate
DEFAULT_GOTO_TIMEOUT_S = 300.0


# Timing Constants


# Delay between steps of the arm -> guided -> takeoff sequence (seconds)
# This provides time for the autopilot to process each command
ARMING_SEQUENCE_DELAY_S = 2.0

# Post-takeoff stabilization delay (seconds)
# Allows the vehicle to stabilize at target altitude before proceeding
POST_TAKEOFF_STABILIZATION_S = 5.0

# Interval for internal state update loop (seconds)
INTERNAL_UPDATE_DELAY_S = 0.1

# State machine delay between state transitions (seconds)
STATE_MACHINE_DELAY_S = 0.01

# Delay for velocity command update loop (seconds)
VELOCITY_UPDATE_DELAY_S = 0.1

# Logging interval when waiting for armable state (seconds)
ARMABLE_STATUS_LOG_INTERVAL_S = 5.0


# Verbose Logging Constants

# Default prefix for verbose log files
VERBOSE_LOG_FILE_PREFIX = "aerpawlib_vehicle_dump"

# Default delay between verbose log entries (seconds)
VERBOSE_LOG_DELAY_S = 0.1


# Validation Limits

# Minimum acceptable position tolerance (meters)
# Prevents users from setting unrealistically tight tolerances
MIN_POSITION_TOLERANCE_M = 0.1

# Maximum acceptable position tolerance (meters)
MAX_POSITION_TOLERANCE_M = 100.0

# Maximum reasonable altitude (meters AGL)
MAX_ALTITUDE_M = 400.0

# Minimum reasonable altitude for flight (meters AGL)
MIN_FLIGHT_ALTITUDE_M = 1.0

# Maximum reasonable ground speed (m/s)
MAX_GROUNDSPEED_M_S = 30.0

# Minimum ground speed (m/s)
MIN_GROUNDSPEED_M_S = 0.0


# AERPAW Platform Constants

# Default Controller VM (C-VM) address and port
DEFAULT_CVM_IP = "192.168.32.25"
DEFAULT_CVM_PORT = 12435

# connect to OEO-CONSOLE, or default to C-VM (will only work on portable nodes)
DEFAULT_FORWARD_SERVER_IP = os.getenv("AP_EXPENV_OEOCVM_XM", "192.168.32.25")
DEFAULT_FORWARD_SERVER_PORT = 12435

DEFAULT_HUMAN_READABLE_AGENT_ID = os.getenv("AP_EXPENV_THIS_CONTAINER_EXP_NODE_NUM")

# OEO Log Severities
OEO_MSG_SEV_INFO = "INFO"
OEO_MSG_SEV_WARN = "WARNING"
OEO_MSG_SEV_ERR = "ERROR"
OEO_MSG_SEV_CRIT = "CRITICAL"
OEO_MSG_SEVS = [
    OEO_MSG_SEV_INFO,
    OEO_MSG_SEV_WARN,
    OEO_MSG_SEV_ERR,
    OEO_MSG_SEV_CRIT,
]


# ZMQ Constants

# Default ZMQ proxy ports
ZMQ_PROXY_IN_PORT = "5570"
ZMQ_PROXY_OUT_PORT = "5571"

# ZMQ Message Types
ZMQ_TYPE_TRANSITION = "state_transition"
ZMQ_TYPE_FIELD_REQUEST = "field_request"
ZMQ_TYPE_FIELD_CALLBACK = "field_callback"


# Safety Checker Constants

# Safety Checker Request Types
SERVER_STATUS_REQ = "server_status_req"
VALIDATE_WAYPOINT_REQ = "validate_waypoint_req"
VALIDATE_CHANGE_SPEED_REQ = "validate_change_speed_req"
VALIDATE_TAKEOFF_REQ = "validate_takeoff_req"
VALIDATE_LANDING_REQ = "validate_landing_req"


# Waypoint and Plan Constants

# Default waypoint speed (m/s)
DEFAULT_WAYPOINT_SPEED = 5

# MAVLink/Plan Commands
PLAN_CMD_TAKEOFF = 22
PLAN_CMD_WAYPOINT = 16
PLAN_CMD_RTL = 20
PLAN_CMD_SPEED = 178
