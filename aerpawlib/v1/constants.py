"""
Central configuration constants for aerpawlib v1.

All timing, tolerance, and configuration values should be defined here
for easy tuning and documentation.
"""

# =============================================================================
# Connection Constants
# =============================================================================

# Maximum time to wait for initial connection to vehicle (seconds)
CONNECTION_TIMEOUT_S = 30.0

# Maximum time to wait for vehicle to become armable (seconds)
ARMABLE_TIMEOUT_S = 60.0

# Interval for checking connection state (seconds)
POLLING_DELAY_S = 0.01

# Interval between heartbeat timeout checks (seconds)
HEARTBEAT_CHECK_INTERVAL_S = 1.0


# =============================================================================
# Movement Constants
# =============================================================================

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


# =============================================================================
# Timing Constants
# =============================================================================

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


# =============================================================================
# Verbose Logging Constants
# =============================================================================

# Default prefix for verbose log files
VERBOSE_LOG_FILE_PREFIX = "aerpawlib_vehicle_dump"

# Default delay between verbose log entries (seconds)
VERBOSE_LOG_DELAY_S = 0.1


# =============================================================================
# Validation Limits
# =============================================================================

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

