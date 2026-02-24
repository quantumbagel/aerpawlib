"""
safety module
Provides a safety server and client for validating vehicle commands
Unchanged from legacy except to provide aliases for new method names
to maintain backward compatibility with existing code.

@author: John Kesler (morzack)
"""

import json
from aerpawlib.log import get_logger, LogComponent
import os
import zlib
from argparse import ArgumentParser
from typing import Dict, Tuple

import yaml
import zmq

from .util import Coordinate, do_intersect, inside, read_geofence
from .constants import (
    SAFETY_CHECKER_REQUEST_TIMEOUT_S,
    SERVER_STATUS_REQ,
    VALIDATE_WAYPOINT_REQ,
    VALIDATE_CHANGE_SPEED_REQ,
    VALIDATE_TAKEOFF_REQ,
    VALIDATE_LANDING_REQ,
)

# Configure logger
logger = get_logger(LogComponent.SAFETY)


# serialize a safety checker request
def serialize_request(request_function: str, params: list):
    """
    Serialize a safety checker request into a compressed JSON format.

    Args:
        request_function (str): The name of the function to request on the server.
        params (list): List of parameters for the request function.

    Returns:
        bytes: Compressed byte string of the serialized JSON.
    """
    raw_msg = json.dumps(
        {
            "request_function": request_function,
            "params": params,
        }
    )
    return serialize_msg(raw_msg)


# serialize a safety checker response
def serialize_response(request_function: str, result: bool, message: str = ""):
    """
    Serialize a safety checker response into a compressed JSON format.

    Args:
        request_function (str): The name of the function that was requested.
        result (bool): Whether the request was successful/valid.
        message (str, optional): Additional information or error reason. Defaults to "".

    Returns:
        bytes: Compressed byte string of the serialized JSON.
    """
    raw_msg = json.dumps(
        {
            "request_function": request_function,
            "result": result,
            "message": message,
        }
    )
    return serialize_msg(raw_msg)


def serialize_msg(raw_json):
    """
    Compress JSON message using zlib.

    Args:
        raw_json (str): The JSON string to compress.

    Returns:
        bytes: Compressed data.
    """
    compressed_msg = zlib.compress(raw_json.encode("utf-8"))
    return compressed_msg


def deserialize_msg(compressed_msg):
    """
    Decompress and parse a JSON message using zlib.

    Args:
        compressed_msg (bytes): The compressed data to decompress.

    Returns:
        dict: The parsed JSON message as a dictionary.
    """
    raw_msg = zlib.decompress(compressed_msg).decode("utf-8")
    msg = json.loads(raw_msg)
    return msg


class SafetyCheckerClient:
    """
    A client for communicating with the SafetyCheckerServer via ZMQ.

    Attributes:
        context (zmq.Context): The ZMQ context.
        socket (zmq.Socket): The REQ socket for sending requests.
    """

    def __init__(
        self, addr: str, port: int, timeout_s: float = SAFETY_CHECKER_REQUEST_TIMEOUT_S
    ):
        """
        Initialize the safety checker client.

        Args:
            addr (str): The IP address of the safety checker server.
            port (int): The port the server is listening on.
            timeout_s (float): Timeout for send/recv in seconds. Prevents indefinite
                block if server is down. Defaults to SAFETY_CHECKER_REQUEST_TIMEOUT_S.
        """
        self._timeout_s = timeout_s
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        timeout_ms = int(timeout_s * 1000)
        self.socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self.socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self.socket.connect(f"tcp://{addr}:{port}")

    def close(self):
        """Close the ZMQ socket and context to free resources."""
        self.socket.close()
        self.context.term()

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
        return False

    def send_request(self, msg):
        """
        Generic function to send a request to the safety checker server.

        Sends the provided raw message, then deserializes the response.

        Args:
            msg (bytes): The serialized request message.

        Returns:
            dict: The deserialized response from the server.

        Raises:
            TimeoutError: If the server does not respond within the configured timeout.
        """
        try:
            self.socket.send(msg)
            raw_msg = self.socket.recv()
        except zmq.Again:
            raise TimeoutError(
                f"Safety checker server did not respond within {self._timeout_s}s"
            )
        message = deserialize_msg(raw_msg)
        logger.debug(f"Received reply [{message}]")
        return message

    def sendRequest(self, msg):
        return self.send_request(msg)

    def parse_response(self, response):
        """
        Parse a response dictionary from the safety checker server.

        Args:
            response (dict): The response dictionary to parse.

        Returns:
            Tuple[bool, str]: A tuple containing (result, message).
        """
        return response["result"], response["message"]

    def parseResponse(self, response):
        return self.parse_response(response)

    def check_server_status(self):
        """
        Verify the safety checker server is reachable and active.

        Returns:
            Tuple[bool, str]: A tuple containing (True, "") if server is up.
        """
        msg = serialize_request(SERVER_STATUS_REQ, None)
        resp = self.send_request(msg)
        return self.parse_response(resp)

    def checkServerStatus(self):
        return self.check_server_status()

    def validate_waypoint_command(self, current_location: Coordinate, next_location: Coordinate):
        """
        Makes sure path from current location to next waypoint stays inside geofence and avoids no-go zones.
        Returns a tuple (bool, str)
        (False, <error message>) if the waypoint violates geofence or no-go zone constraints, else (True, "").
        """
        msg = serialize_request(
            VALIDATE_WAYPOINT_REQ, [current_location.to_json(), next_location.to_json()]
        )
        resp = self.send_request(msg)
        return self.parse_response(resp)

    def validateWaypointCommand(self, curLoc: Coordinate, nextLoc: Coordinate):
        return self.validate_waypoint_command(curLoc, nextLoc)

    def validate_change_speed_command(self, new_speed):
        """
        Makes sure the provided new_speed lies within the configured vehicle constraints
        Returns (False, <error message>) if the speed violates constraints, else (True, "").
        """
        msg = serialize_request(VALIDATE_CHANGE_SPEED_REQ, [new_speed])
        resp = self.send_request(msg)
        return self.parse_response(resp)

    def validateChangeSpeedCommand(self, newSpeed):
        return self.validate_change_speed_command(newSpeed)

    def validate_takeoff_command(self, takeoff_alt, current_lat, current_lon):
        """
        Makes sure the takeoff altitude lies within the vehicle constraints
        Returns (False, <error message>) if the altitude violates constraints, else (True, "").
        """
        msg = serialize_request(
            VALIDATE_TAKEOFF_REQ, [takeoff_alt, current_lat, current_lon]
        )
        resp = self.send_request(msg)
        return self.parse_response(resp)

    def validateTakeoffCommand(self, takeoffAlt, currentLat, currentLon):
        return self.validate_takeoff_command(takeoffAlt, currentLat, currentLon)

    def validate_landing_command(self, current_lat, current_lon):
        """
        Ensure the copter is attempting to land within 5 meters of the takeoff location
        Returns (False, <error message>) if the coper is not within 5 meters, else (True, "").
        """
        msg = serialize_request(VALIDATE_LANDING_REQ, [current_lat, current_lon])
        resp = self.send_request(msg)
        return self.parse_response(resp)

    def validateLandingCommand(self, currentLat, currentLon):
        return self.validate_landing_command(currentLat, currentLon)


def _polygon_edges(polygon):
    """
    Yield consecutive (p1, p2) edge pairs for a polygon, including the
    closing edge from the last vertex back to the first.

    Args:
        polygon (list): List of {'lat': ..., 'lon': ...} points.

    Yields:
        Tuple[dict, dict]: Pairs of adjacent vertices.
    """
    n = len(polygon)
    for i in range(n):
        yield polygon[i], polygon[(i + 1) % n]


# noinspection PyUnusedLocal
class SafetyCheckerServer:
    """
    A server that validates vehicle commands against geofences and constraints.

    Attributes:
        REQUEST_FUNCTIONS (dict): Mapping of request types to handler methods.
        vehicle_type (str): Type of vehicle ('copter' or 'rover').
        include_geofences (list): List of allowed regions.
        exclude_geofences (list): List of no-go zones.
        max_speed (float): Maximum allowed speed (m/s).
        min_speed (float): Minimum allowed speed (m/s).
        max_alt (float, optional): Maximum allowed altitude for copters.
        min_alt (float, optional): Minimum allowed altitude for copters.
        takeoff_location (Coordinate): The location where takeoff was performed.
    """

    # valid vehicle types
    VEHICLE_TYPES = ["rover", "copter"]
    # parameters required for all vehicle types
    REQUIRED_PARAMS = [
        "vehicle_type",
        "max_speed",
        "min_speed",
        "include_geofences",
        "exclude_geofences",
    ]
    # parameters required for copters
    REQUIRED_COPTER_PARAMS = ["max_alt", "min_alt"]

    def __init__(self, vehicle_config_filename: str, server_port=14580):
        """
        Initialize the safety checker server and start listening.

        Args:
            vehicle_config_filename (str): Path to the YAML configuration file.
            server_port (int, optional): Port to bind the server to. Defaults to 14580.
        """
        # Construct map between function request strings and functions
        self.REQUEST_FUNCTIONS = {
            SERVER_STATUS_REQ: self.server_status_handler,
            VALIDATE_WAYPOINT_REQ: self.validate_waypoint_handler,
            VALIDATE_CHANGE_SPEED_REQ: self.validate_change_speed_handler,
            VALIDATE_TAKEOFF_REQ: self.validate_takeoff_handler,
            VALIDATE_LANDING_REQ: self.validate_landing_handler,
        }

        with open(vehicle_config_filename, "r") as vehicle_config_file:
            config = yaml.safe_load(vehicle_config_file)

        self.validate_config(config, vehicle_config_filename)

        self.vehicle_type = config["vehicle_type"]

        (vehicle_config_dir, _) = os.path.split(vehicle_config_filename)
        self.include_geofences = [
            read_geofence(os.path.join(vehicle_config_dir, geofence))
            for geofence in config["include_geofences"]
        ]
        # No go zones to exclude from the geofenced area
        self.exclude_geofences = [
            read_geofence(os.path.join(vehicle_config_dir, geofence))
            for geofence in config["exclude_geofences"]
        ]
        self.max_speed = config["max_speed"]
        self.min_speed = config["min_speed"]

        # Only max altitude for copters
        if self.vehicle_type == "copter":
            self.max_alt = config["max_alt"]
            self.min_alt = config["min_alt"]

        # start server BLOCKING
        self.start_server(server_port)

    def start_server(self, port):
        """
        Start the ZMQ server loop. Blocks until the program is terminated.

        Args:
            port (int): The port to bind to.
        """
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind(f"tcp://*:{port}")

        logger.info("waiting for messages")

        while True:
            raw_msg = socket.recv()
            message = deserialize_msg(raw_msg)
            logger.debug(f"Received request: {message}")
            try:
                function_name = message.get("request_function", "unknown")
                req_function = self.REQUEST_FUNCTIONS[function_name]
                params = message.get("params")
                if params is None:
                    response = req_function(None)
                else:
                    response = req_function(*params)
                socket.send(response)
            except KeyError as e:
                fn = message.get("request_function", "unknown")
                error_resp = serialize_response(
                    request_function=str(e),
                    result=False,
                    message=f"Unimplemented or missing function request <{fn}>",
                )
                socket.send(error_resp)
            except Exception as e:
                logger.debug("Safety checker server handler error: %s", e)
                error_resp = serialize_response(
                    request_function="unknown",
                    result=False,
                    message=f"Server error: {e}",
                )
                socket.send(error_resp)
                # Do NOT re-raise — keep the server running

    def validate_config(self, config: Dict, vehicle_config_filename: str):
        """
        Ensures that the provided config dict contains all necessary parameters.

        Args:
            config (Dict): The configuration dictionary loaded from YAML.
            vehicle_config_filename (str): Filename for error reporting.

        Raises:
            Exception: If the configuration is invalid or missing required keys.
        """
        # Check if all required params exist
        for param in self.REQUIRED_PARAMS:
            if param not in config:
                raise Exception(
                    f"Required parameter {param} not found in {vehicle_config_filename}!"
                )

        # Ensure the vehicle type is valid
        if config["vehicle_type"] not in self.VEHICLE_TYPES:
            raise Exception(
                f"Vehicle type in {vehicle_config_filename} is invalid! Must be one of {self.VEHICLE_TYPES}"
            )

        # If the vehicle is a copter ensure copter-specific required params exist
        if config["vehicle_type"] == "copter":
            for param in self.REQUIRED_COPTER_PARAMS:
                if param not in config:
                    raise Exception(
                        f"Required copter parameter {param} not found in {vehicle_config_filename}!"
                    )

    def validate_waypoint_command(
        self, current_location: Coordinate, next_location: Coordinate
    ) -> Tuple[bool, str]:
        """
        Makes sure path from current location to next waypoint stays inside geofence and avoids no-go zones.
        Returns a tuple (bool, str)
        (False, <error message>) if the waypoint violates geofence or no-go zone constraints, else (True, "").
        """
        logger.debug(f"Validating {next_location}")

        # Makes sure altitude of next waypoint is within regulations
        if self.vehicle_type == "copter":
            if next_location.alt < self.min_alt or next_location.alt > self.max_alt:
                return (
                    False,
                    "Invalid waypoint. Altitude of %s m is not within restrictions! ABORTING!"
                    % next_location.alt,
                )

        # Makes sure next waypoint is inside one of the include geofences
        dest_geofence = None
        for gf in self.include_geofences:
            if inside(next_location.lon, next_location.lat, gf):
                dest_geofence = gf
                break
        if dest_geofence is None:
            return (
                False,
                "Invalid waypoint. Waypoint (%s,%s) is outside of the geofence. ABORTING!"
                % (next_location.lat, next_location.lon),
            )
        # Makes sure next waypoint is not in a no-go zone
        for zone in self.exclude_geofences:
            if inside(next_location.lon, next_location.lat, zone):
                return (
                    False,
                    "Invalid waypoint. Waypoint (%s,%s) is inside a no-go zone. ABORTING!"
                    % (next_location.lat, next_location.lon),
                )
        # Makes sure path between two points does not leave the
        # geofence that the destination was found inside of.
        # Check all edges including the closing edge (last → first vertex).
        for p1, p2 in _polygon_edges(dest_geofence):
            if do_intersect(
                p1["lon"], p1["lat"],
                p2["lon"], p2["lat"],
                current_location.lon, current_location.lat,
                next_location.lon, next_location.lat,
            ):
                return (
                    False,
                    "Invalid waypoint. Path from (%s,%s) to waypoint (%s,%s) leaves geofence. ABORTING!"
                    % (current_location.lat, current_location.lon, next_location.lat, next_location.lon),
                )

        # Makes sure path between two points does not enter no-go zone
        for zone in self.exclude_geofences:
            for p1, p2 in _polygon_edges(zone):
                if do_intersect(
                    p1["lon"], p1["lat"],
                    p2["lon"], p2["lat"],
                    current_location.lon, current_location.lat,
                    next_location.lon, next_location.lat,
                ):
                    return (
                        False,
                        "Invalid waypoint. Path from (%s,%s) to waypoint (%s,%s) enters no-go zone. ABORTING!"
                        % (current_location.lat, current_location.lon, next_location.lat, next_location.lon),
                    )

        # Next waypoint location is valid
        return True, ""

    def validateWaypointCommand(
        self, curLoc: Coordinate, nextLoc: Coordinate
    ) -> Tuple[bool, str]:
        return self.validate_waypoint_command(curLoc, nextLoc)

    def validate_change_speed_command(self, new_speed) -> Tuple[bool, str]:
        """
        Makes sure the provided newSpeed lies within the configured vehicle constraints
        Returns (False, <error message>) if the speed violates constraints, else (True, "").
        """
        if new_speed > self.max_speed:
            return (
                False,
                "Invalid speed (%s) greater than maximum (%s)"
                % (new_speed, self.max_speed),
            )
        if new_speed < self.min_speed:
            return (
                False,
                "Invalid speed (%s) less than minimum (%s)"
                % (new_speed, self.min_speed),
            )
        # New speed is valid
        return True, ""

    def validateChangeSpeedCommand(self, newSpeed) -> Tuple[bool, str]:
        return self.validate_change_speed_command(newSpeed)

    def validate_takeoff_command(self, takeoff_alt, current_lat, current_lon):
        """
        Makes sure the takeoff altitude lies within the vehicle constraints
        Returns (False, <error message>) if the altitude violates constraints, else (True, "").
        """
        # Makes sure altitude is within regulations
        if self.vehicle_type == "copter":
            if takeoff_alt < self.min_alt or takeoff_alt > self.max_alt:
                return (
                    False,
                    "Invalid takeoff altitude of %s m." % takeoff_alt,
                )
        # Save takeoff location for validating landing
        self.takeoff_location = Coordinate(current_lat, current_lon, alt=0)
        # Takeoff command is valid
        return True, ""

    def validateTakeoffCommand(self, takeoffAlt, currentLat, currentLon):
        return self.validate_takeoff_command(takeoffAlt, currentLat, currentLon)

    def validate_landing_command(self, current_lat, current_lon):
        """
        Ensure the copter is attempting to land within 5 meters of the takeoff location
        Returns (False, <error message>) if the coper is not within 5 meters, else (True, "").
        """
        if not hasattr(self, "takeoff_location") or self.takeoff_location is None:
            return (
                False,
                "Cannot validate landing: no takeoff location recorded. "
                "Was validate_takeoff_command called first?",
            )
        current_location = Coordinate(current_lat, current_lon, alt=0)
        distance = self.takeoff_location.ground_distance(current_location)

        if distance > 5:
            return (
                False,
                "Invalid landing location. Must be within 5 meters of takeoff location. Attempted landing location (%s,%s) is %f meters from takeoff location."
                % (current_lat, current_lon, distance),
            )
        # Landing command is valid
        return True, ""

    def validateLandingCommand(self, currentLat, currentLon):
        return self.validate_landing_command(currentLat, currentLon)

    # Client Request Handlers
    def server_status_handler(self, *_params):
        """
        Handler for server status requests.

        Returns:
            bytes: Serialized successful response.
        """
        msg = serialize_response(
            request_function=SERVER_STATUS_REQ, result=True
        )
        return msg

    def serverStatusHandler(self, *_params):
        return self.server_status_handler(*_params)

    def validate_waypoint_handler(self, current_json_location, next_json_location, *_params):
        """
        Handler for waypoint validation requests.

        Args:
            current_json_location (str): JSON string representing the current Coordinate.
            next_json_location (str): JSON string representing the target Coordinate.

        Returns:
            bytes: Serialized validation response.
        """
        current_dict_location = json.loads(current_json_location)
        next_dict_location = json.loads(next_json_location)
        current_location = Coordinate(
            lat=current_dict_location["lat"], lon=current_dict_location["lon"], alt=current_dict_location["alt"]
        )
        next_location = Coordinate(
            lat=next_dict_location["lat"],
            lon=next_dict_location["lon"],
            alt=next_dict_location["alt"],
        )

        result, message = self.validate_waypoint_command(current_location, next_location)
        msg = serialize_response(
            request_function=VALIDATE_WAYPOINT_REQ,
            result=result,
            message=message,
        )
        return msg

    def validateWaypointHandler(self, curLocJSON, nextLocJSON, *_params):
        return self.validate_waypoint_handler(curLocJSON, nextLocJSON, *_params)

    def validate_change_speed_handler(self, new_speed, *_params):
        """
        Handler for speed change validation requests.

        Args:
            new_speed (float): The requested new speed.

        Returns:
            bytes: Serialized validation response.
        """
        result, message = self.validate_change_speed_command(new_speed)
        msg = serialize_response(
            request_function=VALIDATE_CHANGE_SPEED_REQ,
            result=result,
            message=message,
        )
        return msg

    def validateChangeSpeedHandler(self, newSpeed, *_params):
        return self.validate_change_speed_handler(newSpeed, *_params)

    def validate_takeoff_handler(
        self, takeoff_alt, current_lat, current_lon, *_params
    ):
        """
        Handler for takeoff validation requests.

        Args:
            takeoff_alt (float): The requested takeoff altitude.
            current_lat (float): Current latitude.
            current_lon (float): Current longitude.

        Returns:
            bytes: Serialized validation response.
        """
        result, message = self.validate_takeoff_command(
            takeoff_alt, current_lat, current_lon
        )
        msg = serialize_response(
            request_function=VALIDATE_TAKEOFF_REQ,
            result=result,
            message=message,
        )
        return msg

    def validateTakeoffHandler(
        self, takeoffAlt, currentLat, currentLon, *_params
    ):
        return self.validate_takeoff_handler(
            takeoffAlt, currentLat, currentLon, *_params
        )

    def validate_landing_handler(self, current_lat, current_lon, *_params):
        """
        Handler for landing validation requests.

        Args:
            current_lat (float): Current latitude.
            current_lon (float): Current longitude.

        Returns:
            bytes: Serialized validation response.
        """
        result, message = self.validate_landing_command(current_lat, current_lon)
        msg = serialize_response(
            request_function=VALIDATE_LANDING_REQ,
            result=result,
            message=message,
        )
        return msg

    def validateLandingHandler(self, currentLat, currentLon, *_params):
        return self.validate_landing_handler(currentLat, currentLon, *_params)


if __name__ == "__main__":
    # DEPRECATED: This is a stupid way to run the safety checker server,
    # but it allows us to keep the same command line interface for legacy and new safety checkers
    # without needing to maintain two separate entry points.
    # v2 will do away with this and have a more elegant way to launch the server.
    parser = ArgumentParser(
        description="safety - Launch a safety server"
    )
    parser.add_argument(
        "--port",
        help="Port for communication between client and server",
        required=True,
        type=int,
    )
    parser.add_argument(
        "--vehicle_config",
        help="Path to YAML file containing geofences and vehicle constraints",
        required=True,
    )
    args, _ = parser.parse_known_args()

    # This call blocks
    server = SafetyCheckerServer(args.vehicle_config, server_port=args.port)
