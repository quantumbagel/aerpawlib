"""
Functionality exclusive to the AERPAW platform

The AERPAW_Platform singleton will automatically detect if a script is being
run in an AERPAW experiment, in which case it enables additional AERPAW
functionality.

@author: John Kesler (morzack)
"""

import base64
import threading
from urllib.parse import quote as url_quote

import requests

from aerpawlib.log import get_logger, LogComponent

logger = get_logger(LogComponent.AERPAW)
oeo_logger = get_logger(LogComponent.OEO)

from .constants import (
    DEFAULT_FORWARD_SERVER_IP,
    DEFAULT_FORWARD_SERVER_PORT,
    DEFAULT_HUMAN_READABLE_AGENT_ID,
    OEO_MSG_SEV_INFO,
    OEO_MSG_SEV_WARN,
    OEO_MSG_SEV_ERR,
    OEO_MSG_SEV_CRIT,
    OEO_MSG_SEVS,
)


class AERPAW:
    """
    Interface for interacting with the AERPAW platform services.

    This class provides methods for logging to the AERPAW OEO-Console and using the AERPAW checkpoint system.

    Attributes:
        _forw_addr (str): The IP address of the forward server.
        _forw_port (int): The port of the forward server service.
        _connected (bool): Whether the object is successfully connected to the AERPAW platform.
        _connection_warning_displayed (bool): Whether the connection warning has been shown.
        _no_stdout (bool): If True, suppresses printing to standard output.
    """

    _forw_addr: str
    _forw_port: int
    _connected: bool

    _connection_warning_displayed = False

    def __init__(self, forw_addr=DEFAULT_FORWARD_SERVER_IP, forw_port=DEFAULT_FORWARD_SERVER_PORT):
        """
        Initialize the AERPAW platform interface.

        Args:
            forw_addr (str): The IP address of the forward server. Defaults to DEFAULT_FORWARD_SERVER_IP.
            forw_port (int): The port of the forward server service. Defaults to DEFAULT_FORWARD_SERVER_PORT.
        """
        self._forw_addr = forw_addr
        self._forw_port = forw_port
        self._connected = self.attach_to_aerpaw_platform()
        self._no_stdout = False

    def attach_to_aerpaw_platform(self) -> bool:
        """
        Attempts to attach this `AERPAW` object to the AERPAW platform/C-VM
        hosting this experiment. Returns bool depending on success.
        """
        try:
            requests.post(
                f"http://{self._forw_addr}:{self._forw_port}/ping", timeout=1
            )
        except requests.exceptions.RequestException:
            return False
        return True

    def _is_aerpaw_environment(self) -> bool:
        """
        Check if we're running in the AERPAW platform environment.

        Returns:
            True if connected to AERPAW platform, False otherwise (standalone/SITL)
        """
        return self._connected

    def _display_connection_warning(self):
        """
        Displays a warning if the AERPAW platform is used outside of an AERPAW environment.
        """
        if self._connection_warning_displayed:
            return
        if not self._no_stdout:
            logger.info("the user script has attempted to use AERPAW platform functionality without being in the AERPAW environment")
        self._connection_warning_displayed = True

    def log_to_oeo(self, msg: str, severity: str = OEO_MSG_SEV_INFO, agent_id: str = DEFAULT_HUMAN_READABLE_AGENT_ID):
        """
        Send a message to the OEO console, if connected.

        Prints the message to stdout regardless of connection status, unless _no_stdout is True.

        Args:
            msg (str): The message to log.
            severity (str): The severity level of the message. Defaults to OEO_MSG_SEV_INFO.
            agent_id (str): The ID of the agent sending the message. Defaults to DEFAULT_HUMAN_READABLE_AGENT_ID.

        Raises:
            Exception: If the provided severity is not supported.
        """
        if not self._no_stdout:
            if severity == OEO_MSG_SEV_INFO:
                oeo_logger.info(msg)
            elif severity == OEO_MSG_SEV_WARN:
                oeo_logger.warning(msg)
            elif severity == OEO_MSG_SEV_ERR:
                oeo_logger.error(msg)
            elif severity == OEO_MSG_SEV_CRIT:
                oeo_logger.critical(msg)
            else:
                oeo_logger.info(msg)

        if not self._connected:
            self._display_connection_warning()
            return

        if severity not in OEO_MSG_SEVS:
            raise Exception("severity provided for log_to_oeo not supported")
        encoded = base64.urlsafe_b64encode(msg.encode("utf-8"))
        try:
            if agent_id:
                requests.post(
                    f"http://{self._forw_addr}:{self._forw_port}/oeo_msg/{severity}/{encoded.decode('utf-8')}/{agent_id}",
                    timeout=3,
                )
            else:
                requests.post(
                    f"http://{self._forw_addr}:{self._forw_port}/oeo_msg/{severity}/{encoded.decode('utf-8')}",
                    timeout=3,
                )
        except requests.exceptions.RequestException:
            if not self._no_stdout:
                logger.error("unable to send previous message to OEO.")

    def _checkpoint_build_request(self, var_type, var_name):
        """
        Builds a checkpoint request URL.

        Args:
            var_type (str): The type of the checkpoint variable ('bool', 'int', or 'string').
            var_name (str): The name of the checkpoint variable.

        Returns:
            str: The full URL for the checkpoint request.
        """
        return f"http://{self._forw_addr}:{self._forw_port}/checkpoint/{var_type}/{var_name}"

    # NOTE: unlike the above functionality, all checkpoint functions will cause an
    # exception if they are run while not in the AERPAW platform, as there isn't a way
    # to "recover" while maintaining the function's API contract

    def checkpoint_reset_server(self):
        """
        Reset the AERPAW checkpoint server.

        This function should be called at the start of an experiment by an E-VM script
        to ensure that no stored state remains between experiment runs.
        """
        if not self._connected:
            self._display_connection_warning()
            raise Exception(
                "AERPAW checkpoint functionality only works in AERPAW environment"
            )
        response = requests.post(
            f"http://{self._forw_addr}:{self._forw_port}/checkpoint/reset",
            timeout=5,
        )
        if response.status_code != 200:
            raise Exception("error when resetting checkpoint server")

    def checkpoint_set(self, checkpoint_name: str):
        """
        Set a boolean checkpoint in the AERPAW checkpoint system.

        Args:
            checkpoint_name (str): The name of the checkpoint to set.

        Raises:
            Exception: If not in an AERPAW environment or if the server returns an error.
        """
        if not self._connected:
            self._display_connection_warning()
            raise Exception(
                "AERPAW checkpoint functionality only works in AERPAW environment"
            )
        response = requests.post(
            self._checkpoint_build_request("bool", checkpoint_name),
            timeout=5,
        )
        if response.status_code != 200:
            raise Exception("error when posting to checkpoint server")

    def checkpoint_check(self, checkpoint_name: str) -> bool:
        """
        Check if a boolean checkpoint has been set.

        Args:
            checkpoint_name (str): The name of the checkpoint to check.

        Returns:
            bool: True if the checkpoint is set, False otherwise.

        Raises:
            Exception: If not in an AERPAW environment or if the server returns an error/malformed content.
        """
        if not self._connected:
            self._display_connection_warning()
            raise Exception(
                "AERPAW checkpoint functionality only works in AERPAW environment"
            )
        response = requests.get(
            self._checkpoint_build_request("bool", checkpoint_name),
            timeout=5,
        )
        if response.status_code != 200:
            raise Exception("error when getting from checkpoint server")
        response_content = response.content.decode()
        if response_content == "True":
            return True
        elif response_content == "False":
            return False
        raise Exception(
            f"malformed content in response from server: {response_content}"
        )

    def checkpoint_increment_counter(self, counter_name: str):
        """
        Increment an integer counter in the AERPAW checkpoint system.

        Args:
            counter_name (str): The name of the counter to increment.

        Raises:
            Exception: If not in an AERPAW environment or if the server returns an error.
        """
        if not self._connected:
            self._display_connection_warning()
            raise Exception(
                "AERPAW checkpoint functionality only works in AERPAW environment"
            )
        response = requests.post(
            self._checkpoint_build_request("int", counter_name),
            timeout=5,
        )
        if response.status_code != 200:
            raise Exception("error when posting to checkpoint server")

    def checkpoint_check_counter(self, counter_name: str) -> int:
        """
        Get the current value of an integer counter.

        Args:
            counter_name (str): The name of the counter to check.

        Returns:
            int: The current value of the counter. Defaults to 0 if never incremented.

        Raises:
            Exception: If not in an AERPAW environment or if the server returns an error/malformed content.
        """
        if not self._connected:
            self._display_connection_warning()
            raise Exception(
                "AERPAW checkpoint functionality only works in AERPAW environment"
            )
        response = requests.get(
            self._checkpoint_build_request("int", counter_name),
            timeout=5,
        )
        if response.status_code != 200:
            raise Exception("error when getting from checkpoint server")
        response_content = response.content.decode()
        try:
            return int(response_content)
        except (TypeError, ValueError):
            raise Exception(
                f"malformed content in response from server: {response_content}"
            )

    def checkpoint_set_string(self, string_name: str, value: str):
        """
        Set a string value in the AERPAW checkpoint system.

        Args:
            string_name (str): The key for the string value.
            value (str): The string value to store.

        Raises:
            Exception: If not in an AERPAW environment or if the server returns an error.
        """
        if not self._connected:
            self._display_connection_warning()
            raise Exception(
                "AERPAW checkpoint functionality only works in AERPAW environment"
            )
        response = requests.post(
            self._checkpoint_build_request("string", string_name),
            params={"val": value},
            timeout=5,
        )
        if response.status_code != 200:
            raise Exception("error when posting to checkpoint server")

    def checkpoint_check_string(self, string_name: str) -> str:
        """
        Get a string value from the AERPAW checkpoint system.

        Args:
            string_name (str): The key for the string value.

        Returns:
            str: The stored string value.

        Raises:
            Exception: If not in an AERPAW environment or if the server returns an error.
        """
        if not self._connected:
            self._display_connection_warning()
            raise Exception(
                "AERPAW checkpoint functionality only works in AERPAW environment"
            )
        response = requests.get(
            self._checkpoint_build_request("string", string_name),
            timeout=5,
        )
        if response.status_code != 200:
            raise Exception("error when getting from checkpoint server")
        response_content = response.content.decode()
        return response_content

    def publish_user_oeo_topic(self, value: str, topic: str, agent_id: str = DEFAULT_HUMAN_READABLE_AGENT_ID) -> bool:
        """
        Publish `value` to a user topic in the OEO system (can be added/is
        visible on the OEO-console).
        Use `agent_id` to provide an identifier for the message being sent. By
        default, it uses the current node's number.
        `topic` is used to structure the topic as received in the OEO system.
        The message will be received internally "oeo/user/`topic`" and can be
        viewed in the OEO-CONSOLE by adding `topic`. `topic` can include "/"s
        to indicate a hierachy within the message (e.g. "radio_script/snr" and
        "radio_script/throughput" could both be seen on the console by "add"ing
        "radio_script").
        returns bool based on success
        """

        # note for devs, the messages can be sent to http://oeo_console:port/oeo_pub/encoded_topic/encoded_value/(optional)encoded_agent

        if not self._connected:
            self._display_connection_warning()
            return False

        value_b64 = base64.urlsafe_b64encode(str(value).encode('utf-8')).decode('utf-8')
        topic_b64 = base64.urlsafe_b64encode(str(topic).encode('utf-8')).decode('utf-8')
        agent_b64 = None

        if agent_id is not None:
            agent_b64 = base64.urlsafe_b64encode(str(agent_id).encode('utf-8')).decode('utf-8')

        try:
            if not agent_id:
                requests.post(f"http://{self._forw_addr}:{self._forw_port}/oeo_pub/{topic_b64}/{value_b64}", timeout=3)
            else:
                requests.post(f"http://{self._forw_addr}:{self._forw_port}/oeo_pub/{topic_b64}/{value_b64}/{agent_b64}", timeout=3)
        except requests.exceptions.RequestException as e:
            logger.error(f"unable to publish value to OEO system. exception: {e}")
            return False
        return True


class _AERPAWLazyProxy:
    """
    Lazy proxy for the AERPAW singleton.

    Defers construction of the actual AERPAW instance until first attribute
    access, avoiding a ~1 second HTTP timeout on every import when running
    outside the AERPAW environment.
    """

    def __init__(self):
        self.__dict__["_instance"] = None
        self.__dict__["_lock"] = threading.Lock()

    def _get_instance(self):
        if self._instance is None:
            with self._lock:
                # Double-check after acquiring lock
                if self._instance is None:
                    self.__dict__["_instance"] = AERPAW()
        return self._instance

    def __getattr__(self, name):
        return getattr(self._get_instance(), name)


AERPAW_Platform = _AERPAWLazyProxy()
