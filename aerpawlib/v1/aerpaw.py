"""
Functionality exclusive to the AERPAW platform

The AERPAW_Platform singleton will automatically detect if a script is being
run in an AERPAW experiment, in which case it enables additional AERPAW
functionality.

@author: John Kesler (morzack)
"""

import base64
import requests

from ..v2.logging import get_logger, LogComponent

logger = get_logger(LogComponent.AERPAW)
oeo_logger = get_logger(LogComponent.OEO)

from .constants import (
    DEFAULT_CVM_IP,
    DEFAULT_CVM_PORT,
    OEO_MSG_SEV_INFO,
    OEO_MSG_SEV_WARN,
    OEO_MSG_SEV_ERR,
    OEO_MSG_SEV_CRIT,
    OEO_MSG_SEVS,
)


class AERPAW:
    """
    Interface for interacting with the AERPAW platform services.

    This class provides methods for logging to the AERPAW Operational
    Entity Observer (OEO) and using the AERPAW checkpoint system.

    Attributes:
        _cvm_addr (str): The IP address of the Controller VM (C-VM).
        _cvm_port (int): The port of the C-VM service.
        _connected (bool): Whether the object is successfully connected to the AERPAW platform.
        _connection_warning_displayed (bool): Whether the connection warning has been shown.
        _no_stdout (bool): If True, suppresses printing to standard output.
    """

    _cvm_addr: str
    _connected: bool

    _connection_warning_displayed = False

    def __init__(self, cvm_addr=DEFAULT_CVM_IP, cvm_port=DEFAULT_CVM_PORT):
        """
        Initialize the AERPAW platform interface.

        Args:
            cvm_addr (str): The IP address of the C-VM. Defaults to DEFAULT_CVM_IP.
            cvm_port (int): The port of the C-VM service. Defaults to DEFAULT_CVM_PORT.
        """
        self._cvm_addr = cvm_addr
        self._cvm_port = cvm_port
        self._connected = self.attach_to_aerpaw_platform()
        self._no_stdout = False

    def attach_to_aerpaw_platform(self) -> bool:
        """
        Attempts to attach this `AERPAW` object to the AERPAW platform/C-VM
        hosting this experiment. Returns bool depending on success.
        """
        try:
            requests.post(
                f"http://{self._cvm_addr}:{self._cvm_port}/ping", timeout=1
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

    def log_to_oeo(self, msg: str, severity: str = OEO_MSG_SEV_INFO):
        """
        Send a message to the OEO console, if connected.

        Prints the message to stdout regardless of connection status, unless _no_stdout is True.

        Args:
            msg (str): The message to log.
            severity (str): The severity level of the message. Defaults to OEO_MSG_SEV_INFO.

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
            requests.post(
                f"http://{self._cvm_addr}:{self._cvm_port}/oeo_msg/{severity}/{encoded.decode('utf-8')}",
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
        return f"http://{self._cvm_addr}:{self._cvm_port}/checkpoint/{var_type}/{var_name}"

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
            f"http://{self._cvm_addr}:{self._cvm_port}/checkpoint/reset"
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
            self._checkpoint_build_request("bool", checkpoint_name)
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
            self._checkpoint_build_request("bool", checkpoint_name)
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
            self._checkpoint_build_request("int", counter_name)
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
            self._checkpoint_build_request("int", counter_name)
        )
        if response.status_code != 200:
            raise Exception("error when getting from checkpoint server")
        response_content = response.content.decode()
        try:
            return int(response_content)
        except TypeError:
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
            self._checkpoint_build_request("string", string_name)
            + f"?val={value}"
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
            self._checkpoint_build_request("string", string_name)
        )
        if response.status_code != 200:
            raise Exception("error when getting from checkpoint server")
        response_content = response.content.decode()
        return response_content


AERPAW_Platform = AERPAW()
