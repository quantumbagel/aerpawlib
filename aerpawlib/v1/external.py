"""
Utilities allowing for aerpawlib scripts to interact with external processes
running in a userspace.

This is the v1 version of the external process utilities.
"""

import asyncio
from typing import List, Optional
import re


class ExternalProcess:
    """
    Representation of an external process spawned by the script.

    Allows for asynchronous interaction with standard streams (stdin, stdout)
    and dynamic passing of command-line arguments.

    Attributes:
        _executable (str): Path to the executable.
        _params (list): List of command-line arguments.
        _stdin (str, optional): Path to a file to use for stdin redirection.
        _stdout (str, optional): Path to a file to use for stdout redirection.
        process (asyncio.subprocess.Process): The underlying process object.
    """

    def __init__(
        self, executable: str, params=None, stdin: str = None, stdout: str = None
    ):
        """
        Prepare external process for execution.

        Does NOT execute process immediately; call `start()` to run it.

        Args:
            executable (str): The executable path or command.
            params (list, optional): Parameters to pass to the process. Defaults to None.
            stdin (str, optional): Filename for stdin redirection. Defaults to None.
            stdout (str, optional): Filename for stdout redirection. Defaults to None.
        """
        self._executable = executable
        self._params = params if params is not None else []
        self._stdin = stdin
        self._stdout = stdout

    async def start(self):
        """
        Start the executable in an asynchronous process.
        """
        executable = self._executable
        executable += " " + " ".join(self._params)
        if not self._stdin is None:
            executable += f" < {self._stdin}"
        if not self._stdout is None:
            executable += f" > {self._stdout}"

        self.process = await asyncio.create_subprocess_shell(
            executable,
            stdout=(
                None if self._stdout is not None else asyncio.subprocess.PIPE
            ),
            stdin=None if self._stdin is not None else asyncio.subprocess.PIPE,
        )

    async def read_line(self) -> Optional[str]:
        """
        Read one line from the stdout buffer.

        Returns:
            str: The read line decoded to ASCII, or None if the process has stopped
                or stdout is redirected to a file.
        """
        if not self.process.stdout:
            return None
        out = await self.process.stdout.readline()
        if not out:
            return None
        return out.decode("ascii").rstrip()

    async def send_input(self, data: str):
        """
        Send a string to the process's stdin.

        Args:
            data (str): The data to send.

        Raises:
            RuntimeError: If stdin is not available (e.g. redirected to a file).
        """
        if self.process.stdin is None:
            raise RuntimeError(
                "Cannot send input: stdin is not available "
                "(was it redirected to a file?)"
            )
        self.process.stdin.write(data.encode())
        await self.process.stdin.drain()

    async def wait_until_terminated(self):
        """
        Wait until the process is complete.
        """
        await self.process.wait()

    async def wait_until_output(self, output_regex) -> List[str]:
        """
        Block and wait until a line matching the given regex is found in stdout.

        Only works if stdout is not redirected to a file.

        Args:
            output_regex (str): The regular expression to search for.

        Returns:
            List[str]: All lines read from stdout up to and including the matching line.
                Returns the buffer collected so far if the process ends or stdout
                is unavailable before a match is found.
        """
        buff = []
        while True:
            out = await self.read_line()
            if out is None:
                return buff
            buff.append(out)
            if re.search(output_regex, out):
                return buff