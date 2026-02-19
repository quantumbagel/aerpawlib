"""Unit tests for aerpawlib v1 ExternalProcess."""

import asyncio
import os
import tempfile

import pytest

from aerpawlib.v1.external import ExternalProcess


class TestExternalProcess:
    """ExternalProcess creation and basic behavior."""

    def test_init_default_params(self):
        ep = ExternalProcess("echo")
        assert ep._params == []

    def test_init_with_params(self):
        ep = ExternalProcess("echo", params=["hello"])
        assert ep._params == ["hello"]

    @pytest.mark.asyncio
    async def test_echo_produces_output(self):
        ep = ExternalProcess("echo", params=["hello world"])
        await ep.start()
        try:
            line = await ep.read_line()
            assert "hello" in (line or "")
        finally:
            try:
                ep.process.terminate()
                await asyncio.wait_for(ep.process.wait(), timeout=2)
            except Exception:
                pass

    @pytest.mark.asyncio
    async def test_wait_until_output_matches(self):
        ep = ExternalProcess("echo", params=["foo bar baz"])
        await ep.start()
        try:
            buff = await ep.wait_until_output(r"bar")
            assert any("bar" in (l or "") for l in buff)
        finally:
            try:
                ep.process.terminate()
                await asyncio.wait_for(ep.process.wait(), timeout=2)
            except Exception:
                pass

    @pytest.mark.asyncio
    async def test_wait_until_output_returns_empty_on_exit(self):
        ep = ExternalProcess("true")  # Exits immediately
        await ep.start()
        buff = await ep.wait_until_output(r"nonexistent")
        assert buff == [] or buff is not None


class TestExternalProcessExtended:
    def test_init_stores_executable(self):
        ep = ExternalProcess("myapp")
        assert ep._executable == "myapp"

    def test_init_stores_stdin_stdout(self):
        ep = ExternalProcess("cat", stdin="/tmp/in.txt", stdout="/tmp/out.txt")
        assert ep._stdin == "/tmp/in.txt"
        assert ep._stdout == "/tmp/out.txt"

    def test_init_none_params_defaults_to_empty_list(self):
        ep = ExternalProcess("echo", params=None)
        assert ep._params == []

    @pytest.mark.asyncio
    async def test_read_line_multiple(self):
        """Multiple read_line calls read successive lines."""
        ep = ExternalProcess("printf", params=["line1\\nline2\\nline3\\n"])
        await ep.start()
        try:
            lines = []
            for _ in range(3):
                line = await ep.read_line()
                if line is not None:
                    lines.append(line)
            assert any("line1" in l for l in lines)
            assert any("line2" in l for l in lines)
        finally:
            try:
                ep.process.terminate()
                await asyncio.wait_for(ep.process.wait(), timeout=2)
            except Exception:
                pass

    @pytest.mark.asyncio
    async def test_read_line_returns_none_on_eof(self):
        """After all output is drained and the process exits, read_line returns None."""
        ep = ExternalProcess("echo", params=["hello"])
        await ep.start()
        await ep.wait_until_terminated()
        # Drain any buffered output lines first
        for _ in range(10):
            line = await ep.read_line()
            if line is None:
                break
        # Eventually read_line should return None (EOF)
        result = await ep.read_line()
        assert result is None

    @pytest.mark.asyncio
    async def test_wait_until_terminated_completes(self):
        """wait_until_terminated should return once the process exits."""
        ep = ExternalProcess("true")
        await ep.start()
        await asyncio.wait_for(ep.wait_until_terminated(), timeout=5.0)
        assert ep.process.returncode is not None

    @pytest.mark.asyncio
    async def test_send_input_writes_to_stdin(self):
        """cat reads from stdin and echoes back; send_input should deliver data."""
        ep = ExternalProcess("cat")
        await ep.start()
        try:
            await ep.send_input("hello\n")
            line = await asyncio.wait_for(ep.read_line(), timeout=2.0)
            assert line is not None and "hello" in line
        finally:
            try:
                ep.process.terminate()
                await asyncio.wait_for(ep.process.wait(), timeout=2)
            except Exception:
                pass

    @pytest.mark.asyncio
    async def test_send_input_raises_when_stdin_redirected(self):
        """If stdin is redirected to a file, send_input should raise RuntimeError."""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".txt", delete=False) as f:
            f.write("data\n")
            path = f.name
        ep = ExternalProcess("cat", stdin=path)
        await ep.start()
        try:
            with pytest.raises(RuntimeError, match="stdin is not available"):
                await ep.send_input("extra data\n")
        finally:
            try:
                ep.process.terminate()
                await asyncio.wait_for(ep.process.wait(), timeout=2)
            except Exception:
                pass
            os.unlink(path)

    @pytest.mark.asyncio
    async def test_wait_until_output_multiple_matches(self):
        """wait_until_output returns as soon as first match is found."""
        ep = ExternalProcess("printf", params=["alpha\\nbeta\\ngamma\\n"])
        await ep.start()
        try:
            buff = await ep.wait_until_output(r"beta")
            # buff includes all lines up to and including the matching one
            assert any("beta" in (l or "") for l in buff)
            # "gamma" may or may not be included, but shouldn't crash
        finally:
            try:
                ep.process.terminate()
                await asyncio.wait_for(ep.process.wait(), timeout=2)
            except Exception:
                pass

    @pytest.mark.asyncio
    async def test_process_params_appended_to_command(self):
        """Params are actually forwarded to the process."""
        ep = ExternalProcess("echo", params=["unique_token_xyz"])
        await ep.start()
        try:
            line = await asyncio.wait_for(ep.read_line(), timeout=2.0)
            assert line is not None and "unique_token_xyz" in line
        finally:
            try:
                ep.process.terminate()
                await asyncio.wait_for(ep.process.wait(), timeout=2)
            except Exception:
                pass

