from __future__ import annotations

import json
import select
import subprocess
import time
from pathlib import Path
from typing import Any


class SlamGatewayExecutorError(RuntimeError):
    pass


class LocalSlamGatewayExecutor:
    """Persistent local bridge to the C++ slam_llm_command_client."""

    def __init__(
        self,
        executable: str | Path,
        *,
        network_interface: str = "eth0",
        startup_timeout_s: float = 5.0,
        response_timeout_s: float = 15.0,
    ) -> None:
        self.executable = str(executable)
        self.network_interface = network_interface
        self.startup_timeout_s = startup_timeout_s
        self.response_timeout_s = response_timeout_s
        self._process: subprocess.Popen[str] | None = None

    def start(self) -> None:
        if self._process is not None and self._process.poll() is None:
            return

        self._process = subprocess.Popen(
            [self.executable, self.network_interface],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self._wait_until_ready()

    def close(self) -> None:
        if self._process is None:
            return
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=3)
        self._process = None

    def __enter__(self) -> "LocalSlamGatewayExecutor":
        self.start()
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        self.close()

    def send_command(self, command: dict[str, Any]) -> dict[str, Any]:
        self.start()
        assert self._process is not None
        if self._process.stdin is None or self._process.stdout is None:
            raise SlamGatewayExecutorError("slam_llm_command_client stdio is not available")
        if self._process.poll() is not None:
            raise SlamGatewayExecutorError(f"slam_llm_command_client exited with code {self._process.returncode}")

        self._process.stdin.write(json.dumps(command, ensure_ascii=False, separators=(",", ":")) + "\n")
        self._process.stdin.flush()
        return self._read_json_response()

    def _wait_until_ready(self) -> None:
        assert self._process is not None
        if self._process.stdout is None:
            raise SlamGatewayExecutorError("slam_llm_command_client stdout is not available")

        deadline = time.monotonic() + self.startup_timeout_s
        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                stderr = self._read_stderr_tail()
                raise SlamGatewayExecutorError(f"slam_llm_command_client exited during startup: {stderr}")
            line = self._readline_with_timeout(self._process.stdout, deadline - time.monotonic())
            if line is None:
                continue
            if "ready" in line:
                return
        raise SlamGatewayExecutorError("timed out waiting for slam_llm_command_client readiness")

    def _read_json_response(self) -> dict[str, Any]:
        assert self._process is not None
        assert self._process.stdout is not None

        deadline = time.monotonic() + self.response_timeout_s
        lines: list[str] = []
        brace_depth = 0
        in_json = False
        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                raise SlamGatewayExecutorError(f"slam_llm_command_client exited with code {self._process.returncode}")

            line = self._readline_with_timeout(self._process.stdout, deadline - time.monotonic())
            if line is None:
                continue

            stripped = line.strip()
            if not in_json and not stripped.startswith("{"):
                continue

            in_json = True
            lines.append(line)
            brace_depth += line.count("{") - line.count("}")
            if brace_depth <= 0:
                text = "".join(lines)
                try:
                    return json.loads(text)
                except json.JSONDecodeError as exc:
                    raise SlamGatewayExecutorError(f"invalid JSON response from slam gateway: {text}") from exc

        raise SlamGatewayExecutorError("timed out waiting for JSON response from slam gateway")

    def _readline_with_timeout(self, stream: Any, timeout_s: float) -> str | None:
        if timeout_s <= 0:
            return None
        readable, _, _ = select.select([stream], [], [], timeout_s)
        if not readable:
            return None
        line = stream.readline()
        return line if line else None

    def _read_stderr_tail(self) -> str:
        if self._process is None or self._process.stderr is None:
            return ""
        try:
            return self._process.stderr.read()[-1000:]
        except Exception:
            return ""
