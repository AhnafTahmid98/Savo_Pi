"""Process wrapper for external RPLIDAR driver bringup.

This is useful when we want `savo_lidar` to own the hardware launch while still
using a proven ROS driver as the scan publisher.
"""

from __future__ import annotations

import subprocess
import time
from dataclasses import dataclass
from typing import Sequence


@dataclass
class RplidarProcessState:
    running: bool
    returncode: int | None
    pid: int | None
    uptime_s: float
    command: tuple[str, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "running": self.running,
            "returncode": self.returncode,
            "pid": self.pid,
            "uptime_s": self.uptime_s,
            "command": list(self.command),
        }


class RplidarProcess:
    def __init__(self, command: Sequence[str]) -> None:
        if not command:
            raise ValueError("RPLIDAR process command cannot be empty.")

        self.command = tuple(str(part) for part in command)
        self._process: subprocess.Popen[str] | None = None
        self._started_s: float | None = None

    @property
    def running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    @property
    def pid(self) -> int | None:
        if self._process is None:
            return None

        return self._process.pid

    def start(self) -> None:
        if self.running:
            return

        self._process = subprocess.Popen(
            self.command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        self._started_s = time.monotonic()

    def stop(self, timeout_s: float = 3.0) -> None:
        if self._process is None:
            return

        if self.running:
            self._process.terminate()

            try:
                self._process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=timeout_s)

        self._process = None
        self._started_s = None

    def state(self) -> RplidarProcessState:
        now_s = time.monotonic()

        uptime_s = 0.0
        if self._started_s is not None:
            uptime_s = max(0.0, now_s - self._started_s)

        returncode = None
        if self._process is not None:
            returncode = self._process.poll()

        return RplidarProcessState(
            running=self.running,
            returncode=returncode,
            pid=self.pid,
            uptime_s=uptime_s,
            command=self.command,
        )