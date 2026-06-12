"""Small subprocess helpers for LiDAR diagnostics and port checks."""

from __future__ import annotations

import subprocess
from dataclasses import dataclass


@dataclass(frozen=True)
class CommandResult:
    command: tuple[str, ...]
    returncode: int
    stdout: str
    stderr: str

    @property
    def ok(self) -> bool:
        return self.returncode == 0


def run_command(
    command: list[str] | tuple[str, ...],
    *,
    timeout_s: float = 5.0,
) -> CommandResult:
    cmd = tuple(str(part) for part in command)

    try:
        completed = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
        return CommandResult(
            command=cmd,
            returncode=completed.returncode,
            stdout=completed.stdout.strip(),
            stderr=completed.stderr.strip(),
        )
    except subprocess.TimeoutExpired as exc:
        return CommandResult(
            command=cmd,
            returncode=124,
            stdout=(exc.stdout or "").strip() if isinstance(exc.stdout, str) else "",
            stderr=f"Command timed out after {timeout_s:.2f}s",
        )


def command_exists(command: str) -> bool:
    result = run_command(("which", command), timeout_s=2.0)
    return result.ok and bool(result.stdout)


def list_serial_devices() -> list[str]:
    result = run_command(("bash", "-lc", "ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null"), timeout_s=2.0)

    if not result.ok and not result.stdout:
        return []

    return sorted(line.strip() for line in result.stdout.splitlines() if line.strip())


def user_in_group(group_name: str) -> bool:
    result = run_command(("id", "-nG"), timeout_s=2.0)

    if not result.ok:
        return False

    groups = set(result.stdout.split())
    return str(group_name) in groups