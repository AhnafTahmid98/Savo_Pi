#!/usr/bin/env python3
"""Shared fail-closed helpers for Robot SAVO diagnostic commands."""

from __future__ import annotations

import argparse
import json
import os
import platform
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Sequence


@dataclass(frozen=True)
class Result:
    status: str
    diagnostic: str
    reason: str
    started_utc: str
    duration_s: float
    command: list[str]
    details: dict[str, object]


def parser(description: str, *, motion: bool = False) -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(description=description)
    value.add_argument("--timeout", type=float, default=5.0)
    value.add_argument("--output", type=Path)
    value.add_argument("--dry-run", action="store_true")
    if motion:
        value.add_argument("--allow-motion", action="store_true")
    return value


def command_available(name: str) -> bool:
    return subprocess.run(
        ["bash", "-lc", f"command -v {name}"],
        capture_output=True,
        text=True,
        check=False,
    ).returncode == 0


def run(
    diagnostic: str,
    command: Sequence[str],
    *,
    timeout: float,
    output: Path | None,
    dry_run: bool = False,
    blocked_reason: str | None = None,
) -> int:
    started = time.monotonic()
    started_utc = datetime.now(UTC).isoformat()
    status = "BLOCKED"
    reason = blocked_reason or "dry_run"
    details: dict[str, object] = {
        "hostname": platform.node(),
        "platform": platform.platform(),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
    }
    if blocked_reason is None and not dry_run:
        try:
            completed = subprocess.run(
                list(command),
                capture_output=True,
                text=True,
                timeout=max(0.1, timeout),
                check=False,
            )
            details["returncode"] = completed.returncode
            details["stdout"] = completed.stdout[-4000:]
            details["stderr"] = completed.stderr[-4000:]
            if completed.returncode == 0:
                status, reason = "PASS", "command_succeeded"
            else:
                status, reason = "FAIL", "command_failed"
        except subprocess.TimeoutExpired as exc:
            status, reason = "FAIL", "timeout"
            details["stdout"] = (exc.stdout or "")[-4000:]
            details["stderr"] = (exc.stderr or "")[-4000:]
        except FileNotFoundError:
            status, reason = "BLOCKED", "required_command_missing"
    result = Result(
        status=status,
        diagnostic=diagnostic,
        reason=reason,
        started_utc=started_utc,
        duration_s=round(time.monotonic() - started, 3),
        command=list(command),
        details=details,
    )
    encoded = json.dumps(asdict(result), indent=2) + "\n"
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return {"PASS": 0, "FAIL": 1, "BLOCKED": 2}[status]



def emit_result(
    diagnostic: str,
    status: str,
    reason: str,
    details: dict[str, object],
    *,
    output: Path | None,
    started: float,
    started_utc: str,
    command: Sequence[str] = (),
) -> int:
    if status not in {"PASS", "FAIL", "BLOCKED"}:
        raise ValueError("invalid diagnostic status")
    result = Result(
        status=status,
        diagnostic=diagnostic,
        reason=reason,
        started_utc=started_utc,
        duration_s=round(time.monotonic() - started, 3),
        command=list(command),
        details=details,
    )
    encoded = json.dumps(asdict(result), indent=2) + "\n"
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return {"PASS": 0, "FAIL": 1, "BLOCKED": 2}[status]


def topic_once(
    diagnostic: str,
    topic: str,
    message_type: str,
    args: argparse.Namespace,
) -> int:
    command = [
        "ros2",
        "topic",
        "echo",
        "--once",
        topic,
        message_type,
    ]
    blocked = None if command_available("ros2") else "ros2_cli_missing"
    return run(
        diagnostic,
        command,
        timeout=args.timeout,
        output=args.output,
        dry_run=args.dry_run,
        blocked_reason=blocked,
    )


def require_motion_opt_in(args: argparse.Namespace) -> str | None:
    if not getattr(args, "allow_motion", False):
        return "motion_not_authorized_use_allow_motion"
    return "moving_diagnostic_requires_physical_safety_operator"


def main_topic(
    diagnostic: str,
    description: str,
    topic: str,
    message_type: str,
    *,
    motion: bool = False,
) -> int:
    args = parser(description, motion=motion).parse_args()
    if motion:
        reason = require_motion_opt_in(args)
        if reason:
            return run(
                diagnostic,
                [],
                timeout=args.timeout,
                output=args.output,
                dry_run=True,
                blocked_reason=reason,
            )
    return topic_once(diagnostic, topic, message_type, args)


if __name__ == "__main__":
    print("diag_utils is a library; run a specific diagnostic", file=sys.stderr)
    raise SystemExit(2)
