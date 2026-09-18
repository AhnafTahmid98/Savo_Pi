#!/usr/bin/env python3
"""
Inspect Robot Savo runtime ownership or hold the Core lifetime lock.

The preflight subcommand is read-only. The run-core-owner subcommand acquires
an advisory kernel lock and then replaces itself with the requested top-level
runtime process. The inherited descriptor keeps the lock for that process's
entire lifetime and the kernel releases it on every exit path.
"""

from __future__ import annotations

import argparse
import fcntl
import os
from pathlib import Path
import subprocess
import sys


ROBOT_UNITS = {
    "savo_core.service",
    "savo.service",
    "savo_mapping.service",
    "savo-supervisor.service",
}
UNIT_PREFIXES = ("savo-location-stack@",)
PROCESS_MARKERS = (
    "base_driver_node",
    "imu_node",
    "core_ups_node",
    "base_battery_node",
    "wheel_odom_node",
    "kit_battery",
    "ads7830",
    "vl53_mux_node",
    "ultrasonic_node",
    "lidar_driver_node",
    "autonomous_mapping",
    "supervisor_node",
)


def _unit_from_line(line: str) -> str:
    fields = line.strip().split()
    if fields and fields[0] == "●":
        fields = fields[1:]
    return fields[0] if fields else ""


def _active_robot_units(systemctl: str) -> list[str]:
    try:
        result = subprocess.run(
            [
                systemctl,
                "list-units",
                "--type=service",
                "--all",
                "--no-legend",
                "--plain",
                "--state=active,activating",
                "savo*",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
    except FileNotFoundError as exc:
        raise RuntimeError(f"systemctl unavailable: {exc.filename}") from exc
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        raise RuntimeError(f"unable to inspect systemd services: {detail}")
    units = []
    for line in result.stdout.splitlines():
        unit = _unit_from_line(line)
        if unit in ROBOT_UNITS or unit.startswith(UNIT_PREFIXES):
            units.append(unit)
    return sorted(set(units))


def _installed_robot_units(systemctl: str) -> list[str]:
    try:
        result = subprocess.run(
            [systemctl, "list-unit-files", "--no-legend", "--plain", "savo*"],
            capture_output=True,
            text=True,
            check=False,
        )
    except FileNotFoundError as exc:
        raise RuntimeError(f"systemctl unavailable: {exc.filename}") from exc
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        raise RuntimeError(f"unable to inspect installed systemd services: {detail}")
    units = []
    for line in result.stdout.splitlines():
        unit = _unit_from_line(line)
        if unit in ROBOT_UNITS or unit.startswith(UNIT_PREFIXES):
            units.append(unit)
    return sorted(set(units))


def _robot_processes(proc_root: Path, ignore_pids: set[int]) -> list[str]:
    matches: list[str] = []
    try:
        processes = tuple(proc_root.iterdir())
    except OSError as exc:
        raise RuntimeError(f"unable to inspect process table {proc_root}: {exc}") from exc
    for process in processes:
        if not process.name.isdigit() or int(process.name) in ignore_pids | {os.getpid()}:
            continue
        try:
            command = process.joinpath("cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace"
            ).strip()
        except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
            continue
        if command and any(marker in command for marker in PROCESS_MARKERS):
            matches.append(f"pid={process.name} command={command}")
    return sorted(matches)


def preflight(
    owner: str,
    allow_unit: str | None,
    proc_root: Path,
    systemctl: str,
    ignore_pids: set[int],
) -> int:
    """Report overlapping Robot Savo services and processes without changing state."""
    try:
        installed_units = _installed_robot_units(systemctl)
        units = [unit for unit in _active_robot_units(systemctl) if unit != allow_unit]
        processes = _robot_processes(proc_root, ignore_pids)
    except RuntimeError as exc:
        print(f"Robot Savo ownership preflight failed closed: {exc}", file=sys.stderr)
        return 3

    if units or processes:
        print(
            f"Robot Savo ownership preflight failed for requested owner={owner}.",
            file=sys.stderr,
        )
        print(
            "installed Robot Savo units: " + (", ".join(installed_units) or "none"),
            file=sys.stderr,
        )
        for unit in units:
            print(f"conflicting service: {unit}", file=sys.stderr)
        for process in processes:
            print(f"conflicting process: {process}", file=sys.stderr)
        print(
            "Keep the robot in STOP and stop the existing owner explicitly; "
            "nothing was stopped automatically.",
            file=sys.stderr,
        )
        return 4

    print("installed Robot Savo units: " + (", ".join(installed_units) or "none"))
    print(f"Robot Savo ownership preflight: PASS owner={owner}; no overlap detected")
    return 0


def run_core_owner(owner: str, lock_file: Path, command: list[str]) -> int:
    """Acquire the shared Core lock and exec the top-level runtime command."""
    if not command:
        print("run-core-owner requires a command after --", file=sys.stderr)
        return 2
    if command[0] == "--":
        command = command[1:]
    if not command:
        print("run-core-owner requires a command after --", file=sys.stderr)
        return 2
    if not lock_file.is_absolute():
        print("Core ownership lock path must be absolute", file=sys.stderr)
        return 2
    if lock_file.parent.is_symlink():
        print(
            f"Core ownership lock directory must not be a symlink: {lock_file.parent}",
            file=sys.stderr,
        )
        return 3
    if not lock_file.parent.is_dir():
        print(
            f"Core ownership lock directory is missing: {lock_file.parent}. "
            "Use the rendered systemd unit or prepare the runtime directory first.",
            file=sys.stderr,
        )
        return 3

    flags = os.O_RDWR | os.O_CREAT
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(lock_file, flags, 0o640)
    except OSError as exc:
        print(f"Unable to open Core ownership lock {lock_file}: {exc}", file=sys.stderr)
        return 3
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(descriptor)
        print(
            f"Core runtime ownership lock is already held: {lock_file}. "
            "The existing owner was not stopped or replaced.",
            file=sys.stderr,
        )
        return 4
    except OSError as exc:
        os.close(descriptor)
        print(f"Unable to acquire Core ownership lock {lock_file}: {exc}", file=sys.stderr)
        return 3

    try:
        os.fchmod(descriptor, 0o640)
    except OSError as exc:
        os.close(descriptor)
        print(f"Unable to secure Core ownership lock {lock_file}: {exc}", file=sys.stderr)
        return 3
    os.set_inheritable(descriptor, True)
    print(
        f"Robot Savo Core ownership acquired: owner={owner} lock={lock_file}",
        flush=True,
    )
    try:
        os.execvp(command[0], command)
    except OSError as exc:
        os.close(descriptor)
        print(f"Unable to execute Core owner command {command[0]}: {exc}", file=sys.stderr)
        return 3


def parse_args() -> argparse.Namespace:
    """Parse the ownership utility command line."""
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command_name", required=True)

    inspect = subparsers.add_parser("preflight")
    inspect.add_argument("--owner", required=True)
    inspect.add_argument("--allow-unit")
    inspect.add_argument("--proc-root", type=Path, default=Path("/proc"))
    inspect.add_argument("--systemctl", default="systemctl")
    inspect.add_argument("--ignore-pid", action="append", type=int, default=[])

    run = subparsers.add_parser("run-core-owner")
    run.add_argument("--owner", required=True)
    run.add_argument("--lock-file", type=Path, default=Path("/run/robot-savo/core-owner.lock"))
    run.add_argument("command", nargs=argparse.REMAINDER)
    return parser.parse_args()


def main() -> int:
    """Run the selected ownership operation."""
    args = parse_args()
    if args.command_name == "preflight":
        return preflight(
            args.owner,
            args.allow_unit,
            args.proc_root,
            args.systemctl,
            set(args.ignore_pid),
        )
    return run_core_owner(args.owner, args.lock_file, args.command)


if __name__ == "__main__":
    raise SystemExit(main())
