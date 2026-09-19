#!/usr/bin/env python3
"""Report FastDDS SHM evidence without changing participants or files."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path


def _is_fastdds_shm_name(name: str) -> bool:
    lowered = name.lower()
    return (
        lowered.startswith("fastrtps_")
        or lowered.startswith("fastdds_")
        or lowered.startswith("sem.fastrtps_")
    )


def _shm_candidates(shm_root: Path) -> list[dict[str, object]]:
    candidates: list[dict[str, object]] = []
    if not shm_root.is_dir():
        return candidates
    for path in sorted(shm_root.iterdir(), key=lambda item: item.name):
        if not _is_fastdds_shm_name(path.name):
            continue
        try:
            metadata = path.stat()
        except OSError:
            continue
        candidates.append(
            {
                "name": path.name,
                "size_bytes": metadata.st_size,
                "uid": metadata.st_uid,
                "mtime_ns": metadata.st_mtime_ns,
            }
        )
    return candidates


def _active_ros_processes(
    proc_root: Path,
    *,
    ignore_pid: int | None = None,
) -> list[dict[str, object]]:
    processes: list[dict[str, object]] = []
    if not proc_root.is_dir():
        return processes
    for process_dir in sorted(proc_root.iterdir(), key=lambda item: item.name):
        if not process_dir.name.isdigit():
            continue
        if ignore_pid is not None and int(process_dir.name) == ignore_pid:
            continue
        try:
            command = process_dir.joinpath("cmdline").read_bytes().replace(
                b"\0", b" "
            ).decode("utf-8", errors="replace").strip()
        except OSError:
            continue
        lowered = command.lower()
        if any(
            marker in lowered
            for marker in (
                "/ros2",
                " ros2 ",
                "savo_",
                "robot_savo",
                "rmw_fastrtps",
                "fastdds",
            )
        ):
            processes.append({"pid": int(process_dir.name), "command": command})
    return processes


def diagnose(shm_root: Path, proc_root: Path) -> dict[str, object]:
    """Build a deliberately non-authoritative, read-only SHM report."""
    candidates = _shm_candidates(shm_root)
    active_processes = _active_ros_processes(
        proc_root,
        ignore_pid=os.getpid(),
    )
    if active_processes:
        state = "ROS_PARTICIPANTS_ACTIVE"
    elif candidates:
        state = "STALE_CANDIDATES_UNCONFIRMED"
    else:
        state = "NO_FASTDDS_SHM_CANDIDATES"
    return {
        "schema_version": 1,
        "state": state,
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
        "shm_root": str(shm_root),
        "candidate_count": len(candidates),
        "candidates": candidates,
        "active_process_count": len(active_processes),
        "active_processes": active_processes,
        "cleanup_performed": False,
        "cleanup_safe_to_assume": False,
        "note": (
            "Candidate files do not prove that SHM caused a ROS failure. "
            "Stop all ROS owners and participants before an operator considers "
            "the non-force FastDDS cleanup command."
        ),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--shm-root", type=Path, default=Path("/dev/shm"))
    parser.add_argument("--proc-root", type=Path, default=Path("/proc"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print(json.dumps(diagnose(args.shm_root, args.proc_root), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
