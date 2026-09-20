"""Read-only FastDDS shared-memory diagnostic contracts."""

from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "deploy/common/diagnose_fastdds_shm.py"

_SPEC = importlib.util.spec_from_file_location("diagnose_fastdds_shm", SCRIPT)
assert _SPEC and _SPEC.loader
diagnose_fastdds_shm = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(diagnose_fastdds_shm)


def _run(shm_root: Path, proc_root: Path) -> dict[str, object]:
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--shm-root",
            str(shm_root),
            "--proc-root",
            str(proc_root),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    return json.loads(result.stdout)


def test_diagnostic_reports_candidates_without_mutating_them(tmp_path: Path) -> None:
    shm_root = tmp_path / "shm"
    proc_root = tmp_path / "proc"
    shm_root.mkdir()
    proc_root.mkdir()
    candidate = shm_root / "fastrtps_port7421"
    candidate.write_text("sentinel", encoding="utf-8")

    report = _run(shm_root, proc_root)

    assert candidate.read_text(encoding="utf-8") == "sentinel"
    assert report["candidate_count"] == 1
    assert report["state"] == "STALE_CANDIDATES_UNCONFIRMED"
    assert report["cleanup_performed"] is False
    assert report["cleanup_safe_to_assume"] is False


def test_active_ros_process_keeps_diagnosis_uncertain(tmp_path: Path) -> None:
    shm_root = tmp_path / "shm"
    proc_root = tmp_path / "proc"
    shm_root.mkdir()
    process = proc_root / "123"
    process.mkdir(parents=True)
    process.joinpath("cmdline").write_bytes(
        b"/opt/ros/jazzy/bin/ros2\0launch\0savo_bringup\0"
    )

    report = _run(shm_root, proc_root)

    assert report["state"] == "ROS_PARTICIPANTS_ACTIVE"
    assert report["active_process_count"] == 1
    assert report["cleanup_safe_to_assume"] is False


def test_diagnostic_does_not_count_its_own_process(tmp_path: Path) -> None:
    shm_root = tmp_path / "shm"
    proc_root = tmp_path / "proc"
    shm_root.mkdir()
    process = proc_root / str(os.getpid())
    process.mkdir(parents=True)
    process.joinpath("cmdline").write_bytes(
        b"python3\0diagnose_fastdds_shm.py\0"
    )

    report = diagnose_fastdds_shm.diagnose(shm_root, proc_root)

    assert report["state"] == "NO_FASTDDS_SHM_CANDIDATES"
    assert report["active_process_count"] == 0


def test_unrelated_repository_process_is_not_reported_as_ros(
    tmp_path: Path,
) -> None:
    shm_root = tmp_path / "shm"
    proc_root = tmp_path / "proc"
    shm_root.mkdir()
    process = proc_root / "321"
    process.mkdir(parents=True)
    process.joinpath("cmdline").write_bytes(
        b"python3\0/home/savo/Savo_Pi/deploy/common/validate_deployment_assets.py\0"
    )

    report = _run(shm_root, proc_root)

    assert report["state"] == "NO_FASTDDS_SHM_CANDIDATES"
    assert report["active_process_count"] == 0


def test_installed_savo_ros_node_is_reported_as_active(tmp_path: Path) -> None:
    shm_root = tmp_path / "shm"
    proc_root = tmp_path / "proc"
    shm_root.mkdir()
    process = proc_root / "322"
    process.mkdir(parents=True)
    process.joinpath("cmdline").write_bytes(
        b"/home/savo/Savo_Pi/savo_ws/install/savo_power/lib/"
        b"savo_power/core_ups_node\0--ros-args\0"
    )

    report = _run(shm_root, proc_root)

    assert report["state"] == "ROS_PARTICIPANTS_ACTIVE"
    assert report["active_process_count"] == 1


def test_native_ros_install_node_is_reported_as_active(tmp_path: Path) -> None:
    shm_root = tmp_path / "shm"
    proc_root = tmp_path / "proc"
    shm_root.mkdir()
    process = proc_root / "323"
    process.mkdir(parents=True)
    process.joinpath("cmdline").write_bytes(
        b"/opt/ros/jazzy/lib/nav2_controller/controller_server\0--ros-args\0"
    )

    report = _run(shm_root, proc_root)

    assert report["state"] == "ROS_PARTICIPANTS_ACTIVE"
    assert report["active_process_count"] == 1


def test_diagnostic_contains_no_cleanup_implementation() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    assert "fastdds shm clean" not in source
    assert "--force" not in source
    assert ".unlink(" not in source
    assert "shutil.rmtree" not in source
