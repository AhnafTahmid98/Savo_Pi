"""Runtime ownership preflight and lifetime-lock regression tests."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import time

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "deploy" / "common" / "runtime_ownership.py"


def _write_process(proc_root: Path, pid: int, command: str) -> None:
    process = proc_root / str(pid)
    process.mkdir(parents=True, exist_ok=True)
    process.joinpath("cmdline").write_bytes(command.replace(" ", "\0").encode() + b"\0")


def _fake_systemctl(tmp_path: Path, active_units: tuple[str, ...]) -> tuple[Path, dict[str, str]]:
    binary = tmp_path / "systemctl"
    binary.write_text(
        "#!/usr/bin/env bash\n"
        "printf '%s\\n' \"$*\" >> \"${SAVO_TEST_SYSTEMCTL_LOG}\"\n"
        "printf '%s\\n' \"${SAVO_TEST_ACTIVE_UNITS:-}\"\n",
        encoding="utf-8",
    )
    binary.chmod(0o755)
    environment = os.environ.copy()
    environment["PATH"] = f"{tmp_path}:{environment.get('PATH', '')}"
    environment["SAVO_TEST_ACTIVE_UNITS"] = "\n".join(active_units)
    environment["SAVO_TEST_SYSTEMCTL_LOG"] = str(tmp_path / "systemctl.log")
    return binary, environment


def _preflight(
    tmp_path: Path,
    *,
    owner: str,
    active_units: tuple[str, ...] = (),
    processes: tuple[str, ...] = (),
    allow_unit: str | None = None,
    ignore_pids: tuple[int, ...] = (),
) -> subprocess.CompletedProcess[str]:
    proc_root = tmp_path / "proc"
    proc_root.mkdir(exist_ok=True)
    for index, command in enumerate(processes, start=100):
        _write_process(proc_root, index, command)
    _, environment = _fake_systemctl(tmp_path, active_units)
    command = [
        sys.executable,
        str(SCRIPT),
        "preflight",
        "--owner",
        owner,
        "--proc-root",
        str(proc_root),
    ]
    if allow_unit:
        command.extend(("--allow-unit", allow_unit))
    for pid in ignore_pids:
        command.extend(("--ignore-pid", str(pid)))
    return subprocess.run(command, env=environment, capture_output=True, text=True, check=False)


def test_mapping_preflight_rejects_normal_core_service(tmp_path: Path) -> None:
    result = _preflight(
        tmp_path,
        owner="mapping",
        active_units=("● savo_core.service loaded active running Robot SAVO core",),
    )

    assert result.returncode != 0
    assert "savo_core.service" in result.stderr
    assert "stop the existing owner explicitly" in result.stderr


def test_core_preflight_rejects_mapping_and_hardware_processes(tmp_path: Path) -> None:
    result = _preflight(
        tmp_path,
        owner="core",
        active_units=("savo_mapping.service loaded active running Robot SAVO mapping",),
        processes=(
            "/opt/ros/jazzy/lib/savo_base/base_driver_node --ros-args",
            "/opt/robot-savo/savo_ws/install/savo_localization/lib/"
            "savo_localization/wheel_odom_node",
            "/opt/robot-savo/savo_ws/install/savo_perception/lib/savo_perception/ultrasonic_node",
            "/opt/robot-savo/savo_ws/install/savo_lidar/lib/savo_lidar/lidar_driver_node",
        ),
    )

    assert result.returncode != 0
    assert "savo_mapping.service" in result.stderr
    assert "base_driver_node" in result.stderr
    assert "wheel_odom_node" in result.stderr
    assert "ultrasonic_node" in result.stderr
    assert "lidar_driver_node" in result.stderr


def test_expected_service_is_allowed_but_other_owner_is_not(tmp_path: Path) -> None:
    allowed = _preflight(
        tmp_path,
        owner="mapping",
        active_units=("savo_mapping.service loaded activating start Robot SAVO mapping",),
        allow_unit="savo_mapping.service",
    )
    assert allowed.returncode == 0, allowed.stderr

    conflicting = _preflight(
        tmp_path,
        owner="mapping",
        active_units=(
            "savo_mapping.service loaded activating start Robot SAVO mapping",
            "savo-supervisor.service loaded active running Robot SAVO supervisor",
        ),
        allow_unit="savo_mapping.service",
    )
    assert conflicting.returncode != 0
    assert "savo-supervisor.service" in conflicting.stderr


def test_read_only_preflight_does_not_start_or_stop_services(tmp_path: Path) -> None:
    result = _preflight(tmp_path, owner="mapping")

    assert result.returncode == 0, result.stderr
    assert "ownership preflight: PASS" in result.stdout
    calls = tmp_path.joinpath("systemctl.log").read_text(encoding="utf-8")
    assert "list-unit-files" in calls
    assert "list-units" in calls
    assert not any(token in calls for token in (" start ", " stop ", " restart ", " kill "))


def test_preflight_ignores_only_the_calling_runner_process(tmp_path: Path) -> None:
    ignored = _preflight(
        tmp_path,
        owner="mapping",
        processes=("bash deploy/core/run_autonomous_mapping.sh",),
        ignore_pids=(100,),
    )
    assert ignored.returncode == 0, ignored.stderr

    detected = _preflight(
        tmp_path,
        owner="mapping",
        processes=("bash deploy/core/run_autonomous_mapping.sh",),
    )
    assert detected.returncode != 0
    assert "run_autonomous_mapping.sh" in detected.stderr


def test_authoritative_lock_file_has_fixed_non_world_writable_mode(tmp_path: Path) -> None:
    lock_file = tmp_path / "core-owner.lock"
    original_umask = os.umask(0o077)
    try:
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                "run-core-owner",
                "--owner",
                "core",
                "--lock-file",
                str(lock_file),
                "--",
                sys.executable,
                "-c",
                "pass",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
    finally:
        os.umask(original_umask)

    assert result.returncode == 0, result.stderr
    metadata = lock_file.stat()
    assert stat.S_IMODE(metadata.st_mode) == 0o640
    assert metadata.st_uid == os.geteuid()
    assert metadata.st_gid == tmp_path.stat().st_gid


def test_missing_authoritative_lock_directory_fails_closed(tmp_path: Path) -> None:
    marker = tmp_path / "command-started"
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "run-core-owner",
            "--owner",
            "core",
            "--lock-file",
            str(tmp_path / "missing" / "core-owner.lock"),
            "--",
            sys.executable,
            "-c",
            "from pathlib import Path; import sys; Path(sys.argv[1]).write_text('started')",
            str(marker),
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode != 0
    assert "lock directory is missing" in result.stderr
    assert not marker.exists()


def test_symlinked_lock_directory_fails_closed(tmp_path: Path) -> None:
    actual_directory = tmp_path / "actual"
    actual_directory.mkdir()
    alias_directory = tmp_path / "alias"
    alias_directory.symlink_to(actual_directory, target_is_directory=True)
    marker = tmp_path / "command-started"
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "run-core-owner",
            "--owner",
            "core",
            "--lock-file",
            str(alias_directory / "core-owner.lock"),
            "--",
            sys.executable,
            "-c",
            "from pathlib import Path; import sys; Path(sys.argv[1]).write_text('started')",
            str(marker),
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode != 0
    assert "must not be a symlink" in result.stderr
    assert not marker.exists()


def test_competing_core_owners_are_serialized_and_lock_releases_on_exit(
    tmp_path: Path,
) -> None:
    lock_file = tmp_path / "core-owner.lock"
    first_marker = tmp_path / "first-started"
    second_marker = tmp_path / "second-started"
    released_marker = tmp_path / "released-started"
    sleeper = (
        "from pathlib import Path; import sys,time; "
        "import os; Path(sys.argv[1]).write_text(str(os.getpid())); time.sleep(30)"
    )
    first = subprocess.Popen(
        [
            sys.executable,
            str(SCRIPT),
            "run-core-owner",
            "--owner",
            "core",
            "--lock-file",
            str(lock_file),
            "--",
            sys.executable,
            "-c",
            sleeper,
            str(first_marker),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not first_marker.exists():
            time.sleep(0.02)
        assert first_marker.exists(), first.stderr.read() if first.stderr else ""
        assert int(first_marker.read_text(encoding="utf-8")) == first.pid
        assert lock_file.read_bytes() == b""

        second = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                "run-core-owner",
                "--owner",
                "mapping",
                "--lock-file",
                str(lock_file),
                "--",
                sys.executable,
                "-c",
                "from pathlib import Path; import sys; Path(sys.argv[1]).write_text('started')",
                str(second_marker),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        assert second.returncode != 0
        assert not second_marker.exists()
        assert "Core runtime ownership lock is already held" in second.stderr
    finally:
        first.terminate()
        first.wait(timeout=5)

    released = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "run-core-owner",
            "--owner",
            "mapping",
            "--lock-file",
            str(lock_file),
            "--",
            sys.executable,
            "-c",
            "from pathlib import Path; import sys; Path(sys.argv[1]).write_text('started')",
            str(released_marker),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert released.returncode == 0, released.stderr
    assert released_marker.exists()


@pytest.mark.parametrize("termination_signal", (signal.SIGTERM, signal.SIGINT))
def test_signal_targets_exec_runtime_and_lock_releases_only_after_runtime_exit(
    tmp_path: Path,
    termination_signal: signal.Signals,
) -> None:
    lock_file = tmp_path / "core-owner.lock"
    pid_marker = tmp_path / "runtime-pid"
    term_marker = tmp_path / "term-received"
    exit_gate = tmp_path / "allow-exit"
    contender_marker = tmp_path / "contender-started"
    runtime = (
        "import os,signal,sys,time; from pathlib import Path; "
        "pid,term,gate=map(Path,sys.argv[1:4]); "
        "handler=lambda number,_:term.write_text(str(number)); "
        "signal.signal(signal.SIGTERM,handler); signal.signal(signal.SIGINT,handler); "
        "pid.write_text(str(os.getpid())); "
        "\nwhile not gate.exists(): time.sleep(0.02)"
    )
    owner = subprocess.Popen(
        [
            sys.executable,
            str(SCRIPT),
            "run-core-owner",
            "--owner",
            "core",
            "--lock-file",
            str(lock_file),
            "--",
            sys.executable,
            "-c",
            runtime,
            str(pid_marker),
            str(term_marker),
            str(exit_gate),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not pid_marker.exists():
            time.sleep(0.02)
        assert pid_marker.exists(), owner.stderr.read() if owner.stderr else ""
        runtime_pid = int(pid_marker.read_text(encoding="utf-8"))
        assert runtime_pid == owner.pid

        owner.send_signal(termination_signal)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not term_marker.exists():
            time.sleep(0.02)
        assert term_marker.exists()
        assert int(term_marker.read_text(encoding="utf-8")) == termination_signal
        assert owner.poll() is None

        blocked = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                "run-core-owner",
                "--owner",
                "mapping",
                "--lock-file",
                str(lock_file),
                "--",
                sys.executable,
                "-c",
                "from pathlib import Path; import sys; Path(sys.argv[1]).write_text('started')",
                str(contender_marker),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        assert blocked.returncode != 0
        assert not contender_marker.exists()
        assert owner.poll() is None

        exit_gate.write_text("exit", encoding="utf-8")
        assert owner.wait(timeout=5) == 0
        with pytest.raises(ProcessLookupError):
            os.kill(runtime_pid, 0)

        released = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                "run-core-owner",
                "--owner",
                "mapping",
                "--lock-file",
                str(lock_file),
                "--",
                sys.executable,
                "-c",
                "from pathlib import Path; import sys; Path(sys.argv[1]).write_text('started')",
                str(contender_marker),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        assert released.returncode == 0, released.stderr
        assert contender_marker.exists()
    finally:
        if owner.poll() is None:
            exit_gate.write_text("exit", encoding="utf-8")
            owner.wait(timeout=5)
