"""Deployment rendering, ownership, and install-only contract tests."""

from __future__ import annotations

import grp
import os
from pathlib import Path
import pwd
import stat
import subprocess


ROOT = Path(__file__).resolve().parents[2]
SYSTEMD = ROOT / "deploy" / "systemd"


def _read(relative: str) -> str:
    return ROOT.joinpath(relative).read_text(encoding="utf-8")


def _render(tmp_path: Path, deployed_root: str) -> Path:
    output = tmp_path / deployed_root.strip("/").replace("/", "_")
    result = subprocess.run(
        [
            "bash",
            str(SYSTEMD / "render_units.sh"),
            "--user",
            "savo",
            "--group",
            "savo",
            "--source-root",
            str(ROOT),
            "--root",
            deployed_root,
            "--output-dir",
            str(output),
            "--skip-systemd-verify",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return output


def test_core_and_mapping_runners_use_the_same_lifetime_lock() -> None:
    core = _read("deploy/core/run_core.sh")
    mapping = _read("deploy/core/run_mapping_service.sh")
    autonomous = _read("deploy/core/run_autonomous_mapping.sh")

    for runner in (core, mapping, autonomous):
        assert "runtime_ownership.py" in runner
        assert "preflight" in runner
        assert "run-core-owner" in runner
        assert "/run/robot-savo/core-owner.lock" in runner
        assert "SAVO_CORE_OWNER_LOCK" not in runner
    assert "control_startup_mode" in core
    assert "SAVO_CONTROL_STARTUP_MODE:-STOP" in core
    assert "SAVO_CONTROL_STARTUP_MODE:-STOP" in mapping
    assert "SAVO_CONTROL_STARTUP_MODE:-STOP" in autonomous
    assert "autonomous_mapping.launch.py" in autonomous
    assert "start_supervisor:=false" in autonomous
    assert "supervisor_auto_arm:=false" in autonomous
    assert "Dedicated autonomous runner forbids overriding" in autonomous
    assert "action send_goal" not in autonomous


def test_autonomous_mapping_runner_rejects_geometry_policy_overrides() -> None:
    runner = ROOT / "deploy/core/run_autonomous_mapping.sh"

    for argument in (
        "require_locked_geometry:=false",
        "allow_provisional_geometry:=true",
        "geometry_profile:=/tmp/not-production.yaml",
        "geometry_profile:=",
        "geometry_profile:=/tmp/profile:=duplicate.yaml",
    ):
        result = subprocess.run(
            ["bash", str(runner), argument],
            capture_output=True,
            text=True,
            check=False,
        )

        assert result.returncode != 0
        assert "forbids overriding" in result.stderr
        assert argument.split(":=", maxsplit=1)[0] in result.stderr


def test_production_role_runners_reject_unsafe_geometry_environment() -> None:
    cases = (
        ("deploy/core/run_core.sh", "SAVO_REQUIRE_LOCKED_GEOMETRY", "false"),
        ("deploy/core/run_core.sh", "SAVO_ALLOW_PROVISIONAL_GEOMETRY", "true"),
        ("deploy/edge/run_edge.sh", "SAVO_REQUIRE_LOCKED_GEOMETRY", "false"),
        ("deploy/edge/run_edge.sh", "SAVO_ALLOW_PROVISIONAL_GEOMETRY", "true"),
    )

    for relative, variable, value in cases:
        environment = os.environ.copy()
        environment[variable] = value
        result = subprocess.run(
            ["bash", str(ROOT / relative)],
            env=environment,
            capture_output=True,
            text=True,
            check=False,
        )

        assert result.returncode != 0
        assert "production geometry policy" in result.stderr.lower()
        assert variable in result.stderr


def test_production_geometry_helper_uses_the_installed_package_artifact(
    tmp_path: Path,
) -> None:
    workspace = tmp_path / "workspace with spaces"
    result = subprocess.run(
        [
            "bash",
            "-c",
            'SAVO_WS="$1"; source "$2"; savo_production_geometry_profile',
            "bash",
            str(workspace),
            str(ROOT / "deploy/common/production_geometry.sh"),
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == str(
        workspace
        / "install/savo_description/share/savo_description/config/profiles/"
        "robot_savo_core_v1.yaml"
    )
    assert "/src/shared/savo_description" not in result.stdout


def test_runtime_storage_prepares_direct_runner_lock_directory() -> None:
    storage = _read("deploy/core/prepare_runtime_storage.sh")

    assert 'RUNTIME_ROOT="/run/robot-savo"' in storage
    assert '"${RUNTIME_ROOT}"' in storage


def test_runtime_storage_uses_same_private_directory_mode(tmp_path: Path) -> None:
    runtime_root = tmp_path / "run"
    result = subprocess.run(
        [
            str(ROOT / "deploy/core/prepare_runtime_storage.sh"),
            "--owner",
            pwd.getpwuid(os.getuid()).pw_name,
            "--group",
            grp.getgrgid(os.getgid()).gr_name,
            "--state-root",
            str(tmp_path / "state"),
            "--log-root",
            str(tmp_path / "log"),
            "--runtime-root",
            str(runtime_root),
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert stat.S_IMODE(runtime_root.stat().st_mode) == 0o750


def test_core_runtime_directory_is_rendered_for_boot_recreation(tmp_path: Path) -> None:
    output = _render(tmp_path, "/home/savo/Savo_Pi")

    policy = output.joinpath("robot-savo-core-tmpfiles.conf")
    assert policy.read_text(encoding="utf-8") == (
        "# Robot SAVO Core lifetime-lock directory.\n"
        "d /run/robot-savo 0750 savo savo -\n"
    )
    for unit_name in ("savo_core.service", "savo_mapping.service", "savo.service"):
        unit = output.joinpath(unit_name).read_text(encoding="utf-8")
        assert "RuntimeDirectory=robot-savo" in unit
        assert "RuntimeDirectoryMode=0750" in unit
        assert "RuntimeDirectoryPreserve=yes" in unit


def test_mapping_unit_is_an_alternative_owner_not_a_core_overlay() -> None:
    mapping = _read("deploy/systemd/savo_mapping.service")

    assert "After=network-online.target savo_core.service" not in mapping
    assert "After=network-online.target" in mapping
    assert "runtime_ownership.py preflight --owner mapping" in mapping
    assert "ConditionPathExists=/etc/robot-savo/enable-mapping-service" in mapping
    assert "Environment=SAVO_CONTROL_STARTUP_MODE=STOP" in mapping
    assert "RuntimeDirectory=robot-savo" in mapping
    assert "systemctl stop" not in mapping


def test_location_direct_runner_uses_read_only_overlap_preflight() -> None:
    runner = _read("deploy/core/run_location_stack.sh")

    assert "runtime_ownership.py" in runner
    assert "preflight --owner location" in runner
    assert "--ignore-pid" in runner
    assert "run-core-owner" not in runner


def test_renderer_covers_location_and_emits_matching_path_environment(
    tmp_path: Path,
) -> None:
    for deployed_root in ("/home/savo/Savo_Pi", "/opt/robot-savo"):
        output = _render(tmp_path, deployed_root)
        expected = {
            "savo_core.service",
            "savo_edge.service",
            "savo.service",
            "savo_mapping.service",
            "savo-location-stack@.service",
            "savo-ui-runtime.service",
            "savo-ui.service",
        }
        assert expected <= {path.name for path in output.glob("*.service")}
        paths = output.joinpath("robot-savo.paths.env").read_text(encoding="utf-8")
        assert f"SAVO_ROOT={deployed_root}\n" in paths
        assert f"SAVO_WS={deployed_root}/savo_ws\n" in paths

        for unit in output.glob("*.service"):
            text = unit.read_text(encoding="utf-8")
            assert "@SAVO_" not in text
            assert "EnvironmentFile=/etc/robot-savo/robot-savo.paths.env" in text
            assert f"WorkingDirectory={deployed_root}" in text
        location = output.joinpath("savo-location-stack@.service").read_text(encoding="utf-8")
        assert "/home/%i/Savo_Pi" not in location
        assert f"ExecStart={deployed_root}/deploy/core/run_location_stack.sh" in location
        assert "ProtectSystem=strict" in location
        assert "ProtectHome=read-only" in location


def test_environment_example_cannot_override_rendered_root() -> None:
    example = _read("deploy/systemd/robot-savo.env.example")

    assert "SAVO_ROOT=" not in example
    assert "SAVO_WS=" not in example


def test_role_scoped_installer_defaults_to_install_only() -> None:
    installer = _read("deploy/systemd/install_services.sh")

    assert "enable --now" not in installer
    assert "systemctl start" not in installer
    assert "systemctl restart" not in installer
    assert "systemctl stop" not in installer
    assert "daemon-reload" in installer
    assert "core)" in installer
    assert "mapping)" in installer
    assert "edge)" in installer
    assert 'selected=("savo_edge.service" "savo-ui-runtime.service")' in installer


def test_role_scoped_installer_only_installs_selected_owner(tmp_path: Path) -> None:
    expected_profiles = {
        "core": {"savo_core.service"},
        "mapping": {"savo_mapping.service"},
        "edge": {"savo_edge.service", "savo-ui-runtime.service"},
        "generic": {"savo.service"},
        "location": {"savo-location-stack@.service"},
    }
    for profile, expected in expected_profiles.items():
        unit_dir = tmp_path / profile / "units"
        config_dir = tmp_path / profile / "config"
        tmpfiles_dir = tmp_path / profile / "tmpfiles"
        binary_dir = tmp_path / profile / "bin"
        binary_dir.mkdir(parents=True)
        tmpfiles_log = tmp_path / profile / "systemd-tmpfiles.log"
        fake_tmpfiles = binary_dir / "systemd-tmpfiles"
        fake_tmpfiles.write_text(
            "#!/usr/bin/env bash\n"
            "printf '%s\\n' \"$*\" > \"${SAVO_TEST_TMPFILES_LOG}\"\n",
            encoding="utf-8",
        )
        fake_tmpfiles.chmod(0o755)
        environment = os.environ.copy()
        environment["PATH"] = f"{binary_dir}:{environment.get('PATH', '')}"
        environment["SAVO_TEST_TMPFILES_LOG"] = str(tmpfiles_log)
        result = subprocess.run(
            [
                "bash",
                str(SYSTEMD / "install_services.sh"),
                "--profile",
                profile,
                "--user",
                "savo",
                "--group",
                "savo",
                "--source-root",
                str(ROOT),
                "--root",
                str(ROOT),
                "--unit-dir",
                str(unit_dir),
                "--config-dir",
                str(config_dir),
                "--tmpfiles-dir",
                str(tmpfiles_dir),
                "--skip-daemon-reload",
            ],
            env=environment,
            capture_output=True,
            text=True,
            check=False,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        assert {path.name for path in unit_dir.glob("*.service")} == expected
        assert config_dir.joinpath("robot-savo.paths.env").is_file()
        assert "No service was enabled or started" in result.stdout
        policy = tmpfiles_dir / "robot-savo-core.conf"
        if profile in {"core", "mapping", "generic"}:
            assert policy.read_text(encoding="utf-8") == (
                "# Robot SAVO Core lifetime-lock directory.\n"
                "d /run/robot-savo 0750 savo savo -\n"
            )
            assert tmpfiles_log.read_text(encoding="utf-8") == (
                f"--create {policy}\n"
            )
        else:
            assert not policy.exists()
            assert not tmpfiles_log.exists()


def test_edge_systemd_runtime_is_the_only_default_ui_owner() -> None:
    edge = _read("deploy/systemd/savo_edge.service")
    runtime = _read("savo_ws/src/edge/savo_ui/systemd/savo-ui-runtime.service")
    classic = _read("savo_ws/src/edge/savo_ui/systemd/savo-ui.service")

    assert "Wants=network-online.target savo-ui-runtime.service" in edge
    assert "After=network-online.target savo-ui-runtime.service" in edge
    assert "Environment=SAVO_START_UI=false" in edge
    assert "Before=savo_edge.service" in runtime
    assert "Conflicts=savo-ui.service" in runtime
    assert "Conflicts=savo_edge.service" in classic


def test_critical_child_exit_semantics_are_explicit_and_do_not_auto_restart() -> None:
    base_launch = _read("savo_ws/src/core/savo_base/launch/base_bringup.launch.py")
    core_unit = _read("deploy/systemd/savo_core.service")
    operations = _read("docs/deployment/systemd_services.md")

    assert "required=True" not in base_launch
    assert "Restart=on-failure" in core_unit
    assert "parent ros2 launch process may remain active" in operations
    assert "explicit operator recovery" in operations


def test_ownership_conflicts_do_not_enter_automatic_retry_loops() -> None:
    """Ownership refusal requires a fresh operator start, never delayed takeover."""
    core = _read("deploy/systemd/savo_core.service")
    mapping = _read("deploy/systemd/savo_mapping.service")
    generic = _read("deploy/systemd/savo.service")
    location = _read("deploy/systemd/savo-location-stack@.service")

    # The service-level diagnostic gate is a condition: an already-owned robot
    # is skipped rather than failed/retried by Restart=on-failure.
    assert "ExecCondition=/usr/bin/python3" in core
    assert "ExecCondition=/usr/bin/python3" in mapping
    assert "ExecCondition=/usr/bin/python3" in location
    assert "ExecStartPre=/usr/bin/python3" not in core
    assert "ExecStartPre=/usr/bin/python3" not in mapping
    assert "ExecStartPre=/usr/bin/python3" not in location

    # A race can still be caught inside the runner or by flock after the
    # condition. Exit 4 is the authoritative ownership-contention code and
    # must never trigger a later unattended takeover.
    for unit in (core, mapping, generic, location):
        assert "Restart=on-failure" in unit
        assert "RestartPreventExitStatus=4" in unit


def test_direct_deploy_environment_defaults_to_the_checkout_containing_the_script(
    tmp_path: Path,
) -> None:
    """Direct /opt-style runners must not silently source ~/Savo_Pi."""
    environment = os.environ.copy()
    environment.pop("SAVO_ROOT", None)
    environment.pop("SAVO_WS", None)
    environment["HOME"] = str(tmp_path / "unrelated-home")
    result = subprocess.run(
        [
            "bash",
            "-c",
            'source "$1"; printf "%s\\n%s\\n" "$SAVO_ROOT" "$SAVO_WS"',
            "bash",
            str(ROOT / "deploy/common/env_common.sh"),
        ],
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [str(ROOT), str(ROOT / "savo_ws")]


def test_normal_core_runner_refuses_specialized_mapping_modes() -> None:
    """Production Core service must not bypass dedicated mapping gates."""
    runner = _read("deploy/core/run_core.sh")

    assert "manual_mapping" in runner
    assert "run_mapping_service.sh" in runner
    assert "autonomous_mapping" in runner
    assert "run_autonomous_mapping.sh" in runner


def test_manual_mapping_runner_enforces_both_explicit_enable_gates() -> None:
    runner = _read("deploy/core/run_mapping_service.sh")

    assert "SAVO_ENABLE_MAPPING_SERVICE" in runner
    assert "/etc/robot-savo/enable-mapping-service" in runner
    assert '[[ -e "/etc/robot-savo/enable-mapping-service" ]]' in runner
