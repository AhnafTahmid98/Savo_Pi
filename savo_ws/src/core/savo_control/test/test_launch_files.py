#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Source-level tests for launch-file contracts."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = ROOT / "launch"


def read_launch(name: str) -> str:
    return (LAUNCH_DIR / name).read_text(encoding="utf-8")


def test_launch_directory_exists():
    assert LAUNCH_DIR.is_dir()


def test_python_launch_files_compile():
    launch_files = sorted(LAUNCH_DIR.glob("*.launch.py"))

    assert launch_files, "No launch files found."

    for path in launch_files:
        source = path.read_text(encoding="utf-8")
        compile(source, str(path), "exec")


def test_control_bringup_loads_core_control_chain():
    text = read_launch("control_bringup.launch.py")

    assert "control_common.yaml" in text
    assert "control_mode_manager.yaml" in text
    assert "twist_mux.yaml" in text
    assert "cmd_vel_shaper.yaml" in text
    assert "recovery.yaml" in text

    assert "control_mode_manager_node" in text
    assert "twist_mux_node" in text
    assert "cmd_vel_shaper_node" in text
    assert "recovery_manager_node" in text
    assert "backup_escape_node" in text


def test_control_bringup_keeps_distance_approach_optional():
    text = read_launch("control_bringup.launch.py")

    assert "use_distance_approach" in text
    assert 'default_value="false"' in text
    assert "approach_impl" in text
    assert "distance_approach_node" in text
    assert "distance_pid_test_node.py" in text


def test_distance_approach_launch_supports_hybrid_impls():
    text = read_launch("distance_approach.launch.py")

    assert "approach_impl" in text
    assert 'default_value="cpp"' in text
    assert "distance_approach_node" in text
    assert "distance_pid_test_node.py" in text

    assert "control_mode_manager_node" in text
    assert "twist_mux_node" in text
    assert "cmd_vel_shaper_node" in text


def test_distance_approach_launch_uses_typed_runtime_params():
    text = read_launch("distance_approach.launch.py")

    assert "ParameterValue(auto_start, value_type=bool)" in text
    assert "ParameterValue(" in text
    assert "target_distance_m" in text
    assert "value_type=float" in text


def test_main_bringup_uses_typed_distance_params():
    text = read_launch("control_bringup.launch.py")

    assert "ParameterValue(" in text
    assert "distance_auto_start" in text
    assert "target_distance_m" in text
    assert "value_type=bool" in text
    assert "value_type=float" in text


def test_launch_files_do_not_send_commands_directly_to_base():
    for path in sorted(LAUNCH_DIR.glob("*.launch.py")):
        text = path.read_text(encoding="utf-8")

        assert '"/cmd_vel_safe"' not in text
        assert "'/cmd_vel_safe'" not in text


def test_dashboard_is_not_forced_on_by_default():
    for name in [
        "control_bringup.launch.py",
        "distance_approach.launch.py",
    ]:
        text = read_launch(name)
        assert "use_dashboard" in text
        assert 'default_value="false"' in text
