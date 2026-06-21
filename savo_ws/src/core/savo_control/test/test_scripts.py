#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for CLI helper scripts."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = ROOT / "scripts"


def run_script(script_name: str, *args: str) -> subprocess.CompletedProcess:
    env = os.environ.copy()
    env["PYTHONPATH"] = f"{ROOT}:{env.get('PYTHONPATH', '')}"

    return subprocess.run(
        [sys.executable, str(SCRIPTS_DIR / script_name), *args],
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def assert_script_compiles(script_name: str) -> None:
    path = SCRIPTS_DIR / script_name

    assert path.is_file()
    assert os.access(path, os.X_OK)

    source = path.read_text(encoding="utf-8")
    compile(source, str(path), "exec")


def test_scripts_directory_exists():
    assert SCRIPTS_DIR.is_dir()


def test_control_topic_check_script_exists_and_compiles():
    assert_script_compiles("control_topic_check_cli.py")


def test_control_topic_check_cli_text_output():
    result = run_script("control_topic_check_cli.py", "--fail-on-error")

    assert result.returncode == 0, result.stderr
    assert "savo_control topic contract: OK" in result.stdout
    assert "command_chain: OK" in result.stdout
    assert "/cmd_vel_mux -> /cmd_vel -> /cmd_vel_safe" in result.stdout
    assert "direct_base_outputs: OK" in result.stdout


def test_control_topic_check_cli_json_output():
    result = run_script("control_topic_check_cli.py", "--json", "--fail-on-error")

    assert result.returncode == 0, result.stderr

    data = json.loads(result.stdout)

    assert data["package"] == "savo_control"
    assert data["chain_ok"] is True
    assert data["command_chain"]["mux_output"] == "/cmd_vel_mux"
    assert data["command_chain"]["shaped_output"] == "/cmd_vel"
    assert data["command_chain"]["safety_output"] == "/cmd_vel_safe"
    assert data["command_chain"]["base_input"] == "/cmd_vel_safe"
    assert data["command_sources"]["manual"] == "/cmd_vel_manual"
    assert data["command_sources"]["auto"] == "/cmd_vel_auto"
    assert data["command_sources"]["nav"] == "/cmd_vel_nav"
    assert data["command_sources"]["recovery"] == "/cmd_vel_recovery"
    assert data["direct_base_outputs"] == []


def test_control_topic_check_cli_help():
    result = run_script("control_topic_check_cli.py", "--help")

    assert result.returncode == 0
    assert "Validate the static savo_control topic contract" in result.stdout
    assert "--json" in result.stdout
    assert "--fail-on-error" in result.stdout


def test_mode_cmd_script_exists_and_compiles():
    assert_script_compiles("mode_cmd_cli.py")


def test_mode_cmd_cli_list_modes():
    result = run_script("mode_cmd_cli.py", "--list")

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip().splitlines() == [
        "STOP",
        "MANUAL",
        "AUTO",
        "NAV",
        "RECOVERY",
    ]


def test_mode_cmd_cli_dry_run_text_output():
    result = run_script("mode_cmd_cli.py", "MANUAL", "--dry-run")

    assert result.returncode == 0, result.stderr
    assert "savo_control mode command validated" in result.stdout
    assert "mode=MANUAL" in result.stdout
    assert "topic=/savo_control/mode_cmd" in result.stdout
    assert "repeat=3" in result.stdout


def test_mode_cmd_cli_dry_run_json_output():
    result = run_script("mode_cmd_cli.py", "AUTO", "--dry-run", "--json")

    assert result.returncode == 0, result.stderr

    data = json.loads(result.stdout)

    assert data["package"] == "savo_control"
    assert data["topic"] == "/savo_control/mode_cmd"
    assert data["mode"] == "AUTO"
    assert data["repeat"] == 3
    assert data["dry_run"] is True


def test_mode_cmd_cli_accepts_lowercase_mode_in_dry_run():
    result = run_script("mode_cmd_cli.py", "recovery", "--dry-run", "--repeat", "2")

    assert result.returncode == 0, result.stderr
    assert "mode=RECOVERY" in result.stdout
    assert "repeat=2" in result.stdout


def test_mode_cmd_cli_requires_mode_unless_listing():
    result = run_script("mode_cmd_cli.py", "--dry-run")

    assert result.returncode == 2
    assert "mode is required" in result.stderr


def test_mode_cmd_cli_help():
    result = run_script("mode_cmd_cli.py", "--help")

    assert result.returncode == 0
    assert "Publish a mode command" in result.stdout
    assert "--dry-run" in result.stdout
    assert "--list" in result.stdout


def test_control_smoke_script_exists_and_compiles():
    assert_script_compiles("control_smoke_test_cli.py")


def test_control_smoke_cli_text_output():
    result = run_script("control_smoke_test_cli.py", "--fail-on-error")

    assert result.returncode == 0, result.stderr
    assert "savo_control smoke test: OK" in result.stdout
    assert "required_files: OK" in result.stdout
    assert "config_files: OK" in result.stdout
    assert "command_chain: OK" in result.stdout
    assert "distance_approach_hybrid: OK" in result.stdout
    assert "cmake_contract: OK" in result.stdout
    assert "launch_contract: OK" in result.stdout


def test_control_smoke_cli_json_output():
    result = run_script("control_smoke_test_cli.py", "--json", "--fail-on-error")

    assert result.returncode == 0, result.stderr

    data = json.loads(result.stdout)

    assert data["package"] == "savo_control"
    assert data["version"] == "0.1.0"
    assert data["overall"] == "OK"
    assert "items" in data
    assert "topics" in data
    assert data["topics"]["cmd_vel"] == "/cmd_vel"
    assert data["topics"]["cmd_vel_safe"] == "/cmd_vel_safe"


def test_control_smoke_cli_help():
    result = run_script("control_smoke_test_cli.py", "--help")

    assert result.returncode == 0
    assert "Run source-level smoke checks for savo_control" in result.stdout
    assert "--json" in result.stdout
    assert "--fail-on-error" in result.stdout
