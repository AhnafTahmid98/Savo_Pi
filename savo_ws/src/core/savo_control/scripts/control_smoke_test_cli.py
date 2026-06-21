#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Run source-level smoke checks for the savo_control package."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import yaml

from savo_control import VERSION
from savo_control.diagnostics import (
    DiagnosticItem,
    STATUS_ERROR,
    STATUS_OK,
    format_report,
    summarize_items,
)
from savo_control.ros import (
    CMD_VEL,
    CMD_VEL_AUTO,
    CMD_VEL_MUX,
    CMD_VEL_SAFE,
    SAFETY_STOP,
    get_control_topics_dict,
)


ROOT = Path(__file__).resolve().parents[1]


REQUIRED_FILES = [
    "package.xml",
    "setup.py",
    "CMakeLists.txt",
    "savo_control/version.py",
    "savo_control/constants.py",
    "savo_control/models/control_mode.py",
    "savo_control/ros/topic_names.py",
    "savo_control/diagnostics/report_formatter.py",
    "savo_control/utils/time_utils.py",
    "src/nodes/distance_approach_node.cpp",
    "launch/control_bringup.launch.py",
    "launch/distance_approach.launch.py",
    "scripts/control_topic_check_cli.py",
    "scripts/mode_cmd_cli.py",
]


REQUIRED_CONFIGS = [
    "control_common.yaml",
    "cmd_vel_shaper.yaml",
    "twist_mux.yaml",
    "control_mode_manager.yaml",
    "heading_pid.yaml",
    "rotate_to_heading.yaml",
    "recovery.yaml",
    "stuck_detector.yaml",
    "distance_approach.yaml",
    "auto_test_modes.yaml",
    "topic_remaps_example.yaml",
]


def check_required_files() -> DiagnosticItem:
    missing = [name for name in REQUIRED_FILES if not (ROOT / name).is_file()]

    return DiagnosticItem(
        name="required_files",
        status=STATUS_OK if not missing else STATUS_ERROR,
        value="ok" if not missing else missing,
    )


def check_configs_parse() -> DiagnosticItem:
    missing: list[str] = []
    invalid: list[str] = []

    for name in REQUIRED_CONFIGS:
        path = ROOT / "config" / name

        if not path.is_file():
            missing.append(name)
            continue

        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8"))
        except Exception as exc:
            invalid.append(f"{name}: {exc}")
            continue

        if not isinstance(data, dict):
            invalid.append(f"{name}: not a YAML dictionary")

    problems = missing + invalid

    return DiagnosticItem(
        name="config_files",
        status=STATUS_OK if not problems else STATUS_ERROR,
        value="ok" if not problems else problems,
    )


def check_command_chain() -> DiagnosticItem:
    ok = (
        CMD_VEL_AUTO != CMD_VEL_SAFE
        and CMD_VEL_MUX != CMD_VEL_SAFE
        and CMD_VEL != CMD_VEL_SAFE
        and SAFETY_STOP == "/safety/stop"
    )

    return DiagnosticItem(
        name="command_chain",
        status=STATUS_OK if ok else STATUS_ERROR,
        value=f"{CMD_VEL_MUX} -> {CMD_VEL} -> {CMD_VEL_SAFE}",
    )


def check_distance_approach_hybrid() -> DiagnosticItem:
    path = ROOT / "config" / "distance_approach.yaml"

    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return DiagnosticItem("distance_approach_hybrid", STATUS_ERROR, note=str(exc))

    ok = (
        "distance_approach_node" in data
        and "distance_pid_test_node" in data
        and (ROOT / "src/nodes/distance_approach_node.cpp").is_file()
        and (ROOT / "savo_control/nodes/distance_pid_test_node.py").is_file()
    )

    return DiagnosticItem(
        name="distance_approach_hybrid",
        status=STATUS_OK if ok else STATUS_ERROR,
        value="cpp+py" if ok else "missing hybrid section or node",
    )


def check_cmake_contract() -> DiagnosticItem:
    text = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    required = [
        "savo_add_node(distance_approach_node",
        "src/nodes/distance_approach_node.cpp",
        "install(PROGRAMS",
        "distance_pid_test_node.py",
        "add_pytest_if_exists(test_config_files",
        "add_pytest_if_exists(test_scripts",
    ]

    missing = [item for item in required if item not in text]

    return DiagnosticItem(
        name="cmake_contract",
        status=STATUS_OK if not missing else STATUS_ERROR,
        value="ok" if not missing else missing,
    )


def check_launch_contract() -> DiagnosticItem:
    control = (ROOT / "launch/control_bringup.launch.py").read_text(encoding="utf-8")
    distance = (ROOT / "launch/distance_approach.launch.py").read_text(encoding="utf-8")

    required = [
        "control_mode_manager.yaml",
        "distance_approach.yaml",
        "distance_approach_node",
        "distance_pid_test_node.py",
        "approach_impl",
    ]

    missing = [
        item
        for item in required
        if item not in control and item not in distance
    ]

    return DiagnosticItem(
        name="launch_contract",
        status=STATUS_OK if not missing else STATUS_ERROR,
        value="ok" if not missing else missing,
    )


def build_report() -> dict[str, Any]:
    items = [
        check_required_files(),
        check_configs_parse(),
        check_command_chain(),
        check_distance_approach_hybrid(),
        check_cmake_contract(),
        check_launch_contract(),
    ]

    return {
        "package": "savo_control",
        "version": VERSION,
        "root": str(ROOT),
        "overall": summarize_items(items),
        "items": [item.to_line() for item in items],
        "topics": get_control_topics_dict(),
    }


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run source-level smoke checks for savo_control."
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print machine-readable JSON.",
    )
    parser.add_argument(
        "--fail-on-error",
        action="store_true",
        help="Return non-zero if any smoke check fails.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    report = build_report()

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        items = [
            DiagnosticItem("required_files", STATUS_OK if "required_files: OK" in report["items"][0] else STATUS_ERROR),
            DiagnosticItem("config_files", STATUS_OK if "config_files: OK" in report["items"][1] else STATUS_ERROR),
            DiagnosticItem("command_chain", STATUS_OK if "command_chain: OK" in report["items"][2] else STATUS_ERROR),
            DiagnosticItem("distance_approach_hybrid", STATUS_OK if "distance_approach_hybrid: OK" in report["items"][3] else STATUS_ERROR),
            DiagnosticItem("cmake_contract", STATUS_OK if "cmake_contract: OK" in report["items"][4] else STATUS_ERROR),
            DiagnosticItem("launch_contract", STATUS_OK if "launch_contract: OK" in report["items"][5] else STATUS_ERROR),
        ]
        print(format_report("savo_control smoke test", items))

    if args.fail_on_error and report["overall"] != STATUS_OK:
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
