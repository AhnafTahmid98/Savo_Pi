#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Print and validate the static savo_control topic contract."""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any

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
    CMD_VEL_MANUAL,
    CMD_VEL_MUX,
    CMD_VEL_NAV,
    CMD_VEL_RECOVERY,
    CMD_VEL_SAFE,
    CONTROL_MODE_CMD,
    CONTROL_MODE_STATE,
    DEPTH_MIN_FRONT,
    ODOM_FILTERED,
    RECOVERY_ACTIVE,
    RECOVERY_REQUEST,
    SAFETY_STOP,
    get_control_topics_dict,
    outputs_directly_to_base,
)


COMMAND_SOURCES = {
    "manual": CMD_VEL_MANUAL,
    "auto": CMD_VEL_AUTO,
    "nav": CMD_VEL_NAV,
    "recovery": CMD_VEL_RECOVERY,
}


def build_topic_report() -> dict[str, Any]:
    direct_base_outputs = [
        topic for topic in COMMAND_SOURCES.values() if outputs_directly_to_base(topic)
    ]

    chain_ok = (
        CMD_VEL_MANUAL != CMD_VEL_SAFE
        and CMD_VEL_AUTO != CMD_VEL_SAFE
        and CMD_VEL_NAV != CMD_VEL_SAFE
        and CMD_VEL_RECOVERY != CMD_VEL_SAFE
        and CMD_VEL_MUX != CMD_VEL_SAFE
        and CMD_VEL != CMD_VEL_SAFE
        and not direct_base_outputs
    )

    return {
        "package": "savo_control",
        "chain_ok": chain_ok,
        "command_sources": COMMAND_SOURCES,
        "command_chain": {
            "mux_output": CMD_VEL_MUX,
            "shaped_output": CMD_VEL,
            "safety_output": CMD_VEL_SAFE,
            "base_input": CMD_VEL_SAFE,
        },
        "mode_control": {
            "command": CONTROL_MODE_CMD,
            "state": CONTROL_MODE_STATE,
        },
        "recovery": {
            "request": RECOVERY_REQUEST,
            "active": RECOVERY_ACTIVE,
            "command": CMD_VEL_RECOVERY,
        },
        "inputs": {
            "odom": ODOM_FILTERED,
            "front_distance": DEPTH_MIN_FRONT,
            "safety_stop": SAFETY_STOP,
        },
        "direct_base_outputs": direct_base_outputs,
        "all_topics": get_control_topics_dict(),
    }


def build_diagnostic_items(report: dict[str, Any]) -> list[DiagnosticItem]:
    items = [
        DiagnosticItem(
            name="command_chain",
            status=STATUS_OK if report["chain_ok"] else STATUS_ERROR,
            value=(
                f"{CMD_VEL_MUX} -> {CMD_VEL} -> {CMD_VEL_SAFE}"
            ),
        ),
        DiagnosticItem(
            name="manual_source",
            status=STATUS_OK,
            value=CMD_VEL_MANUAL,
        ),
        DiagnosticItem(
            name="auto_source",
            status=STATUS_OK,
            value=CMD_VEL_AUTO,
        ),
        DiagnosticItem(
            name="nav_source",
            status=STATUS_OK,
            value=CMD_VEL_NAV,
        ),
        DiagnosticItem(
            name="recovery_source",
            status=STATUS_OK,
            value=CMD_VEL_RECOVERY,
        ),
        DiagnosticItem(
            name="direct_base_outputs",
            status=STATUS_OK if not report["direct_base_outputs"] else STATUS_ERROR,
            value=report["direct_base_outputs"] or "none",
        ),
    ]

    return items


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate the static savo_control topic contract."
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print machine-readable JSON.",
    )
    parser.add_argument(
        "--fail-on-error",
        action="store_true",
        help="Return non-zero if the topic contract is invalid.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)

    report = build_topic_report()
    items = build_diagnostic_items(report)
    overall = summarize_items(items)

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(format_report("savo_control topic contract", items))

    if args.fail_on_error and overall != STATUS_OK:
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
