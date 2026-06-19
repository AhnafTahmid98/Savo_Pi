#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Dump effective Robot Savo mapping parameters."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, Sequence


# =============================================================================
# Local source-tree import support
# =============================================================================
_PACKAGE_ROOT = Path(__file__).resolve().parents[1]

if str(_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_ROOT))


from savo_mapping.ros.params import build_default_param_bundle
from savo_mapping.utils.param_loader import deep_merge_dicts, load_yaml_file


# =============================================================================
# Arguments
# =============================================================================
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Dump effective params for the Robot Savo mapping package.",
    )

    parser.add_argument(
        "--profile",
        action="append",
        default=[],
        help="YAML profile/config file to merge. Can be used multiple times.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print as JSON instead of YAML-style text.",
    )
    parser.add_argument(
        "--section",
        default="",
        help="Print one section only, for example mapping_supervisor.",
    )

    return parser


# =============================================================================
# Helpers
# =============================================================================
def _load_profiles(paths: Sequence[str]) -> Dict[str, Any]:
    merged: Dict[str, Any] = {}

    for path in paths:
        merged = deep_merge_dicts(merged, load_yaml_file(path))

    return merged


def _format_yaml_like(data: Dict[str, Any]) -> str:
    import yaml

    return yaml.safe_dump(
        data,
        default_flow_style=False,
        sort_keys=False,
    )


def _select_section(data: Dict[str, Any], section: str) -> Dict[str, Any]:
    key = str(section).strip()

    if not key:
        return data

    if key not in data:
        raise KeyError(f"Unknown parameter section: {key}")

    return {key: data[key]}


# =============================================================================
# Main
# =============================================================================
def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    defaults = build_default_param_bundle()
    profile_data = _load_profiles(args.profile)

    effective = deep_merge_dicts(defaults, profile_data)
    selected = _select_section(effective, args.section)

    if args.json:
        print(json.dumps(selected, indent=2, sort_keys=True))
    else:
        print(_format_yaml_like(selected), end="")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))