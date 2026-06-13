#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Dump effective Robot Savo localization parameters after YAML overlays."""

from __future__ import annotations

import argparse
import json
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


DEFAULT_CONFIGS = (
    "config/topics.yaml",
    "config/frames.yaml",
    "config/imu.yaml",
    "config/encoders.yaml",
    "config/wheel_odom.yaml",
    "config/ekf_odom.yaml",
    "config/diagnostics.yaml",
    "config/localization_dashboard.yaml",
)

DEFAULT_PROFILE = "config/profiles/robot_savo_4enc_imu_ekf.yaml"
VO_OVERLAY = "config/vo_fusion_optional.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Dump effective parameters for the savo_localization package."
    )
    parser.add_argument(
        "--package-share",
        default="",
        help="Override package share/source path. Default: auto-detect.",
    )
    parser.add_argument(
        "--config",
        action="append",
        default=[],
        help="Extra YAML config to load after the defaults. Can be used multiple times.",
    )
    parser.add_argument(
        "--profile",
        default=DEFAULT_PROFILE,
        help="Profile YAML loaded after base configs.",
    )
    parser.add_argument(
        "--no-defaults",
        action="store_true",
        help="Only load files passed with --config and --profile.",
    )
    parser.add_argument(
        "--use-vo",
        action="store_true",
        help="Load the optional /vo/odom EKF overlay before the profile.",
    )
    parser.add_argument(
        "--node",
        default="",
        help="Print only one node, for example wheel_odom_node.",
    )
    parser.add_argument(
        "--list-nodes",
        action="store_true",
        help="List node names found in the merged parameter set.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print JSON instead of YAML.",
    )
    parser.add_argument(
        "--compact",
        action="store_true",
        help="Compact JSON output. Only applies with --json.",
    )
    parser.add_argument(
        "--show-files",
        action="store_true",
        help="Print the loaded files before the parameter dump.",
    )
    args = parser.parse_args()

    package_share = resolve_package_share(args.package_share)

    config_paths: list[str] = []

    if not args.no_defaults:
        config_paths.extend(DEFAULT_CONFIGS)

    if args.use_vo:
        config_paths.append(VO_OVERLAY)

    if args.profile:
        config_paths.append(args.profile)

    config_paths.extend(args.config)

    loaded_files: list[Path] = []
    merged: dict[str, Any] = {}

    try:
        for config_path in config_paths:
            resolved = resolve_config_path(config_path, package_share)
            payload = load_yaml_file(resolved)
            merged = merge_ros_param_trees(merged, payload)
            loaded_files.append(resolved)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if args.list_nodes:
        for node_name in sorted(merged.keys()):
            print(node_name)
        return 0

    output: dict[str, Any]

    if args.node:
        if args.node not in merged:
            print(f"ERROR: node not found: {args.node}", file=sys.stderr)
            print("Available nodes:", file=sys.stderr)
            for node_name in sorted(merged.keys()):
                print(f"  - {node_name}", file=sys.stderr)
            return 1

        output = {args.node: merged[args.node]}
    else:
        output = merged

    if args.show_files:
        print("Loaded parameter files:")
        for path in loaded_files:
            print(f"  - {path}")
        print("")

    if args.json:
        print_json(output, compact=args.compact)
    else:
        print_yaml(output)

    return 0


def resolve_package_share(override: str) -> Path:
    if override.strip():
        path = Path(override).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(f"package share path does not exist: {path}")
        return path

    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("savo_localization")).resolve()
    except Exception:
        pass

    script_path = Path(__file__).resolve()

    for parent in script_path.parents:
        if (parent / "package.xml").exists() and parent.name == "savo_localization":
            return parent

    raise FileNotFoundError(
        "Could not auto-detect savo_localization package path. "
        "Pass --package-share /path/to/savo_localization."
    )


def resolve_config_path(config_path: str, package_share: Path) -> Path:
    path = Path(config_path).expanduser()

    if not path.is_absolute():
        path = package_share / path

    path = path.resolve()

    if not path.exists():
        raise FileNotFoundError(f"config file does not exist: {path}")

    if not path.is_file():
        raise FileNotFoundError(f"config path is not a file: {path}")

    return path


def load_yaml_file(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    if data is None:
        return {}

    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")

    return data


def merge_ros_param_trees(
    base: dict[str, Any],
    overlay: dict[str, Any],
) -> dict[str, Any]:
    result = deepcopy(base)

    for node_name, node_payload in overlay.items():
        if node_payload is None:
            continue

        if not isinstance(node_payload, dict):
            result[node_name] = deepcopy(node_payload)
            continue

        current = result.setdefault(node_name, {})

        if not isinstance(current, dict):
            result[node_name] = deepcopy(node_payload)
            continue

        current_params = _ros_params(current)
        overlay_params = _ros_params(node_payload)

        if current_params is not None or overlay_params is not None:
            current.setdefault("ros__parameters", {})
            deep_merge_dicts(
                current["ros__parameters"],
                overlay_params or {},
            )

            for key, value in node_payload.items():
                if key == "ros__parameters":
                    continue
                current[key] = deepcopy(value)

            continue

        deep_merge_dicts(current, node_payload)

    return result


def _ros_params(node_payload: dict[str, Any]) -> dict[str, Any] | None:
    params = node_payload.get("ros__parameters")

    if params is None:
        return None

    if not isinstance(params, dict):
        raise ValueError("ros__parameters must be a mapping")

    return params


def deep_merge_dicts(
    base: dict[str, Any],
    overlay: dict[str, Any],
) -> dict[str, Any]:
    for key, value in overlay.items():
        if (
            key in base
            and isinstance(base[key], dict)
            and isinstance(value, dict)
        ):
            deep_merge_dicts(base[key], value)
        else:
            base[key] = deepcopy(value)

    return base


def print_json(payload: dict[str, Any], *, compact: bool) -> None:
    if compact:
        print(json.dumps(payload, separators=(",", ":"), sort_keys=True))
        return

    print(json.dumps(payload, indent=2, sort_keys=True))


def print_yaml(payload: dict[str, Any]) -> None:
    print(
        yaml.safe_dump(
            payload,
            default_flow_style=False,
            sort_keys=False,
            allow_unicode=True,
        ).rstrip()
    )


if __name__ == "__main__":
    sys.exit(main())
