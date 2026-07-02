# -*- coding: utf-8 -*-

"""Dump effective Robot Savo head configuration."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Optional

import yaml

from savo_head import VERSION
from savo_head.core.validation import validate_config_files
from savo_head.contracts.parameter_names import is_known_parameter


CONFIG_FILES = (
    "head_hardware.yaml",
    "scan_profiles.yaml",
    "head_topics.yaml",
    "head_frames.yaml",
    "camera_stream.yaml",
    "apriltag_semantics.yaml",
    "diagnostics.yaml",
)


def find_package_root(start: str | Path = ".") -> Path:
    current = Path(start).resolve()

    if current.is_file():
        current = current.parent

    for candidate in (current, *current.parents):
        if (candidate / "package.xml").exists() and (candidate / "config").is_dir():
            return candidate

    raise FileNotFoundError("Could not find savo_head package root from current path.")


def load_yaml_params(path: Path, node_name: str = "savo_head") -> dict[str, Any]:
    data = yaml.safe_load(path.read_text()) or {}
    return dict(data.get(node_name, {}).get("ros__parameters", {}) or {})


def load_all_params(package_root: Path) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    config_dir = package_root / "config"

    effective: dict[str, Any] = {}
    by_file: dict[str, dict[str, Any]] = {}

    for filename in CONFIG_FILES:
        path = config_dir / filename
        params = load_yaml_params(path)
        by_file[filename] = params
        effective.update(params)

    return effective, by_file


def unknown_parameter_names(effective: dict[str, Any]) -> list[str]:
    return sorted(key for key in effective if not is_known_parameter(key))


def build_payload(package_root: Path) -> dict[str, Any]:
    effective, by_file = load_all_params(package_root)
    validation = validate_config_files(package_root)
    unknown = unknown_parameter_names(effective)

    return {
        "package": {
            "name": "savo_head",
            "version": VERSION,
            "root": str(package_root),
        },
        "validation": validation.to_dict(),
        "unknown_parameters": unknown,
        "files": by_file,
        "effective": effective,
        "summary": {
            "backend": effective.get("backend"),
            "hardware_profile": effective.get("hardware_profile"),
            "camera_backend": effective.get("camera_backend"),
            "pan_logical_channel": effective.get("pan_logical_channel"),
            "tilt_logical_channel": effective.get("tilt_logical_channel"),
            "pan_pca9685_channel": effective.get("pan_pca9685_channel"),
            "tilt_pca9685_channel": effective.get("tilt_pca9685_channel"),
            "pan_center_deg": effective.get("pan_center_deg"),
            "tilt_center_deg": effective.get("tilt_center_deg"),
            "tilt_max_deg": effective.get("tilt_max_deg"),
            "semantic_scan_pan_targets_deg": effective.get("semantic_scan_pan_targets_deg"),
            "semantic_scan_tilt_sweep_pan_targets_deg": effective.get(
                "semantic_scan_tilt_sweep_pan_targets_deg"
            ),
            "gst_source": effective.get("gst_source"),
            "gst_encoder": effective.get("gst_encoder"),
            "gst_payloader": effective.get("gst_payloader"),
            "gst_sink": effective.get("gst_sink"),
        },
    }


def print_text(payload: dict[str, Any], show_files: bool = False) -> None:
    summary = payload["summary"]
    validation = payload["validation"]

    print("\nRobot Savo — Effective Head Parameters")
    print("--------------------------------------")
    print(f"Package       : {payload['package']['name']} {payload['package']['version']}")
    print(f"Root          : {payload['package']['root']}")
    print(f"Validation    : {'OK' if validation['ok'] else 'FAILED'}")
    print(f"Errors        : {validation['error_count']}")
    print(f"Warnings      : {validation['warning_count']}")

    print("\nHardware")
    print(f"  backend          : {summary['backend']}")
    print(f"  hardware_profile : {summary['hardware_profile']}")
    print(f"  pan              : logical {summary['pan_logical_channel']} / PCA9685 {summary['pan_pca9685_channel']}")
    print(f"  tilt             : logical {summary['tilt_logical_channel']} / PCA9685 {summary['tilt_pca9685_channel']}")
    print(f"  center           : pan={summary['pan_center_deg']}°, tilt={summary['tilt_center_deg']}°")
    print(f"  tilt max         : {summary['tilt_max_deg']}°")

    print("\nScan")
    print(f"  pan targets      : {summary['semantic_scan_pan_targets_deg']}")
    print(f"  tilt sweep at    : {summary['semantic_scan_tilt_sweep_pan_targets_deg']}")

    print("\nCamera")
    print(f"  backend          : {summary['camera_backend']}")
    print(f"  source           : {summary['gst_source']}")
    print(f"  encoder          : {summary['gst_encoder']}")
    print(f"  payloader        : {summary['gst_payloader']}")
    print(f"  sink             : {summary['gst_sink']}")

    if payload["unknown_parameters"]:
        print("\nUnknown parameters")
        for item in payload["unknown_parameters"]:
            print(f"  - {item}")

    if validation["issues"]:
        print("\nValidation issues")
        for issue in validation["issues"]:
            print(f"  [{issue['severity']}] {issue['source']} {issue['key']}: {issue['message']}")

    if show_files:
        print("\nConfig files")
        for filename, params in payload["files"].items():
            print(f"  {filename}: {len(params)} params")


def print_yaml(payload: dict[str, Any]) -> None:
    print(yaml.safe_dump(payload, sort_keys=True))


def print_json(payload: dict[str, Any]) -> None:
    print(json.dumps(payload, indent=2, sort_keys=True))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Dump and validate effective Robot Savo head parameters."
    )

    parser.add_argument(
        "--package-root",
        default=".",
        help="Path inside the savo_head package. Default: current directory.",
    )
    parser.add_argument(
        "--format",
        choices=["text", "json", "yaml"],
        default="text",
    )
    parser.add_argument(
        "--show-files",
        action="store_true",
        help="Show per-file parameter counts in text mode.",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return non-zero when validation fails or unknown parameters exist.",
    )

    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    package_root = find_package_root(args.package_root)
    payload = build_payload(package_root)

    if args.format == "json":
        print_json(payload)
    elif args.format == "yaml":
        print_yaml(payload)
    else:
        print_text(payload, show_files=args.show_files)

    failed = not payload["validation"]["ok"] or bool(payload["unknown_parameters"])
    return 2 if args.strict and failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
