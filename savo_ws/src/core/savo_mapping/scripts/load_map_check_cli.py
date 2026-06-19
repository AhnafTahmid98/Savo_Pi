#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Saved-map load check CLI for Robot Savo mapping."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence


# =============================================================================
# Local source-tree import support
# =============================================================================
_PACKAGE_ROOT = Path(__file__).resolve().parents[1]

if str(_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_ROOT))


from savo_mapping.diagnostics.map_file_check import (  # noqa: E402
    evaluate_map_files,
    evaluate_map_files_from_paths,
    map_file_result_to_diagnostic,
)
from savo_mapping.diagnostics.report_formatter import format_key_value_block  # noqa: E402


# =============================================================================
# Arguments
# =============================================================================
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Check whether a saved Robot Savo map can be loaded safely.",
    )

    parser.add_argument(
        "--map-name",
        default="Savonia Campus Heart",
        help="Map name to check inside maps/saved.",
    )
    parser.add_argument(
        "--maps-dir",
        default="maps/saved",
        help="Directory containing saved map files.",
    )
    parser.add_argument(
        "--yaml-file",
        default="",
        help="Direct map YAML path. Overrides --map-name.",
    )
    parser.add_argument(
        "--image-file",
        default="",
        help="Optional direct map image path.",
    )
    parser.add_argument(
        "--metadata-file",
        default="",
        help="Optional direct metadata JSON path.",
    )
    parser.add_argument(
        "--require-metadata",
        action="store_true",
        help="Require .metadata.json file to exist.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print full JSON result.",
    )

    return parser


# =============================================================================
# Main
# =============================================================================
def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.yaml_file:
        result = evaluate_map_files_from_paths(
            yaml_file=args.yaml_file,
            image_file=args.image_file or None,
            metadata_file=args.metadata_file or None,
            require_metadata=args.require_metadata,
        )
    else:
        result = evaluate_map_files(
            map_name=args.map_name,
            maps_dir=args.maps_dir,
            require_metadata=args.require_metadata,
        )

    diagnostic = map_file_result_to_diagnostic(
        result,
        required=True,
    )

    if args.json:
        print(result.to_json(indent=2))
    else:
        print(
            format_key_value_block(
                "Robot Savo saved-map load check",
                {
                    "ok": result.ok,
                    "level": diagnostic.level,
                    "message": result.message,
                    "map_name": result.map_name,
                    "yaml_exists": result.yaml_exists,
                    "image_exists": result.image_exists,
                    "metadata_exists": result.metadata_exists,
                    "resolution_m": result.resolution_m,
                    "image_ref": result.image_ref,
                    "yaml_file": result.yaml_file,
                    "image_file": result.image_file,
                    "metadata_file": result.metadata_file,
                },
            )
        )

    return 0 if result.ok else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))