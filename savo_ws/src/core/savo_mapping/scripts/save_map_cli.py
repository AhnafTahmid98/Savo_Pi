#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Saved-map metadata helper for Robot Savo mapping."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Sequence


# =============================================================================
# Local source-tree import support
# =============================================================================
_PACKAGE_ROOT = Path(__file__).resolve().parents[1]

if str(_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_ROOT))


from savo_mapping.diagnostics.map_file_check import evaluate_map_files  # noqa: E402
from savo_mapping.diagnostics.report_formatter import format_key_value_block  # noqa: E402
from savo_mapping.models.map_metadata import (  # noqa: E402
    MapOrigin,
    make_map_metadata,
    make_saved_map_paths,
    sanitize_map_name,
)


# =============================================================================
# Arguments
# =============================================================================
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Prepare Robot Savo saved-map paths and metadata.",
    )

    parser.add_argument(
        "--map-name",
        default="Savonia Campus Heart",
        help="Human map name. It will be sanitized for file paths.",
    )
    parser.add_argument(
        "--maps-dir",
        default="maps/saved",
        help="Directory where the map should be saved.",
    )
    parser.add_argument("--image-ext", default=".pgm")

    parser.add_argument("--width", type=int, default=0)
    parser.add_argument("--height", type=int, default=0)
    parser.add_argument("--resolution", type=float, default=0.0)

    parser.add_argument("--origin-x", type=float, default=0.0)
    parser.add_argument("--origin-y", type=float, default=0.0)
    parser.add_argument("--origin-yaw", type=float, default=0.0)

    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--session-id", default="")

    parser.add_argument(
        "--write-metadata",
        action="store_true",
        help="Write .metadata.json next to expected map files.",
    )
    parser.add_argument(
        "--check-existing",
        action="store_true",
        help="Check whether map files already exist.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print full JSON metadata.",
    )

    return parser


# =============================================================================
# Helpers
# =============================================================================
def _write_json(path: str | Path, data: dict) -> Path:
    file_path = Path(path).expanduser().resolve()
    file_path.parent.mkdir(parents=True, exist_ok=True)
    file_path.write_text(
        json.dumps(data, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return file_path


# =============================================================================
# Main
# =============================================================================
def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    clean_name = sanitize_map_name(args.map_name)

    paths = make_saved_map_paths(
        clean_name,
        maps_dir=args.maps_dir,
        image_ext=args.image_ext,
    )

    metadata = make_map_metadata(
        name=clean_name,
        width_cells=args.width,
        height_cells=args.height,
        resolution_m=args.resolution,
        frame_id=args.frame_id,
        origin=MapOrigin(
            x=args.origin_x,
            y=args.origin_y,
            yaw=args.origin_yaw,
        ),
        image_file=paths["image_file"],
        yaml_file=paths["yaml_file"],
        session_id=args.session_id or None,
        extra={
            "metadata_file": paths["metadata_file"],
            "save_command_note": (
                "Use slam_toolbox/map_saver later to create the .yaml and image files."
            ),
        },
    )

    metadata_written = ""

    if args.write_metadata:
        metadata_written = str(
            _write_json(
                paths["metadata_file"],
                metadata.to_dict(),
            )
        )

    existing_result = None

    if args.check_existing:
        existing_result = evaluate_map_files(
            map_name=clean_name,
            maps_dir=args.maps_dir,
            image_ext=args.image_ext,
            require_metadata=args.write_metadata,
        )

    if args.json:
        output = metadata.to_dict()
        output["paths"] = paths
        output["metadata_written"] = metadata_written or None
        output["existing_check"] = (
            existing_result.to_dict()
            if existing_result is not None
            else None
        )
        print(json.dumps(output, indent=2, sort_keys=True))
    else:
        values = {
            "map_name": metadata.name,
            "valid_metadata": metadata.valid,
            "yaml_file": paths["yaml_file"],
            "image_file": paths["image_file"],
            "metadata_file": paths["metadata_file"],
            "metadata_written": metadata_written or "no",
            "width_cells": metadata.width_cells,
            "height_cells": metadata.height_cells,
            "resolution_m": metadata.resolution_m,
            "width_m": f"{metadata.width_m:.3f}",
            "height_m": f"{metadata.height_m:.3f}",
        }

        if existing_result is not None:
            values.update(
                {
                    "existing_map_ok": existing_result.ok,
                    "existing_map_message": existing_result.message,
                }
            )

        print(format_key_value_block("Robot Savo save-map preparation", values))

    return 0 if metadata.name else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))