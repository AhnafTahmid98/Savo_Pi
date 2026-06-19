#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Map metadata CLI for Robot Savo mapping."""

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


from savo_mapping.models.map_metadata import (  # noqa: E402
    MapOrigin,
    make_map_metadata,
    make_saved_map_paths,
    map_metadata_from_dict,
    sanitize_map_name,
)


# =============================================================================
# Arguments
# =============================================================================
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Create or inspect Robot Savo saved-map metadata.",
    )

    parser.add_argument(
        "--name",
        default="Savonia Campus Heart",
        help="Human map name. It will be sanitized for file names.",
    )
    parser.add_argument("--width", type=int, default=400, help="Map width in cells.")
    parser.add_argument("--height", type=int, default=250, help="Map height in cells.")
    parser.add_argument("--resolution", type=float, default=0.05, help="Map resolution in meters.")

    parser.add_argument("--origin-x", type=float, default=0.0)
    parser.add_argument("--origin-y", type=float, default=0.0)
    parser.add_argument("--origin-yaw", type=float, default=0.0)

    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--maps-dir", default="maps/saved")
    parser.add_argument("--image-ext", default=".pgm")
    parser.add_argument("--session-id", default="")

    parser.add_argument(
        "--from-json",
        default="",
        help="Load metadata from a JSON file instead of CLI values.",
    )
    parser.add_argument(
        "--write-json",
        default="",
        help="Write metadata JSON to this path.",
    )
    parser.add_argument(
        "--paths-only",
        action="store_true",
        help="Only print expected saved-map file paths.",
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
def _load_json(path: str | Path) -> dict:
    file_path = Path(path).expanduser().resolve()

    if not file_path.exists():
        raise FileNotFoundError(f"JSON file not found: {file_path}")

    data = json.loads(file_path.read_text(encoding="utf-8"))

    if not isinstance(data, dict):
        raise ValueError("Metadata JSON root must be a dictionary.")

    return data


def _write_json(path: str | Path, data: dict) -> Path:
    file_path = Path(path).expanduser().resolve()
    file_path.parent.mkdir(parents=True, exist_ok=True)
    file_path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return file_path


def _format_key_values(title: str, values: dict) -> str:
    lines = [title]

    for key in sorted(values.keys()):
        lines.append(f"- {key}: {values[key]}")

    return "\n".join(lines)


# =============================================================================
# Main
# =============================================================================
def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.from_json:
        metadata = map_metadata_from_dict(_load_json(args.from_json))
    else:
        clean_name = sanitize_map_name(args.name)
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
            },
        )

    if args.paths_only:
        paths = make_saved_map_paths(
            metadata.name,
            maps_dir=args.maps_dir,
            image_ext=args.image_ext,
        )
        print(_format_key_values("Robot Savo map paths", paths))
        return 0

    data = metadata.to_dict()

    if args.write_json:
        output_path = _write_json(args.write_json, data)
        print(f"wrote={output_path}")

    if args.json:
        print(metadata.to_json(indent=2))
    else:
        print(
            _format_key_values(
                "Robot Savo map metadata",
                {
                    "name": metadata.name,
                    "valid": metadata.valid,
                    "frame_id": metadata.frame_id,
                    "width_cells": metadata.width_cells,
                    "height_cells": metadata.height_cells,
                    "resolution_m": metadata.resolution_m,
                    "width_m": f"{metadata.width_m:.3f}",
                    "height_m": f"{metadata.height_m:.3f}",
                    "image_file": metadata.image_file,
                    "yaml_file": metadata.yaml_file,
                    "session_id": metadata.session_id,
                },
            )
        )

    return 0 if metadata.valid else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))