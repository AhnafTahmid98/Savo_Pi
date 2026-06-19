#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Map quality CLI for Robot Savo mapping."""

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


from savo_mapping.diagnostics.map_quality_check import (  # noqa: E402
    evaluate_map_quality,
    evaluate_occupancy_values_quality,
    map_quality_result_to_diagnostic,
)
from savo_mapping.diagnostics.report_formatter import format_key_value_block  # noqa: E402
from savo_mapping.models.map_quality import MapQualityThresholds  # noqa: E402


# =============================================================================
# Arguments
# =============================================================================
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Evaluate Robot Savo occupancy map quality.",
    )

    parser.add_argument("--width", type=int, default=100, help="Map width in cells.")
    parser.add_argument("--height", type=int, default=80, help="Map height in cells.")
    parser.add_argument("--resolution", type=float, default=0.05, help="Map resolution in meters.")

    parser.add_argument("--free-cells", type=int, default=5000)
    parser.add_argument("--occupied-cells", type=int, default=500)
    parser.add_argument("--unknown-cells", type=int, default=2500)

    parser.add_argument("--min-known-ratio", type=float, default=0.15)
    parser.add_argument("--max-unknown-ratio", type=float, default=0.85)
    parser.add_argument("--min-occupied-cells", type=int, default=50)
    parser.add_argument("--min-map-width-m", type=float, default=2.0)
    parser.add_argument("--min-map-height-m", type=float, default=2.0)

    parser.add_argument(
        "--values-file",
        default="",
        help="Optional text/JSON file containing occupancy values.",
    )
    parser.add_argument(
        "--required",
        action="store_true",
        help="Treat failed map quality as required failure.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print full JSON result.",
    )

    return parser


# =============================================================================
# File loading
# =============================================================================
def _load_values_file(path: str | Path) -> list[int]:
    file_path = Path(path).expanduser().resolve()

    if not file_path.exists():
        raise FileNotFoundError(f"Values file not found: {file_path}")

    text = file_path.read_text(encoding="utf-8").strip()

    if not text:
        return []

    if text.startswith("["):
        data = json.loads(text)

        if not isinstance(data, list):
            raise ValueError("JSON values file must contain a list.")

        return [int(value) for value in data]

    values: list[int] = []

    for raw in text.replace(",", " ").split():
        values.append(int(raw))

    return values


# =============================================================================
# Main
# =============================================================================
def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    thresholds = MapQualityThresholds(
        min_known_ratio=args.min_known_ratio,
        max_unknown_ratio=args.max_unknown_ratio,
        min_occupied_cells=args.min_occupied_cells,
        min_map_width_m=args.min_map_width_m,
        min_map_height_m=args.min_map_height_m,
    )

    if args.values_file:
        values = _load_values_file(args.values_file)
        result = evaluate_occupancy_values_quality(
            width_cells=args.width,
            height_cells=args.height,
            resolution_m=args.resolution,
            values=values,
            thresholds=thresholds,
            required=args.required,
        )
    else:
        result = evaluate_map_quality(
            width_cells=args.width,
            height_cells=args.height,
            resolution_m=args.resolution,
            free_cells=args.free_cells,
            occupied_cells=args.occupied_cells,
            unknown_cells=args.unknown_cells,
            thresholds=thresholds,
            required=args.required,
        )

    diagnostic = map_quality_result_to_diagnostic(result)

    if args.json:
        print(result.to_json(indent=2))
    else:
        print(
            format_key_value_block(
                "Robot Savo map quality",
                {
                    "ok": result.ok,
                    "level": diagnostic.level,
                    "message": result.message,
                    "width_m": result.quality.width_m,
                    "height_m": result.quality.height_m,
                    "known_ratio": result.quality.known_ratio,
                    "unknown_ratio": result.quality.unknown_ratio,
                    "occupied_cells": result.quality.occupied_cells,
                    "free_cells": result.quality.free_cells,
                    "unknown_cells": result.quality.unknown_cells,
                },
            )
        )

    return 0 if result.ok else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))