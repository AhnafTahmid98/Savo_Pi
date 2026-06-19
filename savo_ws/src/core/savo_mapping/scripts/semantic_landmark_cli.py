#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Semantic landmark CLI for Robot Savo mapping."""

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


from savo_mapping.diagnostics.report_formatter import format_key_value_block  # noqa: E402
from savo_mapping.models.semantic_landmark import (  # noqa: E402
    LandmarkState,
    make_landmark_key,
    make_semantic_landmark,
    semantic_landmark_from_dict,
)


# =============================================================================
# Arguments
# =============================================================================
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Create or inspect Robot Savo semantic landmark records.",
    )

    parser.add_argument("--label", default="Info Desk", help="Human-readable location name.")
    parser.add_argument("--x", type=float, default=2.0)
    parser.add_argument("--y", type=float, default=3.5)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=1.57)
    parser.add_argument("--frame-id", default="map")

    parser.add_argument(
        "--state",
        default=LandmarkState.CANDIDATE.value,
        choices=LandmarkState.values(),
        help="Landmark state.",
    )
    parser.add_argument("--source", default="mapping")
    parser.add_argument("--tag-id", type=int, default=-1)
    parser.add_argument("--confidence", type=float, default=0.90)

    parser.add_argument("--map-name", default="savonia_campus_heart")
    parser.add_argument("--session-id", default="")
    parser.add_argument("--notes", default="")

    parser.add_argument(
        "--from-json",
        default="",
        help="Load landmark from JSON file instead of CLI values.",
    )
    parser.add_argument(
        "--write-json",
        default="",
        help="Write semantic landmark JSON to this path.",
    )
    parser.add_argument(
        "--confirm",
        action="store_true",
        help="Mark created/loaded landmark as confirmed.",
    )
    parser.add_argument(
        "--reject",
        action="store_true",
        help="Mark created/loaded landmark as rejected.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print full JSON result.",
    )

    return parser


# =============================================================================
# File helpers
# =============================================================================
def _load_json(path: str | Path) -> dict:
    file_path = Path(path).expanduser().resolve()

    if not file_path.exists():
        raise FileNotFoundError(f"JSON file not found: {file_path}")

    data = json.loads(file_path.read_text(encoding="utf-8"))

    if not isinstance(data, dict):
        raise ValueError("Landmark JSON root must be a dictionary.")

    return data


def _write_json(path: str | Path, data: dict) -> Path:
    file_path = Path(path).expanduser().resolve()
    file_path.parent.mkdir(parents=True, exist_ok=True)
    file_path.write_text(
        json.dumps(data, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return file_path


# =============================================================================
# Landmark builders
# =============================================================================
def _build_landmark(args: argparse.Namespace):
    if args.from_json:
        landmark = semantic_landmark_from_dict(_load_json(args.from_json))
    else:
        tag_id = None if int(args.tag_id) < 0 else int(args.tag_id)

        landmark = make_semantic_landmark(
            label=args.label,
            x=args.x,
            y=args.y,
            z=args.z,
            yaw=args.yaw,
            frame_id=args.frame_id,
            state=args.state,
            source=args.source,
            tag_id=tag_id,
            confidence=args.confidence,
            map_name=args.map_name or None,
            session_id=args.session_id or None,
            notes=args.notes,
        )

    if args.confirm:
        landmark = landmark.with_state(
            LandmarkState.CONFIRMED.value,
            notes=args.notes or landmark.notes,
        )

    if args.reject:
        landmark = landmark.with_state(
            LandmarkState.REJECTED.value,
            notes=args.notes or landmark.notes,
        )

    return landmark


# =============================================================================
# Main
# =============================================================================
def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    landmark = _build_landmark(args)

    written = ""

    if args.write_json:
        written = str(_write_json(args.write_json, landmark.to_dict()))

    if args.json:
        output = landmark.to_dict()
        output["written"] = written or None
        print(json.dumps(output, indent=2, sort_keys=True))
    else:
        print(
            format_key_value_block(
                "Robot Savo semantic landmark",
                {
                    "key": landmark.key,
                    "label": landmark.label,
                    "state": landmark.state,
                    "confirmed": landmark.confirmed,
                    "source": landmark.source,
                    "tag_id": landmark.tag_id,
                    "confidence": landmark.confidence,
                    "map_name": landmark.map_name,
                    "session_id": landmark.session_id,
                    "frame_id": landmark.pose.frame_id,
                    "x": landmark.pose.x,
                    "y": landmark.pose.y,
                    "z": landmark.pose.z,
                    "yaw": landmark.pose.yaw,
                    "written": written or "no",
                },
            )
        )

    return 0 if landmark.key else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))