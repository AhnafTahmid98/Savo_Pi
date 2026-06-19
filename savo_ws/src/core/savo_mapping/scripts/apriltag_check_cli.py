#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""AprilTag readiness CLI for Robot Savo mapping."""

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


from savo_mapping.constants import TOPIC_APRILTAG_DETECTIONS  # noqa: E402
from savo_mapping.diagnostics.apriltag_ready_check import (  # noqa: E402
    apriltag_result_to_diagnostic,
    evaluate_apriltag_observations,
    evaluate_apriltag_ready,
)
from savo_mapping.diagnostics.report_formatter import format_key_value_block  # noqa: E402
from savo_mapping.models.apriltag_status import AprilTagObservation  # noqa: E402
from savo_mapping.models.semantic_landmark import make_landmark_key  # noqa: E402


# =============================================================================
# Arguments
# =============================================================================
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Check AprilTag readiness for Robot Savo semantic mapping.",
    )

    parser.add_argument(
        "--enabled",
        action="store_true",
        help="Evaluate AprilTag detection as enabled.",
    )
    parser.add_argument(
        "--good",
        action="store_true",
        help="Use known-good simulated AprilTag input.",
    )
    parser.add_argument(
        "--bad",
        action="store_true",
        help="Use known-bad simulated AprilTag input.",
    )

    parser.add_argument(
        "--topic",
        default=TOPIC_APRILTAG_DETECTIONS,
        help="AprilTag detection topic.",
    )
    parser.add_argument("--tag-id", type=int, default=21)
    parser.add_argument("--label", default="A201")
    parser.add_argument("--family", default="tag36h11")
    parser.add_argument("--frame-id", default="camera_color_optical_frame")

    parser.add_argument("--x", type=float, default=4.25)
    parser.add_argument("--y", type=float, default=8.70)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=1.57)
    parser.add_argument("--confidence", type=float, default=0.92)

    parser.add_argument("--msg-count", type=int, default=3)
    parser.add_argument("--age", type=float, default=0.05)
    parser.add_argument("--stale-timeout", type=float, default=1.0)
    parser.add_argument("--min-detections", type=int, default=1)

    parser.add_argument(
        "--require-known-label",
        action="store_true",
        help="Require tag to have a known location label.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print full JSON result.",
    )

    return parser


# =============================================================================
# Input builders
# =============================================================================
def _build_observation(args: argparse.Namespace) -> AprilTagObservation:
    label = args.label.strip() or None

    return AprilTagObservation(
        tag_id=int(args.tag_id),
        family=str(args.family),
        frame_id=str(args.frame_id),
        x=float(args.x),
        y=float(args.y),
        z=float(args.z),
        yaw=float(args.yaw),
        confidence=max(0.0, min(1.0, float(args.confidence))),
        label=label,
    )


def _run_good_check(args: argparse.Namespace):
    observation = _build_observation(args)

    return evaluate_apriltag_observations(
        observations=[observation],
        enabled=True,
        msg_count=max(1, int(args.msg_count)),
        age_s=min(float(args.age), float(args.stale_timeout) * 0.5),
        stale_timeout_s=args.stale_timeout,
        topic=args.topic,
        require_known_label=args.require_known_label,
    )


def _run_bad_check(args: argparse.Namespace):
    return evaluate_apriltag_ready(
        enabled=True,
        msg_count=0,
        detection_count=0,
        unique_tag_count=0,
        last_tag_id=None,
        last_label=None,
        age_s=None,
        stale_timeout_s=args.stale_timeout,
        topic=args.topic,
        min_detections=args.min_detections,
        require_known_label=args.require_known_label,
    )


def _run_manual_check(args: argparse.Namespace):
    if not args.enabled:
        return evaluate_apriltag_ready(
            enabled=False,
            topic=args.topic,
        )

    observation = _build_observation(args)
    observations = [observation] if args.msg_count > 0 else []

    return evaluate_apriltag_observations(
        observations=observations,
        enabled=True,
        msg_count=max(0, int(args.msg_count)),
        age_s=args.age if args.msg_count > 0 else None,
        stale_timeout_s=args.stale_timeout,
        topic=args.topic,
        require_known_label=args.require_known_label,
    )


def _select_result(args: argparse.Namespace):
    if args.good:
        return _run_good_check(args)

    if args.bad:
        return _run_bad_check(args)

    return _run_manual_check(args)


# =============================================================================
# Output
# =============================================================================
def _location_key(label: str) -> str:
    try:
        return make_landmark_key(label)
    except ValueError:
        return ""


def _print_result(args: argparse.Namespace, result) -> None:
    diagnostic = apriltag_result_to_diagnostic(result, required=False)

    if args.json:
        print(result.to_json(indent=2))
        return

    print(
        format_key_value_block(
            "Robot Savo AprilTag check",
            {
                "ok": result.ok,
                "level": diagnostic.level,
                "message": result.message,
                "enabled": result.enabled,
                "topic": result.topic,
                "msg_count": result.msg_count,
                "detection_count": result.detection_count,
                "unique_tag_count": result.unique_tag_count,
                "last_tag_id": result.last_tag_id,
                "last_label": result.last_label,
                "location_key": _location_key(result.last_label or ""),
                "age_s": result.age_s,
                "stale": result.stale,
                "require_known_label": args.require_known_label,
            },
        )
    )


# =============================================================================
# Main
# =============================================================================
def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    result = _select_result(args)
    _print_result(args, result)

    return 0 if result.ok or not result.enabled else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))