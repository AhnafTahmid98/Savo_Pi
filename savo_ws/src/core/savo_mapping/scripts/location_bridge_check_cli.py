#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""savo_location bridge check CLI for Robot Savo mapping."""

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


from savo_mapping.diagnostics.location_bridge_check import (  # noqa: E402
    evaluate_location_bridge_ready,
    location_bridge_result_to_diagnostic,
)
from savo_mapping.diagnostics.report_formatter import format_key_value_block  # noqa: E402
from savo_mapping.models.location_bridge_status import (  # noqa: E402
    make_location_bridge_disabled_status,
    make_location_bridge_status,
    make_location_confirmed_status,
    make_location_saved_status,
)


# =============================================================================
# Arguments
# =============================================================================
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Check future savo_mapping → savo_location bridge readiness.",
    )

    parser.add_argument(
        "--enabled",
        action="store_true",
        help="Evaluate location bridge as enabled.",
    )
    parser.add_argument(
        "--good",
        action="store_true",
        help="Use known-good simulated bridge input.",
    )
    parser.add_argument(
        "--bad",
        action="store_true",
        help="Use known-bad simulated bridge input.",
    )

    parser.add_argument(
        "--package-available",
        action="store_true",
        help="Mark savo_location package/interface as available.",
    )
    parser.add_argument(
        "--topic-available",
        action="store_true",
        help="Mark location bridge topic as available.",
    )
    parser.add_argument(
        "--service-available",
        action="store_true",
        help="Mark location bridge service as available.",
    )

    parser.add_argument("--target-package", default="savo_location")
    parser.add_argument("--target-topic", default="/savo_location/landmarks")
    parser.add_argument("--status-topic", default="/savo_mapping/location_bridge_status")

    parser.add_argument("--saved-count", type=int, default=0)
    parser.add_argument("--confirmed-count", type=int, default=0)
    parser.add_argument("--failed-count", type=int, default=0)

    parser.add_argument("--msg-count", type=int, default=3)
    parser.add_argument("--age", type=float, default=0.1)
    parser.add_argument("--stale-timeout", type=float, default=2.0)

    parser.add_argument("--landmark-key", default="a201")
    parser.add_argument("--label", default="A201")
    parser.add_argument("--location-id", default="loc_a201")

    parser.add_argument(
        "--saved",
        action="store_true",
        help="Print a simulated saved-location bridge status.",
    )
    parser.add_argument(
        "--confirmed",
        action="store_true",
        help="Print a simulated confirmed-location bridge status.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print full JSON result.",
    )

    return parser


# =============================================================================
# Status builders
# =============================================================================
def _bridge_status_from_args(args: argparse.Namespace):
    if args.saved:
        return make_location_saved_status(
            landmark_key=args.landmark_key,
            label=args.label,
            location_id=args.location_id or None,
            saved_count=max(1, int(args.saved_count or 1)),
            confirmed_count=max(0, int(args.confirmed_count)),
        )

    if args.confirmed:
        return make_location_confirmed_status(
            landmark_key=args.landmark_key,
            label=args.label,
            location_id=args.location_id or None,
            saved_count=max(1, int(args.saved_count or 1)),
            confirmed_count=max(1, int(args.confirmed_count or 1)),
        )

    if not args.enabled and not args.good and not args.bad:
        return make_location_bridge_disabled_status()

    return make_location_bridge_status(
        enabled=True,
        state="ready",
        ok=True,
        saved_count=max(0, int(args.saved_count)),
        confirmed_count=max(0, int(args.confirmed_count)),
        failed_count=max(0, int(args.failed_count)),
        message="Location bridge ready.",
    )


def _evaluate(args: argparse.Namespace):
    if args.good:
        return evaluate_location_bridge_ready(
            enabled=True,
            package_available=True,
            topic_available=True,
            service_available=False,
            saved_count=max(0, int(args.saved_count)),
            confirmed_count=max(0, int(args.confirmed_count)),
            failed_count=0,
            msg_count=max(1, int(args.msg_count)),
            age_s=min(float(args.age), float(args.stale_timeout) * 0.5),
            stale_timeout_s=args.stale_timeout,
            target_package=args.target_package,
            target_topic=args.target_topic,
            status_topic=args.status_topic,
        )

    if args.bad:
        return evaluate_location_bridge_ready(
            enabled=True,
            package_available=False,
            topic_available=False,
            service_available=False,
            saved_count=0,
            confirmed_count=0,
            failed_count=max(0, int(args.failed_count)),
            msg_count=0,
            age_s=None,
            stale_timeout_s=args.stale_timeout,
            target_package=args.target_package,
            target_topic=args.target_topic,
            status_topic=args.status_topic,
        )

    return evaluate_location_bridge_ready(
        enabled=bool(args.enabled),
        package_available=bool(args.package_available),
        topic_available=bool(args.topic_available),
        service_available=bool(args.service_available),
        saved_count=max(0, int(args.saved_count)),
        confirmed_count=max(0, int(args.confirmed_count)),
        failed_count=max(0, int(args.failed_count)),
        msg_count=max(0, int(args.msg_count)),
        age_s=args.age if args.enabled else None,
        stale_timeout_s=args.stale_timeout,
        target_package=args.target_package,
        target_topic=args.target_topic,
        status_topic=args.status_topic,
    )


# =============================================================================
# Main
# =============================================================================
def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    result = _evaluate(args)
    diagnostic = location_bridge_result_to_diagnostic(result, required=False)
    status = _bridge_status_from_args(args)

    if args.json:
        output = result.to_dict()
        output["example_status"] = status.to_dict()
        print(json.dumps(output, indent=2, sort_keys=True))
    else:
        print(
            format_key_value_block(
                "Robot Savo location bridge check",
                {
                    "ok": result.ok,
                    "level": diagnostic.level,
                    "message": result.message,
                    "enabled": result.enabled,
                    "target_package": result.target_package,
                    "target_topic": result.target_topic,
                    "status_topic": result.status_topic,
                    "package_available": result.package_available,
                    "topic_available": result.topic_available,
                    "service_available": result.service_available,
                    "saved_count": result.saved_count,
                    "confirmed_count": result.confirmed_count,
                    "failed_count": result.failed_count,
                    "msg_count": result.msg_count,
                    "age_s": result.age_s,
                    "stale": result.stale,
                    "example_bridge_state": status.state,
                },
            )
        )

    return 0 if result.ok or not result.enabled else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))