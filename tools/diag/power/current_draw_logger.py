#!/usr/bin/env python3
"""Collect bounded Robot SAVO power telemetry without direct hardware access."""
from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import json
import time
from datetime import UTC, datetime
from typing import Any

from tools.diag.infra.diag_utils import emit_result, parser
from tools.diag.infra.ros_probe import collect, compact


def _decode(sample: dict[str, Any]) -> dict[str, Any] | None:
    raw = sample.get("data")
    if not isinstance(raw, str):
        return None
    try:
        decoded = json.loads(raw)
    except json.JSONDecodeError:
        return {"text": raw}
    return decoded if isinstance(decoded, dict) else {"value": decoded}


def _numeric_values(value: Any, prefix: str = "") -> dict[str, float]:
    result: dict[str, float] = {}
    if isinstance(value, bool):
        return result
    if isinstance(value, (int, float)):
        result[prefix or "value"] = float(value)
    elif isinstance(value, dict):
        for key, item in value.items():
            child = f"{prefix}.{key}" if prefix else str(key)
            result.update(_numeric_values(item, child))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            child = f"{prefix}[{index}]" if prefix else f"[{index}]"
            result.update(_numeric_values(item, child))
    return result


def main() -> int:
    value = parser(__doc__)
    value.set_defaults(timeout=10.0)
    value.add_argument("--maximum-samples", type=int, default=100)
    args = value.parse_args()
    started = time.monotonic()
    started_utc = datetime.now(UTC).isoformat()

    if args.dry_run:
        return emit_result(
            "power_telemetry_logger",
            "BLOCKED",
            "dry_run",
            {},
            output=args.output,
            started=started,
            started_utc=started_utc,
        )
    if args.maximum_samples < 1 or args.maximum_samples > 1000:
        return emit_result(
            "power_telemetry_logger",
            "FAIL",
            "maximum_samples_out_of_bounds",
            {"maximum_samples": args.maximum_samples},
            output=args.output,
            started=started,
            started_utc=started_utc,
        )

    topics = {
        "core_ups": "/savo_power/core/ups",
        "edge_ups": "/savo_power/edge/ups",
        "base_battery": "/savo_power/base/battery",
        "aggregate": "/savo_power/status",
    }
    details: dict[str, Any] = {"topics": {}, "numeric_series": {}}
    publishers = 0
    samples = 0
    try:
        per_topic_duration = max(0.5, args.timeout / len(topics))
        for label, topic in topics.items():
            probe = collect(
                topic,
                "std_msgs/msg/String",
                duration_s=per_topic_duration,
                minimum_samples=1,
                reliable=True,
                maximum_samples=args.maximum_samples,
            )
            publishers += probe.publisher_count
            samples += probe.sample_count
            decoded = [item for sample in probe.samples if (item := _decode(sample)) is not None]
            details["topics"][label] = {**compact(probe), "decoded_samples": decoded}
            series: dict[str, list[float]] = {}
            for item in decoded:
                for key, number in _numeric_values(item).items():
                    series.setdefault(key, []).append(number)
            details["numeric_series"][label] = series
    except (ImportError, RuntimeError, ValueError) as exc:
        return emit_result(
            "power_telemetry_logger",
            "BLOCKED",
            "ros_probe_unavailable",
            {"error": str(exc)},
            output=args.output,
            started=started,
            started_utc=started_utc,
        )

    if publishers == 0:
        status, reason = "BLOCKED", "power_publishers_missing"
    elif samples == 0:
        status, reason = "FAIL", "power_publishers_present_but_no_samples"
    else:
        status, reason = "PASS", "bounded_power_telemetry_collected"
    details["publisher_count_total"] = publishers
    details["sample_count_total"] = samples
    return emit_result(
        "power_telemetry_logger",
        status,
        reason,
        details,
        output=args.output,
        started=started,
        started_utc=started_utc,
    )


if __name__ == "__main__":
    raise SystemExit(main())
