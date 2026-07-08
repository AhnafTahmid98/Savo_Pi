#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Offline range health diagnostic for Robot Savo perception."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from typing import Optional

from savo_perception.constants import STATUS_ERROR, STATUS_OK, STATUS_STALE
from savo_perception.models import RangeSample, SensorHealth


@dataclass(frozen=True)
class HealthScenario:
    name: str
    depth_front_m: Optional[float]
    tof_left_m: Optional[float]
    tof_right_m: Optional[float]
    ultrasonic_front_m: Optional[float]
    stale_sensor: str = ""
    invalid_sensor: str = ""
    expected_status: str = STATUS_OK
    expected_ok: bool = True


def make_sample(
    sensor_name: str,
    value_m: Optional[float],
    *,
    stale: bool = False,
    invalid: bool = False,
    stale_timeout_s: float,
) -> RangeSample:
    if stale:
        return RangeSample(
            sensor_name=sensor_name,
            distance_m=value_m,
            stamp_mono_s=time.monotonic() - stale_timeout_s - 1.0,
            valid=value_m is not None,
            source="diagnostic",
            error="",
        )

    if invalid or value_m is None:
        return RangeSample(
            sensor_name=sensor_name,
            distance_m=None,
            stamp_mono_s=time.monotonic(),
            valid=False,
            source="diagnostic",
            error="invalid_or_missing",
        )

    return RangeSample.now(
        sensor_name=sensor_name,
        distance_m=value_m,
        source="diagnostic",
    )


def default_scenarios() -> list[HealthScenario]:
    return [
        HealthScenario(
            name="all_required_ok",
            depth_front_m=None,
            tof_left_m=0.60,
            tof_right_m=0.62,
            ultrasonic_front_m=0.70,
            expected_status=STATUS_OK,
            expected_ok=True,
        ),
        HealthScenario(
            name="optional_depth_ok_missing",
            depth_front_m=None,
            tof_left_m=0.60,
            tof_right_m=0.62,
            ultrasonic_front_m=0.70,
            expected_status=STATUS_OK,
            expected_ok=True,
        ),
        HealthScenario(
            name="tof_left_stale",
            depth_front_m=0.80,
            tof_left_m=0.60,
            tof_right_m=0.62,
            ultrasonic_front_m=0.70,
            stale_sensor="tof_left",
            expected_status=STATUS_STALE,
            expected_ok=False,
        ),
        HealthScenario(
            name="tof_right_invalid",
            depth_front_m=0.80,
            tof_left_m=0.60,
            tof_right_m=None,
            ultrasonic_front_m=0.70,
            invalid_sensor="tof_right",
            expected_status=STATUS_ERROR,
            expected_ok=False,
        ),
        HealthScenario(
            name="ultrasonic_stale",
            depth_front_m=0.80,
            tof_left_m=0.60,
            tof_right_m=0.62,
            ultrasonic_front_m=0.70,
            stale_sensor="ultrasonic_front",
            expected_status=STATUS_STALE,
            expected_ok=False,
        ),
    ]


def build_health(
    scenario: HealthScenario,
    *,
    stale_timeout_s: float,
    include_depth: bool,
) -> dict:
    samples = {
        "depth_front": make_sample(
            "depth_front",
            scenario.depth_front_m,
            stale=scenario.stale_sensor == "depth_front",
            invalid=scenario.invalid_sensor == "depth_front",
            stale_timeout_s=stale_timeout_s,
        ),
        "tof_left": make_sample(
            "tof_left",
            scenario.tof_left_m,
            stale=scenario.stale_sensor == "tof_left",
            invalid=scenario.invalid_sensor == "tof_left",
            stale_timeout_s=stale_timeout_s,
        ),
        "tof_right": make_sample(
            "tof_right",
            scenario.tof_right_m,
            stale=scenario.stale_sensor == "tof_right",
            invalid=scenario.invalid_sensor == "tof_right",
            stale_timeout_s=stale_timeout_s,
        ),
        "ultrasonic_front": make_sample(
            "ultrasonic_front",
            scenario.ultrasonic_front_m,
            stale=scenario.stale_sensor == "ultrasonic_front",
            invalid=scenario.invalid_sensor == "ultrasonic_front",
            stale_timeout_s=stale_timeout_s,
        ),
    }

    now_s = time.monotonic()
    health = {
        name: SensorHealth.from_sample(
            sample,
            stale_timeout_s=stale_timeout_s,
            now_mono_s=now_s,
        )
        for name, sample in samples.items()
    }

    required = ["tof_left", "tof_right"]
    if include_depth:
        required.append("depth_front")

    required_health = [health[name] for name in required]
    ok = all(item.ok for item in required_health)

    stale_sensors = [item.sensor_name for item in required_health if item.stale]
    error_sensors = [
        item.sensor_name
        for item in required_health
        if not item.ok and not item.stale
    ]

    if ok:
        status = STATUS_OK
    elif stale_sensors:
        status = STATUS_STALE
    else:
        status = STATUS_ERROR

    return {
        "ok": ok,
        "status": status,
        "required_sensors": required,
        "optional_sensors": [] if include_depth else ["depth_front"],
        "stale_sensors": stale_sensors,
        "error_sensors": error_sensors,
        "sensors": {name: item.to_dict() for name, item in health.items()},
    }


def run_scenario(
    scenario: HealthScenario,
    *,
    stale_timeout_s: float,
    include_depth: bool,
) -> dict:
    health = build_health(
        scenario,
        stale_timeout_s=stale_timeout_s,
        include_depth=include_depth,
    )

    passed = (
        health["ok"] == scenario.expected_ok
        and health["status"] == scenario.expected_status
    )

    return {
        "name": scenario.name,
        "passed": passed,
        "expected": {
            "ok": scenario.expected_ok,
            "status": scenario.expected_status,
        },
        "actual": {
            "ok": health["ok"],
            "status": health["status"],
            "stale_sensors": health["stale_sensors"],
            "error_sensors": health["error_sensors"],
        },
        "health": health,
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Offline diagnostic for savo_perception range health"
    )

    parser.add_argument("--stale-timeout", type=float, default=0.30)
    parser.add_argument("--include-depth", action="store_true")
    parser.add_argument("--json", action="store_true")

    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    results = [
        run_scenario(
            scenario,
            stale_timeout_s=args.stale_timeout,
            include_depth=args.include_depth,
        )
        for scenario in default_scenarios()
    ]

    all_passed = all(item["passed"] for item in results)

    if args.json:
        print(json.dumps({"passed": all_passed, "results": results}, indent=2, sort_keys=True))
    else:
        print("[RangeHealthCheck] Offline range health scenarios")
        print("-" * 90)

        for item in results:
            mark = "PASS" if item["passed"] else "FAIL"
            actual = item["actual"]
            expected = item["expected"]

            print(
                f"{mark:4s} | {item['name']:<24s} | "
                f"actual={actual['status']:<6s} ok={str(actual['ok']):<5s} "
                f"stale={actual['stale_sensors']} error={actual['error_sensors']} | "
                f"expected={expected['status']} ok={expected['ok']}"
            )

        print("-" * 90)
        print("Result:", "PASS" if all_passed else "FAIL")

    return 0 if all_passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
