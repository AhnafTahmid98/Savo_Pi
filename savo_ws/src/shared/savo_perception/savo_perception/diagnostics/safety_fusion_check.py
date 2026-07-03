#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Offline safety fusion diagnostic for Robot Savo perception."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from typing import Optional

from savo_perception.models import RangeSample, RangeSnapshot
from savo_perception.safety import RangeFusionConfig, SafetyPolicy, SafetyPolicyConfig


@dataclass(frozen=True)
class Scenario:
    name: str
    depth_front_m: Optional[float]
    tof_left_m: Optional[float]
    tof_right_m: Optional[float]
    ultrasonic_front_m: Optional[float]
    expected_stop: bool
    expected_status: str


def make_sample(sensor_name: str, value_m: Optional[float], *, missing_is_stale: bool = False) -> RangeSample:
    if value_m is None:
        return RangeSample(
            sensor_name=sensor_name,
            distance_m=None,
            stamp_mono_s=0.0 if missing_is_stale else time.monotonic(),
            valid=False,
            source="diagnostic",
            error="missing",
        )

    return RangeSample.now(
        sensor_name=sensor_name,
        distance_m=value_m,
        source="diagnostic",
    )


def make_snapshot(scenario: Scenario) -> RangeSnapshot:
    return RangeSnapshot(
        depth_front=make_sample("depth_front", scenario.depth_front_m),
        tof_left=make_sample("tof_left", scenario.tof_left_m, missing_is_stale=True),
        tof_right=make_sample("tof_right", scenario.tof_right_m, missing_is_stale=True),
        ultrasonic_front=make_sample(
            "ultrasonic_front",
            scenario.ultrasonic_front_m,
            missing_is_stale=True,
        ),
    )


def default_scenarios() -> list[Scenario]:
    return [
        Scenario(
            name="clear_all_far",
            depth_front_m=1.20,
            tof_left_m=0.90,
            tof_right_m=0.90,
            ultrasonic_front_m=1.00,
            expected_stop=False,
            expected_status="OK",
        ),
        Scenario(
            name="front_slow_depth",
            depth_front_m=0.60,
            tof_left_m=0.90,
            tof_right_m=0.90,
            ultrasonic_front_m=0.80,
            expected_stop=False,
            expected_status="SLOW",
        ),
        Scenario(
            name="front_stop_depth",
            depth_front_m=0.20,
            tof_left_m=0.90,
            tof_right_m=0.90,
            ultrasonic_front_m=0.80,
            expected_stop=True,
            expected_status="SAFETY_STOP",
        ),
        Scenario(
            name="front_stop_ultrasonic",
            depth_front_m=1.00,
            tof_left_m=0.90,
            tof_right_m=0.90,
            ultrasonic_front_m=0.20,
            expected_stop=True,
            expected_status="SAFETY_STOP",
        ),
        Scenario(
            name="side_stop_left_tof",
            depth_front_m=1.00,
            tof_left_m=0.06,
            tof_right_m=0.90,
            ultrasonic_front_m=0.90,
            expected_stop=True,
            expected_status="SAFETY_STOP",
        ),
        Scenario(
            name="missing_required_tof",
            depth_front_m=1.00,
            tof_left_m=None,
            tof_right_m=0.90,
            ultrasonic_front_m=0.90,
            expected_stop=True,
            expected_status="SAFETY_STOP",
        ),
        Scenario(
            name="missing_optional_depth",
            depth_front_m=None,
            tof_left_m=0.90,
            tof_right_m=0.90,
            ultrasonic_front_m=0.90,
            expected_stop=False,
            expected_status="OK",
        ),
    ]


def build_policy(args: argparse.Namespace) -> SafetyPolicy:
    fusion = RangeFusionConfig(
        front_stop_m=args.front_stop,
        front_slow_m=args.front_slow,
        side_stop_m=args.side_stop,
        side_slow_m=args.side_slow,
        stale_timeout_s=args.stale_timeout,
        fail_safe_on_stale=not args.no_fail_safe,
        required_sensors=("tof_left", "tof_right", "ultrasonic_front"),
    )

    return SafetyPolicy(
        SafetyPolicyConfig(
            fusion=fusion,
            stop_debounce_count=args.stop_debounce,
            clear_debounce_count=args.clear_debounce,
            front_clear_hysteresis_m=args.front_hysteresis,
            side_clear_hysteresis_m=args.side_hysteresis,
            slowdown_ema_alpha=1.0,
        )
    )


def run_scenario(policy: SafetyPolicy, scenario: Scenario) -> dict:
    update = policy.update(make_snapshot(scenario))
    decision = update.published_decision

    passed = (
        decision.stop_required == scenario.expected_stop
        and decision.status == scenario.expected_status
    )

    return {
        "name": scenario.name,
        "passed": passed,
        "expected": {
            "stop_required": scenario.expected_stop,
            "status": scenario.expected_status,
        },
        "actual": {
            "stop_required": decision.stop_required,
            "status": decision.status,
            "reason": decision.reason,
            "slowdown_factor": decision.slowdown_factor,
            "front_distance_m": decision.front_distance_m,
            "side_distance_m": decision.side_distance_m,
        },
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Offline diagnostic for savo_perception safety fusion"
    )

    parser.add_argument("--front-stop", type=float, default=0.25)
    parser.add_argument("--front-slow", type=float, default=0.80)
    parser.add_argument("--side-stop", type=float, default=0.08)
    parser.add_argument("--side-slow", type=float, default=0.25)
    parser.add_argument("--stale-timeout", type=float, default=0.30)

    parser.add_argument("--stop-debounce", type=int, default=1)
    parser.add_argument("--clear-debounce", type=int, default=1)
    parser.add_argument("--front-hysteresis", type=float, default=0.010)
    parser.add_argument("--side-hysteresis", type=float, default=0.010)

    parser.add_argument("--no-fail-safe", action="store_true")
    parser.add_argument("--json", action="store_true")

    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    results = []
    for scenario in default_scenarios():
        policy = build_policy(args)
        results.append(run_scenario(policy, scenario))

    all_passed = all(item["passed"] for item in results)

    if args.json:
        print(json.dumps({"passed": all_passed, "results": results}, indent=2, sort_keys=True))
    else:
        print("[SafetyFusionCheck] Offline safety fusion scenarios")
        print("-" * 78)

        for item in results:
            mark = "PASS" if item["passed"] else "FAIL"
            actual = item["actual"]
            expected = item["expected"]

            print(
                f"{mark:4s} | {item['name']:<24s} | "
                f"actual={actual['status']:<11s} stop={str(actual['stop_required']):<5s} "
                f"reason={actual['reason']:<28s} | "
                f"expected={expected['status']} stop={expected['stop_required']}"
            )

        print("-" * 78)
        print("Result:", "PASS" if all_passed else "FAIL")

    return 0 if all_passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
