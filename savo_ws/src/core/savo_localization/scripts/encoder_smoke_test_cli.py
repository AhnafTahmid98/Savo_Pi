#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Smoke-test Robot Savo four-wheel encoder odometry topics."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from dataclasses import dataclass
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


WHEEL_NAMES = ("FL", "FR", "RL", "RR")


@dataclass(frozen=True)
class OdomSample:
    stamp_s: float
    x_m: float
    y_m: float
    yaw_z: float
    yaw_w: float
    vx_mps: float
    vy_mps: float
    omega_rad_s: float

    @property
    def linear_speed_mps(self) -> float:
        return math.hypot(self.vx_mps, self.vy_mps)

    @property
    def angular_speed_rad_s(self) -> float:
        return abs(self.omega_rad_s)


class EncoderSmokeTestNode(Node):
    def __init__(
        self,
        *,
        odom_topic: str,
        state_topic: str,
        use_state: bool,
    ) -> None:
        super().__init__("encoder_smoke_test_cli")

        self.odom_samples: list[OdomSample] = []
        self.state_samples: list[dict[str, Any]] = []
        self.state_parse_errors = 0

        self.create_subscription(
            Odometry,
            odom_topic,
            self._on_odom,
            QoSProfile(
                depth=20,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

        if use_state:
            self.create_subscription(
                String,
                state_topic,
                self._on_state,
                QoSProfile(
                    depth=20,
                    reliability=ReliabilityPolicy.RELIABLE,
                ),
            )

    def _on_odom(self, msg: Odometry) -> None:
        self.odom_samples.append(
            OdomSample(
                stamp_s=time.monotonic(),
                x_m=float(msg.pose.pose.position.x),
                y_m=float(msg.pose.pose.position.y),
                yaw_z=float(msg.pose.pose.orientation.z),
                yaw_w=float(msg.pose.pose.orientation.w),
                vx_mps=float(msg.twist.twist.linear.x),
                vy_mps=float(msg.twist.twist.linear.y),
                omega_rad_s=float(msg.twist.twist.angular.z),
            )
        )

    def _on_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.state_parse_errors += 1
            return

        if isinstance(data, dict):
            self.state_samples.append(data)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Smoke-test Robot Savo four-wheel encoder odometry topics."
    )
    parser.add_argument("--odom-topic", default="/wheel/odom")
    parser.add_argument(
        "--state-topic",
        default="/savo_localization/wheel_odom_state",
    )
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--min-rate-hz", type=float, default=5.0)
    parser.add_argument("--max-linear-mps", type=float, default=1.5)
    parser.add_argument("--max-angular-rad-s", type=float, default=4.0)
    parser.add_argument("--max-illegal", type=int, default=20)
    parser.add_argument("--expect-motion", action="store_true")
    parser.add_argument("--min-active-wheels", type=int, default=4)
    parser.add_argument("--no-state", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    if args.samples <= 0:
        raise SystemExit("--samples must be > 0")

    if args.timeout <= 0.0:
        raise SystemExit("--timeout must be > 0.0")

    if args.min_active_wheels < 0 or args.min_active_wheels > 4:
        raise SystemExit("--min-active-wheels must be 0..4")

    rclpy.init()

    node = EncoderSmokeTestNode(
        odom_topic=args.odom_topic,
        state_topic=args.state_topic,
        use_state=not args.no_state,
    )

    deadline_s = time.monotonic() + args.timeout

    try:
        while (
            rclpy.ok()
            and len(node.odom_samples) < args.samples
            and time.monotonic() < deadline_s
        ):
            rclpy.spin_once(node, timeout_sec=0.1)

        report = build_report(
            odom_samples=node.odom_samples,
            state_samples=node.state_samples,
            state_parse_errors=node.state_parse_errors,
            requested_samples=args.samples,
            timeout_s=args.timeout,
            odom_topic=args.odom_topic,
            state_topic=args.state_topic,
            state_topic_enabled=not args.no_state,
            min_rate_hz=args.min_rate_hz,
            max_linear_mps=args.max_linear_mps,
            max_angular_rad_s=args.max_angular_rad_s,
            max_illegal=args.max_illegal,
            expect_motion=args.expect_motion,
            min_active_wheels=args.min_active_wheels,
        )

        if args.json:
            print(json.dumps(report, indent=2, sort_keys=True))
        else:
            print_text_report(report)

        return 0 if report["ok"] else 1

    finally:
        node.destroy_node()
        rclpy.shutdown()


def build_report(
    *,
    odom_samples: list[OdomSample],
    state_samples: list[dict[str, Any]],
    state_parse_errors: int,
    requested_samples: int,
    timeout_s: float,
    odom_topic: str,
    state_topic: str,
    state_topic_enabled: bool,
    min_rate_hz: float,
    max_linear_mps: float,
    max_angular_rad_s: float,
    max_illegal: int,
    expect_motion: bool,
    min_active_wheels: int,
) -> dict[str, Any]:
    reasons: list[str] = []

    if not odom_samples:
        return {
            "ok": False,
            "status": "ERROR",
            "message": "No wheel odometry samples received.",
            "reasons": [f"no messages received on {odom_topic}"],
            "odom_topic": odom_topic,
            "state_topic": state_topic,
            "sample_count": 0,
            "requested_samples": requested_samples,
            "timeout_s": timeout_s,
        }

    elapsed_s = max(odom_samples[-1].stamp_s - odom_samples[0].stamp_s, 1e-9)
    measured_rate_hz = (
        (len(odom_samples) - 1) / elapsed_s if len(odom_samples) >= 2 else 0.0
    )

    linear_speeds = [sample.linear_speed_mps for sample in odom_samples]
    angular_speeds = [sample.angular_speed_rad_s for sample in odom_samples]

    if len(odom_samples) < requested_samples:
        reasons.append(
            f"received {len(odom_samples)}/{requested_samples} odom samples before timeout"
        )

    if measured_rate_hz < min_rate_hz:
        reasons.append(
            f"wheel odom rate too low: {measured_rate_hz:.2f} Hz < {min_rate_hz:.2f} Hz"
        )

    max_linear_seen = max(linear_speeds)
    max_angular_seen = max(angular_speeds)

    if max_linear_seen > max_linear_mps:
        reasons.append(
            f"linear speed too high: {max_linear_seen:.3f} m/s > {max_linear_mps:.3f} m/s"
        )

    if max_angular_seen > max_angular_rad_s:
        reasons.append(
            f"angular speed too high: {max_angular_seen:.3f} rad/s > "
            f"{max_angular_rad_s:.3f} rad/s"
        )

    if state_topic_enabled and not state_samples:
        reasons.append(f"no valid JSON messages received on {state_topic}")

    if state_parse_errors > 0:
        reasons.append(f"{state_parse_errors} state messages failed JSON parsing")

    latest_state = state_samples[-1] if state_samples else {}
    encoder_summary = summarize_encoder_state(latest_state)

    total_illegal = int(encoder_summary.get("total_illegal_transitions", 0))
    active_wheel_count = int(encoder_summary.get("active_wheel_count", 0))

    if total_illegal > max_illegal:
        reasons.append(
            f"illegal encoder transitions too high: {total_illegal} > {max_illegal}"
        )

    if expect_motion and active_wheel_count < min_active_wheels:
        reasons.append(
            "not enough active wheels during expected motion: "
            f"{active_wheel_count}/{min_active_wheels}"
        )

    if expect_motion:
        wheel_deltas = encoder_summary.get("wheel_deltas", {})
        if isinstance(wheel_deltas, dict):
            stopped_wheels = [
                name for name in WHEEL_NAMES if int(wheel_deltas.get(name, 0)) == 0
            ]
            if stopped_wheels:
                reasons.append(
                    "wheel delta is zero during expected motion: "
                    + ", ".join(stopped_wheels)
                )

    ok = not reasons

    return {
        "ok": ok,
        "status": "OK" if ok else "WARN",
        "message": (
            "Encoder smoke test passed"
            if ok
            else "Encoder smoke test completed with notes"
        ),
        "reasons": reasons,
        "odom_topic": odom_topic,
        "state_topic": state_topic,
        "state_topic_enabled": state_topic_enabled,
        "odom_sample_count": len(odom_samples),
        "requested_samples": requested_samples,
        "timeout_s": timeout_s,
        "measured_rate_hz": measured_rate_hz,
        "linear_speed_mean_mps": statistics.fmean(linear_speeds),
        "linear_speed_max_mps": max_linear_seen,
        "angular_speed_mean_rad_s": statistics.fmean(angular_speeds),
        "angular_speed_max_rad_s": max_angular_seen,
        "state_sample_count": len(state_samples),
        "state_parse_errors": state_parse_errors,
        "encoder_summary": encoder_summary,
        "latest_state": latest_state,
    }


def summarize_encoder_state(state: dict[str, Any]) -> dict[str, Any]:
    encoders = state.get("encoders", {})
    if not isinstance(encoders, dict):
        encoders = {}

    active_wheel_count = int(encoders.get("active_wheel_count", 0))
    total_illegal = int(encoders.get("total_illegal_transitions", 0))

    wheel_deltas: dict[str, int] = {}
    wheel_counts: dict[str, int] = {}
    wheel_speeds: dict[str, float] = {}
    wheel_directions: dict[str, int] = {}
    wheel_illegal: dict[str, int] = {}

    for wheel_name in WHEEL_NAMES:
        wheel = encoders.get(wheel_name, {})
        if not isinstance(wheel, dict):
            wheel = {}

        wheel_deltas[wheel_name] = int(wheel.get("delta", 0))
        wheel_counts[wheel_name] = int(wheel.get("count", 0))
        wheel_speeds[wheel_name] = float(wheel.get("speed_mps", 0.0))
        wheel_directions[wheel_name] = int(wheel.get("direction", 0))
        wheel_illegal[wheel_name] = int(wheel.get("illegal", 0))

    return {
        "active_wheel_count": active_wheel_count,
        "total_illegal_transitions": total_illegal,
        "wheel_deltas": wheel_deltas,
        "wheel_counts": wheel_counts,
        "wheel_speeds_mps": wheel_speeds,
        "wheel_directions": wheel_directions,
        "wheel_illegal_transitions": wheel_illegal,
    }


def print_text_report(report: dict[str, Any]) -> None:
    print("Robot Savo Encoder Smoke Test")
    print("=============================")
    print(f"Status: {report['status']}")
    print(f"Message: {report['message']}")
    print(f"Odom topic: {report['odom_topic']}")
    print(f"State topic: {report['state_topic']}")
    print(f"Odom samples: {report.get('odom_sample_count', 0)}/{report['requested_samples']}")

    if report.get("odom_sample_count", 0) > 0:
        print(f"Rate: {report['measured_rate_hz']:.2f} Hz")
        print(
            "Linear speed: "
            f"mean={report['linear_speed_mean_mps']:.4f} m/s "
            f"max={report['linear_speed_max_mps']:.4f} m/s"
        )
        print(
            "Angular speed: "
            f"mean={report['angular_speed_mean_rad_s']:.4f} rad/s "
            f"max={report['angular_speed_max_rad_s']:.4f} rad/s"
        )

    if report.get("state_topic_enabled"):
        print(f"State messages: {report.get('state_sample_count', 0)}")
        print(f"State parse errors: {report.get('state_parse_errors', 0)}")

    encoder_summary = report.get("encoder_summary", {})
    if isinstance(encoder_summary, dict):
        print("Encoder summary:")
        print(f"  active wheels: {encoder_summary.get('active_wheel_count', 0)}")
        print(
            "  illegal transitions: "
            f"{encoder_summary.get('total_illegal_transitions', 0)}"
        )

        wheel_counts = encoder_summary.get("wheel_counts", {})
        wheel_deltas = encoder_summary.get("wheel_deltas", {})
        wheel_speeds = encoder_summary.get("wheel_speeds_mps", {})
        wheel_directions = encoder_summary.get("wheel_directions", {})
        wheel_illegal = encoder_summary.get("wheel_illegal_transitions", {})

        print("  wheels:")
        for wheel_name in WHEEL_NAMES:
            print(
                "    "
                f"{wheel_name}: "
                f"count={_dict_get(wheel_counts, wheel_name, 0)} "
                f"delta={_dict_get(wheel_deltas, wheel_name, 0)} "
                f"speed={float(_dict_get(wheel_speeds, wheel_name, 0.0)):.4f} m/s "
                f"dir={_dict_get(wheel_directions, wheel_name, 0)} "
                f"illegal={_dict_get(wheel_illegal, wheel_name, 0)}"
            )

    if report["reasons"]:
        print("Reasons:")
        for reason in report["reasons"]:
            print(f"  - {reason}")

    print(f"Result: {'PASS' if report['ok'] else 'CHECK'}")


def _dict_get(data: Any, key: str, default: Any) -> Any:
    if isinstance(data, dict):
        return data.get(key, default)

    return default


if __name__ == "__main__":
    sys.exit(main())
