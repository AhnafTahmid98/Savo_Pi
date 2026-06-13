#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Smoke-test /imu/data and the Robot Savo IMU state topic."""

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
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String


@dataclass(frozen=True)
class ImuSample:
    stamp_s: float
    accel_norm_mps2: float
    gyro_norm_rad_s: float
    gyro_z_rad_s: float
    orientation_available: bool


class ImuSmokeTestNode(Node):
    def __init__(
        self,
        *,
        imu_topic: str,
        state_topic: str,
        use_state: bool,
    ) -> None:
        super().__init__("imu_smoke_test_cli")

        self.samples: list[ImuSample] = []
        self.last_state: dict[str, Any] | None = None
        self.state_count = 0

        self.create_subscription(
            Imu,
            imu_topic,
            self._on_imu,
            QoSProfile(
                depth=20,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
        )

        if use_state:
            self.create_subscription(
                String,
                state_topic,
                self._on_state,
                QoSProfile(
                    depth=10,
                    reliability=ReliabilityPolicy.RELIABLE,
                ),
            )

    def _on_imu(self, msg: Imu) -> None:
        accel_norm = math.sqrt(
            msg.linear_acceleration.x * msg.linear_acceleration.x
            + msg.linear_acceleration.y * msg.linear_acceleration.y
            + msg.linear_acceleration.z * msg.linear_acceleration.z
        )

        gyro_norm = math.sqrt(
            msg.angular_velocity.x * msg.angular_velocity.x
            + msg.angular_velocity.y * msg.angular_velocity.y
            + msg.angular_velocity.z * msg.angular_velocity.z
        )

        orientation_available = (
            len(msg.orientation_covariance) >= 1
            and msg.orientation_covariance[0] >= 0.0
        )

        self.samples.append(
            ImuSample(
                stamp_s=time.monotonic(),
                accel_norm_mps2=accel_norm,
                gyro_norm_rad_s=gyro_norm,
                gyro_z_rad_s=msg.angular_velocity.z,
                orientation_available=orientation_available,
            )
        )

    def _on_state(self, msg: String) -> None:
        self.state_count += 1

        try:
            self.last_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.last_state = {
                "raw": msg.data,
                "parse_error": True,
            }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Smoke-test Robot Savo IMU ROS topics."
    )
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--state-topic", default="/savo_localization/imu_state")
    parser.add_argument("--samples", type=int, default=25)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--min-rate-hz", type=float, default=5.0)
    parser.add_argument("--min-accel-norm", type=float, default=6.0)
    parser.add_argument("--max-accel-norm", type=float, default=14.0)
    parser.add_argument("--no-state", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    if args.samples <= 0:
        raise SystemExit("--samples must be > 0")

    if args.timeout <= 0.0:
        raise SystemExit("--timeout must be > 0.0")

    rclpy.init()

    node = ImuSmokeTestNode(
        imu_topic=args.imu_topic,
        state_topic=args.state_topic,
        use_state=not args.no_state,
    )

    deadline_s = time.monotonic() + args.timeout

    try:
        while (
            rclpy.ok()
            and len(node.samples) < args.samples
            and time.monotonic() < deadline_s
        ):
            rclpy.spin_once(node, timeout_sec=0.1)

        report = build_report(
            samples=node.samples,
            requested_samples=args.samples,
            timeout_s=args.timeout,
            min_rate_hz=args.min_rate_hz,
            min_accel_norm=args.min_accel_norm,
            max_accel_norm=args.max_accel_norm,
            state_topic_enabled=not args.no_state,
            state_count=node.state_count,
            last_state=node.last_state,
            imu_topic=args.imu_topic,
            state_topic=args.state_topic,
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
    samples: list[ImuSample],
    requested_samples: int,
    timeout_s: float,
    min_rate_hz: float,
    min_accel_norm: float,
    max_accel_norm: float,
    state_topic_enabled: bool,
    state_count: int,
    last_state: dict[str, Any] | None,
    imu_topic: str,
    state_topic: str,
) -> dict[str, Any]:
    reasons: list[str] = []

    if not samples:
        return {
            "ok": False,
            "status": "ERROR",
            "message": "No IMU samples received.",
            "reasons": [f"no messages received on {imu_topic}"],
            "imu_topic": imu_topic,
            "state_topic": state_topic,
            "sample_count": 0,
            "requested_samples": requested_samples,
            "timeout_s": timeout_s,
        }

    elapsed_s = max(samples[-1].stamp_s - samples[0].stamp_s, 1e-9)
    measured_rate_hz = (
        (len(samples) - 1) / elapsed_s if len(samples) >= 2 else 0.0
    )

    accel_norms = [sample.accel_norm_mps2 for sample in samples]
    gyro_norms = [sample.gyro_norm_rad_s for sample in samples]
    gyro_z_values = [sample.gyro_z_rad_s for sample in samples]
    orientation_count = sum(1 for sample in samples if sample.orientation_available)

    accel_mean = statistics.fmean(accel_norms)
    gyro_mean = statistics.fmean(gyro_norms)
    gyro_z_mean = statistics.fmean(gyro_z_values)

    if len(samples) < requested_samples:
        reasons.append(f"received {len(samples)}/{requested_samples} samples before timeout")

    if measured_rate_hz < min_rate_hz:
        reasons.append(
            f"IMU rate too low: {measured_rate_hz:.2f} Hz < {min_rate_hz:.2f} Hz"
        )

    if accel_mean < min_accel_norm or accel_mean > max_accel_norm:
        reasons.append(
            "acceleration norm outside expected range: "
            f"{accel_mean:.3f} m/s² not in "
            f"[{min_accel_norm:.3f}, {max_accel_norm:.3f}]"
        )

    if state_topic_enabled and state_count <= 0:
        reasons.append(f"no messages received on {state_topic}")

    state_parse_error = bool(last_state and last_state.get("parse_error"))
    if state_parse_error:
        reasons.append("IMU state topic published non-JSON data")

    ok = not reasons

    return {
        "ok": ok,
        "status": "OK" if ok else "WARN",
        "message": "IMU smoke test passed" if ok else "IMU smoke test completed with notes",
        "reasons": reasons,
        "imu_topic": imu_topic,
        "state_topic": state_topic,
        "sample_count": len(samples),
        "requested_samples": requested_samples,
        "timeout_s": timeout_s,
        "measured_rate_hz": measured_rate_hz,
        "accel_norm_mean_mps2": accel_mean,
        "accel_norm_min_mps2": min(accel_norms),
        "accel_norm_max_mps2": max(accel_norms),
        "gyro_norm_mean_rad_s": gyro_mean,
        "gyro_z_mean_rad_s": gyro_z_mean,
        "orientation_available_count": orientation_count,
        "state_topic_enabled": state_topic_enabled,
        "state_count": state_count,
        "last_state": last_state,
    }


def print_text_report(report: dict[str, Any]) -> None:
    print("Robot Savo IMU Smoke Test")
    print("=========================")
    print(f"Status: {report['status']}")
    print(f"Message: {report['message']}")
    print(f"IMU topic: {report['imu_topic']}")
    print(f"State topic: {report['state_topic']}")
    print(f"Samples: {report['sample_count']}/{report['requested_samples']}")

    if report["sample_count"] > 0:
        print(f"Rate: {report['measured_rate_hz']:.2f} Hz")
        print(
            "Accel norm: "
            f"mean={report['accel_norm_mean_mps2']:.3f} m/s² "
            f"min={report['accel_norm_min_mps2']:.3f} "
            f"max={report['accel_norm_max_mps2']:.3f}"
        )
        print(f"Gyro norm mean: {report['gyro_norm_mean_rad_s']:.6f} rad/s")
        print(f"Gyro Z mean: {report['gyro_z_mean_rad_s']:.6f} rad/s")
        print(f"Orientation samples: {report['orientation_available_count']}")

    if report.get("state_topic_enabled"):
        print(f"State messages: {report['state_count']}")

    last_state = report.get("last_state")
    if isinstance(last_state, dict) and not last_state.get("parse_error"):
        print("Last state:")
        for key in (
            "status",
            "chip_ok",
            "chip_id",
            "system_status",
            "system_error",
            "mode",
            "sample_count",
            "error_count",
        ):
            if key in last_state:
                print(f"  {key}: {last_state[key]}")

        calibration = last_state.get("calibration")
        if isinstance(calibration, dict):
            print(
                "  calibration: "
                f"sys={calibration.get('system')}/3 "
                f"gyro={calibration.get('gyro')}/3 "
                f"accel={calibration.get('accel')}/3 "
                f"mag={calibration.get('mag')}/3"
            )

    if report["reasons"]:
        print("Reasons:")
        for reason in report["reasons"]:
            print(f"  - {reason}")

    print(f"Result: {'PASS' if report['ok'] else 'CHECK'}")


if __name__ == "__main__":
    sys.exit(main())
