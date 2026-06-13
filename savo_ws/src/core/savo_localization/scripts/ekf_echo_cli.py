#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Echo Robot Savo EKF odometry in a compact terminal view."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class EkfEchoNode(Node):
    def __init__(
        self,
        *,
        filtered_odom_topic: str,
        ekf_state_topic: str,
        use_state: bool,
    ) -> None:
        super().__init__("ekf_echo_cli")

        self.last_odom: Odometry | None = None
        self.last_odom_time_s: float | None = None
        self.odom_count = 0

        self.last_state: dict[str, Any] | None = None
        self.last_state_time_s: float | None = None
        self.state_count = 0
        self.state_parse_errors = 0

        self.create_subscription(
            Odometry,
            filtered_odom_topic,
            self._on_odom,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE),
        )

        if use_state:
            self.create_subscription(
                String,
                ekf_state_topic,
                self._on_state,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            )

    def _on_odom(self, msg: Odometry) -> None:
        self.last_odom = msg
        self.last_odom_time_s = time.monotonic()
        self.odom_count += 1

    def _on_state(self, msg: String) -> None:
        self.state_count += 1
        self.last_state_time_s = time.monotonic()

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.state_parse_errors += 1
            return

        if isinstance(data, dict):
            self.last_state = data


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Echo Robot Savo EKF filtered odometry."
    )
    parser.add_argument(
        "--filtered-odom-topic",
        default="/odometry/filtered",
    )
    parser.add_argument(
        "--ekf-state-topic",
        default="/savo_localization/ekf_state",
    )
    parser.add_argument("--rate", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--stale-timeout", type=float, default=0.5)
    parser.add_argument("--no-state", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    if args.rate <= 0.0:
        raise SystemExit("--rate must be > 0.0")

    if args.duration < 0.0:
        raise SystemExit("--duration must be >= 0.0")

    if args.stale_timeout <= 0.0:
        raise SystemExit("--stale-timeout must be > 0.0")

    rclpy.init()

    node = EkfEchoNode(
        filtered_odom_topic=args.filtered_odom_topic,
        ekf_state_topic=args.ekf_state_topic,
        use_state=not args.no_state,
    )

    period_s = 1.0 / args.rate
    next_print_s = time.monotonic()
    start_s = time.monotonic()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            now_s = time.monotonic()

            if args.duration > 0.0 and now_s - start_s >= args.duration:
                return 0

            if args.once and node.last_odom is not None:
                report = build_report(
                    node=node,
                    filtered_odom_topic=args.filtered_odom_topic,
                    ekf_state_topic=args.ekf_state_topic,
                    use_state=not args.no_state,
                    stale_timeout_s=args.stale_timeout,
                )
                print_report(report, as_json=args.json)
                return 0 if report["ok"] else 1

            if now_s < next_print_s:
                continue

            report = build_report(
                node=node,
                filtered_odom_topic=args.filtered_odom_topic,
                ekf_state_topic=args.ekf_state_topic,
                use_state=not args.no_state,
                stale_timeout_s=args.stale_timeout,
            )
            print_report(report, as_json=args.json)

            next_print_s = now_s + period_s

    finally:
        node.destroy_node()
        rclpy.shutdown()


def build_report(
    *,
    node: EkfEchoNode,
    filtered_odom_topic: str,
    ekf_state_topic: str,
    use_state: bool,
    stale_timeout_s: float,
) -> dict[str, Any]:
    now_s = time.monotonic()

    odom_age_s = (
        None
        if node.last_odom_time_s is None
        else max(0.0, now_s - node.last_odom_time_s)
    )

    state_age_s = (
        None
        if node.last_state_time_s is None
        else max(0.0, now_s - node.last_state_time_s)
    )

    odom_data = odom_to_dict(node.last_odom) if node.last_odom else None
    state_data = node.last_state if isinstance(node.last_state, dict) else None

    reasons: list[str] = []

    if node.last_odom is None:
        reasons.append(f"no messages received on {filtered_odom_topic}")
    elif odom_age_s is not None and odom_age_s > stale_timeout_s:
        reasons.append(
            f"filtered odom stale: age_s={odom_age_s:.3f} > "
            f"timeout_s={stale_timeout_s:.3f}"
        )

    if use_state and node.state_parse_errors > 0:
        reasons.append(f"{node.state_parse_errors} EKF state messages failed JSON parsing")

    if use_state and node.state_count > 0 and state_data is None:
        reasons.append("EKF state messages received but no valid JSON state available")

    status = "OK" if not reasons else "WARN"
    ok = node.last_odom is not None and not reasons

    return {
        "ok": ok,
        "status": status if node.last_odom is not None else "WAITING",
        "message": "EKF odometry healthy" if ok else "EKF echo waiting or has notes",
        "reasons": reasons,
        "filtered_odom_topic": filtered_odom_topic,
        "ekf_state_topic": ekf_state_topic,
        "use_state": use_state,
        "odom_count": node.odom_count,
        "state_count": node.state_count,
        "state_parse_errors": node.state_parse_errors,
        "odom_age_s": odom_age_s,
        "state_age_s": state_age_s,
        "stale_timeout_s": stale_timeout_s,
        "odom": odom_data,
        "state": state_data,
    }


def odom_to_dict(msg: Odometry) -> dict[str, Any]:
    qx = float(msg.pose.pose.orientation.x)
    qy = float(msg.pose.pose.orientation.y)
    qz = float(msg.pose.pose.orientation.z)
    qw = float(msg.pose.pose.orientation.w)

    yaw_rad = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )

    vx = float(msg.twist.twist.linear.x)
    vy = float(msg.twist.twist.linear.y)
    omega = float(msg.twist.twist.angular.z)

    pose_cov_diag = [
        float(msg.pose.covariance[0]),
        float(msg.pose.covariance[7]),
        float(msg.pose.covariance[35]),
    ]

    twist_cov_diag = [
        float(msg.twist.covariance[0]),
        float(msg.twist.covariance[7]),
        float(msg.twist.covariance[35]),
    ]

    return {
        "frame_id": msg.header.frame_id,
        "child_frame_id": msg.child_frame_id,
        "pose": {
            "x_m": float(msg.pose.pose.position.x),
            "y_m": float(msg.pose.pose.position.y),
            "z_m": float(msg.pose.pose.position.z),
            "yaw_rad": yaw_rad,
        },
        "twist": {
            "vx_mps": vx,
            "vy_mps": vy,
            "omega_rad_s": omega,
            "linear_speed_mps": math.hypot(vx, vy),
            "angular_speed_rad_s": abs(omega),
        },
        "covariance": {
            "pose_diag_x_y_yaw": pose_cov_diag,
            "twist_diag_vx_vy_wz": twist_cov_diag,
        },
        "finite": odom_is_finite(msg),
    }


def odom_is_finite(msg: Odometry) -> bool:
    values = [
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.angular.z,
    ]

    return all(math.isfinite(float(value)) for value in values)


def print_report(report: dict[str, Any], *, as_json: bool) -> None:
    if as_json:
        print(json.dumps(report, indent=2, sort_keys=True))
        return

    print("")
    print("Robot Savo EKF Echo")
    print("===================")
    print(f"Status: {report['status']}")
    print(f"Message: {report['message']}")
    print(f"Filtered odom topic: {report['filtered_odom_topic']}")
    print(f"EKF state topic: {report['ekf_state_topic']}")
    print(f"Odom messages: {report['odom_count']}")
    print(f"State messages: {report['state_count']}")
    print(f"State parse errors: {report['state_parse_errors']}")

    if report["odom_age_s"] is not None:
        print(f"Odom age: {report['odom_age_s']:.3f}s")

    if report["state_age_s"] is not None:
        print(f"State age: {report['state_age_s']:.3f}s")

    odom = report.get("odom")
    if isinstance(odom, dict):
        pose = odom["pose"]
        twist = odom["twist"]
        covariance = odom["covariance"]

        print("")
        print("Filtered odometry:")
        print(f"  frames: {odom['frame_id']} -> {odom['child_frame_id']}")
        print(f"  finite: {odom['finite']}")
        print(
            "  pose: "
            f"x={pose['x_m']:.3f} m "
            f"y={pose['y_m']:.3f} m "
            f"yaw={pose['yaw_rad']:.3f} rad"
        )
        print(
            "  twist: "
            f"vx={twist['vx_mps']:.3f} m/s "
            f"vy={twist['vy_mps']:.3f} m/s "
            f"omega={twist['omega_rad_s']:.3f} rad/s "
            f"speed={twist['linear_speed_mps']:.3f} m/s"
        )
        print(
            "  pose covariance diag x/y/yaw: "
            + ", ".join(f"{value:.3f}" for value in covariance["pose_diag_x_y_yaw"])
        )
        print(
            "  twist covariance diag vx/vy/wz: "
            + ", ".join(f"{value:.3f}" for value in covariance["twist_diag_vx_vy_wz"])
        )

    state = report.get("state")
    if isinstance(state, dict):
        print("")
        print("EKF state:")
        for key in (
            "status",
            "message",
            "ready",
            "usable",
            "rate_hz",
            "last_age_s",
        ):
            if key in state:
                print(f"  {key}: {state[key]}")

    if report["reasons"]:
        print("")
        print("Reasons:")
        for reason in report["reasons"]:
            print(f"  - {reason}")

    print("")
    print(f"Result: {'PASS' if report['ok'] else 'CHECK'}")


if __name__ == "__main__":
    sys.exit(main())