#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Echo Robot Savo wheel odometry and encoder state in a compact terminal view."""

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


WHEEL_NAMES = ("FL", "FR", "RL", "RR")


class WheelOdomEchoNode(Node):
    def __init__(
        self,
        *,
        odom_topic: str,
        state_topic: str,
        use_state: bool,
    ) -> None:
        super().__init__("wheel_odom_echo_cli")

        self.last_odom: Odometry | None = None
        self.last_odom_time_s: float | None = None
        self.odom_count = 0

        self.last_state: dict[str, Any] | None = None
        self.last_state_time_s: float | None = None
        self.state_count = 0
        self.state_parse_errors = 0

        self.create_subscription(
            Odometry,
            odom_topic,
            self._on_odom,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE),
        )

        if use_state:
            self.create_subscription(
                String,
                state_topic,
                self._on_state,
                QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE),
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
        description="Echo Robot Savo /wheel/odom and wheel encoder state."
    )
    parser.add_argument("--odom-topic", default="/wheel/odom")
    parser.add_argument(
        "--state-topic",
        default="/savo_localization/wheel_odom_state",
    )
    parser.add_argument("--rate", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--no-state", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    if args.rate <= 0.0:
        raise SystemExit("--rate must be > 0.0")

    if args.duration < 0.0:
        raise SystemExit("--duration must be >= 0.0")

    rclpy.init()

    node = WheelOdomEchoNode(
        odom_topic=args.odom_topic,
        state_topic=args.state_topic,
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
                    odom_topic=args.odom_topic,
                    state_topic=args.state_topic,
                    use_state=not args.no_state,
                )
                print_report(report, as_json=args.json)
                return 0

            if now_s < next_print_s:
                continue

            report = build_report(
                node=node,
                odom_topic=args.odom_topic,
                state_topic=args.state_topic,
                use_state=not args.no_state,
            )
            print_report(report, as_json=args.json)

            next_print_s = now_s + period_s

    finally:
        node.destroy_node()
        rclpy.shutdown()


def build_report(
    *,
    node: WheelOdomEchoNode,
    odom_topic: str,
    state_topic: str,
    use_state: bool,
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
    state_data = state_to_summary(node.last_state) if node.last_state else None

    return {
        "odom_topic": odom_topic,
        "state_topic": state_topic,
        "use_state": use_state,
        "odom_count": node.odom_count,
        "state_count": node.state_count,
        "state_parse_errors": node.state_parse_errors,
        "odom_age_s": odom_age_s,
        "state_age_s": state_age_s,
        "odom": odom_data,
        "state": state_data,
        "status": "OK" if node.last_odom is not None else "WAITING",
    }


def odom_to_dict(msg: Odometry) -> dict[str, Any]:
    qz = float(msg.pose.pose.orientation.z)
    qw = float(msg.pose.pose.orientation.w)

    yaw_rad = math.atan2(
        2.0 * qw * qz,
        1.0 - 2.0 * qz * qz,
    )

    vx = float(msg.twist.twist.linear.x)
    vy = float(msg.twist.twist.linear.y)
    omega = float(msg.twist.twist.angular.z)

    return {
        "frame_id": msg.header.frame_id,
        "child_frame_id": msg.child_frame_id,
        "pose": {
            "x_m": float(msg.pose.pose.position.x),
            "y_m": float(msg.pose.pose.position.y),
            "yaw_rad": yaw_rad,
        },
        "twist": {
            "vx_mps": vx,
            "vy_mps": vy,
            "omega_rad_s": omega,
            "linear_speed_mps": math.hypot(vx, vy),
        },
    }


def state_to_summary(state: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(state, dict):
        return None

    encoders = state.get("encoders", {})
    if not isinstance(encoders, dict):
        encoders = {}

    wheels: dict[str, dict[str, Any]] = {}

    for wheel_name in WHEEL_NAMES:
        wheel = encoders.get(wheel_name, {})
        if not isinstance(wheel, dict):
            wheel = {}

        wheels[wheel_name] = {
            "count": int(wheel.get("count", 0)),
            "delta": int(wheel.get("delta", 0)),
            "cps": float(wheel.get("cps", 0.0)),
            "speed_mps": float(wheel.get("speed_mps", 0.0)),
            "direction": int(wheel.get("direction", 0)),
            "illegal": int(wheel.get("illegal", 0)),
        }

    return {
        "status": state.get("status", "UNKNOWN"),
        "stamp_s": state.get("stamp_s"),
        "dt_s": state.get("dt_s"),
        "active_wheel_count": int(encoders.get("active_wheel_count", 0)),
        "total_illegal_transitions": int(
            encoders.get("total_illegal_transitions", 0)
        ),
        "sample_count": int(state.get("sample_count", 0)),
        "total_distance_m": float(state.get("total_distance_m", 0.0)),
        "total_rotation_rad": float(state.get("total_rotation_rad", 0.0)),
        "wheels": wheels,
    }


def print_report(report: dict[str, Any], *, as_json: bool) -> None:
    if as_json:
        print(json.dumps(report, indent=2, sort_keys=True))
        return

    print("")
    print("Robot Savo Wheel Odometry")
    print("=========================")
    print(f"Status: {report['status']}")
    print(f"Odom topic: {report['odom_topic']}")
    print(f"State topic: {report['state_topic']}")
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

        print("")
        print("Odometry:")
        print(f"  frames: {odom['frame_id']} -> {odom['child_frame_id']}")
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

    state = report.get("state")
    if isinstance(state, dict):
        print("")
        print("Encoder state:")
        print(f"  status: {state['status']}")
        print(f"  dt: {float(state.get('dt_s') or 0.0):.4f}s")
        print(f"  active wheels: {state['active_wheel_count']}")
        print(f"  illegal transitions: {state['total_illegal_transitions']}")
        print(f"  total distance: {state['total_distance_m']:.3f} m")
        print(f"  total rotation: {state['total_rotation_rad']:.3f} rad")

        print("")
        print("  Wheel table:")
        print("  wheel | count | delta | cps | speed m/s | dir | illegal")
        print("  ------|-------|-------|-----|-----------|-----|--------")

        wheels = state.get("wheels", {})
        if isinstance(wheels, dict):
            for wheel_name in WHEEL_NAMES:
                wheel = wheels.get(wheel_name, {})
                if not isinstance(wheel, dict):
                    wheel = {}

                print(
                    "  "
                    f"{wheel_name:<5} | "
                    f"{int(wheel.get('count', 0)):>5} | "
                    f"{int(wheel.get('delta', 0)):>5} | "
                    f"{float(wheel.get('cps', 0.0)):>5.1f} | "
                    f"{float(wheel.get('speed_mps', 0.0)):>9.4f} | "
                    f"{direction_symbol(int(wheel.get('direction', 0))):>3} | "
                    f"{int(wheel.get('illegal', 0)):>7}"
                )


def direction_symbol(value: int) -> str:
    if value > 0:
        return "+"

    if value < 0:
        return "-"

    return "0"


if __name__ == "__main__":
    sys.exit(main())
