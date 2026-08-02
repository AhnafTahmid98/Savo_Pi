#!/usr/bin/env python3
"""Validate Robot SAVO pan/tilt state and optionally command approved ROS motion."""
from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import math
import time
from datetime import UTC, datetime

from tools.diag.infra.diag_utils import emit_result, parser
from tools.diag.infra.ros_probe import collect, compact


def _read_only(args, started: float, started_utc: str) -> int:
    try:
        probe = collect(
            "/savo_head/pan_tilt_state",
            "sensor_msgs/msg/JointState",
            duration_s=args.timeout,
            minimum_samples=1,
            reliable=False,
            maximum_samples=100,
        )
        details = compact(probe)
        changed = False
        if len(probe.samples) >= 2:
            first = probe.samples[0].get("position", [])
            changed = any(sample.get("position", []) != first for sample in probe.samples[1:])
        details["position_changed"] = changed
        if probe.publisher_count == 0:
            status, reason = "BLOCKED", "head_state_publisher_missing"
        elif not probe.samples:
            status, reason = "FAIL", "head_state_not_observed"
        elif args.require_change and not changed:
            status, reason = "FAIL", "operator_head_motion_not_observed"
        else:
            status, reason = "PASS", "head_state_observed"
    except (ImportError, RuntimeError, ValueError) as exc:
        status, reason, details = "BLOCKED", "ros_probe_unavailable", {"error": str(exc)}
    return emit_result(
        "head_pan_tilt",
        status,
        reason,
        details,
        output=args.output,
        started=started,
        started_utc=started_utc,
    )


def _position(message) -> tuple[float, float] | None:
    if len(message.position) < 2:
        return None
    # The production state uses degrees even though JointState usually carries radians.
    return float(message.position[0]), float(message.position[1])


def _motion(args, started: float, started_utc: str) -> int:
    if not args.allow_motion or not args.head_clear:
        return emit_result(
            "head_pan_tilt",
            "BLOCKED",
            "requires_allow_motion_and_head_clear",
            {},
            output=args.output,
            started=started,
            started_utc=started_utc,
        )
    if args.pan is None or args.tilt is None:
        return emit_result(
            "head_pan_tilt",
            "FAIL",
            "pan_and_tilt_must_be_provided_together",
            {},
            output=args.output,
            started=started,
            started_utc=started_utc,
        )
    if not 0.0 <= args.pan <= 170.0 or not 0.0 <= args.tilt <= 130.0:
        return emit_result(
            "head_pan_tilt",
            "FAIL",
            "target_out_of_robot_savo_bounds",
            {"pan": args.pan, "tilt": args.tilt},
            output=args.output,
            started=started,
            started_utc=started_utc,
        )

    try:
        import rclpy
        from geometry_msgs.msg import Vector3
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except ImportError as exc:
        return emit_result(
            "head_pan_tilt",
            "BLOCKED",
            "ros_runtime_unavailable",
            {"error": str(exc)},
            output=args.output,
            started=started,
            started_utc=started_utc,
        )

    rclpy.init(args=None)
    node = Node("robot_savo_head_pan_tilt_test")
    last_state: dict[str, object] = {"position": None}

    def on_state(message: JointState) -> None:
        value = _position(message)
        if value is not None:
            last_state["position"] = value

    subscription = node.create_subscription(
        JointState, "/savo_head/pan_tilt_state", on_state, 10
    )
    publisher = node.create_publisher(Vector3, "/savo_head/pan_tilt_cmd", 10)
    del subscription

    details: dict[str, object] = {
        "target_pan_deg": args.pan,
        "target_tilt_deg": args.tilt,
        "return_center": not args.leave_at_target,
    }
    status, reason = "BLOCKED", "head_controller_unavailable"
    try:
        deadline = time.monotonic() + min(3.0, args.timeout)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if publisher.get_subscription_count() > 0 and last_state["position"] is not None:
                break
        details["command_subscribers"] = publisher.get_subscription_count()
        details["initial_position"] = last_state["position"]
        if publisher.get_subscription_count() == 0:
            status, reason = "BLOCKED", "head_controller_command_subscription_missing"
        elif last_state["position"] is None:
            status, reason = "BLOCKED", "head_state_not_available_before_motion"
        else:
            command = Vector3()
            command.x = float(args.pan)
            command.y = float(args.tilt)
            command.z = 0.0  # production controller absolute-command mode
            for _ in range(5):
                publisher.publish(command)
                rclpy.spin_once(node, timeout_sec=0.05)
            deadline = time.monotonic() + args.timeout
            reached = False
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                observed = last_state["position"]
                if isinstance(observed, tuple):
                    if math.isclose(observed[0], args.pan, abs_tol=args.tolerance) and math.isclose(
                        observed[1], args.tilt, abs_tol=args.tolerance
                    ):
                        reached = True
                        break
            details["observed_position"] = last_state["position"]
            if reached:
                status, reason = "PASS", "approved_head_command_reached_target"
            else:
                status, reason = "FAIL", "approved_head_command_target_not_observed"
    finally:
        if not args.leave_at_target and publisher.get_subscription_count() > 0:
            center = Vector3()
            center.x = 72.0
            center.y = 55.0
            center.z = 0.0
            for _ in range(5):
                publisher.publish(center)
                rclpy.spin_once(node, timeout_sec=0.05)
            details["center_command_sent"] = True
        node.destroy_node()
        rclpy.shutdown()

    return emit_result(
        "head_pan_tilt",
        status,
        reason,
        details,
        output=args.output,
        started=started,
        started_utc=started_utc,
    )


def main() -> int:
    value = parser(__doc__, motion=True)
    value.add_argument("--require-change", action="store_true")
    value.add_argument("--head-clear", action="store_true")
    value.add_argument("--pan", type=float)
    value.add_argument("--tilt", type=float)
    value.add_argument("--tolerance", type=float, default=3.0)
    value.add_argument("--leave-at-target", action="store_true")
    args = value.parse_args()
    started = time.monotonic()
    started_utc = datetime.now(UTC).isoformat()
    if args.tolerance <= 0.0 or args.tolerance > 15.0:
        return emit_result(
            "head_pan_tilt",
            "FAIL",
            "invalid_tolerance",
            {"tolerance": args.tolerance},
            output=args.output,
            started=started,
            started_utc=started_utc,
        )
    if args.pan is not None or args.tilt is not None:
        return _motion(args, started, started_utc)
    if args.require_change and not args.allow_motion:
        return emit_result(
            "head_pan_tilt",
            "BLOCKED",
            "motion_not_authorized_use_allow_motion",
            {},
            output=args.output,
            started=started,
            started_utc=started_utc,
        )
    return _read_only(args, started, started_utc)


if __name__ == "__main__":
    raise SystemExit(main())
