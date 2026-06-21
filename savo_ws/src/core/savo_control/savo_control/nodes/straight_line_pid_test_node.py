#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Python fallback straight-line PID test node."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64, String

from savo_control.adapters import twist_to_ros_msg
from savo_control.nodes.straight_line_test_helpers import (
    Pose2D,
    StraightLineConfig,
    StraightLineState,
    start_run,
    status_text,
    step_straight_line,
    stop_command,
)
from savo_control.ros import (
    CMD_VEL_AUTO,
    ODOM_FILTERED,
    SAFETY_STOP,
    bool_msg_value,
    float_msg_value,
    make_string_msg,
)
from savo_control.utils import finite_or_zero, validate_rate, validate_timeout


STRAIGHT_LINE_ENABLE_TOPIC = "/savo_control/straight_line_enable"
STRAIGHT_LINE_TARGET_TOPIC = "/savo_control/straight_line_target_m"
STRAIGHT_LINE_STATE_TOPIC = "/savo_control/straight_line_state"


class StraightLinePidTestNode(Node):
    """Diagnostic straight-line drive test using odometry feedback."""

    def __init__(self) -> None:
        super().__init__("straight_line_pid_test_node")

        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("stale_timeout_s", 0.50)
        self.declare_parameter("status_hz", 2.0)

        self.declare_parameter("odom_topic", ODOM_FILTERED)
        self.declare_parameter("cmd_vel_out_topic", CMD_VEL_AUTO)
        self.declare_parameter("enable_topic", STRAIGHT_LINE_ENABLE_TOPIC)
        self.declare_parameter("target_topic", STRAIGHT_LINE_TARGET_TOPIC)
        self.declare_parameter("state_topic", STRAIGHT_LINE_STATE_TOPIC)
        self.declare_parameter("safety_stop_topic", SAFETY_STOP)

        self.declare_parameter("auto_start", False)
        self.declare_parameter("respect_safety_stop", True)
        self.declare_parameter("publish_zero_when_disabled", True)

        self.declare_parameter("target_distance_m", 0.80)
        self.declare_parameter("goal_tolerance_m", 0.04)
        self.declare_parameter("forward_vx_m_s", 0.10)
        self.declare_parameter("max_vx_m_s", 0.14)
        self.declare_parameter("max_vy_m_s", 0.08)
        self.declare_parameter("max_wz_rad_s", 0.35)
        self.declare_parameter("lateral_kp", 0.80)
        self.declare_parameter("yaw_kp", 1.20)
        self.declare_parameter("max_duration_s", 12.0)

        self._publish_hz = validate_rate(
            self.get_parameter("publish_hz").value,
            name="publish_hz",
        )
        self._stale_timeout_s = validate_timeout(
            self.get_parameter("stale_timeout_s").value,
            name="stale_timeout_s",
        )
        if self._stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be > 0.0")

        self._status_hz = float(self.get_parameter("status_hz").value)
        if self._status_hz < 0.0:
            raise ValueError("status_hz must be >= 0.0")

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._cmd_vel_out_topic = str(self.get_parameter("cmd_vel_out_topic").value)
        self._enable_topic = str(self.get_parameter("enable_topic").value)
        self._target_topic = str(self.get_parameter("target_topic").value)
        self._state_topic = str(self.get_parameter("state_topic").value)
        self._safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)

        self._enabled = bool(self.get_parameter("auto_start").value)
        self._respect_safety_stop = bool(
            self.get_parameter("respect_safety_stop").value
        )
        self._publish_zero_when_disabled = bool(
            self.get_parameter("publish_zero_when_disabled").value
        )

        self._config = self._load_config()

        self._latest_pose: Pose2D | None = None
        self._latest_odom_stamp_s: float | None = None
        self._safety_stop = False
        self._safety_stamp_s: float | None = None

        self._run = None
        self._last_status_s = 0.0

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_out_topic, 10)
        self._state_pub = self.create_publisher(String, self._state_topic, 10)

        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(Bool, self._enable_topic, self._on_enable, 10)
        self.create_subscription(Bool, self._safety_stop_topic, self._on_safety_stop, 10)

        self.create_subscription(Float32, self._target_topic, self._on_target, 10)
        self.create_subscription(Float64, self._target_topic, self._on_target, 10)

        self._timer = self.create_timer(1.0 / self._publish_hz, self._on_timer)

        self.get_logger().info(
            "StraightLinePidTestNode started | "
            f"odom={self._odom_topic} | cmd_out={self._cmd_vel_out_topic} | "
            f"target={self._config.target_distance_m:.2f}m | "
            f"enabled={self._enabled}"
        )

    def _load_config(self) -> StraightLineConfig:
        return StraightLineConfig(
            target_distance_m=float(self.get_parameter("target_distance_m").value),
            goal_tolerance_m=float(self.get_parameter("goal_tolerance_m").value),
            forward_vx_m_s=float(self.get_parameter("forward_vx_m_s").value),
            max_vx_m_s=float(self.get_parameter("max_vx_m_s").value),
            max_vy_m_s=float(self.get_parameter("max_vy_m_s").value),
            max_wz_rad_s=float(self.get_parameter("max_wz_rad_s").value),
            lateral_kp=float(self.get_parameter("lateral_kp").value),
            yaw_kp=float(self.get_parameter("yaw_kp").value),
            max_duration_s=float(self.get_parameter("max_duration_s").value),
        ).sanitized()

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        q = pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        self._latest_pose = Pose2D(
            x=finite_or_zero(pose.position.x),
            y=finite_or_zero(pose.position.y),
            yaw=finite_or_zero(yaw),
        ).sanitized()
        self._latest_odom_stamp_s = time.monotonic()

    def _on_enable(self, msg: Bool) -> None:
        enabled = bool_msg_value(msg)

        if enabled and not self._enabled:
            if self._latest_pose is not None:
                self._run = start_run(
                    pose=self._latest_pose,
                    config=self._config,
                    now_s=time.monotonic(),
                )

        if not enabled:
            self._run = None

        self._enabled = enabled
        self.get_logger().info(f"Straight-line PID enable: {self._enabled}")

    def _on_safety_stop(self, msg: Bool) -> None:
        self._safety_stop = bool_msg_value(msg)
        self._safety_stamp_s = time.monotonic()

    def _on_target(self, msg: Float32 | Float64) -> None:
        target = max(0.0, finite_or_zero(float_msg_value(msg)))
        self._config = StraightLineConfig(
            target_distance_m=target,
            goal_tolerance_m=self._config.goal_tolerance_m,
            forward_vx_m_s=self._config.forward_vx_m_s,
            max_vx_m_s=self._config.max_vx_m_s,
            max_vy_m_s=self._config.max_vy_m_s,
            max_wz_rad_s=self._config.max_wz_rad_s,
            lateral_kp=self._config.lateral_kp,
            yaw_kp=self._config.yaw_kp,
            max_duration_s=self._config.max_duration_s,
        ).sanitized()
        self._run = None
        self.get_logger().info(f"Straight-line target updated: {target:.3f} m")

    def _on_timer(self) -> None:
        now_s = time.monotonic()

        odom_fresh = self._odom_fresh(now_s=now_s)
        safety_active = self._safety_active(now_s=now_s)

        step = step_straight_line(
            run=self._run,
            pose=self._latest_pose,
            config=self._config,
            now_s=now_s,
            enabled=self._enabled,
            safety_stop=safety_active,
            odom_fresh=odom_fresh,
        )

        self._run = step.run

        if step.finished:
            self._enabled = False

        if self._enabled or self._publish_zero_when_disabled:
            self._cmd_pub.publish(twist_to_ros_msg(step.command, msg_type=Twist))

        self._publish_status_throttled(step=step, now_s=now_s)

    def _odom_fresh(self, *, now_s: float) -> bool:
        if self._latest_odom_stamp_s is None:
            return False
        return (now_s - self._latest_odom_stamp_s) <= self._stale_timeout_s

    def _safety_active(self, *, now_s: float) -> bool:
        if not self._respect_safety_stop:
            return False

        if self._safety_stamp_s is None:
            return False

        if (now_s - self._safety_stamp_s) > self._stale_timeout_s:
            return False

        return self._safety_stop

    def _publish_status_throttled(self, *, step, now_s: float) -> None:
        if self._status_hz <= 0.0:
            return

        period_s = 1.0 / self._status_hz
        if (now_s - self._last_status_s) < period_s:
            return

        self._last_status_s = now_s

        text = status_text(
            state=step.state,
            enabled=self._enabled,
            reason=step.reason,
            forward_progress_m=step.forward_progress_m,
            lateral_error_m=step.lateral_error_m,
            yaw_error_rad=step.yaw_error_rad,
            remaining_m=step.remaining_m,
            command=step.command,
        )
        self._state_pub.publish(make_string_msg(text, msg_type=String))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StraightLinePidTestNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Exiting straight-line PID test node.")

    finally:
        try:
            node._cmd_pub.publish(
                twist_to_ros_msg(stop_command(stamp_sec=time.monotonic()), msg_type=Twist)
            )
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
