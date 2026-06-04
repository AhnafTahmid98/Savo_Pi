#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Savo — savo_control / recovery_test_manager_node.py
=========================================================

Recovery test manager for Robot Savo.

Purpose
-------
This node runs controlled recovery test sequences and publishes recovery
velocity commands to:

    /cmd_vel_recovery

Correct command chain:

    recovery_test_manager_node
        -> /cmd_vel_recovery
        -> twist_mux_node
        -> /cmd_vel_mux
        -> cmd_vel_shaper_node
        -> /cmd_vel
        -> savo_perception/cmd_vel_safety_gate
        -> /cmd_vel_safe
        -> savo_base/base_driver_node
        -> motors

Architecture rules
------------------
- This node does NOT publish directly to /cmd_vel_safe.
- This node does NOT control hardware directly.
- This node does NOT touch PCA9685, GPIO, PWM, Freenove board, or motors.
- The safety gate and savo_base watchdog always have final authority.

Test control
------------
The node listens for recovery test commands on:

    /savo_control/recovery_test_cmd

Message type:

    std_msgs/String

Supported commands:
    BACKUP_ONLY
    ROTATE_LEFT
    ROTATE_RIGHT
    BACKUP_THEN_ROTATE_LEFT
    BACKUP_THEN_ROTATE_RIGHT
    STOP

Example:
    ros2 topic pub /savo_control/recovery_test_cmd std_msgs/msg/String "{data: 'BACKUP_ONLY'}" --once

Safety
------
First test with wheels lifted. Then test on open floor only.
Keep recovery velocities very low.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class RecoveryTestState(str, Enum):
    IDLE = "IDLE"
    PRE_STOP = "PRE_STOP"
    RUNNING = "RUNNING"
    POST_STOP = "POST_STOP"
    FINISHED = "FINISHED"
    SAFETY_STOP = "SAFETY_STOP"
    UNKNOWN_TEST = "UNKNOWN_TEST"
    TIMEOUT = "TIMEOUT"
    DISABLED = "DISABLED"


@dataclass
class RecoveryStage:
    name: str
    duration_s: float
    vx: float
    vy: float
    wz: float


@dataclass
class ActiveRecoveryTest:
    name: str
    stages: List[RecoveryStage]
    start_time: float
    stage_start_time: float
    current_stage_index: int = 0


class RecoveryTestManagerNode(Node):
    """Controlled recovery test sequence manager."""

    def __init__(self) -> None:
        super().__init__("recovery_test_manager_node")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("enabled", True)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("default_test_name", "BACKUP_ONLY")

        # Topics
        self.declare_parameter("cmd_vel_out_topic", "/cmd_vel_recovery")
        self.declare_parameter("test_cmd_topic", "/savo_control/recovery_test_cmd")
        self.declare_parameter("mode_cmd_topic", "/savo_control/mode_cmd")
        self.declare_parameter("safety_stop_topic", "/safety/stop")
        self.declare_parameter("status_topic", "/savo_control/recovery_test_status")

        # Mode behavior
        self.declare_parameter("request_recovery_mode_on_start", True)
        self.declare_parameter("required_mode", "RECOVERY")
        self.declare_parameter("return_to_stop_on_finish", True)

        # Timing
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("pre_stop_hold_s", 0.50)
        self.declare_parameter("post_stop_hold_s", 0.80)
        self.declare_parameter("max_test_duration_s", 4.0)
        self.declare_parameter("shutdown_zero_count", 5)

        # Recovery command values
        self.declare_parameter("backup_vx", -0.08)
        self.declare_parameter("rotate_wz", 0.25)
        self.declare_parameter("backup_duration_s", 0.80)
        self.declare_parameter("rotate_duration_s", 0.70)
        self.declare_parameter("between_stage_stop_s", 0.40)

        # Hard command limits
        self.declare_parameter("max_abs_vx", 0.10)
        self.declare_parameter("max_abs_vy", 0.00)
        self.declare_parameter("max_abs_wz", 0.28)

        # Safety
        self.declare_parameter("respect_safety_stop", True)
        self.declare_parameter("block_start_if_safety_stop_true", True)
        self.declare_parameter("zero_on_safety_stop", True)
        self.declare_parameter("allow_direct_cmd_vel_safe_publish", False)

        # Diagnostics
        self.declare_parameter("publish_status", True)
        self.declare_parameter("status_hz", 5.0)
        self.declare_parameter("log_throttle_s", 2.0)

        self._load_parameters()
        self._validate_parameters()

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_out_topic, 10)
        self._mode_pub = self.create_publisher(String, self._mode_cmd_topic, 10)

        self._status_pub = None
        if self._publish_status:
            self._status_pub = self.create_publisher(String, self._status_topic, 10)

        self.create_subscription(String, self._test_cmd_topic, self._on_test_cmd, 10)
        self.create_subscription(Bool, self._safety_stop_topic, self._on_safety_stop, 10)

        self._timer = self.create_timer(1.0 / self._publish_hz, self._on_timer)

        # ------------------------------------------------------------------
        # Runtime state
        # ------------------------------------------------------------------
        self._state = RecoveryTestState.IDLE if self._enabled else RecoveryTestState.DISABLED
        self._active_test: Optional[ActiveRecoveryTest] = None
        self._pending_test_name: Optional[str] = None

        self._safety_stop = False
        self._pre_stop_until: Optional[float] = None
        self._post_stop_until: Optional[float] = None

        self._last_status_time = 0.0
        self._last_log_time = 0.0

        if self._auto_start and self._enabled:
            self._request_start_test(self._default_test_name)

        self.get_logger().info(
            "RecoveryTestManagerNode started | "
            f"output={self._cmd_vel_out_topic} | cmd_topic={self._test_cmd_topic} | "
            f"default={self._default_test_name} | auto_start={self._auto_start}"
        )

    # ----------------------------------------------------------------------
    # Parameter loading
    # ----------------------------------------------------------------------
    def _load_parameters(self) -> None:
        self._enabled = bool(self.get_parameter("enabled").value)
        self._auto_start = bool(self.get_parameter("auto_start").value)
        self._default_test_name = str(self.get_parameter("default_test_name").value).upper()

        self._cmd_vel_out_topic = str(self.get_parameter("cmd_vel_out_topic").value)
        self._test_cmd_topic = str(self.get_parameter("test_cmd_topic").value)
        self._mode_cmd_topic = str(self.get_parameter("mode_cmd_topic").value)
        self._safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)
        self._status_topic = str(self.get_parameter("status_topic").value)

        self._request_recovery_mode_on_start = bool(
            self.get_parameter("request_recovery_mode_on_start").value
        )
        self._required_mode = str(self.get_parameter("required_mode").value)
        self._return_to_stop_on_finish = bool(
            self.get_parameter("return_to_stop_on_finish").value
        )

        self._publish_hz = float(self.get_parameter("publish_hz").value)
        self._pre_stop_hold_s = float(self.get_parameter("pre_stop_hold_s").value)
        self._post_stop_hold_s = float(self.get_parameter("post_stop_hold_s").value)
        self._max_test_duration_s = float(self.get_parameter("max_test_duration_s").value)
        self._shutdown_zero_count = int(self.get_parameter("shutdown_zero_count").value)

        self._backup_vx = float(self.get_parameter("backup_vx").value)
        self._rotate_wz = float(self.get_parameter("rotate_wz").value)
        self._backup_duration_s = float(self.get_parameter("backup_duration_s").value)
        self._rotate_duration_s = float(self.get_parameter("rotate_duration_s").value)
        self._between_stage_stop_s = float(self.get_parameter("between_stage_stop_s").value)

        self._max_abs_vx = abs(float(self.get_parameter("max_abs_vx").value))
        self._max_abs_vy = abs(float(self.get_parameter("max_abs_vy").value))
        self._max_abs_wz = abs(float(self.get_parameter("max_abs_wz").value))

        self._respect_safety_stop = bool(self.get_parameter("respect_safety_stop").value)
        self._block_start_if_safety_stop_true = bool(
            self.get_parameter("block_start_if_safety_stop_true").value
        )
        self._zero_on_safety_stop = bool(self.get_parameter("zero_on_safety_stop").value)
        self._allow_direct_cmd_vel_safe_publish = bool(
            self.get_parameter("allow_direct_cmd_vel_safe_publish").value
        )

        self._publish_status = bool(self.get_parameter("publish_status").value)
        self._status_hz = float(self.get_parameter("status_hz").value)
        self._log_throttle_s = float(self.get_parameter("log_throttle_s").value)

    def _validate_parameters(self) -> None:
        if self._publish_hz <= 0.0:
            raise ValueError("publish_hz must be > 0")

        if self._max_test_duration_s <= 0.0:
            raise ValueError("max_test_duration_s must be > 0")

        if self._pre_stop_hold_s < 0.0:
            raise ValueError("pre_stop_hold_s must be >= 0")

        if self._post_stop_hold_s < 0.0:
            raise ValueError("post_stop_hold_s must be >= 0")

        if self._backup_duration_s <= 0.0:
            raise ValueError("backup_duration_s must be > 0")

        if self._rotate_duration_s <= 0.0:
            raise ValueError("rotate_duration_s must be > 0")

        if self._between_stage_stop_s < 0.0:
            raise ValueError("between_stage_stop_s must be >= 0")

        if self._max_abs_vx <= 0.0:
            raise ValueError("max_abs_vx must be > 0")

        if self._max_abs_wz <= 0.0:
            raise ValueError("max_abs_wz must be > 0")

        if self._allow_direct_cmd_vel_safe_publish:
            raise ValueError(
                "allow_direct_cmd_vel_safe_publish must remain false in savo_control"
            )

        # Recovery test must publish only to the recovery command source.
        if self._cmd_vel_out_topic == "/cmd_vel_safe":
            raise ValueError("recovery_test_manager_node must not publish to /cmd_vel_safe")

    # ----------------------------------------------------------------------
    # Subscribers
    # ----------------------------------------------------------------------
    def _on_test_cmd(self, msg: String) -> None:
        name = str(msg.data).strip().upper()

        if not name:
            return

        if name in ("STOP", "CANCEL", "ABORT"):
            self._stop_test(reason="operator_stop")
            return

        self._request_start_test(name)

    def _on_safety_stop(self, msg: Bool) -> None:
        self._safety_stop = bool(msg.data)

    # ----------------------------------------------------------------------
    # Test lifecycle
    # ----------------------------------------------------------------------
    def _request_start_test(self, name: str) -> None:
        if not self._enabled:
            self._state = RecoveryTestState.DISABLED
            self._publish_zero()
            self._publish_status()
            return

        if self._block_start_if_safety_stop_true and self._safety_stop:
            self._state = RecoveryTestState.SAFETY_STOP
            self._publish_zero()
            self._publish_status()
            self.get_logger().warning("Refusing recovery test because safety_stop=True")
            return

        stages = self._build_test_stages(name)
        if not stages:
            self._state = RecoveryTestState.UNKNOWN_TEST
            self._publish_zero()
            self._publish_status()
            self.get_logger().error(f"Unknown recovery test: {name}")
            return

        self._pending_test_name = name

        if self._pre_stop_hold_s > 0.0:
            self._state = RecoveryTestState.PRE_STOP
            self._pre_stop_until = time.monotonic() + self._pre_stop_hold_s
            self._publish_zero()
        else:
            self._start_test_now(name)

    def _start_test_now(self, name: str) -> None:
        stages = self._build_test_stages(name)
        if not stages:
            self._state = RecoveryTestState.UNKNOWN_TEST
            self._publish_zero()
            return

        if self._request_recovery_mode_on_start:
            mode_msg = String()
            mode_msg.data = self._required_mode
            self._mode_pub.publish(mode_msg)

        now = time.monotonic()
        self._active_test = ActiveRecoveryTest(
            name=name,
            stages=stages,
            start_time=now,
            stage_start_time=now,
            current_stage_index=0,
        )

        self._state = RecoveryTestState.RUNNING
        self.get_logger().info(f"Started recovery test: {name}")

    def _stop_test(self, reason: str = "stop") -> None:
        self._active_test = None
        self._pending_test_name = None
        self._pre_stop_until = None
        self._post_stop_until = None
        self._state = RecoveryTestState.FINISHED
        self._publish_zero()

        if self._return_to_stop_on_finish:
            self._publish_mode("STOP")

        self.get_logger().info(f"Recovery test stopped: {reason}")

    def _finish_test(self) -> None:
        name = self._active_test.name if self._active_test else "unknown"
        self._active_test = None
        self._publish_zero()

        if self._post_stop_hold_s > 0.0:
            self._state = RecoveryTestState.POST_STOP
            self._post_stop_until = time.monotonic() + self._post_stop_hold_s
        else:
            self._state = RecoveryTestState.FINISHED
            if self._return_to_stop_on_finish:
                self._publish_mode("STOP")

        self.get_logger().info(f"Recovery test finished: {name}")

    # ----------------------------------------------------------------------
    # Stage construction
    # ----------------------------------------------------------------------
    def _build_test_stages(self, name: str) -> List[RecoveryStage]:
        backup = RecoveryStage(
            name="backup",
            duration_s=self._backup_duration_s,
            vx=self._limit_vx(self._backup_vx),
            vy=0.0,
            wz=0.0,
        )

        stop_between = RecoveryStage(
            name="stop",
            duration_s=self._between_stage_stop_s,
            vx=0.0,
            vy=0.0,
            wz=0.0,
        )

        rotate_left = RecoveryStage(
            name="rotate_left",
            duration_s=self._rotate_duration_s,
            vx=0.0,
            vy=0.0,
            wz=self._limit_wz(abs(self._rotate_wz)),
        )

        rotate_right = RecoveryStage(
            name="rotate_right",
            duration_s=self._rotate_duration_s,
            vx=0.0,
            vy=0.0,
            wz=self._limit_wz(-abs(self._rotate_wz)),
        )

        if name == "BACKUP_ONLY":
            return [backup]

        if name == "ROTATE_LEFT":
            return [rotate_left]

        if name == "ROTATE_RIGHT":
            return [rotate_right]

        if name == "BACKUP_THEN_ROTATE_LEFT":
            return [backup, stop_between, rotate_left]

        if name == "BACKUP_THEN_ROTATE_RIGHT":
            return [backup, stop_between, rotate_right]

        if name == "STOP_ONLY":
            return [stop_between]

        return []

    # ----------------------------------------------------------------------
    # Timer
    # ----------------------------------------------------------------------
    def _on_timer(self) -> None:
        if not self._enabled:
            self._state = RecoveryTestState.DISABLED
            self._publish_zero()
            self._publish_status_if_needed()
            return

        if self._respect_safety_stop and self._safety_stop:
            if self._zero_on_safety_stop:
                self._publish_zero()
            self._state = RecoveryTestState.SAFETY_STOP
            self._active_test = None
            self._pending_test_name = None
            self._publish_status_if_needed()
            return

        if self._state == RecoveryTestState.PRE_STOP:
            self._publish_zero()
            if self._pre_stop_until is not None and time.monotonic() >= self._pre_stop_until:
                pending = self._pending_test_name
                self._pending_test_name = None
                self._pre_stop_until = None
                if pending:
                    self._start_test_now(pending)
                else:
                    self._state = RecoveryTestState.IDLE
            self._publish_status_if_needed()
            return

        if self._state == RecoveryTestState.POST_STOP:
            self._publish_zero()
            if self._post_stop_until is not None and time.monotonic() >= self._post_stop_until:
                self._post_stop_until = None
                self._state = RecoveryTestState.FINISHED
                if self._return_to_stop_on_finish:
                    self._publish_mode("STOP")
            self._publish_status_if_needed()
            return

        if self._active_test is None:
            self._publish_zero()
            self._publish_status_if_needed()
            return

        total_elapsed = time.monotonic() - self._active_test.start_time
        if total_elapsed > self._max_test_duration_s:
            self._state = RecoveryTestState.TIMEOUT
            self._active_test = None
            self._publish_zero()
            if self._return_to_stop_on_finish:
                self._publish_mode("STOP")
            self._publish_status_if_needed()
            self._throttled_warn("Recovery test timeout. Publishing zero.")
            return

        self._run_active_stage()
        self._publish_status_if_needed()

    def _run_active_stage(self) -> None:
        assert self._active_test is not None

        now = time.monotonic()
        stage = self._active_test.stages[self._active_test.current_stage_index]

        if (now - self._active_test.stage_start_time) <= stage.duration_s:
            self._publish_cmd(stage.vx, stage.vy, stage.wz)
            return

        self._active_test.current_stage_index += 1

        if self._active_test.current_stage_index >= len(self._active_test.stages):
            self._finish_test()
            return

        self._active_test.stage_start_time = now
        next_stage = self._active_test.stages[self._active_test.current_stage_index]
        self.get_logger().info(
            f"Recovery test '{self._active_test.name}' stage -> {next_stage.name}"
        )
        self._publish_cmd(next_stage.vx, next_stage.vy, next_stage.wz)

    # ----------------------------------------------------------------------
    # Publishing
    # ----------------------------------------------------------------------
    def _publish_cmd(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = self._safe_float(self._limit_vx(vx))
        msg.linear.y = self._safe_float(self._limit_vy(vy))
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self._safe_float(self._limit_wz(wz))
        self._cmd_pub.publish(msg)

    def _publish_zero(self) -> None:
        self._publish_cmd(0.0, 0.0, 0.0)

    def _publish_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)

    def _publish_status_if_needed(self) -> None:
        if not self._publish_status or self._status_pub is None:
            return

        now = time.monotonic()
        period = 1.0 / max(0.1, self._status_hz)
        if (now - self._last_status_time) < period:
            return

        self._last_status_time = now
        self._publish_status()

    def _publish_status(self) -> None:
        if self._active_test is not None:
            active = self._active_test.name
            stage = self._active_test.current_stage_index
            stage_name = self._active_test.stages[stage].name
        else:
            active = "none"
            stage = -1
            stage_name = "none"

        msg = String()
        msg.data = (
            f"state={self._state.value}; "
            f"active_test={active}; "
            f"stage_index={stage}; "
            f"stage_name={stage_name}; "
            f"safety_stop={self._safety_stop}; "
            f"output={self._cmd_vel_out_topic}"
        )
        self._status_pub.publish(msg)

    # ----------------------------------------------------------------------
    # Limits / helpers
    # ----------------------------------------------------------------------
    def _limit_vx(self, value: float) -> float:
        return self._clamp(self._safe_float(value), -self._max_abs_vx, self._max_abs_vx)

    def _limit_vy(self, value: float) -> float:
        return self._clamp(self._safe_float(value), -self._max_abs_vy, self._max_abs_vy)

    def _limit_wz(self, value: float) -> float:
        return self._clamp(self._safe_float(value), -self._max_abs_wz, self._max_abs_wz)

    @staticmethod
    def _safe_float(value: float) -> float:
        value = float(value)
        if not math.isfinite(value):
            return 0.0
        return value

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _throttled_warn(self, text: str) -> None:
        now = time.monotonic()
        if (now - self._last_log_time) >= self._log_throttle_s:
            self._last_log_time = now
            self.get_logger().warning(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecoveryTestManagerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Sending zero recovery command.")

    finally:
        try:
            for _ in range(max(1, node._shutdown_zero_count)):
                node._publish_zero()
                time.sleep(0.05)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()