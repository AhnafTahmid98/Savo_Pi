#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/diagnostics/emergency_stop_check.py
----------------------------------------------------------
Professional emergency-stop diagnostic for Robot Savo base/safety pipeline.

Purpose
-------
Validate that an emergency stop signal (`/safety/stop`) is correctly propagated
through the control chain and that the base command output (`/cmd_vel_safe`)
goes to zero (or remains zero) when stop is active.

This script is NON-INTRUSIVE by default:
- It subscribes to topics and observes behavior.
- It can optionally publish a test stop pulse for validation.

What it checks
--------------
1) `/safety/stop` topic is present and updates.
2) Optional `/cmd_vel_safe` is observed.
3) When stop becomes TRUE:
   - `/cmd_vel_safe` should be zero (within epsilon), or become zero within timeout.
4) Optional recovery observation when stop becomes FALSE.

Typical use-cases
-----------------
- Bringup validation after `savo_perception` + `cmd_vel_safety_gate`
- Regression check after refactoring `savo_base` / `savo_control`
- Demo-day preflight safety verification

Examples
--------
# Passive observe only (recommended first)
ros2 run savo_base emergency_stop_check.py

# Observe custom topics
ros2 run savo_base emergency_stop_check.py --stop-topic /safety/stop --cmd-topic /cmd_vel_safe

# Publish a 1-second test stop pulse and verify cmd_vel_safe zeroing
ros2 run savo_base emergency_stop_check.py --pulse-stop --pulse-duration 1.0

# Stricter zero threshold
ros2 run savo_base emergency_stop_check.py --epsilon-lin 0.005 --epsilon-ang 0.01

Notes
-----
- This script checks EFFECT on `/cmd_vel_safe`. It does not directly command motors.
- For full end-to-end hardware validation, combine with:
  - `base_smoke_test_cli.py`
  - dashboard (`/savo_base/base_state`)
  - visual confirmation that the robot does not move
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


# =============================================================================
# Helpers
# =============================================================================
def now_mono_s() -> float:
    return float(time.monotonic())


def fmt_s(v: Optional[float]) -> str:
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return "n/a"
    return f"{v:.3f}s"


def safe_float(x) -> float:
    try:
        return float(x)
    except Exception:
        return 0.0


# =============================================================================
# Data Models
# =============================================================================
@dataclass
class BoolSample:
    value: bool = False
    t_mono: float = 0.0
    received: bool = False


@dataclass
class TwistSample:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    t_mono: float = 0.0
    received: bool = False


@dataclass
class StopEventResult:
    seen_stop_true: bool = False
    stop_true_time: Optional[float] = None
    cmd_zero_before_deadline: bool = False
    cmd_zero_time: Optional[float] = None
    cmd_nonzero_during_stop: int = 0
    timeout_hit: bool = False
    message: str = ""


# =============================================================================
# Diagnostic Node
# =============================================================================
class EmergencyStopCheckNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("emergency_stop_check")

        self.args = args

        # Runtime samples
        self.stop_sample = BoolSample()
        self.cmd_sample = TwistSample()

        self._stop_rise_count = 0
        self._stop_fall_count = 0
        self._last_stop_value: Optional[bool] = None

        self._cmd_count = 0
        self._stop_count = 0

        # Optional test publisher
        self.pub_stop_test = None
        if args.pulse_stop:
            self.pub_stop_test = self.create_publisher(Bool, args.stop_topic, 10)

        # QoS choices:
        # - safety topics are often BEST_EFFORT in your stack
        # - command topics often RELIABLE
        # Use compatible subscriptions
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscriptions
        self.sub_stop = self.create_subscription(Bool, args.stop_topic, self._on_stop, qos_sensor)
        self.sub_cmd = None
        if args.cmd_topic:
            self.sub_cmd = self.create_subscription(Twist, args.cmd_topic, self._on_cmd, qos_cmd)

        self.get_logger().info(
            "Emergency stop diagnostic started | "
            f"stop_topic={args.stop_topic} cmd_topic={args.cmd_topic or 'DISABLED'} "
            f"observe_timeout={args.observe_timeout:.2f}s reaction_timeout={args.reaction_timeout:.2f}s"
        )

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------
    def _on_stop(self, msg: Bool) -> None:
        t = now_mono_s()
        val = bool(msg.data)

        self.stop_sample.value = val
        self.stop_sample.t_mono = t
        self.stop_sample.received = True
        self._stop_count += 1

        if self._last_stop_value is None:
            self._last_stop_value = val
            return

        if (self._last_stop_value is False) and (val is True):
            self._stop_rise_count += 1
            self.get_logger().info(f"STOP rise detected (count={self._stop_rise_count})")
        elif (self._last_stop_value is True) and (val is False):
            self._stop_fall_count += 1
            self.get_logger().info(f"STOP clear detected (count={self._stop_fall_count})")

        self._last_stop_value = val

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd_sample = TwistSample(
            vx=safe_float(msg.linear.x),
            vy=safe_float(msg.linear.y),
            wz=safe_float(msg.angular.z),
            t_mono=now_mono_s(),
            received=True,
        )
        self._cmd_count += 1

    # -------------------------------------------------------------------------
    # Diagnostics logic
    # -------------------------------------------------------------------------
    def cmd_is_zero(self) -> bool:
        if not self.cmd_sample.received:
            return False
        return (
            abs(self.cmd_sample.vx) <= self.args.epsilon_lin and
            abs(self.cmd_sample.vy) <= self.args.epsilon_lin and
            abs(self.cmd_sample.wz) <= self.args.epsilon_ang
        )

    def cmd_age_s(self) -> Optional[float]:
        if not self.cmd_sample.received:
            return None
        return max(0.0, now_mono_s() - self.cmd_sample.t_mono)

    def stop_age_s(self) -> Optional[float]:
        if not self.stop_sample.received:
            return None
        return max(0.0, now_mono_s() - self.stop_sample.t_mono)

    def wait_for_topic_presence(self) -> bool:
        """
        Wait until stop topic is observed (and cmd topic too if enabled), or timeout.
        """
        deadline = now_mono_s() + self.args.observe_timeout
        rate_sleep = 1.0 / max(5.0, self.args.spin_hz)

        stop_ok = False
        cmd_ok = (self.args.cmd_topic is None)

        while rclpy.ok() and now_mono_s() < deadline:
            stop_ok = self.stop_sample.received
            if self.args.cmd_topic:
                cmd_ok = self.cmd_sample.received

            if stop_ok and cmd_ok:
                self.get_logger().info("Required topic samples received.")
                return True

            time.sleep(rate_sleep)

        if not stop_ok:
            self.get_logger().error(
                f"No messages received on stop topic '{self.args.stop_topic}' within "
                f"{self.args.observe_timeout:.2f}s"
            )
        if self.args.cmd_topic and not cmd_ok:
            self.get_logger().warn(
                f"No messages received on cmd topic '{self.args.cmd_topic}' within "
                f"{self.args.observe_timeout:.2f}s "
                "(check if upstream is publishing; stop logic can still be tested partially)."
            )
        return stop_ok

    def publish_stop_pulse(self) -> None:
        if self.pub_stop_test is None:
            return

        pulse_s = max(0.05, float(self.args.pulse_duration))
        self.get_logger().warn(
            f"Publishing TEST emergency stop pulse on {self.args.stop_topic}: TRUE for {pulse_s:.2f}s, then FALSE"
        )

        msg = Bool()
        msg.data = True
        self.pub_stop_test.publish(msg)

        t_end = now_mono_s() + pulse_s
        rate_sleep = 1.0 / max(5.0, self.args.spin_hz)
        while rclpy.ok() and now_mono_s() < t_end:
            time.sleep(rate_sleep)
            # Re-publish periodically to avoid a single-message miss
            self.pub_stop_test.publish(msg)

        msg.data = False
        self.pub_stop_test.publish(msg)

    def run_stop_reaction_check(self) -> StopEventResult:
        """
        Observe one stop-TRUE event and verify cmd_vel_safe zeroing reaction.
        """
        result = StopEventResult()

        # If already TRUE at startup, treat current state as stop-active
        if self.stop_sample.received and self.stop_sample.value:
            result.seen_stop_true = True
            result.stop_true_time = self.stop_sample.t_mono
            self.get_logger().info("Stop already TRUE at check start.")
        else:
            # Wait for rising edge / stop true
            deadline_wait_stop = now_mono_s() + self.args.wait_stop_event_timeout
            rate_sleep = 1.0 / max(5.0, self.args.spin_hz)
            while rclpy.ok() and now_mono_s() < deadline_wait_stop:
                if self.stop_sample.received and self.stop_sample.value:
                    result.seen_stop_true = True
                    result.stop_true_time = self.stop_sample.t_mono
                    self.get_logger().info("Observed STOP=TRUE event.")
                    break
                time.sleep(rate_sleep)

            if not result.seen_stop_true:
                result.timeout_hit = True
                result.message = (
                    f"Timeout waiting for STOP=TRUE on {self.args.stop_topic} "
                    f"({self.args.wait_stop_event_timeout:.2f}s)"
                )
                return result

        # If cmd topic disabled, we can only report stop event presence
        if self.args.cmd_topic is None:
            result.message = "STOP event observed (cmd_vel_safe check disabled)."
            return result

        # Check zero reaction window
        reaction_deadline = now_mono_s() + self.args.reaction_timeout
        rate_sleep = 1.0 / max(10.0, self.args.spin_hz)

        while rclpy.ok() and now_mono_s() < reaction_deadline:
            # Count non-zero samples while stop is active (if cmd is being published)
            if self.stop_sample.received and self.stop_sample.value and self.cmd_sample.received:
                if not self.cmd_is_zero():
                    result.cmd_nonzero_during_stop += 1

            if self.cmd_sample.received and self.cmd_is_zero():
                result.cmd_zero_before_deadline = True
                result.cmd_zero_time = self.cmd_sample.t_mono
                result.message = "PASS: /cmd_vel_safe is zero while stop is active."
                return result

            time.sleep(rate_sleep)

        result.timeout_hit = True
        if self.cmd_sample.received:
            result.message = (
                "FAIL: /cmd_vel_safe did not become zero before reaction timeout. "
                f"Last cmd: vx={self.cmd_sample.vx:.4f}, vy={self.cmd_sample.vy:.4f}, wz={self.cmd_sample.wz:.4f}"
            )
        else:
            result.message = (
                "INCONCLUSIVE: No /cmd_vel_safe samples received during reaction window. "
                "Stop event was observed."
            )
        return result

    def print_summary(self, result: StopEventResult) -> int:
        """
        Print professional summary and return process exit code.
        """
        print("\n" + "=" * 72)
        print("Robot SAVO Emergency Stop Diagnostic Summary")
        print("=" * 72)
        print(f"Stop topic           : {self.args.stop_topic}")
        print(f"Cmd topic            : {self.args.cmd_topic or 'DISABLED'}")
        print(f"Stop msgs received   : {self._stop_count}")
        print(f"Cmd msgs received    : {self._cmd_count}")
        print(f"STOP rises/falls     : {self._stop_rise_count}/{self._stop_fall_count}")
        print(f"Latest STOP value    : {self.stop_sample.value if self.stop_sample.received else 'n/a'}")
        print(f"Latest STOP age      : {fmt_s(self.stop_age_s())}")
        print(f"Latest CMD age       : {fmt_s(self.cmd_age_s())}")

        if self.cmd_sample.received:
            print(
                "Latest CMD (/cmd_vel_safe): "
                f"vx={self.cmd_sample.vx:.4f}, vy={self.cmd_sample.vy:.4f}, wz={self.cmd_sample.wz:.4f}"
            )
            print(
                "Zero thresholds      : "
                f"|lin|<={self.args.epsilon_lin:.4f}, |ang|<={self.args.epsilon_ang:.4f}"
            )
            print(f"CMD considered zero  : {self.cmd_is_zero()}")

        print("-" * 72)
        print(f"Result message       : {result.message}")

        # Exit code policy:
        # 0 = pass
        # 1 = fail/inconclusive hard issue
        # 2 = partial/inconclusive but stop topic observed (cmd topic missing)
        if not result.seen_stop_true:
            print("Final status         : FAIL (no STOP event observed)")
            print("=" * 72)
            return 1

        if self.args.cmd_topic is None:
            print("Final status         : PASS (STOP observed, cmd check disabled)")
            print("=" * 72)
            return 0

        if result.cmd_zero_before_deadline:
            print("Final status         : PASS")
            print("=" * 72)
            return 0

        # Inconclusive vs fail
        if self._cmd_count == 0:
            print("Final status         : INCONCLUSIVE (no cmd samples)")
            print("=" * 72)
            return 2

        print("Final status         : FAIL (cmd not zero under STOP)")
        print("=" * 72)
        return 1


# =============================================================================
# Argument Parsing
# =============================================================================
def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="emergency_stop_check.py",
        description="Robot Savo emergency stop diagnostic (observe / pulse stop and verify /cmd_vel_safe zeroing).",
    )

    # Topics
    p.add_argument("--stop-topic", default="/safety/stop", help="Emergency stop Bool topic (default: /safety/stop)")
    p.add_argument(
        "--cmd-topic",
        default="/cmd_vel_safe",
        help="Command topic to verify zeroing (Twist). Use '' to disable cmd check.",
    )

    # Timing
    p.add_argument("--observe-timeout", type=float, default=3.0, help="Time to wait for initial topic samples (s)")
    p.add_argument("--wait-stop-event-timeout", type=float, default=8.0, help="Time to wait for STOP=TRUE event (s)")
    p.add_argument("--reaction-timeout", type=float, default=1.0, help="Time allowed for cmd to become zero after STOP=TRUE (s)")
    p.add_argument("--spin-hz", type=float, default=50.0, help="Internal polling/spin helper rate (Hz)")

    # Zero thresholds
    p.add_argument("--epsilon-lin", type=float, default=0.01, help="Linear zero threshold for vx/vy (m/s)")
    p.add_argument("--epsilon-ang", type=float, default=0.02, help="Angular zero threshold for wz (rad/s)")

    # Optional active pulse test
    p.add_argument(
        "--pulse-stop",
        action="store_true",
        help="Publish a TEST stop pulse (TRUE then FALSE) on stop topic after initial topic presence check.",
    )
    p.add_argument("--pulse-duration", type=float, default=1.0, help="Duration of test STOP=TRUE pulse (s)")

    return p


# =============================================================================
# Main
# =============================================================================
def main(argv=None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    # Normalize disabled cmd topic
    if isinstance(args.cmd_topic, str) and args.cmd_topic.strip() == "":
        args.cmd_topic = None

    # Sanity clamps
    args.observe_timeout = max(0.5, float(args.observe_timeout))
    args.wait_stop_event_timeout = max(0.5, float(args.wait_stop_event_timeout))
    args.reaction_timeout = max(0.05, float(args.reaction_timeout))
    args.spin_hz = max(5.0, float(args.spin_hz))
    args.epsilon_lin = max(0.0, float(args.epsilon_lin))
    args.epsilon_ang = max(0.0, float(args.epsilon_ang))
    args.pulse_duration = max(0.05, float(args.pulse_duration))

    rclpy.init(args=None)
    node = EmergencyStopCheckNode(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    exit_code = 1
    try:
        # Start a lightweight manual spin thread pattern via timed polling loop
        t0 = now_mono_s()
        while rclpy.ok() and (now_mono_s() - t0) < 0.05:
            executor.spin_once(timeout_sec=0.01)

        # Initial topic presence check (spin while waiting)
        deadline = now_mono_s() + args.observe_timeout
        while rclpy.ok() and now_mono_s() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if node.stop_sample.received and (args.cmd_topic is None or node.cmd_sample.received):
                break

        presence_ok = node.wait_for_topic_presence()

        # Continue spinning in small steps during test windows
        if presence_ok and args.pulse_stop:
            # Give a short pre-pulse spin
            t_pre = now_mono_s() + 0.2
            while rclpy.ok() and now_mono_s() < t_pre:
                executor.spin_once(timeout_sec=0.02)
            node.publish_stop_pulse()

        # Run main reaction check with active spinning
        result = StopEventResult()
        if presence_ok:
            # We need to interleave node logic with executor spinning.
            # We'll run node logic in a lightweight staged manner.
            # Stage 1: wait for stop event
            start_wait = now_mono_s()
            if node.stop_sample.received and node.stop_sample.value:
                result.seen_stop_true = True
                result.stop_true_time = node.stop_sample.t_mono
            else:
                while rclpy.ok() and (now_mono_s() - start_wait) < args.wait_stop_event_timeout:
                    executor.spin_once(timeout_sec=0.02)
                    if node.stop_sample.received and node.stop_sample.value:
                        result.seen_stop_true = True
                        result.stop_true_time = node.stop_sample.t_mono
                        node.get_logger().info("Observed STOP=TRUE event.")
                        break

            if not result.seen_stop_true:
                result.timeout_hit = True
                result.message = (
                    f"Timeout waiting for STOP=TRUE on {args.stop_topic} "
                    f"({args.wait_stop_event_timeout:.2f}s)"
                )
            elif args.cmd_topic is None:
                result.message = "STOP event observed (cmd_vel_safe check disabled)."
            else:
                # Stage 2: verify cmd zeroing
                reaction_start = now_mono_s()
                while rclpy.ok() and (now_mono_s() - reaction_start) < args.reaction_timeout:
                    executor.spin_once(timeout_sec=0.02)

                    if node.stop_sample.received and node.stop_sample.value and node.cmd_sample.received:
                        if not node.cmd_is_zero():
                            result.cmd_nonzero_during_stop += 1

                    if node.cmd_sample.received and node.cmd_is_zero():
                        result.cmd_zero_before_deadline = True
                        result.cmd_zero_time = node.cmd_sample.t_mono
                        result.message = "PASS: /cmd_vel_safe is zero while stop is active."
                        break

                if not result.cmd_zero_before_deadline and not result.message:
                    result.timeout_hit = True
                    if node.cmd_sample.received:
                        result.message = (
                            "FAIL: /cmd_vel_safe did not become zero before reaction timeout. "
                            f"Last cmd: vx={node.cmd_sample.vx:.4f}, vy={node.cmd_sample.vy:.4f}, wz={node.cmd_sample.wz:.4f}"
                        )
                    else:
                        result.message = (
                            "INCONCLUSIVE: No /cmd_vel_safe samples received during reaction window. "
                            "Stop event was observed."
                        )
        else:
            result.message = "Initial topic presence check failed (see logs)."

        exit_code = node.print_summary(result)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        exit_code = 130
    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()

    return int(exit_code)


if __name__ == "__main__":
    sys.exit(main())