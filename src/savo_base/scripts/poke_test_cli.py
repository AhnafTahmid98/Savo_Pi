#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/scripts/poke_test_cli.py
-----------------------------------------------
Professional ROS 2 Jazzy CLI utility for *short motion poke tests* of the base stack.

Purpose
-------
This tool publishes brief Twist commands (small "pokes") to verify:
- base_driver_node command path is alive
- mecanum wheel direction/sign conventions
- motor board writes are happening
- safety stop topic behavior (optional quick test)
- slowdown factor effect (optional)

It is intentionally short-duration and low-speed by default.

Typical use cases
-----------------
- After bringup of `savo_base/base_driver_node.py`
- After changing invert flags / sign conventions
- After wiring/motor board changes
- During dry-run backend validation (no hardware movement expected)

Default behavior
----------------
Publishes short pulses on:
1) +vx  (forward test in robot command frame)
2) -vx
3) +vy  (strafe left/right depending conventions)
4) -vy
5) +wz  (CCW/CW depending conventions)
6) -wz

Safety notes
------------
- Keep robot lifted or wheels off ground for first test after changes.
- Start with low amplitudes.
- This script does NOT bypass your `cmd_vel_safety_gate`; it should publish to /cmd_vel_safe by default.
- Optional `--test-safety-stop` briefly toggles /safety/stop and confirms motor stop behavior via observation.

Examples
--------
# Default quick poke sequence
ros2 run savo_base poke_test_cli.py

# Publish to raw /cmd_vel (only if you intentionally want that path)
ros2 run savo_base poke_test_cli.py --cmd-topic /cmd_vel

# Smaller, safer amplitudes
ros2 run savo_base poke_test_cli.py --vx 0.08 --vy 0.08 --wz 0.15 --pulse 0.25

# Test only rotation pokes
ros2 run savo_base poke_test_cli.py --only rotate

# Include safety stop toggle test
ros2 run savo_base poke_test_cli.py --test-safety-stop

# Apply slowdown factor during test (if base_driver_node uses slowdown topic)
ros2 run savo_base poke_test_cli.py --set-slowdown 0.40
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


# -----------------------------------------------------------------------------
# Optional topic names helper (falls back to canonical defaults)
# -----------------------------------------------------------------------------
try:
    from savo_base.utils.topic_names import (
        CMD_VEL_SAFE as TOPIC_CMD_VEL_SAFE,
        SAFETY_STOP as TOPIC_SAFETY_STOP,
        SAFETY_SLOWDOWN_FACTOR as TOPIC_SAFETY_SLOWDOWN_FACTOR,
    )
except Exception:
    TOPIC_CMD_VEL_SAFE = "/cmd_vel_safe"
    TOPIC_SAFETY_STOP = "/safety/stop"
    TOPIC_SAFETY_SLOWDOWN_FACTOR = "/safety/slowdown_factor"


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------
def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def make_twist(vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> Twist:
    msg = Twist()
    msg.linear.x = float(vx)
    msg.linear.y = float(vy)
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = float(wz)
    return msg


# -----------------------------------------------------------------------------
# CLI node
# -----------------------------------------------------------------------------
class PokeTestCli(Node):
    def __init__(
        self,
        *,
        cmd_topic: str,
        safety_stop_topic: str,
        slowdown_topic: str,
        publish_hz: float,
    ) -> None:
        super().__init__("poke_test_cli")

        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub_cmd = self.create_publisher(Twist, cmd_topic, qos_cmd)
        self.pub_stop = self.create_publisher(Bool, safety_stop_topic, qos_sensor)
        self.pub_slowdown = self.create_publisher(Float32, slowdown_topic, qos_sensor)

        self.cmd_topic = cmd_topic
        self.safety_stop_topic = safety_stop_topic
        self.slowdown_topic = slowdown_topic
        self.publish_hz = max(5.0, float(publish_hz))
        self.dt = 1.0 / self.publish_hz

        self.get_logger().info(
            f"poke_test_cli ready | cmd={self.cmd_topic} stop={self.safety_stop_topic} "
            f"slowdown={self.slowdown_topic} publish_hz={self.publish_hz:.1f}"
        )

    # ---------------------------- publish primitives ---------------------------
    def publish_twist(self, vx: float, vy: float, wz: float) -> None:
        self.pub_cmd.publish(make_twist(vx=vx, vy=vy, wz=wz))

    def publish_zero(self) -> None:
        self.publish_twist(0.0, 0.0, 0.0)

    def publish_stop(self, stop: bool) -> None:
        msg = Bool()
        msg.data = bool(stop)
        self.pub_stop.publish(msg)

    def publish_slowdown(self, factor: float) -> None:
        msg = Float32()
        msg.data = float(factor)
        self.pub_slowdown.publish(msg)

    # ----------------------------- timing helpers -----------------------------
    def spin_sleep(self, executor: SingleThreadedExecutor, seconds: float) -> None:
        end_t = time.monotonic() + max(0.0, float(seconds))
        while rclpy.ok() and time.monotonic() < end_t:
            executor.spin_once(timeout_sec=min(0.05, self.dt))

    def hold_command(
        self,
        executor: SingleThreadedExecutor,
        *,
        vx: float,
        vy: float,
        wz: float,
        duration_s: float,
    ) -> None:
        """
        Re-publish command during the pulse to avoid watchdog timeout
        in base_driver_node (default timeout is ~0.30s).
        """
        end_t = time.monotonic() + max(0.0, float(duration_s))
        while rclpy.ok() and time.monotonic() < end_t:
            self.publish_twist(vx, vy, wz)
            executor.spin_once(timeout_sec=0.0)
            time.sleep(self.dt)

    def send_zero_for(
        self, executor: SingleThreadedExecutor, duration_s: float
    ) -> None:
        end_t = time.monotonic() + max(0.0, float(duration_s))
        while rclpy.ok() and time.monotonic() < end_t:
            self.publish_zero()
            executor.spin_once(timeout_sec=0.0)
            time.sleep(self.dt)


# -----------------------------------------------------------------------------
# Sequence building
# -----------------------------------------------------------------------------
PokeStep = Tuple[str, float, float, float]


def build_sequence(mode: str, vx: float, vy: float, wz: float) -> List[PokeStep]:
    mode = str(mode).strip().lower()

    all_steps: List[PokeStep] = [
        ("vx+", +vx, 0.0, 0.0),
        ("vx-", -vx, 0.0, 0.0),
        ("vy+", 0.0, +vy, 0.0),
        ("vy-", 0.0, -vy, 0.0),
        ("wz+", 0.0, 0.0, +wz),
        ("wz-", 0.0, 0.0, -wz),
    ]

    if mode in ("all", "full"):
        return all_steps
    if mode in ("forward", "vx"):
        return [("vx+", +vx, 0.0, 0.0), ("vx-", -vx, 0.0, 0.0)]
    if mode in ("strafe", "vy"):
        return [("vy+", 0.0, +vy, 0.0), ("vy-", 0.0, -vy, 0.0)]
    if mode in ("rotate", "wz", "yaw"):
        return [("wz+", 0.0, 0.0, +wz), ("wz-", 0.0, 0.0, -wz)]

    raise ValueError(f"Unsupported --only mode: {mode}")


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="poke_test_cli.py",
        description="Short motion poke tests for Robot Savo base_driver_node.",
    )

    # Topics
    p.add_argument("--cmd-topic", default=TOPIC_CMD_VEL_SAFE, help=f"Twist topic (default: {TOPIC_CMD_VEL_SAFE})")
    p.add_argument("--safety-stop-topic", default=TOPIC_SAFETY_STOP, help=f"Safety stop topic (default: {TOPIC_SAFETY_STOP})")
    p.add_argument("--slowdown-topic", default=TOPIC_SAFETY_SLOWDOWN_FACTOR, help=f"Slowdown factor topic (default: {TOPIC_SAFETY_SLOWDOWN_FACTOR})")

    # Sequence / amplitude
    p.add_argument("--only", default="all", choices=["all", "forward", "strafe", "rotate"], help="Run subset of poke sequence")
    p.add_argument("--vx", type=float, default=0.12, help="Amplitude for vx poke (default: 0.12)")
    p.add_argument("--vy", type=float, default=0.12, help="Amplitude for vy poke (default: 0.12)")
    p.add_argument("--wz", type=float, default=0.20, help="Amplitude for wz poke (default: 0.20)")

    # Timing
    p.add_argument("--pulse", type=float, default=0.30, help="Pulse duration per poke, seconds (default: 0.30)")
    p.add_argument("--gap", type=float, default=0.35, help="Zero-command gap between pokes, seconds (default: 0.35)")
    p.add_argument("--start-delay", type=float, default=1.0, help="Delay before first poke, seconds (default: 1.0)")
    p.add_argument("--publish-hz", type=float, default=20.0, help="Republish rate during pulse/zero hold (default: 20 Hz)")

    # Safety / optional integration
    p.add_argument("--set-slowdown", type=float, default=None, help="Optional slowdown factor to publish before sequence (0..1)")
    p.add_argument("--test-safety-stop", action="store_true", help="Briefly assert /safety/stop during test")
    p.add_argument("--safety-stop-hold", type=float, default=0.5, help="Duration to hold safety stop true (default: 0.5s)")

    # Behavior controls
    p.add_argument("--no-final-zero", action="store_true", help="Skip final zero hold (not recommended)")
    p.add_argument("--repeat", type=int, default=1, help="Repeat sequence N times (default: 1)")
    p.add_argument("--label", type=str, default="", help="Optional operator note shown in logs")

    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    # Clamp / validate
    try:
        vx = clamp(float(args.vx), -1.0, 1.0)
        vy = clamp(float(args.vy), -1.0, 1.0)
        wz = clamp(float(args.wz), -1.0, 1.0)
        pulse = max(0.05, float(args.pulse))
        gap = max(0.05, float(args.gap))
        start_delay = max(0.0, float(args.start_delay))
        publish_hz = max(5.0, float(args.publish_hz))
        repeat = max(1, int(args.repeat))
        safety_stop_hold = max(0.10, float(args.safety_stop_hold))
    except Exception as e:
        print(f"[poke_test_cli.py] Invalid arguments: {e}", file=sys.stderr)
        return 2

    slowdown_factor = None
    if args.set_slowdown is not None:
        slowdown_factor = clamp(float(args.set_slowdown), 0.0, 1.0)

    try:
        sequence = build_sequence(args.only, abs(vx), abs(vy), abs(wz))
    except Exception as e:
        print(f"[poke_test_cli.py] {e}", file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node: Optional[PokeTestCli] = None
    executor: Optional[SingleThreadedExecutor] = None

    try:
        node = PokeTestCli(
            cmd_topic=str(args.cmd_topic),
            safety_stop_topic=str(args.safety_stop_topic),
            slowdown_topic=str(args.slowdown_topic),
            publish_hz=publish_hz,
        )
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        # Header log
        if args.label:
            node.get_logger().info(f"Operator label: {args.label}")

        node.get_logger().info(
            "Poke test configuration | "
            f"only={args.only} repeat={repeat} pulse={pulse:.2f}s gap={gap:.2f}s "
            f"vx={abs(vx):.3f} vy={abs(vy):.3f} wz={abs(wz):.3f} start_delay={start_delay:.2f}s"
        )

        # Ensure initial zero
        node.send_zero_for(executor, 0.20)

        # Optional slowdown factor
        if slowdown_factor is not None:
            node.get_logger().info(f"Publishing slowdown factor: {slowdown_factor:.2f}")
            # Publish a few times for robustness with best-effort subscribers
            for _ in range(5):
                node.publish_slowdown(slowdown_factor)
                executor.spin_once(timeout_sec=0.0)
                time.sleep(0.03)

        # Optional safety stop test before motion sequence
        if bool(args.test_safety_stop):
            node.get_logger().warn(
                f"Testing safety stop topic ({args.safety_stop_topic}) for {safety_stop_hold:.2f}s"
            )
            for _ in range(5):
                node.publish_stop(True)
                node.publish_zero()
                executor.spin_once(timeout_sec=0.0)
                time.sleep(0.03)

            node.send_zero_for(executor, safety_stop_hold)

            node.get_logger().info("Releasing safety stop")
            for _ in range(5):
                node.publish_stop(False)
                node.publish_zero()
                executor.spin_once(timeout_sec=0.0)
                time.sleep(0.03)

            node.spin_sleep(executor, 0.20)

        # Start delay (operator move away)
        if start_delay > 0.0:
            node.get_logger().info(f"Starting in {start_delay:.1f}s...")
            node.send_zero_for(executor, start_delay)

        # Run sequence
        for rep in range(1, repeat + 1):
            node.get_logger().info(f"=== Poke sequence round {rep}/{repeat} ===")
            for name, sx, sy, sz in sequence:
                node.get_logger().info(
                    f"POKE {name:<3} | vx={sx:+.3f} vy={sy:+.3f} wz={sz:+.3f} for {pulse:.2f}s"
                )
                node.hold_command(executor, vx=sx, vy=sy, wz=sz, duration_s=pulse)

                node.get_logger().info(f"GAP  {name:<3} | zero for {gap:.2f}s")
                node.send_zero_for(executor, gap)

        # Final zero (recommended)
        if not bool(args.no_final_zero):
            node.get_logger().info("Final zero hold (0.5s)")
            node.send_zero_for(executor, 0.5)

        node.get_logger().info("Poke test completed successfully.")
        return 0

    except KeyboardInterrupt:
        if node is not None and executor is not None:
            try:
                node.get_logger().warn("Interrupted by user. Sending zero command...")
                node.send_zero_for(executor, 0.3)
            except Exception:
                pass
        return 130

    except Exception as e:
        print(f"[poke_test_cli.py] ERROR: {e}", file=sys.stderr)
        # Best-effort zero on error
        if node is not None and executor is not None:
            try:
                node.send_zero_for(executor, 0.3)
            except Exception:
                pass
        return 1

    finally:
        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
            except Exception:
                pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())