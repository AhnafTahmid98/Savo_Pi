#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/scripts/base_smoke_test_cli.py
-----------------------------------------------------
CLI smoke-test tool for `savo_base` / `base_driver_node`.

Purpose
-------
Quickly validate the base command path on real robot or dry-run:
  cmd source -> /cmd_vel_safe -> base_driver_node -> motor board

Also validates optional safety inputs:
  - /safety/stop
  - /safety/slowdown_factor

Typical use cases
-----------------
1) Dry-run bringup validation (no hardware damage risk)
2) Real hardware low-speed motion sanity checks
3) Safety stop topic test
4) Slowdown factor scaling test
5) Watchdog timeout behavior (stop publishing and observe zero output)

Examples
--------
# 1) Publish zeros once
ros2 run savo_base base_smoke_test_cli.py zero

# 2) Forward at low speed for 2 seconds
ros2 run savo_base base_smoke_test_cli.py cmd --vx 0.15 --duration 2.0

# 3) Strafe right
ros2 run savo_base base_smoke_test_cli.py cmd --vy 0.12 --duration 2.0

# 4) Rotate in place
ros2 run savo_base base_smoke_test_cli.py cmd --wz 0.20 --duration 2.0

# 5) Set slowdown factor to 0.40
ros2 run savo_base base_smoke_test_cli.py slowdown --value 0.40

# 6) Trigger safety stop latch path (if enabled in base_driver_node)
ros2 run savo_base base_smoke_test_cli.py estop --set true

# 7) Run a professional smoke sequence (safe low-speed)
ros2 run savo_base base_smoke_test_cli.py sequence --profile basic --confirm-real

# 8) Watchdog test: send command briefly then stop publishing
ros2 run savo_base base_smoke_test_cli.py pulse --vx 0.15 --on 0.5 --off 1.0
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Optional, Iterable, List

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


# -----------------------------------------------------------------------------
# Defaults (aligned with current Robot Savo stack)
# -----------------------------------------------------------------------------
DEFAULT_CMD_TOPIC = "/cmd_vel_safe"
DEFAULT_SAFETY_STOP_TOPIC = "/safety/stop"
DEFAULT_SLOWDOWN_TOPIC = "/safety/slowdown_factor"


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------
def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def parse_bool_text(text: str) -> bool:
    s = str(text).strip().lower()
    if s in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if s in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Invalid boolean value: {text!r}")


@dataclass
class MotionCmd:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    duration_s: float = 1.0
    label: str = "cmd"


# -----------------------------------------------------------------------------
# ROS CLI publisher node
# -----------------------------------------------------------------------------
class BaseSmokePublisher(Node):
    def __init__(
        self,
        *,
        cmd_topic: str,
        safety_stop_topic: str,
        slowdown_topic: str,
    ) -> None:
        super().__init__("base_smoke_test_cli")

        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.pub_stop = self.create_publisher(Bool, safety_stop_topic, 10)
        self.pub_slowdown = self.create_publisher(Float32, slowdown_topic, 10)

        self.cmd_topic = cmd_topic
        self.safety_stop_topic = safety_stop_topic
        self.slowdown_topic = slowdown_topic

        self.get_logger().info(
            "Base smoke test CLI ready | "
            f"cmd_topic={self.cmd_topic} "
            f"safety_stop_topic={self.safety_stop_topic} "
            f"slowdown_topic={self.slowdown_topic}"
        )

    def publish_cmd(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(wz)
        self.pub_cmd.publish(msg)

    def publish_zero(self) -> None:
        self.publish_cmd(0.0, 0.0, 0.0)

    def publish_safety_stop(self, stop: bool) -> None:
        msg = Bool()
        msg.data = bool(stop)
        self.pub_stop.publish(msg)

    def publish_slowdown(self, value: float) -> None:
        msg = Float32()
        msg.data = float(value)
        self.pub_slowdown.publish(msg)


# -----------------------------------------------------------------------------
# Execution helpers
# -----------------------------------------------------------------------------
def spin_once(executor: SingleThreadedExecutor, timeout_s: float = 0.01) -> None:
    executor.spin_once(timeout_sec=timeout_s)


def publish_for_duration(
    node: BaseSmokePublisher,
    executor: SingleThreadedExecutor,
    *,
    vx: float,
    vy: float,
    wz: float,
    duration_s: float,
    rate_hz: float,
    tail_zero: bool = True,
    quiet: bool = False,
) -> None:
    rate_hz = max(1.0, float(rate_hz))
    duration_s = max(0.0, float(duration_s))
    period = 1.0 / rate_hz

    if not quiet:
        node.get_logger().info(
            f"Publishing Twist for {duration_s:.2f}s @ {rate_hz:.1f}Hz | "
            f"vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}"
        )

    t_end = time.monotonic() + duration_s
    while time.monotonic() < t_end and rclpy.ok():
        node.publish_cmd(vx, vy, wz)
        spin_once(executor, timeout_s=0.0)
        time.sleep(period)

    if tail_zero and rclpy.ok():
        node.publish_zero()
        spin_once(executor, timeout_s=0.0)
        if not quiet:
            node.get_logger().info("Published trailing zero Twist.")


def run_sequence(
    node: BaseSmokePublisher,
    executor: SingleThreadedExecutor,
    *,
    rate_hz: float,
    pause_s: float,
    cmds: Iterable[MotionCmd],
    send_zero_between: bool = True,
) -> None:
    for i, cmd in enumerate(cmds, start=1):
        node.get_logger().info(
            f"[{i}] {cmd.label}: vx={cmd.vx:.3f} vy={cmd.vy:.3f} wz={cmd.wz:.3f} "
            f"for {cmd.duration_s:.2f}s"
        )
        publish_for_duration(
            node,
            executor,
            vx=cmd.vx,
            vy=cmd.vy,
            wz=cmd.wz,
            duration_s=cmd.duration_s,
            rate_hz=rate_hz,
            tail_zero=send_zero_between,
            quiet=True,
        )
        if pause_s > 0.0:
            t_end = time.monotonic() + pause_s
            while time.monotonic() < t_end and rclpy.ok():
                spin_once(executor, timeout_s=0.0)
                time.sleep(0.02)

    node.get_logger().info("Sequence complete.")


def build_basic_profile(speed: float, rot: float, step_s: float) -> List[MotionCmd]:
    s = float(speed)
    r = float(rot)
    d = float(step_s)
    return [
        MotionCmd(vx=0.0, vy=0.0, wz=0.0, duration_s=0.4, label="zero"),
        MotionCmd(vx=+s, vy=0.0, wz=0.0, duration_s=d, label="forward"),
        MotionCmd(vx=-s, vy=0.0, wz=0.0, duration_s=d, label="reverse"),
        MotionCmd(vx=0.0, vy=+s, wz=0.0, duration_s=d, label="strafe_left"),
        MotionCmd(vx=0.0, vy=-s, wz=0.0, duration_s=d, label="strafe_right"),
        MotionCmd(vx=0.0, vy=0.0, wz=+r, duration_s=d, label="rotate_ccw"),
        MotionCmd(vx=0.0, vy=0.0, wz=-r, duration_s=d, label="rotate_cw"),
        MotionCmd(vx=0.0, vy=0.0, wz=0.0, duration_s=0.4, label="zero_end"),
    ]


def build_xydiag_profile(speed: float, rot: float, step_s: float) -> List[MotionCmd]:
    s = float(speed)
    r = float(rot)
    d = float(step_s)
    # Small diagonals + mixed motion (use conservative values)
    return [
        MotionCmd(vx=+s, vy=+s, wz=0.0, duration_s=d, label="diag_fwd_left"),
        MotionCmd(vx=+s, vy=-s, wz=0.0, duration_s=d, label="diag_fwd_right"),
        MotionCmd(vx=-s, vy=+s, wz=0.0, duration_s=d, label="diag_rev_left"),
        MotionCmd(vx=-s, vy=-s, wz=0.0, duration_s=d, label="diag_rev_right"),
        MotionCmd(vx=+s, vy=0.0, wz=+r, duration_s=d, label="arc_fwd_ccw"),
        MotionCmd(vx=+s, vy=0.0, wz=-r, duration_s=d, label="arc_fwd_cw"),
        MotionCmd(vx=0.0, vy=0.0, wz=0.0, duration_s=0.4, label="zero_end"),
    ]


# -----------------------------------------------------------------------------
# Argument parser
# -----------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="base_smoke_test_cli.py",
        description="Robot Savo base smoke-test CLI for /cmd_vel_safe and safety topics.",
    )

    # Global topic / publish controls
    p.add_argument("--cmd-topic", default=DEFAULT_CMD_TOPIC, help="Twist command topic (default: /cmd_vel_safe)")
    p.add_argument("--safety-stop-topic", default=DEFAULT_SAFETY_STOP_TOPIC, help="Bool safety stop topic")
    p.add_argument("--slowdown-topic", default=DEFAULT_SLOWDOWN_TOPIC, help="Float32 slowdown topic")
    p.add_argument("--rate", type=float, default=20.0, help="Publish rate for command streams (Hz)")
    p.add_argument("--repeat", type=int, default=1, help="Repeat count for selected action/sequence")
    p.add_argument("--no-tail-zero", action="store_true", help="Do not publish zero command after cmd/pulse/sequence")
    p.add_argument(
        "--confirm-real",
        action="store_true",
        help="Acknowledge you may be running on real hardware (required for sequence profile).",
    )

    sub = p.add_subparsers(dest="subcmd", required=True)

    # zero
    sub.add_parser("zero", help="Publish zero Twist once (and exit)")

    # cmd
    p_cmd = sub.add_parser("cmd", help="Publish constant Twist for duration")
    p_cmd.add_argument("--vx", type=float, default=0.0, help="linear.x")
    p_cmd.add_argument("--vy", type=float, default=0.0, help="linear.y")
    p_cmd.add_argument("--wz", type=float, default=0.0, help="angular.z")
    p_cmd.add_argument("--duration", type=float, default=1.0, help="seconds")

    # pulse
    p_pulse = sub.add_parser("pulse", help="Pulse command then stop publishing (watchdog test)")
    p_pulse.add_argument("--vx", type=float, default=0.15)
    p_pulse.add_argument("--vy", type=float, default=0.0)
    p_pulse.add_argument("--wz", type=float, default=0.0)
    p_pulse.add_argument("--on", type=float, default=0.4, help="command publish duration in seconds")
    p_pulse.add_argument("--off", type=float, default=1.0, help="silent wait after pulse (observe watchdog)")

    # estop
    p_estop = sub.add_parser("estop", help="Publish safety stop Bool")
    p_estop.add_argument("--set", type=parse_bool_text, required=True, help="true/false")

    # slowdown
    p_slow = sub.add_parser("slowdown", help="Publish slowdown factor Float32")
    p_slow.add_argument("--value", type=float, required=True, help="0.0..1.0 (recommended)")

    # sequence
    p_seq = sub.add_parser("sequence", help="Run a low-speed professional smoke sequence")
    p_seq.add_argument("--profile", choices=["basic", "xydiag"], default="basic")
    p_seq.add_argument("--speed", type=float, default=0.12, help="translation command magnitude")
    p_seq.add_argument("--rot", type=float, default=0.18, help="rotation command magnitude")
    p_seq.add_argument("--step", type=float, default=1.0, help="seconds per step")
    p_seq.add_argument("--pause", type=float, default=0.4, help="pause between steps (s)")
    p_seq.add_argument(
        "--set-slowdown",
        type=float,
        default=None,
        help="Optionally publish slowdown factor before sequence (e.g., 0.5)",
    )
    p_seq.add_argument(
        "--clear-estop-first",
        action="store_true",
        help="Publish /safety/stop false before sequence start",
    )

    return p


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    # Basic sanity checks (safety-oriented)
    if args.rate <= 0.0:
        print("ERROR: --rate must be > 0", file=sys.stderr)
        return 2
    if hasattr(args, "duration") and args.duration < 0.0:
        print("ERROR: --duration must be >= 0", file=sys.stderr)
        return 2
    if hasattr(args, "value"):
        if not math.isfinite(args.value):
            print("ERROR: slowdown value must be finite", file=sys.stderr)
            return 2

    rclpy.init(args=None)
    node: Optional[BaseSmokePublisher] = None
    executor: Optional[SingleThreadedExecutor] = None

    try:
        node = BaseSmokePublisher(
            cmd_topic=args.cmd_topic,
            safety_stop_topic=args.safety_stop_topic,
            slowdown_topic=args.slowdown_topic,
        )
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        tail_zero = not args.no_tail_zero

        for rep in range(1, max(1, int(args.repeat)) + 1):
            if args.repeat > 1:
                node.get_logger().info(f"=== Repeat {rep}/{args.repeat} ===")

            if args.subcmd == "zero":
                node.publish_zero()
                spin_once(executor, timeout_s=0.0)
                node.get_logger().info("Published zero Twist once.")

            elif args.subcmd == "cmd":
                publish_for_duration(
                    node,
                    executor,
                    vx=float(args.vx),
                    vy=float(args.vy),
                    wz=float(args.wz),
                    duration_s=float(args.duration),
                    rate_hz=float(args.rate),
                    tail_zero=tail_zero,
                )

            elif args.subcmd == "pulse":
                publish_for_duration(
                    node,
                    executor,
                    vx=float(args.vx),
                    vy=float(args.vy),
                    wz=float(args.wz),
                    duration_s=float(args.on),
                    rate_hz=float(args.rate),
                    tail_zero=tail_zero,
                )
                node.get_logger().info(
                    f"Pulse complete. Silent waiting for {float(args.off):.2f}s "
                    "(observe watchdog stop behavior on base_driver_node)."
                )
                t_end = time.monotonic() + max(0.0, float(args.off))
                while time.monotonic() < t_end and rclpy.ok():
                    spin_once(executor, timeout_s=0.0)
                    time.sleep(0.02)

            elif args.subcmd == "estop":
                val = bool(args.set)
                node.publish_safety_stop(val)
                spin_once(executor, timeout_s=0.0)
                node.get_logger().info(f"Published safety stop: {val}")

            elif args.subcmd == "slowdown":
                val = clamp(float(args.value), 0.0, 1.0)
                if abs(val - float(args.value)) > 1e-9:
                    node.get_logger().warn(
                        f"Slowdown value clamped from {float(args.value):.3f} to {val:.3f}"
                    )
                node.publish_slowdown(val)
                spin_once(executor, timeout_s=0.0)
                node.get_logger().info(f"Published slowdown factor: {val:.3f}")

            elif args.subcmd == "sequence":
                # Require explicit acknowledgement for sequence because it moves the robot
                if not args.confirm_real:
                    node.get_logger().error(
                        "Sequence command requires --confirm-real "
                        "(safety acknowledgement for motion sequence)."
                    )
                    return 2

                speed = clamp(float(args.speed), -1.0, 1.0)
                rot = clamp(float(args.rot), -1.0, 1.0)
                step = max(0.1, float(args.step))
                pause = max(0.0, float(args.pause))

                # Conservative defaults warning if user set large values
                if abs(speed) > 0.25 or abs(rot) > 0.35:
                    node.get_logger().warn(
                        "High sequence magnitudes detected. Recommended smoke-test range: "
                        "speed <= 0.20, rot <= 0.25 on real hardware."
                    )

                if args.clear_estop_first:
                    node.publish_safety_stop(False)
                    spin_once(executor, timeout_s=0.0)
                    node.get_logger().info("Published /safety/stop = false (clear-estop-first).")
                    time.sleep(0.05)

                if args.set_slowdown is not None:
                    sf = clamp(float(args.set_slowdown), 0.0, 1.0)
                    node.publish_slowdown(sf)
                    spin_once(executor, timeout_s=0.0)
                    node.get_logger().info(f"Published slowdown factor before sequence: {sf:.3f}")
                    time.sleep(0.05)

                if args.profile == "basic":
                    cmds = build_basic_profile(speed=speed, rot=rot, step_s=step)
                else:
                    cmds = build_xydiag_profile(speed=speed, rot=rot, step_s=step)

                run_sequence(
                    node,
                    executor,
                    rate_hz=float(args.rate),
                    pause_s=pause,
                    cmds=cmds,
                    send_zero_between=tail_zero,
                )

                # Final zero (extra safe)
                node.publish_zero()
                spin_once(executor, timeout_s=0.0)
                node.get_logger().info("Final zero Twist published.")

            else:
                node.get_logger().error(f"Unknown subcommand: {args.subcmd}")
                return 2

        return 0

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Interrupted by user. Publishing final zero Twist.")
            try:
                node.publish_zero()
            except Exception:
                pass
        return 130

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