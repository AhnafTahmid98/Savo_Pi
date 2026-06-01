#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/scripts/teleop_letters_cli.py
----------------------------------------------------
Professional ROS 2 Jazzy terminal teleop (letters-only) for Robot Savo.

Purpose
-------
Manual keyboard teleoperation for quick bringup and diagnostics of the ROS base stack:
- Publishes Twist to /cmd_vel_safe (default) or custom topic
- Optional /safety/stop toggle publishing
- Optional /safety/slowdown_factor publishing
- Re-publishes commands at fixed rate to avoid base watchdog timeout
- Linux terminal raw-key mode (single key press, no Enter needed)

Design notes
------------
- "Letters-only" control style (matches your Robot Savo test workflow)
- Keeps command values normalized (typically -1..+1) unless you choose otherwise
- Safe defaults: low speeds + immediate stop key
- Clean zero command on exit (Ctrl+C / quit)

Default keymap (letters only)
-----------------------------
Motion:
  w = forward (+vx)
  s = backward (-vx)
  a = strafe left (+vy)
  d = strafe right (-vy)
  q = rotate left (+wz)
  e = rotate right (-wz)
  x = stop (zero cmd)

Diagonals / combined:
  r = forward + strafe left
  t = forward + strafe right
  f = backward + strafe left
  g = backward + strafe right

Speed scaling:
  z = decrease speed scale
  c = increase speed scale
  v = reset speed scale to 1.0

Safety / helpers:
  p = toggle /safety/stop publish (True/False)
  o = publish slowdown factor once (current slowdown value)
  k = decrease slowdown factor (if using slowdown topic)
  l = increase slowdown factor (if using slowdown topic)
  i = print help/status
  m = toggle hold-mode (latched command) vs pulse-mode (default hold-mode ON)
  h = help
  . = quit

Important
---------
If your sign conventions differ in base_driver_node (forward_sign/strafe_sign/rotate_sign),
the robot's real motion may look inverted relative to the keyboard labels. That is OK during
bringup—this tool is for verification and operator testing.

Examples
--------
# Default (publishes to /cmd_vel_safe)
ros2 run savo_base teleop_letters_cli.py

# Publish to raw /cmd_vel (only if you intentionally want this path)
ros2 run savo_base teleop_letters_cli.py --cmd-topic /cmd_vel

# Lower amplitudes for first hardware test
ros2 run savo_base teleop_letters_cli.py --vx 0.10 --vy 0.10 --wz 0.15

# Start with slowdown factor and allow publishing to /safety/slowdown_factor
ros2 run savo_base teleop_letters_cli.py --enable-slowdown-pub --slowdown 0.40
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import tty
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

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


@dataclass
class TeleopConfig:
    vx: float
    vy: float
    wz: float
    publish_hz: float
    speed_scale_step: float
    slowdown_step: float
    cmd_topic: str
    safety_stop_topic: str
    slowdown_topic: str
    enable_safety_stop_pub: bool
    enable_slowdown_pub: bool
    pulse_mode: bool  # if True, key press sends momentary pulse; else latched until next key/stop


# -----------------------------------------------------------------------------
# Terminal raw mode context
# -----------------------------------------------------------------------------
class RawTerminal:
    def __init__(self, stream) -> None:
        self.stream = stream
        self.fd = stream.fileno()
        self._old = None

    def __enter__(self):
        self._old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._old)


# -----------------------------------------------------------------------------
# Teleop node
# -----------------------------------------------------------------------------
class TeleopLettersCli(Node):
    def __init__(self, cfg: TeleopConfig) -> None:
        super().__init__("teleop_letters_cli")
        self.cfg = cfg

        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_aux = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub_cmd = self.create_publisher(Twist, cfg.cmd_topic, qos_cmd)
        self.pub_stop = self.create_publisher(Bool, cfg.safety_stop_topic, qos_aux)
        self.pub_slowdown = self.create_publisher(Float32, cfg.slowdown_topic, qos_aux)

        self.speed_scale = 1.0
        self.slowdown_factor = 1.0
        self.safety_stop_state = False
        self.hold_mode = not cfg.pulse_mode

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0

        self.last_key = "-"
        self.tick_count = 0

        self.timer = self.create_timer(1.0 / max(5.0, cfg.publish_hz), self._tick_publish)

        self.get_logger().info(
            f"teleop_letters_cli started | cmd={cfg.cmd_topic} "
            f"stop={cfg.safety_stop_topic} slowdown={cfg.slowdown_topic} "
            f"publish_hz={cfg.publish_hz:.1f} hold_mode={self.hold_mode}"
        )

    # ----------------------------- publishing ---------------------------------
    def _tick_publish(self) -> None:
        self.tick_count += 1
        self.pub_cmd.publish(make_twist(self.current_vx, self.current_vy, self.current_wz))

    def publish_zero_now(self) -> None:
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.pub_cmd.publish(make_twist(0.0, 0.0, 0.0))

    def publish_safety_stop_now(self, state: bool) -> None:
        msg = Bool()
        msg.data = bool(state)
        # Publish a few times (best-effort robustness)
        for _ in range(3):
            self.pub_stop.publish(msg)

    def publish_slowdown_now(self, factor: float) -> None:
        msg = Float32()
        msg.data = float(factor)
        for _ in range(3):
            self.pub_slowdown.publish(msg)

    # ----------------------------- command logic -------------------------------
    def _scaled(self, vx: float, vy: float, wz: float) -> Tuple[float, float, float]:
        s = clamp(self.speed_scale, 0.05, 2.0)
        return (vx * s, vy * s, wz * s)

    def _set_motion(self, vx: float, vy: float, wz: float) -> None:
        self.current_vx, self.current_vy, self.current_wz = self._scaled(vx, vy, wz)
        self.pub_cmd.publish(make_twist(self.current_vx, self.current_vy, self.current_wz))

    def handle_key(self, ch: str) -> bool:
        """
        Returns False to quit, True to continue.
        """
        self.last_key = ch

        # ---------------------------- quit/help --------------------------------
        if ch == ".":
            self.get_logger().info("Quit requested (.)")
            return False

        if ch in ("h", "i"):
            self.print_help()
            self.print_status()
            return True

        # ----------------------------- stop ------------------------------------
        if ch == "x":
            self.publish_zero_now()
            self.get_logger().info("STOP -> zero command")
            return True

        # ---------------------------- speed scale ------------------------------
        if ch == "z":
            self.speed_scale = clamp(self.speed_scale - self.cfg.speed_scale_step, 0.05, 2.0)
            self.get_logger().info(f"speed_scale = {self.speed_scale:.2f}")
            if self.hold_mode:
                # re-apply current normalized direction if any
                self._renormalize_current()
            return True

        if ch == "c":
            self.speed_scale = clamp(self.speed_scale + self.cfg.speed_scale_step, 0.05, 2.0)
            self.get_logger().info(f"speed_scale = {self.speed_scale:.2f}")
            if self.hold_mode:
                self._renormalize_current()
            return True

        if ch == "v":
            self.speed_scale = 1.0
            self.get_logger().info("speed_scale reset to 1.00")
            if self.hold_mode:
                self._renormalize_current()
            return True

        # ------------------------- slowdown factor pub -------------------------
        if ch == "k":
            self.slowdown_factor = clamp(self.slowdown_factor - self.cfg.slowdown_step, 0.0, 1.0)
            self.get_logger().info(f"slowdown_factor = {self.slowdown_factor:.2f}")
            if self.cfg.enable_slowdown_pub:
                self.publish_slowdown_now(self.slowdown_factor)
            return True

        if ch == "l":
            self.slowdown_factor = clamp(self.slowdown_factor + self.cfg.slowdown_step, 0.0, 1.0)
            self.get_logger().info(f"slowdown_factor = {self.slowdown_factor:.2f}")
            if self.cfg.enable_slowdown_pub:
                self.publish_slowdown_now(self.slowdown_factor)
            return True

        if ch == "o":
            if self.cfg.enable_slowdown_pub:
                self.publish_slowdown_now(self.slowdown_factor)
                self.get_logger().info(
                    f"Published slowdown factor to {self.cfg.slowdown_topic}: {self.slowdown_factor:.2f}"
                )
            else:
                self.get_logger().warn(
                    "Slowdown publisher disabled. Re-run with --enable-slowdown-pub to use key 'o/k/l'."
                )
            return True

        # --------------------------- safety stop pub ---------------------------
        if ch == "p":
            if self.cfg.enable_safety_stop_pub:
                self.safety_stop_state = not self.safety_stop_state
                self.publish_safety_stop_now(self.safety_stop_state)
                self.get_logger().warn(
                    f"Published safety_stop={self.safety_stop_state} to {self.cfg.safety_stop_topic}"
                )
                if self.safety_stop_state:
                    self.publish_zero_now()
            else:
                self.get_logger().warn(
                    "Safety stop publisher disabled. Re-run with --enable-safety-stop-pub to use key 'p'."
                )
            return True

        # -------------------------- mode toggle --------------------------------
        if ch == "m":
            self.hold_mode = not self.hold_mode
            mode_name = "HOLD (latched)" if self.hold_mode else "PULSE (momentary)"
            self.get_logger().info(f"Teleop mode -> {mode_name}")
            if not self.hold_mode:
                self.publish_zero_now()
            return True

        # -------------------------- motion keys --------------------------------
        motion_map: Dict[str, Tuple[float, float, float, str]] = {
            "w": (+self.cfg.vx, 0.0, 0.0, "forward"),
            "s": (-self.cfg.vx, 0.0, 0.0, "backward"),
            "a": (0.0, +self.cfg.vy, 0.0, "strafe_left"),
            "d": (0.0, -self.cfg.vy, 0.0, "strafe_right"),
            "q": (0.0, 0.0, +self.cfg.wz, "rotate_left"),
            "e": (0.0, 0.0, -self.cfg.wz, "rotate_right"),
            "r": (+self.cfg.vx, +self.cfg.vy, 0.0, "diag_fwd_left"),
            "t": (+self.cfg.vx, -self.cfg.vy, 0.0, "diag_fwd_right"),
            "f": (-self.cfg.vx, +self.cfg.vy, 0.0, "diag_back_left"),
            "g": (-self.cfg.vx, -self.cfg.vy, 0.0, "diag_back_right"),
        }

        if ch in motion_map:
            vx, vy, wz, label = motion_map[ch]
            self._set_motion(vx, vy, wz)
            self.get_logger().info(
                f"{label} | cmd=({self.current_vx:+.3f}, {self.current_vy:+.3f}, {self.current_wz:+.3f}) "
                f"speed_scale={self.speed_scale:.2f}"
            )
            if not self.hold_mode:
                # In pulse mode, momentary publish; timer continues but we zero immediately on next loop by caller.
                pass
            return True

        # Unknown key: ignore quietly (but provide occasional hint)
        if ch.strip():
            self.get_logger().info(f"Unknown key '{ch}'. Press h for help.")
        return True

    def pulse_decay_if_needed(self) -> None:
        if not self.hold_mode:
            # Momentary mode: after one key cycle, go back to zero
            self.current_vx = 0.0
            self.current_vy = 0.0
            self.current_wz = 0.0

    def _renormalize_current(self) -> None:
        """
        Reapply speed scale to current direction while preserving normalized intent
        approximately (only used after z/c/v in hold mode).
        """
        # Infer sign-only intent from current command (best effort)
        def sign_or_zero(x: float) -> float:
            if x > 1e-9:
                return 1.0
            if x < -1e-9:
                return -1.0
            return 0.0

        sx = sign_or_zero(self.current_vx)
        sy = sign_or_zero(self.current_vy)
        sz = sign_or_zero(self.current_wz)

        # Use configured amplitudes for the active directions
        self.current_vx = sx * self.cfg.vx * self.speed_scale
        self.current_vy = sy * self.cfg.vy * self.speed_scale
        self.current_wz = sz * self.cfg.wz * self.speed_scale
        self.pub_cmd.publish(make_twist(self.current_vx, self.current_vy, self.current_wz))

    # ------------------------------ UI ----------------------------------------
    def print_status(self) -> None:
        mode_name = "HOLD" if self.hold_mode else "PULSE"
        print(
            "\n[teleop_letters_cli status]\n"
            f"  cmd_topic      : {self.cfg.cmd_topic}\n"
            f"  safety_topic   : {self.cfg.safety_stop_topic} (enabled={self.cfg.enable_safety_stop_pub})\n"
            f"  slowdown_topic : {self.cfg.slowdown_topic} (enabled={self.cfg.enable_slowdown_pub})\n"
            f"  mode           : {mode_name}\n"
            f"  speed_scale    : {self.speed_scale:.2f}\n"
            f"  slowdown_factor: {self.slowdown_factor:.2f}\n"
            f"  safety_stop    : {self.safety_stop_state}\n"
            f"  current_cmd    : vx={self.current_vx:+.3f}, vy={self.current_vy:+.3f}, wz={self.current_wz:+.3f}\n"
        )
        sys.stdout.flush()

    @staticmethod
    def print_help() -> None:
        help_text = r"""
================ Robot Savo Teleop (letters-only) ================

Motion:
  w forward      s backward
  a strafe left  d strafe right
  q rotate left  e rotate right
  r fwd+left     t fwd+right
  f back+left    g back+right
  x stop (zero)

Speed scale:
  z slower   c faster   v reset scale=1.0

Safety / slowdown:
  p toggle safety stop publish (requires --enable-safety-stop-pub)
  k decrease slowdown factor (requires --enable-slowdown-pub)
  l increase slowdown factor (requires --enable-slowdown-pub)
  o publish current slowdown factor once

Modes / info:
  m toggle HOLD/PULSE mode
  h help
  i info/status
  . quit

Notes:
- HOLD mode: key sets command and keeps publishing it until next key/stop
- PULSE mode: key gives brief command pulse, then returns to zero
- Script re-publishes Twist to keep base_driver watchdog fed
===============================================================
"""
        print(help_text)
        sys.stdout.flush()


# -----------------------------------------------------------------------------
# CLI args
# -----------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="teleop_letters_cli.py",
        description="Letters-only ROS2 keyboard teleop for Robot Savo base stack.",
    )

    # Topics
    p.add_argument("--cmd-topic", default=TOPIC_CMD_VEL_SAFE, help=f"Twist topic (default: {TOPIC_CMD_VEL_SAFE})")
    p.add_argument("--safety-stop-topic", default=TOPIC_SAFETY_STOP, help=f"Safety stop topic (default: {TOPIC_SAFETY_STOP})")
    p.add_argument("--slowdown-topic", default=TOPIC_SAFETY_SLOWDOWN_FACTOR, help=f"Slowdown topic (default: {TOPIC_SAFETY_SLOWDOWN_FACTOR})")

    # Magnitudes
    p.add_argument("--vx", type=float, default=0.12, help="Base forward/backward magnitude (default: 0.12)")
    p.add_argument("--vy", type=float, default=0.12, help="Base strafe magnitude (default: 0.12)")
    p.add_argument("--wz", type=float, default=0.20, help="Base rotate magnitude (default: 0.20)")

    # Runtime behavior
    p.add_argument("--publish-hz", type=float, default=20.0, help="Twist republish rate (default: 20 Hz)")
    p.add_argument("--pulse-mode", action="store_true", help="Start in pulse mode instead of hold mode")
    p.add_argument("--speed-scale-step", type=float, default=0.10, help="z/c step for speed_scale (default: 0.10)")
    p.add_argument("--slowdown-step", type=float, default=0.10, help="k/l step for slowdown factor (default: 0.10)")

    # Optional publisher enables
    p.add_argument("--enable-safety-stop-pub", action="store_true", help="Enable key 'p' to publish /safety/stop")
    p.add_argument("--enable-slowdown-pub", action="store_true", help="Enable keys o/k/l to publish /safety/slowdown_factor")
    p.add_argument("--slowdown", type=float, default=1.0, help="Initial slowdown factor value for o/k/l keys (default: 1.0)")

    return p


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def _stdin_key_ready() -> bool:
    r, _, _ = select.select([sys.stdin], [], [], 0.0)
    return bool(r)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    cfg = TeleopConfig(
        vx=abs(clamp(float(args.vx), 0.0, 1.0)),
        vy=abs(clamp(float(args.vy), 0.0, 1.0)),
        wz=abs(clamp(float(args.wz), 0.0, 1.0)),
        publish_hz=max(5.0, float(args.publish_hz)),
        speed_scale_step=clamp(float(args.speed_scale_step), 0.01, 0.50),
        slowdown_step=clamp(float(args.slowdown_step), 0.01, 0.50),
        cmd_topic=str(args.cmd_topic),
        safety_stop_topic=str(args.safety_stop_topic),
        slowdown_topic=str(args.slowdown_topic),
        enable_safety_stop_pub=bool(args.enable_safety_stop_pub),
        enable_slowdown_pub=bool(args.enable_slowdown_pub),
        pulse_mode=bool(args.pulse_mode),
    )

    # Basic terminal check (Linux/macOS terminal expected)
    if not sys.stdin.isatty():
        print("[teleop_letters_cli.py] ERROR: stdin is not a TTY. Run in a terminal.", file=sys.stderr)
        return 1

    rclpy.init(args=None)
    node: Optional[TeleopLettersCli] = None
    executor: Optional[SingleThreadedExecutor] = None

    try:
        node = TeleopLettersCli(cfg)
        node.slowdown_factor = clamp(float(args.slowdown), 0.0, 1.0)

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        node.print_help()
        node.print_status()

        with RawTerminal(sys.stdin):
            while rclpy.ok():
                # Process ROS callbacks/timers regularly
                executor.spin_once(timeout_sec=0.02)

                if _stdin_key_ready():
                    ch = sys.stdin.read(1)
                    if not ch:
                        continue
                    keep_running = node.handle_key(ch.lower())
                    if not keep_running:
                        break

                    # In pulse mode, decay back to zero immediately after processing key
                    node.pulse_decay_if_needed()

        # Graceful stop on normal exit
        node.get_logger().info("Exiting teleop_letters_cli -> sending zero command")
        for _ in range(5):
            node.publish_zero_now()
            executor.spin_once(timeout_sec=0.01)

        return 0

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warn("Interrupted by user (Ctrl+C) -> sending zero command")
            try:
                if executor is not None:
                    for _ in range(5):
                        node.publish_zero_now()
                        executor.spin_once(timeout_sec=0.01)
                else:
                    node.publish_zero_now()
            except Exception:
                pass
        return 130

    except Exception as e:
        print(f"[teleop_letters_cli.py] ERROR: {e}", file=sys.stderr)
        if node is not None:
            try:
                node.publish_zero_now()
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