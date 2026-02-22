#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/nodes/base_watchdog_node.py
--------------------------------------------------
Professional ROS 2 Jazzy watchdog node for Robot Savo base command safety.

Purpose
-------
This node supervises the command path feeding the mobile base and publishes a
clear watchdog state for safety, dashboards, and diagnostics. It can also
optionally publish a stop request topic when commands go stale.

Why this node exists
--------------------
In real robot testing, if command messages stop arriving (teleop crash, network
delay, upstream node failure, etc.), the robot must not continue moving based on
old commands.

This watchdog provides:
- command freshness monitoring
- warning/trip thresholds
- debouncing to avoid false trips
- optional stop request publishing
- JSON state summary for dashboards/logging

Typical usage in Robot Savo
---------------------------
Monitor:
- /cmd_vel_safe   (preferred, after safety gate)
or
- /cmd_vel

Publish:
- /savo_base/watchdog_state          (std_msgs/String, JSON)
- /savo_base/watchdog_trip           (std_msgs/Bool)
- /savo_base/watchdog_stop_request   (std_msgs/Bool) [optional]

Important
---------
This node does NOT directly drive motors. `base_driver_node` remains the hardware
execution layer. This watchdog is a safety supervisor / telemetry layer.
"""

from __future__ import annotations

import json
import math
import time
import traceback
from dataclasses import dataclass
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


# =============================================================================
# Helpers
# =============================================================================
def _now_mono() -> float:
    return float(time.monotonic())


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


# =============================================================================
# Internal state containers
# =============================================================================
@dataclass
class CmdSample:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    t_mono: float = 0.0
    seq: int = 0
    count: int = 0


# =============================================================================
# Node
# =============================================================================
class BaseWatchdogNode(Node):
    """
    Supervises the freshness of base motion commands.

    States (high-level)
    -------------------
    - OK      : command age below warn threshold
    - WARN    : command age approaching timeout
    - TRIPPED : command stale for >= timeout (with optional debounce)
    """

    def __init__(self) -> None:
        super().__init__("base_watchdog_node")

        # ---------------------------------------------------------------------
        # Parameters (ROS 2 Jazzy style)
        # ---------------------------------------------------------------------
        self.declare_parameter("robot_name", "Robot Savo")
        self.declare_parameter("cmd_topic", "/cmd_vel_safe")

        # Timing
        self.declare_parameter("loop_hz", 20.0)
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("timeout_s", 0.30)
        self.declare_parameter("warn_ratio", 0.70)  # warn when age >= warn_ratio * timeout

        # Debounce / hysteresis behavior
        self.declare_parameter("trip_debounce_count", 2)
        self.declare_parameter("clear_debounce_count", 2)
        self.declare_parameter("auto_rearm", True)

        # Outputs
        self.declare_parameter("watchdog_state_topic", "/savo_base/watchdog_state")
        self.declare_parameter("watchdog_trip_topic", "/savo_base/watchdog_trip")
        self.declare_parameter("publish_trip_topic", True)

        # Optional stop request publishing (for integration if desired)
        self.declare_parameter("publish_stop_request", False)
        self.declare_parameter("stop_request_topic", "/savo_base/watchdog_stop_request")

        # JSON formatting
        self.declare_parameter("pretty_json", False)

        # ---------------------------------------------------------------------
        # Read parameters
        # ---------------------------------------------------------------------
        self.robot_name = str(self.get_parameter("robot_name").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)

        self.loop_hz = max(2.0, float(self.get_parameter("loop_hz").value))
        self.publish_hz = max(0.5, float(self.get_parameter("publish_hz").value))
        self.timeout_s = max(0.05, float(self.get_parameter("timeout_s").value))
        self.warn_ratio = _clamp(float(self.get_parameter("warn_ratio").value), 0.1, 0.99)

        self.trip_debounce_count = max(1, int(self.get_parameter("trip_debounce_count").value))
        self.clear_debounce_count = max(1, int(self.get_parameter("clear_debounce_count").value))
        self.auto_rearm = bool(self.get_parameter("auto_rearm").value)

        self.watchdog_state_topic = str(self.get_parameter("watchdog_state_topic").value)
        self.watchdog_trip_topic = str(self.get_parameter("watchdog_trip_topic").value)
        self.publish_trip_topic = bool(self.get_parameter("publish_trip_topic").value)

        self.publish_stop_request = bool(self.get_parameter("publish_stop_request").value)
        self.stop_request_topic = str(self.get_parameter("stop_request_topic").value)

        self.pretty_json = bool(self.get_parameter("pretty_json").value)

        # ---------------------------------------------------------------------
        # Runtime state
        # ---------------------------------------------------------------------
        self._started_mono = _now_mono()
        self._last_cmd = CmdSample()
        self._seq_counter = 0

        # Watchdog flags
        self._initialized = True
        self._active = True
        self._warning = False
        self._tripped = False
        self._stale = True  # true until first command arrives
        self._zero_action_pending = False  # telemetry only; actual zeroing is done by base_driver_node

        # Debounce counters
        self._stale_streak = 0
        self._fresh_streak = 0

        # Loop / event counters
        self._tick_count = 0
        self._trip_count = 0
        self._clear_count = 0
        self._cmd_rx_count = 0
        self._trip_pub_count = 0
        self._state_pub_count = 0
        self._stop_request_pub_count = 0

        # Last transition info
        self._last_event = "startup"
        self._last_event_mono = self._started_mono
        self._last_summary = "Awaiting first command"

        # ---------------------------------------------------------------------
        # QoS
        # ---------------------------------------------------------------------
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------------------------------------------------------------------
        # Subscriber
        # ---------------------------------------------------------------------
        self.sub_cmd = self.create_subscription(Twist, self.cmd_topic, self._on_cmd, qos_cmd)

        # ---------------------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------------------
        self.pub_state = self.create_publisher(String, self.watchdog_state_topic, 10)

        self.pub_trip = None
        if self.publish_trip_topic:
            self.pub_trip = self.create_publisher(Bool, self.watchdog_trip_topic, 10)

        self.pub_stop_request = None
        if self.publish_stop_request:
            self.pub_stop_request = self.create_publisher(Bool, self.stop_request_topic, 10)

        # ---------------------------------------------------------------------
        # Timers
        # ---------------------------------------------------------------------
        self.loop_timer = self.create_timer(1.0 / self.loop_hz, self._tick)
        self.state_timer = self.create_timer(1.0 / self.publish_hz, self._publish_state)

        self.get_logger().info(
            "base_watchdog_node started | "
            f"cmd_topic={self.cmd_topic} timeout={self.timeout_s:.3f}s "
            f"warn_ratio={self.warn_ratio:.2f} loop_hz={self.loop_hz:.1f} "
            f"trip_db={self.trip_debounce_count} clear_db={self.clear_debounce_count}"
        )

    # =========================================================================
    # Subscribers
    # =========================================================================
    def _on_cmd(self, msg: Twist) -> None:
        now = _now_mono()
        self._seq_counter += 1
        self._cmd_rx_count += 1

        self._last_cmd = CmdSample(
            vx=float(msg.linear.x),
            vy=float(msg.linear.y),
            wz=float(msg.angular.z),
            t_mono=now,
            seq=self._seq_counter,
            count=self._cmd_rx_count,
        )

        # If we are tripped and auto-rearm is enabled, clearing happens via
        # debounce in _tick() (fresh_streak). This callback only records input.

    # =========================================================================
    # Core watchdog logic
    # =========================================================================
    def _tick(self) -> None:
        self._tick_count += 1
        now = _now_mono()

        age_s = self._cmd_age(now)
        warn_s = self.warn_ratio * self.timeout_s

        stale_now = (age_s >= self.timeout_s)
        warn_now = (age_s >= warn_s) if math.isfinite(age_s) else True

        # Update flags
        self._warning = warn_now
        self._stale = stale_now

        # Debounce streak tracking
        if stale_now:
            self._stale_streak += 1
            self._fresh_streak = 0
        else:
            self._fresh_streak += 1
            self._stale_streak = 0

        # Trip logic
        if (not self._tripped) and stale_now and (self._stale_streak >= self.trip_debounce_count):
            self._tripped = True
            self._zero_action_pending = True
            self._trip_count += 1
            self._set_event("trip", now)
            self._publish_trip(True)
            self._publish_stop_request_if_enabled(True)

        # Clear logic (auto-rearm)
        if self._tripped and self.auto_rearm and (not stale_now) and (self._fresh_streak >= self.clear_debounce_count):
            self._tripped = False
            self._zero_action_pending = False
            self._clear_count += 1
            self._set_event("clear", now)
            self._publish_trip(False)
            self._publish_stop_request_if_enabled(False)

        # Status summary text (human-readable)
        if not math.isfinite(age_s):
            self._last_summary = "[INIT] No command received yet"
        elif self._tripped:
            self._last_summary = (
                f"[TRIPPED] cmd age={age_s:.3f}s >= timeout={self.timeout_s:.3f}s "
                f"(stale_streak={self._stale_streak})"
            )
        elif self._warning:
            self._last_summary = (
                f"[WARN] cmd age={age_s:.3f}s (warn at {warn_s:.3f}s, timeout {self.timeout_s:.3f}s)"
            )
        else:
            self._last_summary = f"[OK] cmd age={age_s:.3f}s < warn={warn_s:.3f}s"

    def _cmd_age(self, now: Optional[float] = None) -> float:
        if now is None:
            now = _now_mono()
        if self._last_cmd.t_mono <= 0.0:
            return math.inf
        return max(0.0, now - self._last_cmd.t_mono)

    def _set_event(self, name: str, now: Optional[float] = None) -> None:
        if now is None:
            now = _now_mono()
        self._last_event = name
        self._last_event_mono = now

    # =========================================================================
    # Publishers
    # =========================================================================
    def _publish_trip(self, value: bool) -> None:
        if self.pub_trip is None:
            return
        msg = Bool()
        msg.data = bool(value)
        self.pub_trip.publish(msg)
        self._trip_pub_count += 1

    def _publish_stop_request_if_enabled(self, value: bool) -> None:
        if self.pub_stop_request is None:
            return
        msg = Bool()
        msg.data = bool(value)
        self.pub_stop_request.publish(msg)
        self._stop_request_pub_count += 1

    def _publish_state(self) -> None:
        now = _now_mono()
        uptime_s = max(0.0, now - self._started_mono)
        age_s = self._cmd_age(now)
        warn_s = self.warn_ratio * self.timeout_s
        timeout_ratio = (age_s / self.timeout_s) if math.isfinite(age_s) else float("inf")

        # High-level status label
        if not math.isfinite(age_s):
            status_level = "INIT"
        elif self._tripped:
            status_level = "TRIPPED"
        elif self._warning:
            status_level = "WARN"
        else:
            status_level = "OK"

        payload: Dict[str, Any] = {
            "node": "base_watchdog_node",
            "robot_name": self.robot_name,
            "name": "base_command_watchdog",
            "status_level": status_level,
            "flags": {
                "initialized": self._initialized,
                "active": self._active,
                "warning": self._warning,
                "tripped": self._tripped,
                "stale": self._stale,
                "zero_action_pending": self._zero_action_pending,
                "auto_rearm": self.auto_rearm,
            },
            "timing": {
                "loop_hz": self.loop_hz,
                "publish_hz": self.publish_hz,
                "command_age_s": age_s,
                "warn_threshold_s": warn_s,
                "timeout_s": self.timeout_s,
                "timeout_ratio": timeout_ratio,
                "uptime_s": uptime_s,
            },
            "debounce": {
                "trip_debounce_count": self.trip_debounce_count,
                "clear_debounce_count": self.clear_debounce_count,
                "stale_streak": self._stale_streak,
                "fresh_streak": self._fresh_streak,
            },
            "command": {
                "topic": self.cmd_topic,
                "received_any": self._last_cmd.count > 0,
                "rx_count": self._cmd_rx_count,
                "last_seq": self._last_cmd.seq,
                "last_vx": self._last_cmd.vx,
                "last_vy": self._last_cmd.vy,
                "last_wz": self._last_cmd.wz,
            },
            "counters": {
                "tick_count": self._tick_count,
                "trip_count": self._trip_count,
                "clear_count": self._clear_count,
                "state_pub_count": self._state_pub_count + 1,
                "trip_pub_count": self._trip_pub_count,
                "stop_request_pub_count": self._stop_request_pub_count,
            },
            "events": {
                "last_event": self._last_event,
                "last_event_age_s": max(0.0, now - self._last_event_mono),
            },
            "outputs": {
                "watchdog_state_topic": self.watchdog_state_topic,
                "watchdog_trip_topic": self.watchdog_trip_topic if self.publish_trip_topic else None,
                "stop_request_topic": self.stop_request_topic if self.publish_stop_request else None,
            },
            "diagnostics": {
                "summary": self._last_summary,
                "notes": [
                    "This node supervises command freshness only.",
                    "Motor zeroing should be enforced by base_driver_node on trip/stale.",
                    "Recommended monitored topic: /cmd_vel_safe",
                ],
            },
        }

        msg = String()
        if self.pretty_json:
            msg.data = json.dumps(payload, ensure_ascii=False, indent=2, default=str)
        else:
            msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"), default=str)

        self.pub_state.publish(msg)
        self._state_pub_count += 1

    # =========================================================================
    # Shutdown
    # =========================================================================
    def destroy_node(self) -> bool:
        try:
            # Publish a final stop request if enabled (defensive safety)
            if self.publish_stop_request:
                self._publish_stop_request_if_enabled(True)
            self.get_logger().info("Shutting down base_watchdog_node.")
        finally:
            return super().destroy_node()


# =============================================================================
# Entry point
# =============================================================================
def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[BaseWatchdogNode] = None
    try:
        node = BaseWatchdogNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[base_watchdog_node] Fatal error: {e}")
        traceback.print_exc()
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()