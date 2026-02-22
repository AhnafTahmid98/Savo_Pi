#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/nodes/base_heartbeat_node.py
---------------------------------------------------
Professional ROS 2 Jazzy heartbeat node for the `savo_base` package.

Purpose
-------
This node provides a lightweight heartbeat/liveness signal for the Robot Savo
base stack during real robot testing and staged bringup.

Why this node is useful
-----------------------
- Confirms that the base stack is alive even before motors move
- Gives a simple signal for dashboards, launch validation, and supervisors
- Publishes both a boolean heartbeat and a compact JSON status summary
- Can optionally monitor other base topics and include "seen/stale" flags

Primary outputs
---------------
- /savo_base/heartbeat         (std_msgs/Bool)
- /savo_base/heartbeat_state   (std_msgs/String, JSON)

Optional monitored topics
-------------------------
- /cmd_vel_safe               (geometry_msgs/Twist)
- /savo_base/watchdog_trip    (std_msgs/Bool)
- /savo_base/watchdog_state   (std_msgs/String, JSON)
- /savo_base/base_state       (std_msgs/String, JSON)

Design notes
------------
- This node does NOT control motors.
- It is intentionally simple and robust for real hardware bringup.
- Missing optional topics do not cause failures (graceful degradation).
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

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist


# =============================================================================
# Helpers
# =============================================================================
def _now_mono() -> float:
    return float(time.monotonic())


def _safe_json_loads(text: str) -> Optional[Dict[str, Any]]:
    if not text:
        return None
    try:
        val = json.loads(text)
        if isinstance(val, dict):
            return val
        return {"_raw_value": val}
    except Exception:
        return None


# =============================================================================
# Topic caches
# =============================================================================
@dataclass
class BoolCache:
    value: bool = False
    t_mono: float = 0.0
    count: int = 0


@dataclass
class TwistCache:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    t_mono: float = 0.0
    count: int = 0


@dataclass
class JsonCache:
    data: Optional[Dict[str, Any]] = None
    t_mono: float = 0.0
    count: int = 0
    parse_error_count: int = 0
    last_raw_preview: str = ""


# =============================================================================
# Node
# =============================================================================
class BaseHeartbeatNode(Node):
    """
    Publishes base liveness heartbeat and optional heartbeat summary JSON.
    """

    def __init__(self) -> None:
        super().__init__("base_heartbeat_node")

        # ---------------------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------------------
        self.declare_parameter("robot_name", "Robot Savo")

        # Output topics
        self.declare_parameter("heartbeat_topic", "/savo_base/heartbeat")
        self.declare_parameter("heartbeat_state_topic", "/savo_base/heartbeat_state")

        # Publish rates
        self.declare_parameter("heartbeat_hz", 2.0)       # fast/simple liveness pulse
        self.declare_parameter("state_publish_hz", 1.0)   # JSON status summary

        # Heartbeat signal style
        self.declare_parameter("pulse_mode", "toggle")    # "toggle" or "always_true"

        # Optional topic monitoring (for richer state summary)
        self.declare_parameter("monitor_cmd", True)
        self.declare_parameter("monitor_watchdog_trip", True)
        self.declare_parameter("monitor_watchdog_state", True)
        self.declare_parameter("monitor_base_state", False)

        self.declare_parameter("cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("watchdog_trip_topic", "/savo_base/watchdog_trip")
        self.declare_parameter("watchdog_state_topic", "/savo_base/watchdog_state")
        self.declare_parameter("base_state_topic", "/savo_base/base_state")

        # Stale thresholds (used only for monitored-topic summary fields)
        self.declare_parameter("cmd_stale_s", 0.30)
        self.declare_parameter("watchdog_trip_stale_s", 1.00)
        self.declare_parameter("watchdog_state_stale_s", 1.50)
        self.declare_parameter("base_state_stale_s", 1.50)

        # JSON formatting
        self.declare_parameter("pretty_json", False)

        # ---------------------------------------------------------------------
        # Read parameters
        # ---------------------------------------------------------------------
        self.robot_name = str(self.get_parameter("robot_name").value)

        self.heartbeat_topic = str(self.get_parameter("heartbeat_topic").value)
        self.heartbeat_state_topic = str(self.get_parameter("heartbeat_state_topic").value)

        self.heartbeat_hz = max(0.2, float(self.get_parameter("heartbeat_hz").value))
        self.state_publish_hz = max(0.2, float(self.get_parameter("state_publish_hz").value))

        self.pulse_mode = str(self.get_parameter("pulse_mode").value).strip().lower()
        if self.pulse_mode not in ("toggle", "always_true"):
            self.get_logger().warn(
                f"Invalid pulse_mode='{self.pulse_mode}', falling back to 'toggle'."
            )
            self.pulse_mode = "toggle"

        self.monitor_cmd = bool(self.get_parameter("monitor_cmd").value)
        self.monitor_watchdog_trip = bool(self.get_parameter("monitor_watchdog_trip").value)
        self.monitor_watchdog_state = bool(self.get_parameter("monitor_watchdog_state").value)
        self.monitor_base_state = bool(self.get_parameter("monitor_base_state").value)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.watchdog_trip_topic = str(self.get_parameter("watchdog_trip_topic").value)
        self.watchdog_state_topic = str(self.get_parameter("watchdog_state_topic").value)
        self.base_state_topic = str(self.get_parameter("base_state_topic").value)

        self.cmd_stale_s = max(0.05, float(self.get_parameter("cmd_stale_s").value))
        self.watchdog_trip_stale_s = max(0.10, float(self.get_parameter("watchdog_trip_stale_s").value))
        self.watchdog_state_stale_s = max(0.10, float(self.get_parameter("watchdog_state_stale_s").value))
        self.base_state_stale_s = max(0.10, float(self.get_parameter("base_state_stale_s").value))

        self.pretty_json = bool(self.get_parameter("pretty_json").value)

        # ---------------------------------------------------------------------
        # Runtime state
        # ---------------------------------------------------------------------
        self._started_mono = _now_mono()
        self._hb_value = False  # used in toggle mode
        self._heartbeat_count = 0
        self._state_pub_count = 0

        self._cmd = TwistCache()
        self._watchdog_trip = BoolCache()
        self._watchdog_state = JsonCache()
        self._base_state = JsonCache()

        # ---------------------------------------------------------------------
        # QoS
        # ---------------------------------------------------------------------
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------------------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------------------
        self.pub_heartbeat = self.create_publisher(Bool, self.heartbeat_topic, 10)
        self.pub_heartbeat_state = self.create_publisher(String, self.heartbeat_state_topic, 10)

        # ---------------------------------------------------------------------
        # Optional subscribers
        # ---------------------------------------------------------------------
        self.sub_cmd = None
        self.sub_watchdog_trip = None
        self.sub_watchdog_state = None
        self.sub_base_state = None

        if self.monitor_cmd:
            self.sub_cmd = self.create_subscription(Twist, self.cmd_topic, self._on_cmd, qos_reliable)

        if self.monitor_watchdog_trip:
            self.sub_watchdog_trip = self.create_subscription(
                Bool, self.watchdog_trip_topic, self._on_watchdog_trip, qos_best_effort
            )

        if self.monitor_watchdog_state:
            self.sub_watchdog_state = self.create_subscription(
                String, self.watchdog_state_topic, self._on_watchdog_state, qos_reliable
            )

        if self.monitor_base_state:
            self.sub_base_state = self.create_subscription(
                String, self.base_state_topic, self._on_base_state, qos_reliable
            )

        # ---------------------------------------------------------------------
        # Timers
        # ---------------------------------------------------------------------
        self.heartbeat_timer = self.create_timer(1.0 / self.heartbeat_hz, self._publish_heartbeat)
        self.state_timer = self.create_timer(1.0 / self.state_publish_hz, self._publish_heartbeat_state)

        self.get_logger().info(
            "base_heartbeat_node started | "
            f"heartbeat_topic={self.heartbeat_topic} ({self.heartbeat_hz:.1f} Hz), "
            f"state_topic={self.heartbeat_state_topic} ({self.state_publish_hz:.1f} Hz), "
            f"pulse_mode={self.pulse_mode}, "
            f"monitors(cmd={self.monitor_cmd}, wd_trip={self.monitor_watchdog_trip}, "
            f"wd_state={self.monitor_watchdog_state}, base_state={self.monitor_base_state})"
        )

    # =========================================================================
    # Subscribers
    # =========================================================================
    def _on_cmd(self, msg: Twist) -> None:
        self._cmd.vx = float(msg.linear.x)
        self._cmd.vy = float(msg.linear.y)
        self._cmd.wz = float(msg.angular.z)
        self._cmd.t_mono = _now_mono()
        self._cmd.count += 1

    def _on_watchdog_trip(self, msg: Bool) -> None:
        self._watchdog_trip.value = bool(msg.data)
        self._watchdog_trip.t_mono = _now_mono()
        self._watchdog_trip.count += 1

    def _on_watchdog_state(self, msg: String) -> None:
        self._update_json_cache(self._watchdog_state, msg.data)

    def _on_base_state(self, msg: String) -> None:
        self._update_json_cache(self._base_state, msg.data)

    def _update_json_cache(self, cache: JsonCache, raw: str) -> None:
        parsed = _safe_json_loads(raw)
        cache.t_mono = _now_mono()
        cache.count += 1
        cache.last_raw_preview = (raw[:200] + "...") if len(raw) > 200 else raw
        if parsed is None:
            cache.parse_error_count += 1
            return
        cache.data = parsed

    # =========================================================================
    # Helpers
    # =========================================================================
    def _topic_age(self, t_mono: float) -> float:
        if t_mono <= 0.0:
            return math.inf
        return max(0.0, _now_mono() - t_mono)

    def _is_stale(self, t_mono: float, timeout_s: float) -> bool:
        return self._topic_age(t_mono) >= timeout_s

    def _derive_status_level(self) -> str:
        """
        Heartbeat node health + monitored-topic summary status.
        Conservative ordering:
        - WD_TRIPPED
        - WARN (stale monitored topics)
        - OK
        """
        # Strong signal: watchdog trip true and fresh
        if self.monitor_watchdog_trip and self._watchdog_trip.count > 0:
            if (not self._is_stale(self._watchdog_trip.t_mono, self.watchdog_trip_stale_s)) and self._watchdog_trip.value:
                return "WD_TRIPPED"

        # Warnings if monitored critical topics are stale
        if self.monitor_cmd and self._cmd.count > 0 and self._is_stale(self._cmd.t_mono, self.cmd_stale_s):
            return "WARN"

        if self.monitor_watchdog_state and self._watchdog_state.count > 0 and self._is_stale(
            self._watchdog_state.t_mono, self.watchdog_state_stale_s
        ):
            return "WARN"

        if self.monitor_base_state and self._base_state.count > 0 and self._is_stale(
            self._base_state.t_mono, self.base_state_stale_s
        ):
            return "WARN"

        return "OK"

    # =========================================================================
    # Publishers
    # =========================================================================
    def _publish_heartbeat(self) -> None:
        msg = Bool()

        if self.pulse_mode == "always_true":
            msg.data = True
        else:  # toggle
            self._hb_value = not self._hb_value
            msg.data = self._hb_value

        self.pub_heartbeat.publish(msg)
        self._heartbeat_count += 1

    def _publish_heartbeat_state(self) -> None:
        now = _now_mono()
        uptime_s = max(0.0, now - self._started_mono)

        cmd_age = self._topic_age(self._cmd.t_mono)
        wd_trip_age = self._topic_age(self._watchdog_trip.t_mono)
        wd_state_age = self._topic_age(self._watchdog_state.t_mono)
        base_state_age = self._topic_age(self._base_state.t_mono)

        payload: Dict[str, Any] = {
            "node": "base_heartbeat_node",
            "robot_name": self.robot_name,
            "status_level": self._derive_status_level(),
            "timestamp_monotonic_s": now,
            "uptime_s": uptime_s,
            "heartbeat": {
                "topic": self.heartbeat_topic,
                "mode": self.pulse_mode,
                "count": self._heartbeat_count,
                "last_value_internal_toggle_state": self._hb_value if self.pulse_mode == "toggle" else True,
                "publish_hz": self.heartbeat_hz,
            },
            "state_summary": {
                "topic": self.heartbeat_state_topic,
                "publish_hz": self.state_publish_hz,
                "count": self._state_pub_count + 1,
            },
            "monitors": {
                "cmd_vel_safe": {
                    "enabled": self.monitor_cmd,
                    "topic": self.cmd_topic if self.monitor_cmd else None,
                    "seen": self._cmd.count > 0,
                    "count": self._cmd.count,
                    "stale": self._is_stale(self._cmd.t_mono, self.cmd_stale_s) if self.monitor_cmd else None,
                    "age_s": cmd_age if self.monitor_cmd else None,
                    "stale_timeout_s": self.cmd_stale_s if self.monitor_cmd else None,
                    "last_vx": self._cmd.vx if self.monitor_cmd else None,
                    "last_vy": self._cmd.vy if self.monitor_cmd else None,
                    "last_wz": self._cmd.wz if self.monitor_cmd else None,
                },
                "watchdog_trip": {
                    "enabled": self.monitor_watchdog_trip,
                    "topic": self.watchdog_trip_topic if self.monitor_watchdog_trip else None,
                    "seen": self._watchdog_trip.count > 0,
                    "count": self._watchdog_trip.count,
                    "stale": self._is_stale(self._watchdog_trip.t_mono, self.watchdog_trip_stale_s)
                             if self.monitor_watchdog_trip else None,
                    "age_s": wd_trip_age if self.monitor_watchdog_trip else None,
                    "stale_timeout_s": self.watchdog_trip_stale_s if self.monitor_watchdog_trip else None,
                    "value": self._watchdog_trip.value if self.monitor_watchdog_trip else None,
                },
                "watchdog_state": {
                    "enabled": self.monitor_watchdog_state,
                    "topic": self.watchdog_state_topic if self.monitor_watchdog_state else None,
                    "seen": self._watchdog_state.count > 0,
                    "count": self._watchdog_state.count,
                    "stale": self._is_stale(self._watchdog_state.t_mono, self.watchdog_state_stale_s)
                             if self.monitor_watchdog_state else None,
                    "age_s": wd_state_age if self.monitor_watchdog_state else None,
                    "stale_timeout_s": self.watchdog_state_stale_s if self.monitor_watchdog_state else None,
                    "parse_error_count": self._watchdog_state.parse_error_count if self.monitor_watchdog_state else None,
                    "data": self._watchdog_state.data if self.monitor_watchdog_state else None,
                },
                "base_state": {
                    "enabled": self.monitor_base_state,
                    "topic": self.base_state_topic if self.monitor_base_state else None,
                    "seen": self._base_state.count > 0,
                    "count": self._base_state.count,
                    "stale": self._is_stale(self._base_state.t_mono, self.base_state_stale_s)
                             if self.monitor_base_state else None,
                    "age_s": base_state_age if self.monitor_base_state else None,
                    "stale_timeout_s": self.base_state_stale_s if self.monitor_base_state else None,
                    "parse_error_count": self._base_state.parse_error_count if self.monitor_base_state else None,
                    "data": self._base_state.data if self.monitor_base_state else None,
                },
            },
            "diagnostics": {
                "notes": [
                    "Heartbeat confirms liveness of the base stack node/process.",
                    "This node does not drive motors and does not enforce stop.",
                    "Use base_watchdog_node for command freshness safety supervision.",
                ]
            },
        }

        msg = String()
        if self.pretty_json:
            msg.data = json.dumps(payload, ensure_ascii=False, indent=2, default=str)
        else:
            msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"), default=str)

        self.pub_heartbeat_state.publish(msg)
        self._state_pub_count += 1

    # =========================================================================
    # Shutdown
    # =========================================================================
    def destroy_node(self) -> bool:
        self.get_logger().info("Shutting down base_heartbeat_node.")
        return super().destroy_node()


# =============================================================================
# Entry point
# =============================================================================
def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[BaseHeartbeatNode] = None
    try:
        node = BaseHeartbeatNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[base_heartbeat_node] Fatal error: {e}")
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