#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/nodes/base_state_publisher_node.py
----------------------------------------------------------
Professional ROS 2 Jazzy state aggregation / publishing node for `savo_base`.

Purpose
-------
This node collects runtime signals related to the mobile base and publishes a
single consolidated JSON state summary for dashboards, logging, and debugging.

Why this node is useful
-----------------------
- Keeps `base_driver_node` focused on motor execution (real hardware path)
- Centralizes status reporting in one place
- Makes it easier to monitor Robot Savo during real robot testing
- Works even if some topics are missing (graceful degradation)

Typical inputs
--------------
- /cmd_vel_safe                     (geometry_msgs/Twist)
- /safety/stop                      (std_msgs/Bool)
- /safety/slowdown_factor           (std_msgs/Float32)
- /savo_base/watchdog_state         (std_msgs/String JSON from base_driver_node)
- /savo_base/base_state             (std_msgs/String JSON from base_driver_node)
- /savo_base/motor_board_status     (std_msgs/String JSON, optional future topic)

Primary output
--------------
- /savo_base/state_summary          (std_msgs/String JSON)

Optional outputs
----------------
- /savo_base/state_heartbeat        (std_msgs/Bool)

Design notes
------------
- JSON string transport is intentional for easy dashboard integration and logs.
- Uses stale detection per input topic.
- Safe for partial bringup (missing topics do not crash the node).
"""

from __future__ import annotations

import json
import math
import time
import traceback
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String


# =============================================================================
# Helpers
# =============================================================================
def _now_mono() -> float:
    return float(time.monotonic())


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


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
# Topic cache structures
# =============================================================================
@dataclass
class TwistCache:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    t_mono: float = 0.0
    count: int = 0


@dataclass
class BoolCache:
    value: bool = False
    t_mono: float = 0.0
    count: int = 0


@dataclass
class FloatCache:
    value: float = 0.0
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
class BaseStatePublisherNode(Node):
    """
    Aggregate base-related states into a single JSON summary topic.

    This node is intentionally lightweight and dashboard-friendly.
    """

    def __init__(self) -> None:
        super().__init__("base_state_publisher_node")

        # ---------------------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------------------
        self.declare_parameter("robot_name", "Robot Savo")

        # Input topics
        self.declare_parameter("cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("safety_stop_topic", "/safety/stop")
        self.declare_parameter("slowdown_topic", "/safety/slowdown_factor")
        self.declare_parameter("watchdog_topic", "/savo_base/watchdog_state")
        self.declare_parameter("base_state_topic", "/savo_base/base_state")
        self.declare_parameter("motor_board_status_topic", "/savo_base/motor_board_status")

        # Feature toggles (useful during staged bringup)
        self.declare_parameter("subscribe_cmd", True)
        self.declare_parameter("subscribe_safety_stop", True)
        self.declare_parameter("subscribe_slowdown", True)
        self.declare_parameter("subscribe_watchdog", True)
        self.declare_parameter("subscribe_base_state", True)
        self.declare_parameter("subscribe_motor_board_status", False)

        # Output topics
        self.declare_parameter("summary_topic", "/savo_base/state_summary")
        self.declare_parameter("heartbeat_topic", "/savo_base/state_heartbeat")

        # Timing
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("heartbeat_hz", 1.0)

        # Stale thresholds
        self.declare_parameter("cmd_stale_s", 0.30)
        self.declare_parameter("safety_stale_s", 0.50)
        self.declare_parameter("watchdog_stale_s", 1.50)
        self.declare_parameter("base_state_stale_s", 1.50)
        self.declare_parameter("motor_board_status_stale_s", 1.50)

        # JSON formatting
        self.declare_parameter("pretty_json", False)

        # ---------------------------------------------------------------------
        # Read parameters
        # ---------------------------------------------------------------------
        self.robot_name = str(self.get_parameter("robot_name").value)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.safety_stop_topic = str(self.get_parameter("safety_stop_topic").value)
        self.slowdown_topic = str(self.get_parameter("slowdown_topic").value)
        self.watchdog_topic = str(self.get_parameter("watchdog_topic").value)
        self.base_state_topic = str(self.get_parameter("base_state_topic").value)
        self.motor_board_status_topic = str(self.get_parameter("motor_board_status_topic").value)

        self.subscribe_cmd = bool(self.get_parameter("subscribe_cmd").value)
        self.subscribe_safety_stop = bool(self.get_parameter("subscribe_safety_stop").value)
        self.subscribe_slowdown = bool(self.get_parameter("subscribe_slowdown").value)
        self.subscribe_watchdog = bool(self.get_parameter("subscribe_watchdog").value)
        self.subscribe_base_state = bool(self.get_parameter("subscribe_base_state").value)
        self.subscribe_motor_board_status = bool(self.get_parameter("subscribe_motor_board_status").value)

        self.summary_topic = str(self.get_parameter("summary_topic").value)
        self.heartbeat_topic = str(self.get_parameter("heartbeat_topic").value)

        self.publish_hz = max(0.5, float(self.get_parameter("publish_hz").value))
        self.heartbeat_hz = max(0.2, float(self.get_parameter("heartbeat_hz").value))

        self.cmd_stale_s = max(0.05, float(self.get_parameter("cmd_stale_s").value))
        self.safety_stale_s = max(0.05, float(self.get_parameter("safety_stale_s").value))
        self.watchdog_stale_s = max(0.10, float(self.get_parameter("watchdog_stale_s").value))
        self.base_state_stale_s = max(0.10, float(self.get_parameter("base_state_stale_s").value))
        self.motor_board_status_stale_s = max(0.10, float(self.get_parameter("motor_board_status_stale_s").value))

        self.pretty_json = bool(self.get_parameter("pretty_json").value)

        # ---------------------------------------------------------------------
        # Runtime caches
        # ---------------------------------------------------------------------
        self._started_mono = _now_mono()

        self._cmd = TwistCache()
        self._safety_stop = BoolCache(value=False)
        self._slowdown = FloatCache(value=1.0)

        self._watchdog = JsonCache()
        self._base_state = JsonCache()
        self._motor_board_status = JsonCache()

        self._summary_pub_count = 0
        self._heartbeat_pub_count = 0

        # ---------------------------------------------------------------------
        # QoS profiles
        # ---------------------------------------------------------------------
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------------------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------------------
        self.pub_summary = self.create_publisher(String, self.summary_topic, 10)
        self.pub_heartbeat = self.create_publisher(Bool, self.heartbeat_topic, 10)

        # ---------------------------------------------------------------------
        # Subscriptions
        # ---------------------------------------------------------------------
        self.sub_cmd = None
        self.sub_safety_stop = None
        self.sub_slowdown = None
        self.sub_watchdog = None
        self.sub_base_state = None
        self.sub_motor_board_status = None

        if self.subscribe_cmd:
            self.sub_cmd = self.create_subscription(Twist, self.cmd_topic, self._on_cmd_vel, qos_reliable)

        if self.subscribe_safety_stop:
            self.sub_safety_stop = self.create_subscription(Bool, self.safety_stop_topic, self._on_safety_stop, qos_sensor)

        if self.subscribe_slowdown:
            self.sub_slowdown = self.create_subscription(Float32, self.slowdown_topic, self._on_slowdown, qos_sensor)

        if self.subscribe_watchdog:
            self.sub_watchdog = self.create_subscription(String, self.watchdog_topic, self._on_watchdog_json, qos_reliable)

        if self.subscribe_base_state:
            self.sub_base_state = self.create_subscription(String, self.base_state_topic, self._on_base_state_json, qos_reliable)

        if self.subscribe_motor_board_status:
            self.sub_motor_board_status = self.create_subscription(
                String, self.motor_board_status_topic, self._on_motor_board_status_json, qos_reliable
            )

        # ---------------------------------------------------------------------
        # Timers
        # ---------------------------------------------------------------------
        self.summary_timer = self.create_timer(1.0 / self.publish_hz, self._publish_summary)
        self.heartbeat_timer = self.create_timer(1.0 / self.heartbeat_hz, self._publish_heartbeat)

        self.get_logger().info(
            "base_state_publisher_node started | "
            f"summary_topic={self.summary_topic} publish_hz={self.publish_hz:.1f} "
            f"subs(cmd={self.subscribe_cmd}, stop={self.subscribe_safety_stop}, "
            f"slowdown={self.subscribe_slowdown}, watchdog={self.subscribe_watchdog}, "
            f"base_state={self.subscribe_base_state}, board_status={self.subscribe_motor_board_status})"
        )

    # =========================================================================
    # Subscribers
    # =========================================================================
    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd.vx = float(msg.linear.x)
        self._cmd.vy = float(msg.linear.y)
        self._cmd.wz = float(msg.angular.z)
        self._cmd.t_mono = _now_mono()
        self._cmd.count += 1

    def _on_safety_stop(self, msg: Bool) -> None:
        self._safety_stop.value = bool(msg.data)
        self._safety_stop.t_mono = _now_mono()
        self._safety_stop.count += 1

    def _on_slowdown(self, msg: Float32) -> None:
        self._slowdown.value = float(msg.data)
        self._slowdown.t_mono = _now_mono()
        self._slowdown.count += 1

    def _on_watchdog_json(self, msg: String) -> None:
        self._update_json_cache(self._watchdog, msg.data)

    def _on_base_state_json(self, msg: String) -> None:
        self._update_json_cache(self._base_state, msg.data)

    def _on_motor_board_status_json(self, msg: String) -> None:
        self._update_json_cache(self._motor_board_status, msg.data)

    def _update_json_cache(self, cache: JsonCache, raw: str) -> None:
        parsed = _safe_json_loads(raw)
        cache.t_mono = _now_mono()
        cache.count += 1
        cache.last_raw_preview = (raw[:200] + "...") if len(raw) > 200 else raw
        if parsed is None:
            cache.parse_error_count += 1
            # Keep previous parsed data if current one is malformed
            return
        cache.data = parsed

    # =========================================================================
    # Status helpers
    # =========================================================================
    def _topic_age(self, t_mono: float) -> float:
        if t_mono <= 0.0:
            return math.inf
        return max(0.0, _now_mono() - t_mono)

    def _is_stale(self, t_mono: float, timeout_s: float) -> bool:
        return self._topic_age(t_mono) >= timeout_s

    def _overall_status(self) -> str:
        """
        Consolidated health/status level with conservative priority ordering.
        """
        safety_active = self._safety_stop.value if self._safety_stop.count > 0 else False
        if safety_active:
            return "SAFETY_STOP"

        # Watchdog from base_driver is strongest signal if present and fresh
        if self._watchdog.data and not self._is_stale(self._watchdog.t_mono, self.watchdog_stale_s):
            try:
                wd_level = str(self._watchdog.data.get("status_level", "")).upper()
                if wd_level in ("ERROR", "STALE", "TRIPPED", "FAULT"):
                    return wd_level
            except Exception:
                pass

        # base_state from base_driver if present
        if self._base_state.data and not self._is_stale(self._base_state.t_mono, self.base_state_stale_s):
            try:
                bs_level = str(self._base_state.data.get("status_level", "")).upper()
                if bs_level in ("ERROR", "FAULT", "STALE", "WARN", "WARNING"):
                    return "WARN" if bs_level == "WARNING" else bs_level
            except Exception:
                pass

        # Missing fresh command path during live testing should be visible
        if self.subscribe_cmd and self._is_stale(self._cmd.t_mono, self.cmd_stale_s):
            return "CMD_STALE"

        return "OK"

    # =========================================================================
    # Publishers
    # =========================================================================
    def _publish_summary(self) -> None:
        now = _now_mono()
        uptime_s = max(0.0, now - self._started_mono)

        cmd_age = self._topic_age(self._cmd.t_mono)
        stop_age = self._topic_age(self._safety_stop.t_mono)
        slowdown_age = self._topic_age(self._slowdown.t_mono)
        watchdog_age = self._topic_age(self._watchdog.t_mono)
        base_state_age = self._topic_age(self._base_state.t_mono)
        board_age = self._topic_age(self._motor_board_status.t_mono)

        payload: Dict[str, Any] = {
            "node": "base_state_publisher_node",
            "robot_name": self.robot_name,
            "status_level": self._overall_status(),
            "timestamp_monotonic_s": now,
            "uptime_s": uptime_s,
            "summary_pub_count": self._summary_pub_count + 1,
            "topics": {
                "inputs": {
                    "cmd_topic": self.cmd_topic if self.subscribe_cmd else None,
                    "safety_stop_topic": self.safety_stop_topic if self.subscribe_safety_stop else None,
                    "slowdown_topic": self.slowdown_topic if self.subscribe_slowdown else None,
                    "watchdog_topic": self.watchdog_topic if self.subscribe_watchdog else None,
                    "base_state_topic": self.base_state_topic if self.subscribe_base_state else None,
                    "motor_board_status_topic": self.motor_board_status_topic if self.subscribe_motor_board_status else None,
                },
                "outputs": {
                    "summary_topic": self.summary_topic,
                    "heartbeat_topic": self.heartbeat_topic,
                },
            },
            "command_path": {
                "cmd_vel_safe": {
                    "seen": self._cmd.count > 0,
                    "count": self._cmd.count,
                    "stale": self._is_stale(self._cmd.t_mono, self.cmd_stale_s),
                    "age_s": cmd_age,
                    "stale_timeout_s": self.cmd_stale_s,
                    "vx": self._cmd.vx,
                    "vy": self._cmd.vy,
                    "wz": self._cmd.wz,
                },
                "safety_stop": {
                    "seen": self._safety_stop.count > 0,
                    "count": self._safety_stop.count,
                    "stale": self._is_stale(self._safety_stop.t_mono, self.safety_stale_s),
                    "age_s": stop_age,
                    "stale_timeout_s": self.safety_stale_s,
                    "value": self._safety_stop.value,
                },
                "slowdown_factor": {
                    "seen": self._slowdown.count > 0,
                    "count": self._slowdown.count,
                    "stale": self._is_stale(self._slowdown.t_mono, self.safety_stale_s),
                    "age_s": slowdown_age,
                    "stale_timeout_s": self.safety_stale_s,
                    "value": self._slowdown.value,
                },
            },
            "driver_state_topics": {
                "watchdog_state": {
                    "seen": self._watchdog.count > 0,
                    "count": self._watchdog.count,
                    "stale": self._is_stale(self._watchdog.t_mono, self.watchdog_stale_s),
                    "age_s": watchdog_age,
                    "stale_timeout_s": self.watchdog_stale_s,
                    "parse_error_count": self._watchdog.parse_error_count,
                    "data": self._watchdog.data,
                },
                "base_state": {
                    "seen": self._base_state.count > 0,
                    "count": self._base_state.count,
                    "stale": self._is_stale(self._base_state.t_mono, self.base_state_stale_s),
                    "age_s": base_state_age,
                    "stale_timeout_s": self.base_state_stale_s,
                    "parse_error_count": self._base_state.parse_error_count,
                    "data": self._base_state.data,
                },
                "motor_board_status": {
                    "seen": self._motor_board_status.count > 0,
                    "count": self._motor_board_status.count,
                    "stale": self._is_stale(self._motor_board_status.t_mono, self.motor_board_status_stale_s),
                    "age_s": board_age,
                    "stale_timeout_s": self.motor_board_status_stale_s,
                    "parse_error_count": self._motor_board_status.parse_error_count,
                    "data": self._motor_board_status.data,
                },
            },
            "diagnostics": {
                "notes": [
                    "This node aggregates status for dashboards and logging.",
                    "Motor execution remains in base_driver_node.",
                    "Missing inputs are tolerated during staged bringup.",
                ],
            },
        }

        msg = String()
        if self.pretty_json:
            msg.data = json.dumps(payload, ensure_ascii=False, indent=2, default=str)
        else:
            msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"), default=str)

        self.pub_summary.publish(msg)
        self._summary_pub_count += 1

    def _publish_heartbeat(self) -> None:
        """
        Heartbeat is True when the node is alive and publishing summaries.
        It does NOT mean the base is healthy; check state_summary.status_level for that.
        """
        msg = Bool()
        msg.data = True
        self.pub_heartbeat.publish(msg)
        self._heartbeat_pub_count += 1

    # =========================================================================
    # Shutdown
    # =========================================================================
    def destroy_node(self) -> bool:
        self.get_logger().info("Shutting down base_state_publisher_node.")
        return super().destroy_node()


# =============================================================================
# Entry point
# =============================================================================
def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[BaseStatePublisherNode] = None
    try:
        node = BaseStatePublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[base_state_publisher_node] Fatal error: {e}")
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