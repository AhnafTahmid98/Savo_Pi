#!/usr/bin/env python3
"""
Robot Savo — UI mode router node (v2)

Purpose
=======
Decides which UI mode to show on the 7" display based on:

- Mapping state
- Latest LLM intent result

Inputs
======
- /savo_mapping/mapping_active         (std_msgs/Bool)
- /savo_intent/intent_result           (savo_msgs/IntentResult)

Outputs
=======
- /savo_ui/mode                        (std_msgs/String)
    "INTERACT" / "NAVIGATE" / "MAP"

- /savo_ui/status_text                 (std_msgs/String)
    Short human-readable message for the current mode.

Parameters (ROS)
================
- robot_id                (string, default: "robot_savo_pi")
    Only process IntentResult with matching robot_id (if non-empty).

- mapping_active_topic    (string, default: "/savo_mapping/mapping_active")

- idle_status_text        (string, default: "Hello, I am Robot Savo!")
    Used when in INTERACT mode with no special condition.

- mapping_status_text     (string, default: "Mapping in progress, please keep distance.")

- stopped_status_text     (string, default: "Stopped here.")

- status_from_reply       (bool, default: True)
    If true and last IntentResult has non-empty reply_text, use it as status in
    NAVIGATE mode (truncated).

- nav_mode_timeout_s      (double, default: 20.0)
    How long after a NAVIGATE/FOLLOW intent we keep NAVIGATE mode active.

- stopped_mode_timeout_s  (double, default: 10.0)
    How long after a STOP intent we keep showing stopped_status_text.

Notes
=====
- This node is **optional**. You can disable it and publish /savo_ui/mode
  and /savo_ui/status_text yourself.
"""

from __future__ import annotations

import sys
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from savo_msgs.msg import IntentResult  # requires savo_ui to depend on savo_msgs


class UIModeRouterNode(Node):
    """Router that chooses UI mode based on mapping + LLM intent result."""

    def __init__(self) -> None:
        super().__init__("savo_ui_mode_router")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("robot_id", "robot_savo_pi")
        self.declare_parameter("mapping_active_topic", "/savo_mapping/mapping_active")
        self.declare_parameter("idle_status_text", "Hello, I am Robot Savo!")
        self.declare_parameter(
            "mapping_status_text",
            "Mapping in progress, please keep distance.",
        )
        self.declare_parameter("stopped_status_text", "Stopped here.")
        self.declare_parameter("status_from_reply", True)
        self.declare_parameter("nav_mode_timeout_s", 20.0)
        self.declare_parameter("stopped_mode_timeout_s", 10.0)

        self.robot_id: str = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        )
        self.mapping_active_topic: str = (
            self.get_parameter("mapping_active_topic")
            .get_parameter_value()
            .string_value
        )
        self.idle_status_text: str = (
            self.get_parameter("idle_status_text")
            .get_parameter_value()
            .string_value
        )
        self.mapping_status_text: str = (
            self.get_parameter("mapping_status_text")
            .get_parameter_value()
            .string_value
        )
        self.stopped_status_text: str = (
            self.get_parameter("stopped_status_text")
            .get_parameter_value()
            .string_value
        )
        self.status_from_reply: bool = (
            self.get_parameter("status_from_reply")
            .get_parameter_value()
            .bool_value
        )
        self.nav_mode_timeout_s: float = (
            self.get_parameter("nav_mode_timeout_s")
            .get_parameter_value()
            .double_value
        )
        self.stopped_mode_timeout_s: float = (
            self.get_parameter("stopped_mode_timeout_s")
            .get_parameter_value()
            .double_value
        )

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self.pub_mode = self.create_publisher(String, "/savo_ui/mode", 10)
        self.pub_status = self.create_publisher(String, "/savo_ui/status_text", 10)

        # ------------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------------
        self.create_subscription(
            Bool,
            self.mapping_active_topic,
            self._on_mapping_active,
            10,
        )

        self.create_subscription(
            IntentResult,
            "/savo_intent/intent_result",
            self._on_intent_result,
            10,
        )

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self._mapping_active: bool = False

        self._last_intent: str = ""      # NAVIGATE/FOLLOW/STOP/STATUS/CHATBOT
        self._last_nav_goal: str = ""    # from nav_goal in IntentResult
        self._last_reply_text: str = ""  # from reply_text in IntentResult
        self._last_event_time_s: float = 0.0  # last time we got any intent
        self._last_stop_time_s: float = 0.0   # last STOP

        self._current_mode: str = "INTERACT"
        self._current_status: str = self.idle_status_text

        # Evaluate mode at 5 Hz
        self._timer_dt = 0.2
        self._timer = self.create_timer(self._timer_dt, self._on_timer)

        self.get_logger().info(
            "UIModeRouterNode started with:\n"
            f"  robot_id                = {self.robot_id}\n"
            f"  mapping_active_topic    = {self.mapping_active_topic}\n"
            f"  idle_status_text        = {self.idle_status_text}\n"
            f"  mapping_status_text     = {self.mapping_status_text}\n"
            f"  stopped_status_text     = {self.stopped_status_text}\n"
            f"  status_from_reply       = {self.status_from_reply}\n"
            f"  nav_mode_timeout_s      = {self.nav_mode_timeout_s:.1f}\n"
            f"  stopped_mode_timeout_s  = {self.stopped_mode_timeout_s:.1f}"
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _on_mapping_active(self, msg: Bool) -> None:
        self._mapping_active = bool(msg.data)

    def _on_intent_result(self, msg: IntentResult) -> None:
        """
        Track latest intent for this robot_id and when it happened.
        """
        if self.robot_id:
            if msg.robot_id and msg.robot_id != self.robot_id:
                # Ignore intents for other robots
                return

        intent = (msg.intent or "").strip().upper()
        self._last_intent = intent
        self._last_nav_goal = (msg.nav_goal or "").strip()
        self._last_reply_text = (msg.reply_text or "").strip()

        now = self._now_s()
        self._last_event_time_s = now
        if intent == "STOP":
            self._last_stop_time_s = now

    # ------------------------------------------------------------------
    # Timer: decide mode + status
    # ------------------------------------------------------------------
    def _on_timer(self) -> None:
        now = self._now_s()
        new_mode = self._current_mode
        new_status = self._current_status

        # 1) Mapping has highest priority
        if self._mapping_active:
            new_mode = "MAP"
            new_status = self.mapping_status_text

        else:
            # Time since last intent
            dt_since_event = now - self._last_event_time_s
            dt_since_stop = now - self._last_stop_time_s

            # 2) Recent NAVIGATE/FOLLOW intent → NAVIGATE mode
            if (
                self._last_intent in ("NAVIGATE", "FOLLOW")
                and dt_since_event <= self.nav_mode_timeout_s
            ):
                new_mode = "NAVIGATE"

                # Build status text
                if self.status_from_reply and self._last_reply_text:
                    # Use LLM reply_text, truncated to something safe
                    txt = self._last_reply_text
                    if len(txt) > 120:
                        txt = txt[:117] + "..."
                    new_status = txt
                else:
                    if self._last_nav_goal:
                        new_status = f"Guiding to: {self._last_nav_goal}"
                    else:
                        new_status = "Guiding…"

            # 3) Recent STOP → INTERACT with stopped text
            elif (
                self._last_intent == "STOP"
                and dt_since_stop <= self.stopped_mode_timeout_s
            ):
                new_mode = "INTERACT"
                new_status = self.stopped_status_text

            # 4) Otherwise, idle INTERACT
            else:
                new_mode = "INTERACT"
                new_status = self.idle_status_text

        # Only publish when something changed
        if new_mode != self._current_mode:
            self.get_logger().info(
                f"UI mode router: {self._current_mode} -> {new_mode}"
            )
            self._publish_mode(new_mode)
            self._current_mode = new_mode

        if new_status != self._current_status:
            self._publish_status(new_status)
            self._current_status = new_status

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _now_s(self) -> float:
        """Return steady time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self.pub_mode.publish(msg)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)


# ======================================================================#
# main()
# ======================================================================#

def main(argv: Optional[list] = None) -> None:
    rclpy.init(args=argv)
    node = UIModeRouterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down UIModeRouterNode.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
