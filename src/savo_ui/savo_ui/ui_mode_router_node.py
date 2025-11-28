#!/usr/bin/env python3
"""
Robot Savo — UI mode router node

This node decides which high-level UI mode the screen should show and what
status text to display, based on intent results and (optionally) mapping state.

Inputs
------
- /savo_intent/intent_result (savo_msgs/IntentResult)
    Result from the LLM/intent pipeline. We use:
      - robot_id : to filter events for this robot
      - intent   : NAVIGATE / FOLLOW / STOP / STATUS / CHATBOT
      - nav_goal : target location name (e.g. "A201")
      - reply_text : final spoken reply
      - success : whether the LLM call and parsing succeeded

- /savo_mapping/mapping_active (std_msgs/Bool) [optional]
    If enabled and True, UI is forced into MAP mode, independent of the intent.

Outputs
-------
- /savo_ui/mode (std_msgs/String)
    "INTERACT" | "NAVIGATE" | "MAP"

- /savo_ui/status_text (std_msgs/String)
    Short, human-readable status, e.g.:
      - "Guiding to A201"
      - "Mapping in progress, please keep distance."
      - "Stopped here."
      - "Hello, I am Robot Savo!"

Parameters
----------
- robot_id (string)
    Only react to IntentResult messages with the same robot_id.
    If empty, accept all.

- mapping_active_topic (string, default "/savo_mapping/mapping_active")
    If empty, mapping override is disabled. If non-empty, subscribe to this
    Bool topic and force MAP mode while it is True.

- idle_status_text (string)
    Default text in INTERACT mode when nothing specific is happening.

- mapping_status_text (string)
    Text to show when in MAP mode.

- stopped_status_text (string)
    Text to show when a STOP intent is received.

- status_from_reply (bool)
    If True, STATUS intents will use reply_text as status_text
    (clamped to a reasonable length). Otherwise, keep idle_status_text.

This node does not render anything itself; it only decides mode + status text
and lets display_manager_node handle actual drawing.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from savo_msgs.msg import IntentResult  # NavState/RobotStatus can be added later


@dataclass
class LastIntentState:
    """Small struct to remember the most recent intent result."""
    intent: str = ""
    nav_goal: str = ""
    reply_text: str = ""
    success: bool = False


class UIModeRouter(Node):
    """Node that routes robot state into simple UI mode + status text."""

    def __init__(self) -> None:
        super().__init__("savo_ui_mode_router")

        self.get_logger().info("Initializing UIModeRouter node")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("robot_id", "")
        self.declare_parameter("mapping_active_topic", "/savo_mapping/mapping_active")
        self.declare_parameter(
            "idle_status_text",
            "Hello, I am Robot Savo!",
        )
        self.declare_parameter(
            "mapping_status_text",
            "Mapping in progress, please keep distance.",
        )
        self.declare_parameter(
            "stopped_status_text",
            "Stopped here.",
        )
        self.declare_parameter(
            "status_from_reply",
            True,
        )

        self.robot_id_filter: str = (
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

        if self.robot_id_filter:
            self.get_logger().info(
                f"Filtering IntentResult by robot_id='{self.robot_id_filter}'"
            )
        else:
            self.get_logger().warn(
                "robot_id parameter is empty: accepting IntentResult from all robots."
            )

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self.pub_mode = self.create_publisher(String, "/savo_ui/mode", 10)
        self.pub_status = self.create_publisher(String, "/savo_ui/status_text", 10)

        # ------------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------------
        self.sub_intent = self.create_subscription(
            IntentResult,
            "/savo_intent/intent_result",
            self._on_intent_result,
            10,
        )

        self.mapping_active: bool = False
        self.sub_mapping: Optional[rclpy.subscription.Subscription] = None

        if self.mapping_active_topic:
            self.sub_mapping = self.create_subscription(
                Bool,
                self.mapping_active_topic,
                self._on_mapping_active,
                10,
            )
            self.get_logger().info(
                f"Subscribed to mapping active topic: {self.mapping_active_topic}"
            )
        else:
            self.get_logger().info(
                "mapping_active_topic is empty; MAP mode override disabled."
            )

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self._last_intent = LastIntentState()
        self._current_mode: str = ""       # last published mode
        self._current_status: str = ""     # last published status text

        # Immediately publish an initial state
        self._update_and_publish_ui_state(reason="initial")

    # ======================================================================
    # Callbacks
    # ======================================================================

    def _on_intent_result(self, msg: IntentResult) -> None:
        """Handle a new intent result from the LLM/intent pipeline."""

        # Robot ID filtering
        if self.robot_id_filter:
            if msg.robot_id != self.robot_id_filter:
                # Ignore messages intended for other robots
                return

        intent = (msg.intent or "").strip().upper()
        nav_goal = (msg.nav_goal or "").strip()
        reply_text = (msg.reply_text or "").strip()
        success = bool(msg.success)

        self._last_intent = LastIntentState(
            intent=intent,
            nav_goal=nav_goal,
            reply_text=reply_text,
            success=success,
        )

        self.get_logger().debug(
            f"Received IntentResult: intent={intent!r}, nav_goal={nav_goal!r}, "
            f"success={success}, reply_text='{reply_text[:60]}...'"
        )

        self._update_and_publish_ui_state(reason="intent_result")

    def _on_mapping_active(self, msg: Bool) -> None:
        """Handle mapping active flag, used to override UI into MAP mode."""
        self.mapping_active = bool(msg.data)
        self.get_logger().info(
            f"Mapping active flag updated: {self.mapping_active}"
        )
        self._update_and_publish_ui_state(reason="mapping_active")

    # ======================================================================
    # UI mode decision logic
    # ======================================================================

    def _update_and_publish_ui_state(self, *, reason: str) -> None:
        """
        Compute desired mode + status text and publish changes, if any.

        Priority:
        1. Mapping active (MAP mode override)
        2. NAVIGATE/FOLLOW intents with nav_goal
        3. STOP intent
        4. STATUS intent (optional: use reply_text)
        5. Fallback: INTERACT idle
        """

        mode, status_text = self._compute_ui_state()

        # Only log & publish when something actually changed
        if mode != self._current_mode or status_text != self._current_status:
            self.get_logger().info(
                f"UI state update ({reason}): mode={mode}, status='{status_text}'"
            )
            self._publish_mode_and_status(mode, status_text)
            self._current_mode = mode
            self._current_status = status_text

    def _compute_ui_state(self) -> tuple[str, str]:
        """Return (mode, status_text) according to current internal state."""

        # 1) Mapping override has highest priority
        if self.mapping_active:
            return "MAP", self.mapping_status_text

        intent = self._last_intent.intent
        nav_goal = self._last_intent.nav_goal
        reply_text = self._last_intent.reply_text
        success = self._last_intent.success

        # 2) Navigation / follow intent with a goal → NAVIGATE mode
        if intent in ("NAVIGATE", "FOLLOW") and nav_goal:
            # If the last request failed, keep INTERACT but show a gentle status
            if not success:
                msg = f"Tried to guide to {nav_goal}, but something went wrong."
                return "INTERACT", msg
            # Normal case: guiding to a known location
            return "NAVIGATE", f"Guiding to {nav_goal}"

        # 3) STOP intent → INTERACT with stopped text
        if intent == "STOP":
            return "INTERACT", self.stopped_status_text

        # 4) STATUS intent → INTERACT with robot's explanation
        if intent == "STATUS" and reply_text:
            if self.status_from_reply:
                # Clamp to a reasonable length so we don't overflow
                text = reply_text.strip()
                if len(text) > 120:
                    text = text[:117] + "..."
                return "INTERACT", text
            else:
                return "INTERACT", self.idle_status_text

        # 5) CHATBOT or anything else → INTERACT idle
        return "INTERACT", self.idle_status_text

    def _publish_mode_and_status(self, mode: str, status_text: str) -> None:
        """Publish /savo_ui/mode and /savo_ui/status_text."""
        msg_mode = String()
        msg_mode.data = mode
        self.pub_mode.publish(msg_mode)

        msg_status = String()
        msg_status.data = status_text
        self.pub_status.publish(msg_status)


# ==========================================================================#
# main()
# ==========================================================================#

def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = UIModeRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down UIModeRouter.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
