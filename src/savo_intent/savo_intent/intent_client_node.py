#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — ROS 2 Intent Client Node
-------------------------------------

This node is the ROS 2 bridge between the robot (Pi) and the LLM server.

Subscriptions
-------------
- /savo_intent/user_text   (std_msgs/String)
    * Text that the robot heard from the user (after STT).

Publications
------------
- /savo_intent/reply_text  (std_msgs/String)
    * Short spoken answer that TTS should say.

- /savo_intent/intent_result  (savo_msgs/IntentResult)
    * Full result from the LLM server:
        - source     ("system", "mic", "keyboard", "test", ...)
        - robot_id   (this robot, e.g. "robot_savo_pi")
        - user_text  (what the human said)
        - reply_text (what the robot should say)
        - intent     ("NAVIGATE", "FOLLOW", "STOP", "STATUS", "CHATBOT")
        - nav_goal   (canonical location like "A201" or "Info Desk" or "")
        - tier_used  ("tier1", "tier2", "tier3", or "tier_error")
        - success    (True if the LLM call succeeded)
        - error      (empty string on success, short reason on failure)

Parameters
----------
- llm_server_url (string)
    Default: "http://127.0.0.1:8000"
    (Typically overridden via speech_bringup.launch.py, which reads
     LLM_SERVER_URL from tools/scripts/env_llm.sh.)

- robot_id (string)
    Default: "robot_savo"

- timeout_s (double)
    Default: 20.0 seconds (clamped to [0.5, 20.0])

Usage example on the Pi
-----------------------

    cd ~/Savo_Pi
    source tools/scripts/env.sh
    source install/setup.bash

    ros2 run savo_intent intent_client_node \
      --ros-args \
      -p llm_server_url:="http://server_IP:8000" \
      -p robot_id:="robot_savo_pi"

In other terminals:

    # Send a test user text
    ros2 topic pub --once /savo_intent/user_text std_msgs/msg/String \
      "{data: 'Robot Savo, can you guide me to A201?'}"

    # See only the spoken reply
    ros2 topic echo /savo_intent/reply_text

    # See the full structured result
    ros2 topic echo /savo_intent/intent_result
"""

from __future__ import annotations

import json
import sys
import urllib.error
import urllib.request
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from savo_msgs.msg import IntentResult


class IntentClientNode(Node):
    """
    ROS 2 node that bridges ROS topics to the LLM HTTP API.
    """

    MIN_TIMEOUT_S: float = 0.5
    MAX_TIMEOUT_S: float = 20.0

    def __init__(self) -> None:
        super().__init__("intent_client_node")

        # ------------------------------------------------------------------
        # Declare parameters with defaults
        # ------------------------------------------------------------------
        self.declare_parameter("llm_server_url", "http://127.0.0.1:8000")
        self.declare_parameter("robot_id", "robot_savo")
        self.declare_parameter("timeout_s", 20.0)  # default 20 seconds

        # Read parameters safely
        self.llm_server_url = (
            self.get_parameter("llm_server_url").get_parameter_value().string_value
        ) or "http://127.0.0.1:8000"

        # Normalize base URL (strip trailing slash once)
        self.base_url = self.llm_server_url.rstrip("/")

        self.robot_id = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        ) or "robot_savo"

        # timeout_s may come as double/float; clamp into [MIN_TIMEOUT_S, MAX_TIMEOUT_S]
        timeout_value = 20.0
        try:
            timeout_value = (
                self.get_parameter("timeout_s").get_parameter_value().double_value
            )
        except Exception:
            # Keep default if parameter type is odd
            pass

        self.timeout_s = self._clamp_timeout(timeout_value)

        # ------------------------------------------------------------------
        # Publishers and subscribers
        # ------------------------------------------------------------------
        # Simple reply text for TTS
        self.reply_text_pub = self.create_publisher(
            String,
            "/savo_intent/reply_text",
            10,
        )

        # Full structured result from the LLM server
        self.intent_result_pub = self.create_publisher(
            IntentResult,
            "/savo_intent/intent_result",
            10,
        )

        # Incoming user text from speech system
        self.user_text_sub = self.create_subscription(
            String,
            "/savo_intent/user_text",
            self.user_text_callback,
            10,
        )

        # Log startup info (single string to avoid rcutils formatting issues)
        self.get_logger().info(
            "IntentClientNode starting with LLM server URL: %s, robot_id: %s, timeout: %.2fs"
            % (self.base_url, self.robot_id, self.timeout_s)
        )

        # Optional: health check at startup
        self._check_health_once()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _clamp_timeout(self, value: float) -> float:
        """Clamp timeout into [MIN_TIMEOUT_S, MAX_TIMEOUT_S]."""
        try:
            t = float(value)
        except Exception:
            t = 20.0
        if t < self.MIN_TIMEOUT_S:
            t = self.MIN_TIMEOUT_S
        if t > self.MAX_TIMEOUT_S:
            t = self.MAX_TIMEOUT_S
        return t

    # ------------------------------------------------------------------
    # Health check helper
    # ------------------------------------------------------------------
    def _check_health_once(self) -> None:
        """Call /health on the LLM server once at startup."""
        url = self.base_url + "/health"
        self.get_logger().info("Checking LLM server /health at: %s" % url)
        try:
            req = urllib.request.Request(url, method="GET")
            with urllib.request.urlopen(req, timeout=5.0) as resp:
                data = resp.read().decode("utf-8", errors="ignore")
            self.get_logger().info("LLM server /health response: %s" % data)
        except Exception as exc:
            # Don't crash if health is down; just warn.
            self.get_logger().warn(
                "LLM server /health check failed: %r" % (exc,)
            )

    # ------------------------------------------------------------------
    # ROS callback
    # ------------------------------------------------------------------
    def user_text_callback(self, msg: String) -> None:
        """Handle incoming user_text, call LLM, and publish reply_text + IntentResult."""
        user_text = msg.data or ""
        user_text_stripped = user_text.strip()

        if not user_text_stripped:
            # Ignore empty strings, but log once for debugging
            self.get_logger().info("Received empty user_text; ignoring.")
            return

        self.get_logger().info("Received user_text: %r" % (user_text_stripped,))

        (
            reply_text,
            intent,
            nav_goal,
            tier_used,
            session_id,
            success,
            error_msg,
        ) = self.call_llm_chat(user_text_stripped)

        # ------------------------------------------------------------------
        # Publish reply_text (for TTS)
        # ------------------------------------------------------------------
        reply_msg = String()
        reply_msg.data = reply_text
        self.reply_text_pub.publish(reply_msg)

        self.get_logger().info("Published reply_text: %r" % (reply_text,))

        # ------------------------------------------------------------------
        # Publish full IntentResult
        # ------------------------------------------------------------------
        result_msg = IntentResult()

        # If later you add a stamp field to IntentResult.msg, you can set:
        #   result_msg.stamp = self.get_clock().now().to_msg()

        result_msg.source = "system"
        result_msg.robot_id = self.robot_id

        result_msg.user_text = user_text_stripped
        result_msg.reply_text = reply_text
        result_msg.intent = intent or ""
        result_msg.nav_goal = nav_goal or ""
        result_msg.tier_used = tier_used or ""
        result_msg.success = success
        result_msg.error = error_msg or ""

        self.intent_result_pub.publish(result_msg)
        self.get_logger().info(
            "Published IntentResult: intent=%r, nav_goal=%r, tier=%r, success=%r, error=%r"
            % (
                result_msg.intent,
                result_msg.nav_goal,
                result_msg.tier_used,
                result_msg.success,
                result_msg.error,
            )
        )

    # ------------------------------------------------------------------
    # HTTP helper
    # ------------------------------------------------------------------
    def call_llm_chat(
        self,
        user_text: str,
    ) -> Tuple[
        str,
        Optional[str],
        Optional[str],
        Optional[str],
        Optional[str],
        bool,
        str,
    ]:
        """
        Call /chat on the LLM server.

        Returns:
            (reply_text, intent, nav_goal, tier_used, session_id, success, error_msg)

        On any error or timeout, we return a safe fallback reply and
        success = False with a short error string.
        """
        url = self.base_url + "/chat"
        self.get_logger().info("Calling LLM /chat at: %s" % url)

        # Build JSON body following ChatRequest schema
        body: Dict[str, Any] = {
            "user_text": user_text,
            "source": "system",  # from robot pipeline / ROS bridge
            "session_id": self.robot_id,
            "language": "en",
            "meta": {
                "client": "savo_intent",
                "robot_id": self.robot_id,
                "via": "ros2_bridge",
            },
        }

        data_bytes = json.dumps(body).encode("utf-8")
        req = urllib.request.Request(
            url,
            data=data_bytes,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        # Default fallback values
        fallback_reply = (
            "Sorry, my brain server has a problem now. I cannot handle this request."
        )

        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                resp_raw = resp.read().decode("utf-8", errors="ignore")
        except urllib.error.HTTPError as exc:
            self.get_logger().error("HTTPError calling LLM /chat: %s" % (exc,))
            return fallback_reply, None, None, "tier_error", None, False, "http_error"
        except urllib.error.URLError as exc:
            self.get_logger().error("URLError calling LLM /chat: %s" % (exc,))
            # URLError often wraps timeouts as well
            reason = getattr(exc, "reason", None)
            if isinstance(reason, TimeoutError):
                slow_reply = (
                    "Sorry, my brain server is too slow right now. I cannot answer this request."
                )
                return slow_reply, None, None, "tier_error", None, False, "timeout"
            return fallback_reply, None, None, "tier_error", None, False, "url_error"
        except TimeoutError as exc:
            self.get_logger().error(
                "Error calling LLM /chat (TimeoutError): %r" % (exc,)
            )
            slow_reply = (
                "Sorry, my brain server is too slow right now. I cannot answer this request."
            )
            return slow_reply, None, None, "tier_error", None, False, "timeout"
        except Exception as exc:
            self.get_logger().error(
                "Unexpected error calling LLM /chat: %r" % (exc,)
            )
            return fallback_reply, None, None, "tier_error", None, False, "exception"

        # Parse JSON reply
        try:
            obj = json.loads(resp_raw)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                "Failed to parse LLM /chat JSON: %r, raw=%r" % (exc, resp_raw)
            )
            return fallback_reply, None, None, "tier_error", None, False, "json_error"

        reply_text = obj.get("reply_text") or ""
        intent = obj.get("intent")
        nav_goal = obj.get("nav_goal")
        session_id = obj.get("session_id")
        tier_used = obj.get("tier_used")

        if not reply_text:
            self.get_logger().warn(
                "LLM /chat response missing reply_text; raw=%r" % (obj,)
            )
            reply_text = fallback_reply
            success = False
            error_msg = "missing_reply_text"
        else:
            success = True
            error_msg = ""

        self.get_logger().info(
            "LLM /chat success. intent=%r, nav_goal=%r, tier=%r, session_id=%r, reply_text=%r"
            % (intent, nav_goal, tier_used, session_id, reply_text)
        )

        return reply_text, intent, nav_goal, tier_used, session_id, success, error_msg


# ---------------------------------------------------------------------------
# main()
# ---------------------------------------------------------------------------


def main(argv: Optional[list[str]] = None) -> None:
    if argv is None:
        argv = sys.argv

    rclpy.init(args=argv)
    node = IntentClientNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down IntentClientNode (Ctrl+C).")
    finally:
        # Best-effort shutdown; ignore "already shutdown" errors
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
