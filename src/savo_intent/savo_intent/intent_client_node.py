#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — ROS2 Intent Client Node
------------------------------------

This node is the ROS2 bridge between the robot (Pi) and the LLM server.

- Subscribes to:  /savo_intent/user_text   (std_msgs/String)
    * Text that the robot heard from the user (after STT).

- Calls:          HTTP POST /chat on the LLM server
    * Sends user_text, robot_id (as session_id), and a small meta block.

- Publishes to:   /savo_intent/reply_text  (std_msgs/String)
    * Short spoken answer that TTS should say.

Parameters:
- llm_server_url (string)
    Default: "http://127.0.0.1:8000"
- robot_id (string)
    Default: "robot_savo"
- timeout_s (double)
    Default: 20.0 seconds (max allowed; you can override via ROS params)

Usage example on the Pi:

    cd ~/Savo_Pi
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash

    ros2 run savo_intent intent_client_node \
      --ros-args \
      -p llm_server_url:="http://192.168.164.119:8000" \
      -p robot_id:="robot_savo_pi"

Then in another terminal:

    ros2 topic pub --once /savo_intent/user_text std_msgs/msg/String \
      "{data: 'Robot Savo, can you guide me to A201?'}"

    ros2 topic echo /savo_intent/reply_text
"""

from __future__ import annotations

import json
import sys
import urllib.error
import urllib.request
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class IntentClientNode(Node):
    """
    ROS2 node that bridges ROS topics to the LLM HTTP API.
    """

    def __init__(self) -> None:
        super().__init__("intent_client_node")

        # ------------------------------------------------------------------
        # Declare parameters with defaults
        # ------------------------------------------------------------------
        self.declare_parameter("llm_server_url", "http://127.0.0.1:8000")
        self.declare_parameter("robot_id", "robot_savo")
        self.declare_parameter("timeout_s", 20.0)  # default 20 seconds

        # Read parameters safely
        self.llm_server_url: str = (
            self.get_parameter("llm_server_url").get_parameter_value().string_value
        )
        self.robot_id: str = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        )

        try:
            self.timeout_s: float = float(
                self.get_parameter("timeout_s").get_parameter_value().double_value
            )
        except Exception:
            self.timeout_s = 20.0

        # Clamp timeout between 0.5 and 20.0 seconds
        if self.timeout_s < 0.5:
            self.timeout_s = 0.5
        if self.timeout_s > 20.0:
            self.timeout_s = 20.0

        # ------------------------------------------------------------------
        # Publishers and subscribers
        # ------------------------------------------------------------------
        self.reply_text_pub = self.create_publisher(
            String,
            "/savo_intent/reply_text",
            10,
        )
        self.user_text_sub = self.create_subscription(
            String,
            "/savo_intent/user_text",
            self.user_text_callback,
            10,
        )

        # Log startup info (only ONE formatted string, to avoid rcutils error)
        self.get_logger().info(
            "IntentClientNode starting with LLM server URL: %s, robot_id: %s, timeout: %.2fs"
            % (self.llm_server_url, self.robot_id, self.timeout_s)
        )

        # Optional: health check at startup
        self._check_health_once()

    # ------------------------------------------------------------------
    # Health check helper
    # ------------------------------------------------------------------
    def _check_health_once(self) -> None:
        """Call /health on the LLM server once at startup."""
        url = self.llm_server_url.rstrip("/") + "/health"
        self.get_logger().info(
            "Checking LLM server /health at: %s" % url
        )
        try:
            req = urllib.request.Request(url, method="GET")
            with urllib.request.urlopen(req, timeout=5.0) as resp:
                data = resp.read().decode("utf-8", errors="ignore")
            self.get_logger().info(
                "LLM server /health response: %s" % data
            )
        except Exception as exc:
            self.get_logger().warn(
                "LLM server /health check failed: %r" % (exc,)
            )

    # ------------------------------------------------------------------
    # ROS callback
    # ------------------------------------------------------------------
    def user_text_callback(self, msg: String) -> None:
        """Handle incoming user_text, call LLM, and publish reply_text."""
        user_text = msg.data or ""
        user_text_stripped = user_text.strip()

        if not user_text_stripped:
            # Ignore empty strings, but log once for debugging
            self.get_logger().info("Received empty user_text; ignoring.")
            return

        self.get_logger().info(
            "Received user_text: %r" % (user_text_stripped,)
        )

        reply_text, intent, nav_goal = self.call_llm_chat(user_text_stripped)

        # For now, we only publish reply_text.
        out = String()
        out.data = reply_text
        self.reply_text_pub.publish(out)

        self.get_logger().info(
            "Published reply_text: %r" % (reply_text,)
        )
        # Later we can publish intent/nav_goal via a custom message.

    # ------------------------------------------------------------------
    # HTTP helper
    # ------------------------------------------------------------------
    def call_llm_chat(self, user_text: str) -> tuple[str, Optional[str], Optional[str]]:
        """
        Call /chat on the LLM server and return (reply_text, intent, nav_goal).

        On any error or timeout, we return a safe fallback reply and (None, None).
        """
        url = self.llm_server_url.rstrip("/") + "/chat"
        self.get_logger().info(
            "Calling LLM /chat at: %s" % url
        )

        # Build JSON body following ChatRequest schema
        body: Dict[str, Any] = {
            "user_text": user_text,
            "source": "system",  # from robot pipeline
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

        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                resp_raw = resp.read().decode("utf-8", errors="ignore")
        except urllib.error.HTTPError as exc:
            self.get_logger().error(
                "HTTPError calling LLM /chat: %s" % (exc,)
            )
            return (
                "Sorry, my brain server has a problem now. I cannot handle this request.",
                None,
                None,
            )
        except urllib.error.URLError as exc:
            self.get_logger().error(
                "URLError calling LLM /chat: %s" % (exc,)
            )
            return (
                "Sorry, my brain server has a problem now. I cannot handle this request.",
                None,
                None,
            )
        except TimeoutError as exc:
            self.get_logger().error(
                "Error calling LLM /chat (TimeoutError): %r" % (exc,)
            )
            return (
                "Sorry, my brain server is too slow right now. I cannot answer this request.",
                None,
                None,
            )
        except Exception as exc:
            self.get_logger().error(
                "Unexpected error calling LLM /chat: %r" % (exc,)
            )
            return (
                "Sorry, my brain server has a problem now. I cannot handle this request.",
                None,
                None,
            )

        # Parse JSON reply
        try:
            obj = json.loads(resp_raw)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                "Failed to parse LLM /chat JSON: %r, raw=%r" % (exc, resp_raw)
            )
            return (
                "Sorry, my brain server has a problem now. I cannot handle this request.",
                None,
                None,
            )

        reply_text = obj.get("reply_text") or ""
        intent = obj.get("intent")
        nav_goal = obj.get("nav_goal")

        if not reply_text:
            self.get_logger().warn(
                "LLM /chat response missing reply_text; raw=%r" % (obj,)
            )
            reply_text = (
                "Sorry, my brain server has a problem now. I cannot handle this request."
            )

        self.get_logger().info(
            "LLM /chat success. intent=%r, nav_goal=%r, reply_text=%r"
            % (intent, nav_goal, reply_text)
        )

        return reply_text, intent, nav_goal


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
