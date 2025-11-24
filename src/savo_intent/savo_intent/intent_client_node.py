#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Intent Client Node (ROS2 ↔ LLM server bridge)

This node:
  - Subscribes to:  /savo_intent/user_text   (std_msgs/String)
  - Calls LLM HTTP: POST {llm_server_url}/chat
  - Publishes to:   /savo_intent/reply_text  (std_msgs/String)

Parameters:
  - llm_server_url (string)
      Default: "http://127.0.0.1:8000"
      Example: "http://192.168.164.119:8000"
  - robot_id (string)
      Default: "robot_savo_pi"
      Used to build session_id for the LLM server.
  - timeout_s (double)
      Default: 20.0
      Socket timeout for the HTTP call to /chat (seconds).
      For navigation/simple questions we will usually use 8–12 s,
      but 20.0 gives some headroom for heavy live-info questions.

Usage example (on Pi):

  cd ~/Savo_Pi
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash

  ros2 run savo_intent intent_client_node \
    --ros-args \
    -p llm_server_url:="http://192.168.164.119:8000" \
    -p robot_id:="robot_savo_pi" \
    -p timeout_s:=10.0
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
    """ROS2 node that bridges /savo_intent/user_text to the LLM /chat endpoint."""

    def __init__(self) -> None:
        super().__init__("intent_client_node")

        # ------------------------------------------------------------------
        # Declare parameters with sensible defaults
        # ------------------------------------------------------------------
        self.declare_parameter("llm_server_url", "http://127.0.0.1:8000")
        self.declare_parameter("robot_id", "robot_savo_pi")
        self.declare_parameter("timeout_s", 20.0)  # <-- default 20 seconds

        self.llm_server_url: str = (
            self.get_parameter("llm_server_url").get_parameter_value().string_value
        )
        self.robot_id: str = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        )

        # Timeout as float (no fancy type logic, keep it simple)
        try:
            self.timeout_s: float = float(
                self.get_parameter("timeout_s").get_parameter_value().double_value
            )
        except Exception:  # noqa: BLE001
            # Fallback if something weird happens
            self.timeout_s = 20.0

        # ------------------------------------------------------------------
        # Publishers and subscribers
        # ------------------------------------------------------------------
        self.reply_pub = self.create_publisher(String, "/savo_intent/reply_text", 10)
        self.user_sub = self.create_subscription(
            String,
            "/savo_intent/user_text",
            self._on_user_text,
            10,
        )

        # ------------------------------------------------------------------
        # Log configuration + initial /health check
        # ------------------------------------------------------------------
        self.get_logger().info(
            "IntentClientNode starting with LLM server URL: %s, robot_id: %s, timeout: %.2fs",
            self.llm_server_url,
            self.robot_id,
            self.timeout_s,
        )

        self._check_llm_health()

    # ----------------------------------------------------------------------
    # HTTP helpers
    # ----------------------------------------------------------------------

    def _build_url(self, path: str) -> str:
        base = self.llm_server_url.rstrip("/")
        if not path.startswith("/"):
            path = "/" + path
        return base + path

    def _http_get(self, path: str) -> Optional[str]:
        """Simple HTTP GET with timeout, returns response text or None on error."""
        url = self._build_url(path)
        self.get_logger().info("Checking LLM server /health at: %s", url)

        try:
            with urllib.request.urlopen(url, timeout=self.timeout_s) as resp:
                data = resp.read().decode("utf-8", errors="replace")
                return data
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                "Failed GET %s: %r",
                url,
                exc,
            )
            return None

    def _http_post_json(self, path: str, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Simple HTTP POST with JSON body.

        Returns parsed JSON dict on success, or None on any error/timeout.
        """
        url = self._build_url(path)
        body = json.dumps(payload).encode("utf-8")

        req = urllib.request.Request(
            url,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        self.get_logger().info("Calling LLM /chat at: %s", url)

        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                resp_text = resp.read().decode("utf-8", errors="replace")
        except urllib.error.HTTPError as exc:
            self.get_logger().error(
                "HTTPError calling LLM /chat: %s %s",
                exc.code,
                exc.reason,
            )
            return None
        except urllib.error.URLError as exc:
            # URLError wraps connection issues and timeouts
            self.get_logger().error(
                "URLError calling LLM /chat: %r",
                exc,
            )
            return None
        except TimeoutError as exc:  # rarely raised directly
            self.get_logger().error(
                "TimeoutError calling LLM /chat: %r",
                exc,
            )
            return None
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                "Unexpected error calling LLM /chat: %r",
                exc,
            )
            return None

        # Try to parse JSON
        try:
            return json.loads(resp_text)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                "Failed to decode /chat JSON response: %r (text=%r)",
                exc,
                resp_text,
            )
            return None

    # ----------------------------------------------------------------------
    # Node logic
    # ----------------------------------------------------------------------

    def _check_llm_health(self) -> None:
        """Call /health once on startup and log the result."""
        resp_text = self._http_get("/health")
        if resp_text is None:
            self.get_logger().warn(
                "LLM server /health check failed. The server may be down."
            )
            return

        self.get_logger().info(
            "LLM server /health response: %s",
            resp_text,
        )

    def _on_user_text(self, msg: String) -> None:
        """Callback for /savo_intent/user_text messages."""
        user_text = (msg.data or "").strip()
        if not user_text:
            self.get_logger().warn("Received empty user_text; ignoring.")
            return

        self.get_logger().info(
            "Received user_text: %r",
            user_text,
        )

        payload: Dict[str, Any] = {
            "user_text": user_text,
            # For now we use 'system' as source (from ROS).
            # Later we can switch to 'mic' or 'keyboard' depending on real input.
            "source": "system",
            "session_id": f"ros2-{self.robot_id}",
            "meta": {
                "client": "ros2-intent-bridge",
                "robot_id": self.robot_id,
            },
        }

        data = self._http_post_json("/chat", payload)

        # If call failed, publish fallback reply.
        if data is None:
            fallback = "Sorry, my brain server has a problem now. I cannot handle this request."
            self._publish_reply(fallback)
            return

        # Extract reply_text / intent / nav_goal from ChatResponse JSON
        reply_text = data.get("reply_text") or ""
        intent = data.get("intent")
        nav_goal = data.get("nav_goal")

        self.get_logger().info(
            "LLM /chat success. intent=%s, nav_goal=%s, reply_text=%r",
            intent,
            nav_goal,
            reply_text,
        )

        if not reply_text:
            reply_text = (
                "Sorry, I did not get a valid answer from my brain server. "
                "Please try again."
            )

        self._publish_reply(reply_text)

    def _publish_reply(self, text: str) -> None:
        """Publish reply_text on /savo_intent/reply_text."""
        msg = String()
        msg.data = text
        self.reply_pub.publish(msg)
        self.get_logger().info(
            "Published reply_text: %r",
            text,
        )


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
        # Best-effort clean shutdown; ignore "already shutdown" errors.
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
