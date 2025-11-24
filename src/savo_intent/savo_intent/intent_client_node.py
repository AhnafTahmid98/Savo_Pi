# src/savo_intent/savo_intent/intent_client_node.py
# -*- coding: utf-8 -*-
"""
Robot Savo — Intent Client Node (ROS 2 ↔ LLM bridge)

This node:
- Subscribes to: /savo_intent/user_text   (std_msgs/String)
- Calls LLM server: POST /chat            (FastAPI server on PC/laptop)
- Publishes to: /savo_intent/reply_text   (std_msgs/String, for TTS etc.)

Workflow:
    /savo_intent/user_text  --->  LLM /chat  --->  /savo_intent/reply_text

It is intentionally simple:
- Only sends user_text, source="system", and a stable session_id.
- Only publishes reply_text (later we will add intent + nav_goal as a custom msg).
- Uses a configurable HTTP timeout (timeout_s), default 12.0 s.

ROS params:
    llm_server_url (string)  : default "http://127.0.0.1:8000"
    robot_id       (string)  : default "robot_savo"
    timeout_s      (double)  : default 12.0
"""

from __future__ import annotations

import json
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import urllib.request
import urllib.error


class IntentClientNode(Node):
    """
    ROS2 node that bridges Robot Savo <-> LLM server.

    - Listens for plain text on /savo_intent/user_text.
    - Calls the LLM server's /chat endpoint.
    - Publishes the reply text on /savo_intent/reply_text.
    """

    def __init__(self) -> None:
        super().__init__("intent_client_node")

        # ------------------------------------------------------------------
        # Declare & read parameters
        # ------------------------------------------------------------------
        self.declare_parameter("llm_server_url", "http://127.0.0.1:8000")
        self.declare_parameter("robot_id", "robot_savo")
        # Default timeout: 12 seconds (can be overridden from CLI / launch)
        self.declare_parameter("timeout_s", 12.0)

        llm_url_param = self.get_parameter("llm_server_url")
        robot_id_param = self.get_parameter("robot_id")
        timeout_param = self.get_parameter("timeout_s")

        self.llm_server_url: str = str(llm_url_param.value).rstrip("/")
        self.robot_id: str = str(robot_id_param.value)
        try:
            self.timeout_s: float = float(timeout_param.value)
        except (TypeError, ValueError):
            self.get_logger().warn(
                "Invalid timeout_s parameter value; falling back to 12.0 s"
            )
            self.timeout_s = 12.0

        # Stable session_id so the LLM can keep context per robot
        self.session_id: str = f"{self.robot_id}-session"

        # Fallback message if LLM/server fails
        self.fallback_reply: str = (
            "Sorry, my brain server has a problem now. I cannot handle this request."
        )

        self.get_logger().info(
            "IntentClientNode starting with LLM server URL: %s, robot_id: %s, "
            "timeout: %.2fs",
            self.llm_server_url,
            self.robot_id,
            self.timeout_s,
        )

        # ------------------------------------------------------------------
        # ROS publishers & subscribers
        # ------------------------------------------------------------------
        self.reply_pub = self.create_publisher(String, "/savo_intent/reply_text", 10)
        self.user_sub = self.create_subscription(
            String,
            "/savo_intent/user_text",
            self._on_user_text,
            10,
        )

        # ------------------------------------------------------------------
        # Initial /health check (best-effort)
        # ------------------------------------------------------------------
        self._check_llm_health()

    # ----------------------------------------------------------------------
    # Health check
    # ----------------------------------------------------------------------

    def _check_llm_health(self) -> None:
        """Call LLM server /health once at startup (best-effort)."""
        health_url = f"{self.llm_server_url}/health"
        self.get_logger().info("Checking LLM server /health at: %s", health_url)

        try:
            req = urllib.request.Request(health_url, method="GET")
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                body = resp.read().decode("utf-8", errors="replace")
                self.get_logger().info("LLM server /health response: %s", body)
        except Exception as exc:
            # We just log; node can still run and publish fallback replies.
            self.get_logger().error(
                "Failed to reach LLM server /health: %r", exc
            )

    # ----------------------------------------------------------------------
    # ROS callback: incoming user_text from robot
    # ----------------------------------------------------------------------

    def _on_user_text(self, msg: String) -> None:
        """Handle incoming text from /savo_intent/user_text."""
        user_text = (msg.data or "").strip()
        if not user_text:
            self.get_logger().warn("Received empty user_text; ignoring.")
            return

        self.get_logger().info("Received user_text: %r", user_text)
        reply = self._call_llm_chat(user_text)
        self._publish_reply(reply)

    # ----------------------------------------------------------------------
    # HTTP call to LLM /chat
    # ----------------------------------------------------------------------

    def _call_llm_chat(self, user_text: str) -> str:
        """
        Call LLM server /chat and return the reply_text.

        On any error or timeout, returns the fallback reply.
        """
        chat_url = f"{self.llm_server_url}/chat"
        self.get_logger().info("Calling LLM /chat at: %s", chat_url)

        payload = {
            "user_text": user_text,
            # For the API: valid sources are: "mic", "keyboard", "system", "test"
            "source": "system",
            # Stable per-robot session for conversation history:
            "session_id": self.session_id,
        }

        data_bytes = json.dumps(payload).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        req = urllib.request.Request(chat_url, data=data_bytes, headers=headers, method="POST")

        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                body = resp.read().decode("utf-8", errors="replace")

            # Parse JSON response
            parsed = json.loads(body)
            reply_text: Optional[str] = parsed.get("reply_text")
            intent: Optional[str] = parsed.get("intent")
            nav_goal: Optional[str] = parsed.get("nav_goal")

            if reply_text:
                self.get_logger().info(
                    "LLM /chat success. intent=%s, nav_goal=%s, reply_text=%r",
                    intent,
                    nav_goal,
                    reply_text,
                )
                return reply_text

            # If no reply_text, log and fallback
            self.get_logger().error(
                "LLM /chat response missing 'reply_text'. Raw body: %s",
                body,
            )
            return self.fallback_reply

        except json.JSONDecodeError as exc:
            self.get_logger().error(
                "Failed to decode LLM /chat JSON response: %r", exc
            )
        except urllib.error.URLError as exc:
            self.get_logger().error(
                "Error calling LLM /chat (URLError): %r", exc
            )
        except TimeoutError as exc:
            self.get_logger().error(
                "Error calling LLM /chat (TimeoutError): %r", exc
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                "Unexpected error calling LLM /chat: %r", exc
            )

        # Any exception path returns fallback
        return self.fallback_reply

    # ----------------------------------------------------------------------
    # Publish helpers
    # ----------------------------------------------------------------------

    def _publish_reply(self, text: str) -> None:
        """Publish reply_text onto /savo_intent/reply_text."""
        msg = String()
        msg.data = text
        self.reply_pub.publish(msg)
        self.get_logger().info("Published reply_text: %r", text)


def main(args=None) -> None:
    """ROS2 entry point."""
    rclpy.init(args=args)
    node: Optional[IntentClientNode] = None

    try:
        node = IntentClientNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Shutting down IntentClientNode (Ctrl+C).")
        else:
            print("Shutting down IntentClientNode (Ctrl+C).")
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as exc:  # noqa: BLE001
            # Avoid crashing on "rcl_shutdown already called"
            print(f"rclpy.shutdown() failed: {exc!r}")


if __name__ == "__main__":
    main()
