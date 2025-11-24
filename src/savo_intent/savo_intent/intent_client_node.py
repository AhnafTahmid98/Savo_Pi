#!/usr/bin/env python3
"""
Robot Savo — LLM Intent Client Node

This ROS2 node acts as a bridge between ROS and the Robot Savo LLM server.

ROS topics:
- Subscribes: /savo_intent/user_text   (std_msgs/String)
- Publishes : /savo_intent/reply_text  (std_msgs/String)

It sends user_text to the LLM server /chat endpoint and publishes the reply_text.
"""

import json
import urllib.request
import urllib.error
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class IntentClientNode(Node):
    """
    ROS2 node that talks to the LLM server and bridges user_text <-> reply_text.
    """

    def __init__(self) -> None:
        super().__init__("intent_client_node")

        # --- Parameters -----------------------------------------------------
        llm_server_param = self.declare_parameter(
            "llm_server_url",
            "http://127.0.0.1:8000",
        )
        robot_id_param = self.declare_parameter(
            "robot_id",
            "robot_savo_pi",
        )
        timeout_param = self.declare_parameter(
            "timeout_s",
            1.8,
        )

        self.llm_server_url = str(llm_server_param.value).rstrip("/")
        self.robot_id = str(robot_id_param.value)

        # Parse timeout as float safely
        try:
            self.timeout_s = float(timeout_param.value)
        except Exception:
            self.get_logger().warn(
                f"Invalid timeout_s parameter '{timeout_param.value}', "
                "falling back to 1.8s"
            )
            self.timeout_s = 1.8

        # Session ID: we keep one session per robot instance
        self.session_id = f"ros2-{self.robot_id}"

        self.get_logger().info(
            "IntentClientNode starting with "
            f"LLM server URL: {self.llm_server_url}, "
            f"robot_id: {self.robot_id}, "
            f"timeout: {self.timeout_s:.2f}s"
        )

        # --- ROS I/O --------------------------------------------------------
        self.user_sub = self.create_subscription(
            String,
            "/savo_intent/user_text",
            self.user_text_callback,
            10,
        )

        self.reply_pub = self.create_publisher(
            String,
            "/savo_intent/reply_text",
            10,
        )

        # --- Optional: initial health check --------------------------------
        self._check_health_once()

    # ----------------------------------------------------------------------
    # Internal helpers
    # ----------------------------------------------------------------------
    def _check_health_once(self) -> None:
        """Check LLM /health endpoint once at startup and log the result."""
        url = f"{self.llm_server_url}/health"
        self.get_logger().info(f"Checking LLM server /health at: {url}")

        try:
            with urllib.request.urlopen(url, timeout=self.timeout_s) as resp:
                data = resp.read().decode("utf-8", errors="ignore")
                self.get_logger().info(f"LLM server /health response: {data}")
        except Exception as exc:
            self.get_logger().error(
                f"Failed to reach LLM server /health: {exc!r}. "
                "Node will still run, but calls may fail."
            )

    def _call_llm_chat(self, user_text: str) -> Optional[str]:
        """
        Call the LLM server /chat endpoint with user_text.

        Returns:
            reply_text as string if successful, otherwise None.
        """
        url = f"{self.llm_server_url}/chat"
        payload = {
            "user_text": user_text,
            # For now we use 'system'. Later we can distinguish 'mic' vs 'keyboard'.
            "source": "system",
            "session_id": self.session_id,
        }

        body = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            url,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        self.get_logger().info(f"Calling LLM /chat at: {url}")

        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                raw = resp.read().decode("utf-8", errors="ignore")
        except urllib.error.URLError as exc:
            self.get_logger().error(f"Error calling LLM /chat: {exc!r}")
            return None
        except Exception as exc:
            self.get_logger().error(f"Unexpected error calling LLM /chat: {exc!r}")
            return None

        # Parse JSON
        try:
            data = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f"Failed to parse LLM /chat JSON: {exc!r}, raw={raw!r}"
            )
            return None

        reply_text = data.get("reply_text")
        if not isinstance(reply_text, str):
            self.get_logger().error(
                f"LLM /chat response missing valid 'reply_text': {data!r}"
            )
            return None

        self.get_logger().info(
            f"LLM /chat success. intent={data.get('intent')}, "
            f"nav_goal={data.get('nav_goal')}, reply_text={reply_text!r}"
        )
        return reply_text

    def _publish_reply(self, text: str) -> None:
        """Publish reply_text on /savo_intent/reply_text."""
        msg = String()
        msg.data = text
        self.reply_pub.publish(msg)
        self.get_logger().info(f"Published reply_text: {text!r}")

    def _publish_fallback(self) -> None:
        """Publish a safe fallback reply if LLM call fails."""
        fallback = (
            "Sorry, my brain server has a problem now. "
            "I cannot handle this request."
        )
        self._publish_reply(fallback)

    # ----------------------------------------------------------------------
    # ROS callbacks
    # ----------------------------------------------------------------------
    def user_text_callback(self, msg: String) -> None:
        """Handle incoming user_text from ROS."""
        user_text = msg.data.strip()
        if not user_text:
            self.get_logger().warn("Received empty user_text; ignoring.")
            return

        self.get_logger().info(f"Received user_text: {user_text!r}")

        reply = self._call_llm_chat(user_text)
        if reply is None:
            self._publish_fallback()
        else:
            self._publish_reply(reply)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = IntentClientNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Shutting down IntentClientNode (Ctrl+C).")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
