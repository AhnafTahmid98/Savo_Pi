#!/usr/bin/env python3
"""
Robot Savo — Intent Client Node (ROS2 ↔ LLM server bridge)

- Subscribes to: /savo_intent/user_text   (std_msgs/String)
- Calls:        LLM server /health and /chat (HTTP)
- Publishes to: /savo_intent/reply_text  (std_msgs/String)

Parameters:
  - llm_server_url (string): "http://<PC_IP>:8000"
  - robot_id       (string): "robot_savo_pi" (for session_id)
  - timeout_s      (double): HTTP timeout in seconds (default 4.0)
"""

import json
import sys
import urllib.error
import urllib.request
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class IntentClientNode(Node):
    def __init__(self) -> None:
        super().__init__("intent_client_node")

        # --- Declare parameters with defaults ---
        self.declare_parameter("llm_server_url", "http://127.0.0.1:8000")
        self.declare_parameter("robot_id", "robot_savo_pi")
        self.declare_parameter("timeout_s", 4.0)

        # --- Read parameters (values from launch/CLI override defaults) ---
        self.llm_server_url: str = str(
            self.get_parameter("llm_server_url").get_parameter_value().string_value
        )
        self.robot_id: str = str(
            self.get_parameter("robot_id").get_parameter_value().string_value
        )
        # Use double_value; fallback to default if not set as double
        timeout_param = self.get_parameter("timeout_s").get_parameter_value()
        if timeout_param.type_ == timeout_param.Type.DOUBLE:
            self.timeout_s = float(timeout_param.double_value)
        else:
            # Fallback path (e.g. if parameter came as integer)
            try:
                self.timeout_s = float(timeout_param.integer_value)
            except Exception:
                self.timeout_s = 4.0

        self.get_logger().info(
            f"IntentClientNode starting with LLM server URL: {self.llm_server_url}, "
            f"robot_id: {self.robot_id}, timeout: {self.timeout_s:.2f}s"
        )

        # --- Publishers / Subscribers ---
        self.reply_pub = self.create_publisher(String, "/savo_intent/reply_text", 10)
        self.user_sub = self.create_subscription(
            String,
            "/savo_intent/user_text",
            self.user_text_callback,
            10,
        )

        # Check /health once on startup so we see if the LLM server is alive
        self._check_health_once()

    # ------------------------------------------------------------------
    # HTTP helpers
    # ------------------------------------------------------------------
    def _build_url(self, path: str) -> str:
        base = self.llm_server_url.rstrip("/")
        path = path.lstrip("/")
        return f"{base}/{path}"

    def _http_post_json(self, url: str, payload: dict) -> Optional[dict]:
        """
        Simple blocking HTTP POST with timeout and JSON decode.
        Returns dict on success, None on any error.
        """
        try:
            data = json.dumps(payload).encode("utf-8")
            req = urllib.request.Request(
                url,
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                body = resp.read().decode("utf-8")
            return json.loads(body)
        except urllib.error.URLError as e:
            self.get_logger().error(
                f"Error calling {url}: {e}. Payload={payload!r}"
            )
        except Exception as e:
            self.get_logger().error(
                f"Unexpected error calling {url}: {e}. Payload={payload!r}"
            )
        return None

    # ------------------------------------------------------------------
    # Startup /health check
    # ------------------------------------------------------------------
    def _check_health_once(self) -> None:
        url = self._build_url("/health")
        self.get_logger().info(f"Checking LLM server /health at: {url}")
        try:
            with urllib.request.urlopen(url, timeout=self.timeout_s) as resp:
                body = resp.read().decode("utf-8")
            self.get_logger().info(f"LLM server /health response: {body}")
        except Exception as e:
            self.get_logger().error(
                f"Failed to reach LLM server /health: {e}. "
                f"Robot will still run but will use fallback replies."
            )

    # ------------------------------------------------------------------
    # ROS callback for incoming text
    # ------------------------------------------------------------------
    def user_text_callback(self, msg: String) -> None:
        user_text = msg.data.strip()
        if not user_text:
            return

        self.get_logger().info(f"Received user_text: {user_text!r}")

        url = self._build_url("/chat")
        payload = {
            "user_text": user_text,
            # 'source' must be one of: 'mic', 'keyboard', 'system', 'test'
            "source": "system",
            # Use robot_id as session_id so the server keeps history per robot
            "session_id": self.robot_id,
        }

        response = self._http_post_json(url, payload)
        if response is None:
            # HTTP error or timeout -> publish fallback
            self._publish_fallback()
            return

        reply_text = str(response.get("reply_text", "")).strip()
        if not reply_text:
            self.get_logger().warn(
                "LLM /chat response had no reply_text. Using fallback."
            )
            self._publish_fallback()
            return

        # Normal happy path: publish LLM reply_text to ROS
        out = String()
        out.data = reply_text
        self.reply_pub.publish(out)
        self.get_logger().info(f"Published reply_text: {reply_text!r}")

    def _publish_fallback(self) -> None:
        """Safe fallback sentence when LLM server fails or times out."""
        fallback = (
            "Sorry, my brain server has a problem now. "
            "I cannot handle this request."
        )
        msg = String()
        msg.data = fallback
        self.reply_pub.publish(msg)
        self.get_logger().warn("Published fallback reply_text.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntentClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down IntentClientNode (Ctrl+C).")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
