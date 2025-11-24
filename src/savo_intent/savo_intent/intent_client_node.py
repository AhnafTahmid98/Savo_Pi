import json
import urllib.error
import urllib.request
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class IntentClientNode(Node):
    """
    Robot Savo — Intent client node.

    This node is the ROS2 <-> LLM server bridge:

    - Subscribes to:  /savo_intent/user_text   (std_msgs/String)
      -> human text from STT or debug tools.

    - Sends HTTP POST to:  <llm_server_url>/chat
      -> FastAPI LLM server (Docker on PC / laptop).

    - Publishes to: /savo_intent/reply_text    (std_msgs/String)
      -> text for TTS (Robot Savo speaking).

    """

    def __init__(self) -> None:
        super().__init__("intent_client_node")

        # --- Parameters ---
        # Base URL of the LLM server (FastAPI) running in Docker.
        self.declare_parameter("llm_server_url", "http://192.168.164.119:8000")
        # Robot ID used as session_id for the LLM (grouping conversations).
        self.declare_parameter("robot_id", "robot_savo_pi")
        # HTTP timeout in seconds for /chat requests.
        self.declare_parameter("request_timeout_sec", 1.8)

        self.llm_server_url: str = (
            self.get_parameter("llm_server_url").get_parameter_value().string_value
        )
        self.robot_id: str = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        )
        self.request_timeout_sec: float = (
            self.get_parameter("request_timeout_sec")
            .get_parameter_value()
            .double_value
        )

        # Normalize URL a little (avoid trailing slash issues)
        self.llm_server_url = self.llm_server_url.rstrip("/")

        self.get_logger().info(
            f"IntentClientNode starting with LLM server URL: {self.llm_server_url}, "
            f"robot_id: {self.robot_id}, timeout: {self.request_timeout_sec:.2f}s"
        )

        # --- ROS interfaces ---

        # Incoming user text (from STT or other nodes).
        self.user_sub = self.create_subscription(
            String,
            "/savo_intent/user_text",
            self.on_user_text,
            10,
        )

        # Outgoing reply text (for TTS).
        self.reply_pub = self.create_publisher(
            String,
            "/savo_intent/reply_text",
            10,
        )

        # Optional: raw JSON debug topic (useful while developing).
        self.raw_response_pub = self.create_publisher(
            String,
            "/savo_intent/raw_response",
            10,
        )

        # Optional: do a one-time /health check on startup so we know if the
        # LLM server is reachable.
        self._startup_health_timer = self.create_timer(
            0.5, self._startup_health_check_once
        )

    # ------------------------------------------------------------------
    # Startup health check
    # ------------------------------------------------------------------
    def _startup_health_check_once(self) -> None:
        """Run /health check exactly once on startup, then stop the timer."""
        self._startup_health_timer.cancel()

        health_url = f"{self.llm_server_url}/health"
        self.get_logger().info(f"Checking LLM server /health at: {health_url}")

        try:
            with urllib.request.urlopen(health_url, timeout=self.request_timeout_sec) as resp:
                body = resp.read().decode("utf-8", errors="replace")
            self.get_logger().info(f"LLM server /health response: {body}")
        except Exception as exc:
            self.get_logger().error(
                f"Failed to reach LLM server /health: {exc!r}. "
                "The node will still run, but /chat calls may fail."
            )

    # ------------------------------------------------------------------
    # Subscriber callback: user text from ROS2
    # ------------------------------------------------------------------
    def on_user_text(self, msg: String) -> None:
        """Handle incoming human text and forward it to the LLM /chat endpoint."""
        text = msg.data.strip()
        if not text:
            self.get_logger().warn("Received empty user_text; ignoring.")
            return

        self.get_logger().info(f"Received user_text: {text!r}")

        try:
            reply = self.call_llm_chat(user_text=text)
        except Exception as exc:
            # Any error here: log and publish a safe fallback reply.
            self.get_logger().error(
                f"Error calling LLM /chat: {exc!r}. "
                "Publishing fallback reply."
            )
            fallback = String()
            fallback.data = (
                "Sorry, my brain server has a problem now. "
                "I cannot handle this request."
            )
            self.reply_pub.publish(fallback)
            return

        # Extract fields with safe defaults
        reply_text = reply.get("reply_text", "").strip()
        intent = reply.get("intent", "")
        nav_goal: Optional[str] = reply.get("nav_goal")

        self.get_logger().info(
            "LLM /chat result: "
            f"intent={intent!r}, nav_goal={nav_goal!r}, reply_text={reply_text!r}"
        )

        # Publish reply_text to TTS topic
        if reply_text:
            out = String()
            out.data = reply_text
            self.reply_pub.publish(out)
        else:
            self.get_logger().warn(
                "LLM /chat returned empty reply_text; nothing published."
            )

        # Also publish the raw JSON response for debugging (optional)
        raw_out = String()
        raw_out.data = json.dumps(reply, ensure_ascii=False)
        self.raw_response_pub.publish(raw_out)

    # ------------------------------------------------------------------
    # HTTP client: /chat
    # ------------------------------------------------------------------
    def call_llm_chat(self, user_text: str) -> Dict[str, Any]:
        """
        Call the FastAPI LLM server /chat endpoint with user_text.

        Returns a parsed JSON dictionary (Python).
        Raises an exception on HTTP or JSON errors.
        """
        url = f"{self.llm_server_url}/chat"

        payload = {
            # Session groups conversation turns. For now, we can just use robot_id.
            "session_id": self.robot_id,
            "user_text": user_text,
            # We keep the payload minimal to match the server's ChatRequest model.
        }

        body = json.dumps(payload).encode("utf-8")

        req = urllib.request.Request(
            url,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )

        self.get_logger().debug(
            f"Sending /chat request to {url} with payload: {payload!r}"
        )

        try:
            with urllib.request.urlopen(req, timeout=self.request_timeout_sec) as resp:
                status_code = resp.getcode()
                resp_body = resp.read().decode("utf-8", errors="replace")
        except urllib.error.URLError as exc:
            raise RuntimeError(f"HTTP error calling /chat: {exc!r}") from exc

        if status_code != 200:
            raise RuntimeError(
                f"/chat returned HTTP {status_code}: {resp_body[:200]!r}"
            )

        try:
            data = json.loads(resp_body)
        except json.JSONDecodeError as exc:
            raise RuntimeError(
                f"Failed to decode /chat JSON: {exc!r}, body={resp_body!r}"
            ) from exc

        # Basic sanity check: must have reply_text and intent keys.
        if "reply_text" not in data or "intent" not in data:
            self.get_logger().warn(
                f"/chat response missing expected keys: {data!r}"
            )

        return data


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
