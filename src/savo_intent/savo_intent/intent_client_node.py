"""
Robot Savo — Intent Client Node

Simple test node for the `savo_intent` package.
It checks that the Robot Savo LLM server (running in Docker on PC or Mac)
is reachable from the robot (or any ROS2 machine) by calling `/health`.


"""

import json
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node


class IntentClientNode(Node):
    """Simple ROS2 node that calls the LLM server /health endpoint once."""

    def __init__(self) -> None:
        super().__init__("intent_client_node")

        # Declare a parameter for the LLM server base URL.
        # This can point to your Ubuntu PC or MacBook where Docker is running.
        self.declare_parameter(
            "llm_server_url",
            "http://192.168.164.119:8000",  # TODO: change default if your server IP changes
        )

        llm_server_url = (
            self.get_parameter("llm_server_url").get_parameter_value().string_value
        ).rstrip("/")

        health_url = f"{llm_server_url}/health"
        self.get_logger().info(f"Calling LLM server /health at: {health_url}")

        try:
            with urllib.request.urlopen(health_url, timeout=5.0) as resp:
                raw_data = resp.read().decode("utf-8", errors="replace")

            # Try to parse JSON for nicer logging
            try:
                parsed = json.loads(raw_data)
                pretty = json.dumps(parsed, indent=2, ensure_ascii=False)
                self.get_logger().info("LLM server /health response:\n" + pretty)
            except json.JSONDecodeError:
                self.get_logger().warn(
                    "Response was not valid JSON, showing raw body instead."
                )
                self.get_logger().info(f"Raw response: {raw_data}")

        except urllib.error.URLError as e:
            self.get_logger().error(f"Failed to reach LLM server: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error calling LLM server: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntentClientNode()
    # No need to spin; the HTTP call is done in __init__
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
