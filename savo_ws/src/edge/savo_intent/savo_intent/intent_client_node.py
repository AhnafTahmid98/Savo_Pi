#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS bridge between user text topics and the LLM HTTP API."""

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
    """Forward user text to the LLM server and publish the structured result."""

    MIN_TIMEOUT_S: float = 0.5
    MAX_TIMEOUT_S: float = 20.0

    def __init__(self) -> None:
        super().__init__("intent_client_node")

        self.declare_parameter("llm_server_url", "http://127.0.0.1:8000")
        self.declare_parameter("robot_id", "robot_savo")
        self.declare_parameter("timeout_s", 20.0)

        self.llm_server_url = (
            self.get_parameter("llm_server_url").get_parameter_value().string_value
        ) or "http://127.0.0.1:8000"

        self.base_url = self.llm_server_url.rstrip("/")

        self.robot_id = (
            self.get_parameter("robot_id").get_parameter_value().string_value
        ) or "robot_savo"

        # Clamp keeps bad launch overrides from hanging the speech loop.
        timeout_value = 20.0
        try:
            timeout_value = (
                self.get_parameter("timeout_s").get_parameter_value().double_value
            )
        except Exception:
            pass

        self.timeout_s = self._clamp_timeout(timeout_value)

        self.reply_text_pub = self.create_publisher(
            String,
            "/savo_intent/reply_text",
            10,
        )

        self.intent_result_pub = self.create_publisher(
            IntentResult,
            "/savo_intent/intent_result",
            10,
        )

        self.user_text_sub = self.create_subscription(
            String,
            "/savo_intent/user_text",
            self.user_text_callback,
            10,
        )

        self.get_logger().info(
            "IntentClientNode starting with LLM server URL: %s, robot_id: %s, timeout: %.2fs"
            % (self.base_url, self.robot_id, self.timeout_s)
        )

        self._check_health_once()

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

    def _check_health_once(self) -> None:
        """Probe /health once; startup still continues if the server is down."""
        url = self.base_url + "/health"
        self.get_logger().info("Checking LLM server /health at: %s" % url)
        try:
            req = urllib.request.Request(url, method="GET")
            with urllib.request.urlopen(req, timeout=5.0) as resp:
                data = resp.read().decode("utf-8", errors="ignore")
            self.get_logger().info("LLM server /health response: %s" % data)
        except Exception as exc:
            self.get_logger().warn(
                "LLM server /health check failed: %r" % (exc,)
            )

    def user_text_callback(self, msg: String) -> None:
        """Call the LLM for non-empty user text and publish both outputs."""
        user_text = msg.data or ""
        user_text_stripped = user_text.strip()

        if not user_text_stripped:
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

        reply_msg = String()
        reply_msg.data = reply_text
        self.reply_text_pub.publish(reply_msg)

        self.get_logger().info("Published reply_text: %r" % (reply_text,))

        result_msg = IntentResult()

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
        """Call /chat and return a speakable fallback on errors."""
        url = self.base_url + "/chat"
        self.get_logger().info("Calling LLM /chat at: %s" % url)

        body: Dict[str, Any] = {
            "user_text": user_text,
            "source": "system",
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
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
