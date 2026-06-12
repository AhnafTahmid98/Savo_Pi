#!/usr/bin/env python3
"""Bridge STT text to intent handling and intent replies to TTS."""

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _normalize_text(text: str) -> str:
    """Trim whitespace and collapse internal spaces."""
    text = text.strip()
    if not text:
        return text
    parts = text.split()
    return " ".join(parts)


class SpeechBridgeNode(Node):
    """Connect STT, intent, and TTS topics."""

    def __init__(self) -> None:
        super().__init__("speech_bridge_node")

        self.declare_parameter("stt_text_topic", "/savo_speech/stt_text")
        self.declare_parameter("user_text_topic", "/savo_intent/user_text")
        self.declare_parameter("reply_text_topic", "/savo_intent/reply_text")
        self.declare_parameter("tts_text_topic", "/savo_speech/tts_text")
        self.declare_parameter("min_text_length", 2)
        self.declare_parameter("log_debug", False)

        self._stt_text_topic = (
            self.get_parameter("stt_text_topic")
            .get_parameter_value()
            .string_value
        )
        self._user_text_topic = (
            self.get_parameter("user_text_topic")
            .get_parameter_value()
            .string_value
        )
        self._reply_text_topic = (
            self.get_parameter("reply_text_topic")
            .get_parameter_value()
            .string_value
        )
        self._tts_text_topic = (
            self.get_parameter("tts_text_topic")
            .get_parameter_value()
            .string_value
        )
        self._min_text_length = (
            self.get_parameter("min_text_length")
            .get_parameter_value()
            .integer_value
        )
        self._log_debug = (
            self.get_parameter("log_debug")
            .get_parameter_value()
            .bool_value
        )

        self._user_text_pub = self.create_publisher(
            String, self._user_text_topic, 10
        )
        self._tts_text_pub = self.create_publisher(
            String, self._tts_text_topic, 10
        )

        self._stt_text_sub = self.create_subscription(
            String,
            self._stt_text_topic,
            self._on_stt_text,
            10,
        )
        self._reply_text_sub = self.create_subscription(
            String,
            self._reply_text_topic,
            self._on_reply_text,
            10,
        )

        self._stt_forward_count = 0
        self._reply_forward_count = 0

        self.get_logger().info(
            "SpeechBridgeNode started\n"
            f"  STT  [{self._stt_text_topic}]  →  [{self._user_text_topic}]\n"
            f"  LLM  [{self._reply_text_topic}] → [{self._tts_text_topic}]\n"
            f"  min_text_length = {self._min_text_length}, "
            f"log_debug = {self._log_debug}"
        )
    def _on_stt_text(self, msg: String) -> None:
        """Forward usable STT text to the intent input topic."""
        raw = msg.data or ""
        text = _normalize_text(raw)

        if not text:
            return

        if len(text) < self._min_text_length:
            # Ignore extremely short tokens (noise, 'a', 'uh', etc.)
            self.get_logger().debug(
                f"[STT→LLM] Ignored too-short text ({len(text)} chars): '{text}'"
            )
            return

        out = String()
        out.data = text
        self._user_text_pub.publish(out)
        self._stt_forward_count += 1

        log_fn = self.get_logger().debug if self._log_debug else self.get_logger().info
        log_fn(
            f"[STT→LLM] #{self._stt_forward_count}: '{text}' "
            f"(len={len(text)})"
        )

    def _on_reply_text(self, msg: String) -> None:
        """Forward non-empty intent replies to TTS."""
        raw = msg.data or ""
        text = _normalize_text(raw)

        if not text:
            return

        out = String()
        out.data = text
        self._tts_text_pub.publish(out)
        self._reply_forward_count += 1

        log_fn = self.get_logger().debug if self._log_debug else self.get_logger().info
        log_fn(
            f"[LLM→TTS] #{self._reply_forward_count}: '{text}' "
            f"(len={len(text)})"
        )


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = SpeechBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("SpeechBridgeNode interrupted, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
