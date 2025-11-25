# File: src/savo_speech/savo_speech/speech_bridge_node.py

#!/usr/bin/env python3
"""
Robot Savo — Speech Bridge Node

Purpose
-------
Glue between:
  - STT (speech-to-text)  → LLM client (intent_client_node)
  - LLM client            → TTS (text-to-speech)

Topic flow
----------
Input:
  /savo_speech/stt_text      (std_msgs/String)  - recognized user speech
  /savo_intent/reply_text    (std_msgs/String)  - LLM reply text

Output:
  /savo_intent/user_text     (std_msgs/String)  - text sent to LLM server
  /savo_speech/tts_text      (std_msgs/String)  - text to be spoken by TTS

The node itself does not "think". It only:
  - cleans / filters text a bit
  - forwards between these topics
  - logs what it is doing.
"""

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _normalize_text(text: str) -> str:
    """Trim whitespace and collapse internal spaces."""
    # Strip leading/trailing whitespace
    text = text.strip()
    if not text:
        return text
    # Collapse multiple spaces / newlines into single spaces
    parts = text.split()
    return " ".join(parts)


class SpeechBridgeNode(Node):
    """
    Connect STT ↔ LLM ↔ TTS in Robot Savo.

    Parameters
    ----------
    stt_text_topic : str
        Topic where STT node publishes recognized text.
        Default: '/savo_speech/stt_text'

    user_text_topic : str
        Topic where LLM intent client listens for user text.
        Default: '/savo_intent/user_text'

    reply_text_topic : str
        Topic where LLM intent client publishes reply text.
        Default: '/savo_intent/reply_text'

    tts_text_topic : str
        Topic where TTS node listens for text to speak.
        Default: '/savo_speech/tts_text'

    min_text_length : int
        Minimum length (in characters) for STT text to be forwarded.
        This avoids sending very short noise / single letters to the LLM.
        Default: 2

    log_debug : bool
        If true, log each forwarded message at DEBUG level instead of INFO.
        Default: False
    """

    def __init__(self) -> None:
        super().__init__("speech_bridge_node")

        # Declare parameters with sensible defaults
        self.declare_parameter("stt_text_topic", "/savo_speech/stt_text")
        self.declare_parameter("user_text_topic", "/savo_intent/user_text")
        self.declare_parameter("reply_text_topic", "/savo_intent/reply_text")
        self.declare_parameter("tts_text_topic", "/savo_speech/tts_text")
        self.declare_parameter("min_text_length", 2)
        self.declare_parameter("log_debug", False)

        # Read parameter values
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

        # Publishers
        self._user_text_pub = self.create_publisher(
            String, self._user_text_topic, 10
        )
        self._tts_text_pub = self.create_publisher(
            String, self._tts_text_topic, 10
        )

        # Subscribers
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

        # Simple counters for diagnostics
        self._stt_forward_count = 0
        self._reply_forward_count = 0

        self.get_logger().info(
            "SpeechBridgeNode started\n"
            f"  STT  [{self._stt_text_topic}]  →  [{self._user_text_topic}]\n"
            f"  LLM  [{self._reply_text_topic}] → [{self._tts_text_topic}]\n"
            f"  min_text_length = {self._min_text_length}, "
            f"log_debug = {self._log_debug}"
        )

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def _on_stt_text(self, msg: String) -> None:
        """
        Handle text from STT and forward to LLM input topic.

        Steps:
          1. Normalize text (trim / collapse spaces).
          2. Drop empty / very short messages.
          3. Publish to /savo_intent/user_text.
        """
        raw = msg.data or ""
        text = _normalize_text(raw)

        if not text:
            # Silently ignore pure whitespace / empty
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
        """
        Handle reply text from LLM and forward to TTS input topic.

        Steps:
          1. Normalize text.
          2. Drop empty messages.
          3. Publish to /savo_speech/tts_text.
        """
        raw = msg.data or ""
        text = _normalize_text(raw)

        if not text:
            # No need to send empty reply to TTS
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
