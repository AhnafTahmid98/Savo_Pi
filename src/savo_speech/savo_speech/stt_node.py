#!/usr/bin/env python3
"""
Robot Savo — STT Node (faster-whisper)

This node:
  - Captures audio from the default input (ReSpeaker on the Pi).
  - Uses a simple energy-based VAD to ignore silence/background noise.
  - Runs faster-whisper on each audio block to get a transcript.
  - Publishes recognized text to /savo_intent/user_text (std_msgs/String).

Dependencies (already installed on your Pi):
  - rclpy
  - std_msgs
  - sounddevice
  - numpy
  - faster-whisper

Configuration is done via ROS 2 parameters or YAML (stt_whisper.yaml).
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .audio_io import record_block
from .whisper_engine import FasterWhisperEngine


class STTNode(Node):
    """ROS 2 node for streaming STT using faster-whisper."""

    def __init__(self) -> None:
        super().__init__("stt_node")

        # ------------------------------------------------------------------
        # 1. Declare parameters (can be overridden via YAML or CLI)
        # ------------------------------------------------------------------
        # Model / inference
        self.declare_parameter("model_size_or_path", "small.en")
        self.declare_parameter("device", "cpu")          # "cpu" on Pi, "cuda" on PC later
        self.declare_parameter("compute_type", "int8")   # good for Pi
        self.declare_parameter("language", "en")
        self.declare_parameter("beam_size", 5)

        # Audio / VAD
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("block_duration_s", 4.0)
        self.declare_parameter("energy_threshold", 0.01)
        self.declare_parameter("min_transcript_chars", 3)

        # ROS topics
        self.declare_parameter("publish_topic", "/savo_intent/user_text")

        # ------------------------------------------------------------------
        # 2. Read parameters
        # ------------------------------------------------------------------
        model_size_or_path = (
            self.get_parameter("model_size_or_path")
            .get_parameter_value()
            .string_value
        )
        device = self.get_parameter("device").get_parameter_value().string_value
        compute_type = (
            self.get_parameter("compute_type").get_parameter_value().string_value
        )
        language = self.get_parameter("language").get_parameter_value().string_value
        beam_size = (
            self.get_parameter("beam_size").get_parameter_value().integer_value
        )

        self.sample_rate = (
            self.get_parameter("sample_rate").get_parameter_value().integer_value
        )
        self.block_duration_s = (
            self.get_parameter("block_duration_s").get_parameter_value().double_value
        )
        self.energy_threshold = (
            self.get_parameter("energy_threshold").get_parameter_value().double_value
        )
        self.min_transcript_chars = (
            self.get_parameter("min_transcript_chars")
            .get_parameter_value()
            .integer_value
        )
        publish_topic = (
            self.get_parameter("publish_topic").get_parameter_value().string_value
        )

        # Clamp block duration to something sane
        if self.block_duration_s <= 0.2:
            self.get_logger().warn(
                f"block_duration_s={self.block_duration_s:.3f} too small; "
                "clamping to 0.5 s"
            )
            self.block_duration_s = 0.5

        # ------------------------------------------------------------------
        # 3. Publisher
        # ------------------------------------------------------------------
        self.text_pub = self.create_publisher(String, publish_topic, 10)

        # ------------------------------------------------------------------
        # 4. Initialize faster-whisper engine
        # ------------------------------------------------------------------
        self.get_logger().info(
            "STTNode starting with faster-whisper:\n"
            f"  model_size_or_path = {model_size_or_path}\n"
            f"  device             = {device}\n"
            f"  compute_type       = {compute_type}\n"
            f"  language           = {language}\n"
            f"  beam_size          = {beam_size}\n"
            f"  sample_rate        = {self.sample_rate} Hz\n"
            f"  block_duration_s   = {self.block_duration_s:.2f} s\n"
            f"  energy_threshold   = {self.energy_threshold:.4f}\n"
            f"  publish_topic      = {publish_topic}"
        )

        try:
            self.engine = FasterWhisperEngine(
                model_size_or_path=model_size_or_path,
                device=device,
                compute_type=compute_type,
                language=language,
                beam_size=beam_size,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Failed to initialize FasterWhisperEngine: {exc}"
            )
            # If we can't init the model, there's no point continuing.
            # Raise so the node exits and systemd/supervisor can restart it.
            raise

        # ------------------------------------------------------------------
        # 5. Timer to periodically capture & transcribe audio
        # ------------------------------------------------------------------
        # NOTE: This callback is blocking for block_duration_s because we use
        # a simple record_block() call. For a single STT node this is fine.
        self.timer = self.create_timer(self.block_duration_s, self._timer_cb)

        self._block_index = 0

    # ----------------------------------------------------------------------
    # Timer callback: record audio block -> VAD -> transcription -> publish
    # ----------------------------------------------------------------------
    def _timer_cb(self) -> None:
        self._block_index += 1
        block_id = self._block_index

        # 1) Record one block of audio
        try:
            audio = record_block(self.sample_rate, self.block_duration_s)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"[block {block_id}] Audio capture failed: {exc}"
            )
            return

        if audio.size == 0:
            self.get_logger().warn(f"[block {block_id}] Empty audio block received")
            return

        # 2) Simple energy-based VAD
        #    energy = mean(x^2)
        try:
            energy = float(np.mean(np.square(audio, dtype=np.float32)))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"[block {block_id}] Failed to compute energy: {exc}"
            )
            return

        # Optionally log at debug level to avoid spamming the console
        self.get_logger().debug(
            f"[block {block_id}] energy={energy:.6f} "
            f"(threshold={self.energy_threshold:.6f})"
        )

        if not math.isfinite(energy):
            self.get_logger().warn(
                f"[block {block_id}] Non-finite energy value, skipping block"
            )
            return

        if energy < self.energy_threshold:
            # Too quiet → treat as silence/background, skip STT
            return

        # 3) Run STT on the block
        try:
            text = self.engine.transcribe_block(audio)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"[block {block_id}] Transcription error: {exc}"
            )
            return

        text = text.strip()
        if len(text) < self.min_transcript_chars:
            # Very short / empty text, likely noise or junk.
            self.get_logger().debug(
                f"[block {block_id}] Ignoring short transcript: '{text}'"
            )
            return

        # 4) Publish transcript
        msg = String()
        msg.data = text
        self.text_pub.publish(msg)

        self.get_logger().info(
            f"[block {block_id}] STT transcript: '{text}'"
        )


def main(args: Optional[list[str]] = None) -> None:
    """Entry point for ROS 2."""
    rclpy.init(args=args)
    node = STTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down STTNode")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
