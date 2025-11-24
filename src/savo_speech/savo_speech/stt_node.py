#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — STT node (faster-whisper, utterance mode)

- Captures audio from ReSpeaker via sounddevice.
- Buffers blocks into "utterances" while speech is active.
- When speech stops, transcribes the whole utterance once.
- Publishes recognized text to /savo_intent/user_text.

We can switch between:
  - utterance_mode = True  → buffer until user stops talking, then STT once
  - utterance_mode = False → old behavior: STT on each block

Parameters (from YAML / CLI):
  model_size_or_path   : "tiny.en", "base.en", "small.en", or path
  device               : "cpu" on Pi
  compute_type         : "int8" on Pi
  language             : "en"
  beam_size            : int (e.g. 3)
  sample_rate          : 16000
  block_duration_s     : 3.0–4.0 recommended
  energy_threshold     : float, VAD gate on block energy
  min_transcript_chars : ignore very short outputs
  input_device_index   : ReSpeaker index (0 on your Pi)
  publish_topic        : "/savo_intent/user_text"
  utterance_mode       : True to enable utterance buffering
  max_utterance_duration_s : safety limit, e.g. 12.0
"""

import math
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .audio_io import record_block
from .whisper_engine import FasterWhisperEngine


class STTNode(Node):
    def __init__(self) -> None:
        super().__init__("stt_node")

        # ---------------------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------------------
        self.model_size_or_path = (
            self.declare_parameter("model_size_or_path", "tiny.en")
            .get_parameter_value()
            .string_value
        )
        self.device = (
            self.declare_parameter("device", "cpu")
            .get_parameter_value()
            .string_value
        )
        self.compute_type = (
            self.declare_parameter("compute_type", "int8")
            .get_parameter_value()
            .string_value
        )
        self.language = (
            self.declare_parameter("language", "en")
            .get_parameter_value()
            .string_value
        )
        self.beam_size = (
            self.declare_parameter("beam_size", 3)
            .get_parameter_value()
            .integer_value
        )

        self.sample_rate = (
            self.declare_parameter("sample_rate", 16000)
            .get_parameter_value()
            .integer_value
        )
        self.block_duration_s = (
            self.declare_parameter("block_duration_s", 3.0)
            .get_parameter_value()
            .double_value
        )
        self.energy_threshold = (
            self.declare_parameter("energy_threshold", 0.0003)
            .get_parameter_value()
            .double_value
        )
        self.min_transcript_chars = (
            self.declare_parameter("min_transcript_chars", 3)
            .get_parameter_value()
            .integer_value
        )
        self.input_device_index = (
            self.declare_parameter("input_device_index", 0)
            .get_parameter_value()
            .integer_value
        )

        self.publish_topic = (
            self.declare_parameter(
                "publish_topic", "/savo_intent/user_text"
            )
            .get_parameter_value()
            .string_value
        )

        # New: utterance-level behavior.
        self.utterance_mode = (
            self.declare_parameter("utterance_mode", True)
            .get_parameter_value()
            .bool_value
        )
        self.max_utterance_duration_s = (
            self.declare_parameter("max_utterance_duration_s", 12.0)
            .get_parameter_value()
            .double_value
        )

        # ---------------------------------------------------------------------
        # Publisher
        # ---------------------------------------------------------------------
        self.publisher_ = self.create_publisher(String, self.publish_topic, 10)

        # ---------------------------------------------------------------------
        # STT engine
        # ---------------------------------------------------------------------
        self.engine = FasterWhisperEngine(
            model_size_or_path=self.model_size_or_path,
            device=self.device,
            compute_type=self.compute_type,
        )

        # ---------------------------------------------------------------------
        # Utterance state
        # ---------------------------------------------------------------------
        self._block_index = 0
        self._in_utterance = False
        self._utterance_blocks: List[np.ndarray] = []
        self._utterance_total_samples = 0
        self._utterance_counter = 0

        # ---------------------------------------------------------------------
        # Timer
        # ---------------------------------------------------------------------
        self.timer = self.create_timer(
            self.block_duration_s, self._timer_callback
        )

        # Log configuration
        self.get_logger().info(
            "STTNode starting with faster-whisper:\n"
            f"  model_size_or_path = {self.model_size_or_path}\n"
            f"  device             = {self.device}\n"
            f"  compute_type       = {self.compute_type}\n"
            f"  language           = {self.language}\n"
            f"  beam_size          = {self.beam_size}\n"
            f"  sample_rate        = {self.sample_rate} Hz\n"
            f"  block_duration_s   = {self.block_duration_s:.2f} s\n"
            f"  energy_threshold   = {self.energy_threshold:.4f}\n"
            f"  input_device_index = {self.input_device_index}\n"
            f"  utterance_mode     = {self.utterance_mode}\n"
            f"  max_utt_duration   = {self.max_utterance_duration_s:.1f} s\n"
            f"  publish_topic      = {self.publish_topic}"
        )

    # -------------------------------------------------------------------------
    # Main timer loop
    # -------------------------------------------------------------------------
    def _timer_callback(self) -> None:
        self._block_index += 1

        try:
            audio = record_block(
                sample_rate=self.sample_rate,
                duration_s=self.block_duration_s,
                input_device_index=self.input_device_index,
            )
        except Exception as exc:
            self.get_logger().error(
                f"[block {self._block_index}] audio capture failed: {exc}"
            )
            return

        if audio is None or len(audio) == 0:
            self.get_logger().warning(
                f"[block {self._block_index}] empty audio block"
            )
            return

        # Convert to float32 and compute simple energy for VAD
        audio_f32 = audio.astype(np.float32)
        energy = float(np.mean(np.square(audio_f32)))

        self.get_logger().debug(
            f"[block {self._block_index}] energy={energy:.6f} "
            f"(threshold={self.energy_threshold:.6f})"
        )

        if not self.utterance_mode:
            # -------------------------------------------------------------
            # LEGACY MODE: per-block STT (no buffering)
            # -------------------------------------------------------------
            if energy < self.energy_threshold:
                return

            try:
                text = self.engine.transcribe_block(
                    audio_f32, self.sample_rate, self.language, self.beam_size
                )
            except Exception as exc:
                self.get_logger().error(f"STT error on block: {exc}")
                return

            text = (text or "").strip()
            if len(text) < self.min_transcript_chars:
                return

            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
            self.get_logger().info(
                f"[block {self._block_index}] STT transcript: '{text}'"
            )
            return

        # -----------------------------------------------------------------
        # UTTERANCE MODE: buffer blocks while speech is active
        # -----------------------------------------------------------------
        if energy >= self.energy_threshold:
            # Speech detected in this block
            if not self._in_utterance:
                self.get_logger().debug(
                    f"[block {self._block_index}] Utterance START"
                )
                self._in_utterance = True
                self._utterance_blocks = []
                self._utterance_total_samples = 0

            self._utterance_blocks.append(audio_f32)
            self._utterance_total_samples += audio_f32.shape[0]

            # Safety: cut very long utterances
            utt_duration = self._utterance_total_duration_s()
            if utt_duration >= self.max_utterance_duration_s:
                self.get_logger().debug(
                    f"[block {self._block_index}] Utterance reached "
                    f"max duration ({utt_duration:.2f}s), finalizing."
                )
                self._finalize_utterance()
                self._in_utterance = False

        else:
            # Silence in this block
            if self._in_utterance:
                # We were in speech and now saw a silent block -> end utterance
                self.get_logger().debug(
                    f"[block {self._block_index}] Utterance END on silence"
                )
                self._finalize_utterance()
                self._in_utterance = False
            # else: still idle, nothing to do

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def _utterance_total_duration_s(self) -> float:
        if self.sample_rate <= 0:
            return 0.0
        return float(self._utterance_total_samples) / float(self.sample_rate)

    def _finalize_utterance(self) -> None:
        """Run STT on buffered utterance and publish one transcript."""
        if not self._utterance_blocks:
            return

        # Concatenate all blocks into one 1-D array
        try:
            audio_all = np.concatenate(self._utterance_blocks, axis=0)
        except Exception as exc:
            self.get_logger().error(f"Failed to concat utterance blocks: {exc}")
            self._utterance_blocks = []
            self._utterance_total_samples = 0
            return

        utt_duration = self._utterance_total_duration_s()
        self._utterance_blocks = []
        self._utterance_total_samples = 0

        self._utterance_counter += 1
        utt_id = self._utterance_counter

        self.get_logger().debug(
            f"[utt {utt_id}] Finalizing utterance, duration={utt_duration:.2f}s"
        )

        try:
            text = self.engine.transcribe_block(
                audio_all, self.sample_rate, self.language, self.beam_size
            )
        except Exception as exc:
            self.get_logger().error(f"STT error in utterance {utt_id}: {exc}")
            return

        text = (text or "").strip()
        if len(text) < self.min_transcript_chars:
            self.get_logger().debug(
                f"[utt {utt_id}] Ignored short transcript: '{text}'"
            )
            return

        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(
            f"[utt {utt_id}] STT transcript: '{text}'"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = STTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down STTNode (Ctrl+C)")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # Already shut down or context invalid; safe to ignore.
            pass


if __name__ == "__main__":
    main()
