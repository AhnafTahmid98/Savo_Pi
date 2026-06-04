#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — STT node (faster-whisper, utterance mode + TTS gate)

- Captures audio from ReSpeaker via sounddevice.
- Buffers blocks into "utterances" while speech is active.
- When speech stops, transcribes the whole utterance once.
- Publishes recognized text to /savo_intent/user_text (or configured topic).

New in this version:
- Subscribes to a TTS state topic (e.g. /savo_speech/tts_speaking, std_msgs/Bool).
- While the robot is speaking (and for a short cooldown) STT is gated OFF
  so we do not transcribe the robot's own replies.

We can switch between:
  - utterance_mode = True  → buffer until user stops talking, then STT once
  - utterance_mode = False → per-block STT (legacy behavior)

Key parameters (from YAML / CLI):
  model_size_or_path           : "tiny.en", "base.en", "small.en", or path
  device                       : "cpu" on Pi
  compute_type                 : "int8" on Pi
  language                     : "en"
  beam_size                    : int (e.g. 3)
  sample_rate                  : 16000
  block_duration_s             : 2.0–4.0 recommended
  energy_threshold             : float, VAD gate on block energy
  min_transcript_chars         : ignore very short outputs
  input_device_index           : ReSpeaker index (0 on your Pi)
  publish_topic                : "/savo_speech/stt_text" or "/savo_intent/user_text"
  utterance_mode               : True to enable utterance buffering
  max_utterance_duration_s     : safety limit, e.g. 15.0

  use_tts_gate                 : True → ignore audio while robot TTS is active
  tts_speaking_topic           : "/savo_speech/tts_speaking"
  tts_cooldown_blocks          : number of blocks to skip after TTS stops
"""

from __future__ import annotations

from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from .audio_io import record_block
from .whisper_engine import FasterWhisperEngine


class STTNode(Node):
    def __init__(self) -> None:
        super().__init__("stt_node")

        # ---------------------------------------------------------------------
        # Parameters: STT / Whisper configuration
        # ---------------------------------------------------------------------
        self.model_size_or_path: str = (
            self.declare_parameter("model_size_or_path", "tiny.en")
            .get_parameter_value()
            .string_value
        )
        self.device: str = (
            self.declare_parameter("device", "cpu")
            .get_parameter_value()
            .string_value
        )
        self.compute_type: str = (
            self.declare_parameter("compute_type", "int8")
            .get_parameter_value()
            .string_value
        )
        self.language: str = (
            self.declare_parameter("language", "en")
            .get_parameter_value()
            .string_value
        )
        self.beam_size: int = (
            self.declare_parameter("beam_size", 3)
            .get_parameter_value()
            .integer_value
        )

        self.sample_rate: int = (
            self.declare_parameter("sample_rate", 16000)
            .get_parameter_value()
            .integer_value
        )
        self.block_duration_s: float = (
            self.declare_parameter("block_duration_s", 2.0)
            .get_parameter_value()
            .double_value
        )
        self.energy_threshold: float = (
            self.declare_parameter("energy_threshold", 0.0003)
            .get_parameter_value()
            .double_value
        )
        self.min_transcript_chars: int = (
            self.declare_parameter("min_transcript_chars", 3)
            .get_parameter_value()
            .integer_value
        )
        self.input_device_index: int = (
            self.declare_parameter("input_device_index", 0)
            .get_parameter_value()
            .integer_value
        )

        self.publish_topic: str = (
            self.declare_parameter("publish_topic", "/savo_speech/stt_text")
            .get_parameter_value()
            .string_value
        )

        # ---------------------------------------------------------------------
        # Parameters: utterance-level behavior
        # ---------------------------------------------------------------------
        self.utterance_mode: bool = (
            self.declare_parameter("utterance_mode", True)
            .get_parameter_value()
            .bool_value
        )
        self.max_utterance_duration_s: float = (
            self.declare_parameter("max_utterance_duration_s", 15.0)
            .get_parameter_value()
            .double_value
        )

        # ---------------------------------------------------------------------
        # Parameters: TTS gate (ignore robot's own voice)
        # ---------------------------------------------------------------------
        self.use_tts_gate: bool = (
            self.declare_parameter("use_tts_gate", True)
            .get_parameter_value()
            .bool_value
        )
        self.tts_speaking_topic: str = (
            self.declare_parameter("tts_speaking_topic", "/savo_speech/tts_speaking")
            .get_parameter_value()
            .string_value
        )
        self.tts_cooldown_blocks: int = (
            self.declare_parameter("tts_cooldown_blocks", 1)
            .get_parameter_value()
            .integer_value
        )

        # ---------------------------------------------------------------------
        # Publisher
        # ---------------------------------------------------------------------
        self.publisher_ = self.create_publisher(String, self.publish_topic, 10)

        # ---------------------------------------------------------------------
        # TTS speaking state subscriber (for gating)
        # ---------------------------------------------------------------------
        self._tts_speaking: bool = False
        self._tts_cooldown_blocks_remaining: int = 0

        self._tts_speaking_sub: Optional[rclpy.subscription.Subscription] = None
        if self.use_tts_gate and self.tts_speaking_topic:
            self._tts_speaking_sub = self.create_subscription(
                Bool,
                self.tts_speaking_topic,
                self._on_tts_speaking,
                10,
            )

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
        self._block_index: int = 0
        self._in_utterance: bool = False
        self._utterance_blocks: List[np.ndarray] = []
        self._utterance_total_samples: int = 0
        self._utterance_counter: int = 0

        # ---------------------------------------------------------------------
        # Timer
        # ---------------------------------------------------------------------
        self.timer = self.create_timer(self.block_duration_s, self._timer_callback)

        # ---------------------------------------------------------------------
        # Log configuration
        # ---------------------------------------------------------------------
        self.get_logger().info(
            "STTNode starting with faster-whisper:\n"
            f"  model_size_or_path     = {self.model_size_or_path}\n"
            f"  device                 = {self.device}\n"
            f"  compute_type           = {self.compute_type}\n"
            f"  language               = {self.language}\n"
            f"  beam_size              = {self.beam_size}\n"
            f"  sample_rate            = {self.sample_rate} Hz\n"
            f"  block_duration_s       = {self.block_duration_s:.2f} s\n"
            f"  energy_threshold       = {self.energy_threshold:.6f}\n"
            f"  input_device_index     = {self.input_device_index}\n"
            f"  utterance_mode         = {self.utterance_mode}\n"
            f"  max_utterance_duration = {self.max_utterance_duration_s:.1f} s\n"
            f"  publish_topic          = {self.publish_topic}\n"
            f"  use_tts_gate           = {self.use_tts_gate}\n"
            f"  tts_speaking_topic     = {self.tts_speaking_topic}\n"
            f"  tts_cooldown_blocks    = {self.tts_cooldown_blocks}"
        )

    # -------------------------------------------------------------------------
    # TTS speaking callback (gate STT while robot is talking)
    # -------------------------------------------------------------------------
    def _on_tts_speaking(self, msg: Bool) -> None:
        """Update TTS speaking state and reset utterance when robot starts talking."""
        new_state = bool(msg.data)
        old_state = self._tts_speaking
        self._tts_speaking = new_state

        if new_state and not old_state:
            # TTS just started: cancel any current utterance and start cooldown.
            self.get_logger().debug("STTNode: TTS started, cancelling current utterance and gating STT.")
            self._in_utterance = False
            self._utterance_blocks = []
            self._utterance_total_samples = 0
            self._tts_cooldown_blocks_remaining = self.tts_cooldown_blocks

        elif not new_state and old_state:
            # TTS just stopped: cooldown will be handled in timer.
            self.get_logger().debug(
                f"STTNode: TTS stopped, starting cooldown for {self.tts_cooldown_blocks} block(s)."
            )
            self._tts_cooldown_blocks_remaining = self.tts_cooldown_blocks

    # -------------------------------------------------------------------------
    # Main timer loop
    # -------------------------------------------------------------------------
    def _timer_callback(self) -> None:
        self._block_index += 1

        # -------------------------------------------------------------
        # Step 1: TTS gate — skip capturing while robot is speaking
        #         or during a short cooldown after TTS stops.
        # -------------------------------------------------------------
        if self.use_tts_gate:
            if self._tts_speaking:
                self.get_logger().debug(
                    f"[block {self._block_index}] STT gated OFF (robot TTS speaking)."
                )
                return

            if self._tts_cooldown_blocks_remaining > 0:
                self._tts_cooldown_blocks_remaining -= 1
                self.get_logger().debug(
                    f"[block {self._block_index}] STT cooling down after TTS "
                    f"({self._tts_cooldown_blocks_remaining} blocks left)."
                )
                return

        # -------------------------------------------------------------
        # Step 2: Capture audio
        # -------------------------------------------------------------
        try:
            audio = record_block(
                sample_rate=self.sample_rate,
                duration_s=self.block_duration_s,
                input_device_index=self.input_device_index,
            )
        except Exception as exc:  # noqa: BLE001
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

        # -------------------------------------------------------------
        # Legacy mode: per-block STT (no utterance buffering)
        # -------------------------------------------------------------
        if not self.utterance_mode:
            if energy < self.energy_threshold:
                return

            try:
                text = self.engine.transcribe_block(
                    audio_f32, self.sample_rate, self.language, self.beam_size
                )
            except Exception as exc:  # noqa: BLE001
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

        # -------------------------------------------------------------
        # Utterance mode: buffer blocks while speech is active
        # -------------------------------------------------------------
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
        except Exception as exc:  # noqa: BLE001
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
        except Exception as exc:  # noqa: BLE001
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
