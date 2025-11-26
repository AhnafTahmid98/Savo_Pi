#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — TTS Node (Piper-based)

This ROS 2 node subscribes to a text topic and uses Piper TTS models
to synthesize speech, then plays it through the Pi's audio output.

Key features:
- Uses local Piper ONNX models (e.g. en_US-ryan-high, en_US-hfc_female-medium).
- Supports a "voice_profile" parameter ("male" / "female").
- Applies configurable gain/output_gain.
- Publishes mouth animation (0.0–1.0) while speaking.
- Publishes a "speech done" notification when playback finishes.
- NEW: publishes a Boolean "tts_speaking" flag so STT can mute itself
  while the robot is talking.

Configuration is provided via:
  src/savo_speech/config/tts_piper.yaml
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional, Dict

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty, Float32, Bool

from .audio_io import play_pcm
from .piper_engine import PiperEngine

logger = logging.getLogger(__name__)

# Target playback sample rate for the audio device.
# ReSpeaker 4-Mic Array is usually fine with 16000 or 48000.
TARGET_PLAYBACK_SR = 16000


class TTSNode(Node):
    """ROS 2 node that wraps Piper TTS via PiperEngine."""

    def __init__(self) -> None:
        super().__init__("tts_node")

        # --------------------------------------------------------------
        # Declare parameters with reasonable defaults
        # (normally overridden by tts_piper.yaml)
        # --------------------------------------------------------------

        self.declare_parameter("model_dir", "/home/savo/Savo_Pi/models/piper")
        self.declare_parameter("voice_profile", "male")

        self.declare_parameter("male_model_file", "en_US-ryan-high.onnx")
        self.declare_parameter("male_config_file", "en_US-ryan-high.onnx.json")

        self.declare_parameter("female_model_file", "en_US-hfc_female-medium.onnx")
        self.declare_parameter("female_config_file", "en_US-hfc_female-medium.onnx.json")

        # Synthesis controls (for logging / future tuning)
        self.declare_parameter("sample_rate", 22050)  # informational
        self.declare_parameter("gain", 1.0)
        self.declare_parameter("length_scale", 0.95)
        self.declare_parameter("noise_scale", 0.667)
        self.declare_parameter("noise_w", 0.8)

        # Output device + gain at playback stage
        self.declare_parameter("output_device_index", -1)
        self.declare_parameter("output_gain", 1.0)

        # ROS topics
        self.declare_parameter("input_text_topic", "/savo_speech/tts_text")
        self.declare_parameter("speech_done_topic", "/savo_speech/tts_done")
        self.declare_parameter("mouth_anim_topic", "/savo_speech/mouth_open")
        self.declare_parameter("mouth_anim_rate_hz", 25.0)

        # NEW: "TTS is speaking" flag for STT gating
        self.declare_parameter("tts_speaking_topic", "/savo_speech/tts_speaking")

        # Logging / debug
        self.declare_parameter("debug_log_text", True)
        self.declare_parameter("max_logged_chars", 512)

        # --------------------------------------------------------------
        # Read parameters
        # --------------------------------------------------------------

        self.model_dir: str = self.get_parameter("model_dir").get_parameter_value().string_value
        self.voice_profile: str = (
            self.get_parameter("voice_profile").get_parameter_value().string_value.lower()
        )

        self.male_model_file: str = (
            self.get_parameter("male_model_file").get_parameter_value().string_value
        )
        self.male_config_file: str = (
            self.get_parameter("male_config_file").get_parameter_value().string_value
        )

        self.female_model_file: str = (
            self.get_parameter("female_model_file").get_parameter_value().string_value
        )
        self.female_config_file: str = (
            self.get_parameter("female_config_file").get_parameter_value().string_value
        )

        # Synthesis params (for logging + gain)
        self.sample_rate_param: int = (
            self.get_parameter("sample_rate").get_parameter_value().integer_value
        )
        self.gain: float = self.get_parameter("gain").get_parameter_value().double_value
        self.length_scale: float = (
            self.get_parameter("length_scale").get_parameter_value().double_value
        )
        self.noise_scale: float = (
            self.get_parameter("noise_scale").get_parameter_value().double_value
        )
        self.noise_w: float = self.get_parameter("noise_w").get_parameter_value().double_value

        # Playback device + extra gain
        self.output_device_index: int = (
            self.get_parameter("output_device_index").get_parameter_value().integer_value
        )
        self.output_gain: float = (
            self.get_parameter("output_gain").get_parameter_value().double_value
        )

        # ROS topics
        self.input_text_topic: str = (
            self.get_parameter("input_text_topic").get_parameter_value().string_value
        )
        self.speech_done_topic: str = (
            self.get_parameter("speech_done_topic").get_parameter_value().string_value
        )
        self.mouth_anim_topic: str = (
            self.get_parameter("mouth_anim_topic").get_parameter_value().string_value
        )
        self.mouth_anim_rate_hz: float = (
            self.get_parameter("mouth_anim_rate_hz").get_parameter_value().double_value
        )
        self.tts_speaking_topic: str = (
            self.get_parameter("tts_speaking_topic").get_parameter_value().string_value
        )

        # Debug logging
        self.debug_log_text: bool = (
            self.get_parameter("debug_log_text").get_parameter_value().bool_value
        )
        self.max_logged_chars: int = (
            self.get_parameter("max_logged_chars").get_parameter_value().integer_value
        )

        # --------------------------------------------------------------
        # Initialize Piper engines (one per voice profile, if available)
        # --------------------------------------------------------------

        self._engines: Dict[str, PiperEngine] = {}
        model_dir_path = Path(self.model_dir)

        try:
            # Male
            male_model_path = model_dir_path / self.male_model_file
            male_config_path = model_dir_path / self.male_config_file
            if male_model_path.is_file() and male_config_path.is_file():
                self._engines["male"] = PiperEngine(
                    model_path=male_model_path,
                    config_path=male_config_path,
                )
            else:
                self.get_logger().warn(
                    "TTSNode: male model/config not found:\n"
                    f"  model  = {male_model_path}\n"
                    f"  config = {male_config_path}"
                )

            # Female
            female_model_path = model_dir_path / self.female_model_file
            female_config_path = model_dir_path / self.female_config_file
            if female_model_path.is_file() and female_config_path.is_file():
                self._engines["female"] = PiperEngine(
                    model_path=female_model_path,
                    config_path=female_config_path,
                )
            else:
                self.get_logger().warn(
                    "TTSNode: female model/config not found:\n"
                    f"  model  = {female_model_path}\n"
                    f"  config = {female_config_path}"
                )

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"TTSNode: failed to initialize PiperEngine(s): {exc}")

        if not self._engines:
            self.get_logger().error(
                "TTSNode: no Piper TTS engines could be initialized. "
                "Check model_dir and model filenames."
            )
        else:
            engine_list = ", ".join(sorted(self._engines.keys()))
            self.get_logger().info(f"TTSNode: initialized voices: {engine_list}")

        # --------------------------------------------------------------
        # Publishers / subscribers
        # --------------------------------------------------------------

        # Input text subscriber
        self._text_sub = self.create_subscription(
            String,
            self.input_text_topic,
            self._on_text_message,
            10,
        )

        # Optional "speech done" publisher
        self._speech_done_pub: Optional[rclpy.publisher.Publisher] = None
        if self.speech_done_topic:
            self._speech_done_pub = self.create_publisher(Empty, self.speech_done_topic, 10)

        # Optional mouth animation publisher (0.0 idle, 1.0 speaking)
        self._mouth_anim_pub: Optional[rclpy.publisher.Publisher] = None
        if self.mouth_anim_topic:
            self._mouth_anim_pub = self.create_publisher(
                Float32,
                self.mouth_anim_topic,
                10,
            )

        # NEW: "TTS is speaking" Boolean publisher
        self._tts_speaking_pub: Optional[rclpy.publisher.Publisher] = None
        if self.tts_speaking_topic:
            self._tts_speaking_pub = self.create_publisher(
                Bool,
                self.tts_speaking_topic,
                10,
            )

        # --------------------------------------------------------------
        # Log final configuration summary
        # --------------------------------------------------------------

        self.get_logger().info(
            "TTSNode starting with Piper:\n"
            f"  model_dir         = {self.model_dir}\n"
            f"  voice_profile     = {self.voice_profile}\n"
            f"  gain              = {self.gain:.3f}\n"
            f"  output_gain       = {self.output_gain:.3f}\n"
            f"  length_scale      = {self.length_scale:.3f}\n"
            f"  noise_scale       = {self.noise_scale:.3f}\n"
            f"  noise_w           = {self.noise_w:.3f}\n"
            f"  sample_rate       = {self.sample_rate_param}\n"
            f"  output_device     = {self.output_device_index}\n"
            f"  input_text_topic  = {self.input_text_topic}\n"
            f"  speech_done_topic = {self.speech_done_topic or '(disabled)'}\n"
            f"  mouth_anim_topic  = {self.mouth_anim_topic or '(disabled)'}\n"
            f"  tts_speaking_topic= {self.tts_speaking_topic or '(disabled)'}\n"
            f"  debug_log_text    = {self.debug_log_text}\n"
            f"  max_logged_chars  = {self.max_logged_chars}"
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _choose_engine(self) -> Optional[PiperEngine]:
        """
        Choose the appropriate PiperEngine based on voice_profile.

        Fallbacks:
        - If requested profile not loaded, fall back to "male" if available.
        - If no engines loaded, return None.
        """
        if not self._engines:
            return None

        profile = self.voice_profile.lower()
        if profile in self._engines:
            return self._engines[profile]

        # Fallbacks
        if "male" in self._engines:
            self.get_logger().warn(
                f"TTSNode: requested voice_profile='{profile}' not available, "
                "falling back to 'male'"
            )
            return self._engines["male"]

        # As a last resort, pick any engine
        key = next(iter(self._engines.keys()))
        self.get_logger().warn(
            f"TTSNode: requested voice_profile='{profile}' not available, "
            f"falling back to '{key}'"
        )
        return self._engines[key]

    def _log_text_preview(self, text: str) -> None:
        """Log a trimmed preview of the text, respecting max_logged_chars."""
        if not self.debug_log_text:
            return

        clean = (text or "").replace("\n", " ").strip()
        if len(clean) > self.max_logged_chars:
            preview = clean[: self.max_logged_chars] + "... [truncated]"
        else:
            preview = clean

        self.get_logger().info(f"TTSNode: TTS text: '{preview}'")

    def _publish_mouth_open(self, value: float) -> None:
        """Publish mouth open value (0.0–1.0) if mouth animation is enabled."""
        if self._mouth_anim_pub is None:
            return
        msg = Float32()
        msg.data = float(max(0.0, min(1.0, value)))
        self._mouth_anim_pub.publish(msg)

    def _publish_speech_done(self) -> None:
        """Publish an Empty message on the speech_done_topic (if configured)."""
        if self._speech_done_pub is None:
            return
        msg = Empty()
        self._speech_done_pub.publish(msg)

    def _publish_tts_speaking(self, speaking: bool) -> None:
        """
        Publish a Bool flag telling the rest of the system whether
        the TTS is currently speaking.
        """
        if self._tts_speaking_pub is None:
            return
        msg = Bool()
        msg.data = bool(speaking)
        self._tts_speaking_pub.publish(msg)

    @staticmethod
    def _resample_linear(
        audio: np.ndarray,
        src_sr: int,
        dst_sr: int,
    ) -> np.ndarray:
        """
        Simple linear resampler for mono float32 audio.

        Parameters
        ----------
        audio : np.ndarray
            1D float32 array, mono audio.
        src_sr : int
            Source sample rate.
        dst_sr : int
            Target sample rate.

        Returns
        -------
        np.ndarray
            Resampled mono float32 array.
        """
        if src_sr <= 0 or dst_sr <= 0:
            return audio
        if src_sr == dst_sr:
            return audio

        n_src = audio.shape[0]
        if n_src == 0:
            return audio

        duration = n_src / float(src_sr)
        n_dst = max(1, int(round(duration * dst_sr)))

        # Interpolation positions in source index space
        src_positions = np.linspace(0, n_src - 1, num=n_dst, dtype=np.float64)
        src_indices = np.floor(src_positions).astype(np.int64)
        frac = src_positions - src_indices
        src_indices_plus = np.clip(src_indices + 1, 0, n_src - 1)

        audio_np = audio.astype(np.float32, copy=False)
        resampled = (1.0 - frac) * audio_np[src_indices] + frac * audio_np[src_indices_plus]
        return resampled.astype(np.float32, copy=False)

    # ------------------------------------------------------------------
    # Subscriber callback
    # ------------------------------------------------------------------

    def _on_text_message(self, msg: String) -> None:
        """
        Handle incoming text messages and perform TTS playback.

        This callback is *blocking* while speech is playing. For Robot Savo,
        that's acceptable because TTS output is short and we do not need
        concurrent overlapping speech from this node.

        NEW: publishes tts_speaking=True while audio is playing,
        and tts_speaking=False after playback completes.
        """
        text = (msg.data or "").strip()
        if not text:
            self.get_logger().debug("TTSNode: received empty text, skipping")
            return

        engine = self._choose_engine()
        if engine is None:
            self.get_logger().error("TTSNode: no TTS engine available, cannot speak")
            return

        self._log_text_preview(text)

        # Synthesize audio (int16 PCM at engine.sample_rate)
        try:
            audio_i16 = engine.synthesize_to_pcm16(text)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"TTSNode: TTS synthesis failed: {exc}")
            return

        if audio_i16.size == 0:
            self.get_logger().warn("TTSNode: TTS produced empty audio, skipping playback")
            return

        # Convert int16 [-32768, 32767] → float32 [-1.0, 1.0]
        audio_f32 = audio_i16.astype(np.float32) / 32768.0
        max_abs_raw = float(np.max(np.abs(audio_f32)))
        self.get_logger().debug(
            f"TTSNode: raw audio: sr={engine.sample_rate}, "
            f"samples={audio_f32.shape[0]}, max_abs={max_abs_raw:.4f}"
        )

        # Resample to a device-friendly rate if needed.
        playback_sr = TARGET_PLAYBACK_SR
        if playback_sr <= 0:
            playback_sr = engine.sample_rate

        if playback_sr != engine.sample_rate:
            audio_resampled = self._resample_linear(
                audio_f32,
                src_sr=engine.sample_rate,
                dst_sr=playback_sr,
            )
            max_abs_res = float(np.max(np.abs(audio_resampled)))
            self.get_logger().debug(
                f"TTSNode: resampled audio to {playback_sr} Hz "
                f"(from {engine.sample_rate} Hz), "
                f"samples={audio_resampled.shape[0]}, max_abs={max_abs_res:.4f}"
            )
        else:
            audio_resampled = audio_f32
            max_abs_res = max_abs_raw

        # Apply gains (Piper-level gain + playback gain)
        total_gain = float(self.gain * self.output_gain)
        if total_gain != 1.0:
            audio_resampled = audio_resampled * total_gain

        # Clip to safe range
        max_abs_after = float(np.max(np.abs(audio_resampled)))
        if max_abs_after > 1.0:
            audio_resampled = np.clip(audio_resampled, -1.0, 1.0)
            max_abs_after = float(np.max(np.abs(audio_resampled)))

        self.get_logger().debug(
            f"TTSNode: after gain/clip: samples={audio_resampled.shape[0]}, "
            f"max_abs={max_abs_after:.4f}"
        )

        # Playback — simple "mouth fully open during speech" model for now.
        # Also publish tts_speaking=True while we are talking.
        self._publish_mouth_open(1.0)
        self._publish_tts_speaking(True)
        try:
            device = None if self.output_device_index < 0 else self.output_device_index
            play_pcm(audio_resampled, sample_rate=playback_sr, device=device)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"TTSNode: audio playback failed: {exc}")
        finally:
            self._publish_mouth_open(0.0)
            self._publish_tts_speaking(False)
            self._publish_speech_done()


# ----------------------------------------------------------------------
# Main entry point
# ----------------------------------------------------------------------


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("TTSNode: Shutting down (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
