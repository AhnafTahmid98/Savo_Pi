#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Mouth Animation Node (audio-driven)

This node converts either:
  1) full TTS PCM audio, or
  2) simple Float32 "mouth activity"
into a smooth, UI-friendly mouth open/close level for the on-robot display.

Modes
-----
1) PCM mode (recommended, real audio):
   - Input:  std_msgs/Int16MultiArray on /savo_speech/tts_pcm
             (mono PCM int16, sample_rate ≈ 16000 Hz)
   - Output: std_msgs/Float32 on /savo_ui/mouth_level
             (smoothed value 0.0–1.0 for face_view)

   Behaviour:
   - On each TTS utterance, TTSNode publishes the full PCM buffer.
   - This node walks through the buffer at update_rate_hz (e.g. 30 Hz),
     computes RMS in a short window, maps RMS→[0,1], then applies
     attack/decay smoothing.

2) Activity mode (fallback / legacy):
   - Input:  std_msgs/Float32 on /savo_speech/mouth_open
             (0.0 = closed, 1.0 = fully open; rough/spiky)
   - Output: std_msgs/Float32 on /savo_ui/mouth_level
             (smoothed 0.0–1.0)

Parameters (typical YAML)
-------------------------
  mode: "pcm"                  # or "activity"
  pcm_topic: "/savo_speech/tts_pcm"
  pcm_sample_rate: 16000       # must match TTS playback
  activity_topic: "/savo_speech/mouth_open"
  output_topic: "/savo_ui/mouth_level"
  update_rate_hz: 30.0

  window_duration_s: 0.04      # RMS window ~40 ms
  rms_floor: 0.02              # below this → level ≈ 0
  rms_ceiling: 0.30            # above this → level ≈ 1

  attack_gain: 0.5             # how fast it opens
  decay_gain: 0.15             # how fast it closes

  min_level: 0.0               # lower cutoff for tiny noise
  max_level: 1.0               # clamp upper bound

  debug_logging: false
"""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16MultiArray


class MouthAnimNode(Node):
    """ROS 2 node that converts TTS audio/activity into a mouth level for the UI."""

    def __init__(self) -> None:
        super().__init__("mouth_anim")

        # ------------------------------------------------------------------
        # 1. Declare parameters
        # ------------------------------------------------------------------
        # Mode selection: "pcm" (real audio, recommended) or "activity"
        self.declare_parameter("mode", "pcm")

        # PCM mode: full audio from TTS
        self.declare_parameter("pcm_topic", "/savo_speech/tts_pcm")
        self.declare_parameter("pcm_sample_rate", 16000.0)   # must match TTS playback
        self.declare_parameter("window_duration_s", 0.04)    # ~40 ms RMS window
        self.declare_parameter("rms_floor", 0.02)            # noise floor
        self.declare_parameter("rms_ceiling", 0.30)          # loud speech level

        # Activity mode: simple Float32 activity (legacy/fallback)
        self.declare_parameter("activity_topic", "/savo_speech/mouth_open")

        # Common output + timing
        self.declare_parameter("output_topic", "/savo_ui/mouth_level")
        self.declare_parameter("update_rate_hz", 30.0)

        # Smoothing / animation behaviour
        self.declare_parameter("attack_gain", 0.5)   # how fast it opens
        self.declare_parameter("decay_gain", 0.15)   # how fast it closes

        self.declare_parameter("min_level", 0.0)     # optional lower cutoff
        self.declare_parameter("max_level", 1.0)     # clamp upper bound

        # Logging
        self.declare_parameter("debug_logging", False)

        # ------------------------------------------------------------------
        # 2. Read parameters
        # ------------------------------------------------------------------
        self.mode: str = (
            self.get_parameter("mode").get_parameter_value().string_value.strip().lower()
        )
        if self.mode not in ("pcm", "activity"):
            self.get_logger().warn(
                f"mode='{self.mode}' is invalid; falling back to 'pcm'"
            )
            self.mode = "pcm"

        self.pcm_topic: str = (
            self.get_parameter("pcm_topic").get_parameter_value().string_value
        )
        self.pcm_sample_rate: float = (
            self.get_parameter("pcm_sample_rate").get_parameter_value().double_value
        )
        self.window_duration_s: float = (
            self.get_parameter("window_duration_s").get_parameter_value().double_value
        )
        self.rms_floor: float = (
            self.get_parameter("rms_floor").get_parameter_value().double_value
        )
        self.rms_ceiling: float = (
            self.get_parameter("rms_ceiling").get_parameter_value().double_value
        )

        self.activity_topic: str = (
            self.get_parameter("activity_topic").get_parameter_value().string_value
        )

        self.output_topic: str = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        update_rate_hz = (
            self.get_parameter("update_rate_hz").get_parameter_value().double_value
        )

        self.attack_gain = (
            self.get_parameter("attack_gain").get_parameter_value().double_value
        )
        self.decay_gain = (
            self.get_parameter("decay_gain").get_parameter_value().double_value
        )
        self.min_level = (
            self.get_parameter("min_level").get_parameter_value().double_value
        )
        self.max_level = (
            self.get_parameter("max_level").get_parameter_value().double_value
        )

        self.debug_logging: bool = (
            self.get_parameter("debug_logging").get_parameter_value().bool_value
        )

        if update_rate_hz <= 0.0:
            self.get_logger().warn(
                f"update_rate_hz={update_rate_hz:.3f} invalid; clamping to 30.0"
            )
            update_rate_hz = 30.0
        self.update_rate_hz: float = update_rate_hz
        self.dt: float = 1.0 / self.update_rate_hz

        # Clamp / sanity
        if self.pcm_sample_rate <= 0.0:
            self.get_logger().warn(
                f"pcm_sample_rate={self.pcm_sample_rate} invalid; clamping to 16000"
            )
            self.pcm_sample_rate = 16000.0

        if self.window_duration_s <= 0.0:
            self.get_logger().warn(
                f"window_duration_s={self.window_duration_s} invalid; clamping to 0.04"
            )
            self.window_duration_s = 0.04

        # Convert durations to samples
        self._window_samples: int = max(
            1, int(round(self.window_duration_s * self.pcm_sample_rate))
        )
        # Step through audio roughly in sync with update_rate_hz
        self._hop_samples: int = max(
            1, int(round(self.pcm_sample_rate / self.update_rate_hz))
        )

        if self.rms_ceiling <= self.rms_floor:
            self.get_logger().warn(
                f"rms_floor ({self.rms_floor:.3f}) >= rms_ceiling "
                f"({self.rms_ceiling:.3f}); using defaults 0.02/0.30"
            )
            self.rms_floor = 0.02
            self.rms_ceiling = 0.30

        if self.max_level <= 0.0:
            self.max_level = 1.0

        # ------------------------------------------------------------------
        # 3. State variables
        # ------------------------------------------------------------------
        # Current "target" activity (0–1) and smoothed level (0–1)
        self._target_activity: float = 0.0
        self._current_level: float = 0.0

        # PCM playback state (PCM mode only)
        self._pcm_buffer: Optional[np.ndarray] = None  # float32 in [-1, 1]
        self._pcm_index: int = 0
        self._pcm_total: int = 0

        # ------------------------------------------------------------------
        # 4. ROS interfaces
        # ------------------------------------------------------------------
        if self.mode == "pcm":
            self.get_logger().info(
                f"MouthAnimNode running in PCM mode, subscribing to {self.pcm_topic}"
            )
            self._pcm_sub = self.create_subscription(
                Int16MultiArray,
                self.pcm_topic,
                self._pcm_cb,
                10,
            )
        else:
            self.get_logger().info(
                f"MouthAnimNode running in ACTIVITY mode, subscribing to {self.activity_topic}"
            )
            self._activity_sub = self.create_subscription(
                Float32,
                self.activity_topic,
                self._activity_cb,
                10,
            )

        self.pub = self.create_publisher(Float32, self.output_topic, 10)
        self.timer = self.create_timer(self.dt, self._timer_cb)

        self.get_logger().info(
            "MouthAnimNode started with:\n"
            f"  mode             = {self.mode}\n"
            f"  pcm_topic        = {self.pcm_topic}\n"
            f"  pcm_sample_rate  = {self.pcm_sample_rate:.1f}\n"
            f"  window_duration  = {self.window_duration_s:.3f} s "
            f"({self._window_samples} samples)\n"
            f"  hop_samples      = {self._hop_samples} samples "
            f"(~{self.update_rate_hz:.1f} Hz)\n"
            f"  rms_floor        = {self.rms_floor:.3f}\n"
            f"  rms_ceiling      = {self.rms_ceiling:.3f}\n"
            f"  activity_topic   = {self.activity_topic}\n"
            f"  output_topic     = {self.output_topic}\n"
            f"  update_rate_hz   = {self.update_rate_hz:.1f}\n"
            f"  attack_gain      = {self.attack_gain:.3f}\n"
            f"  decay_gain       = {self.decay_gain:.3f}\n"
            f"  min_level        = {self.min_level:.3f}\n"
            f"  max_level        = {self.max_level:.3f}\n"
            f"  debug_logging    = {self.debug_logging}"
        )

    # ----------------------------------------------------------------------
    # PCM subscriber: store full utterance audio
    # ----------------------------------------------------------------------
    def _pcm_cb(self, msg: Int16MultiArray) -> None:
        """
        Receive a full utterance PCM buffer (int16), convert to float32,
        and reset playback index so the timer can step through it.
        """
        data = np.array(msg.data, dtype=np.int16)
        if data.size == 0:
            self.get_logger().debug("MouthAnimNode: received empty PCM buffer, ignoring")
            return

        audio_f32 = data.astype(np.float32) / 32768.0
        self._pcm_buffer = audio_f32
        self._pcm_total = int(audio_f32.shape[0])
        self._pcm_index = 0

        if self.debug_logging:
            self.get_logger().debug(
                f"MouthAnimNode: new PCM buffer, samples={self._pcm_total}"
            )

    # ----------------------------------------------------------------------
    # Activity subscriber: update target activity directly (legacy mode)
    # ----------------------------------------------------------------------
    def _activity_cb(self, msg: Float32) -> None:
        # Clamp incoming value to [0, 1]
        raw = float(msg.data)
        if raw < 0.0:
            raw = 0.0
        elif raw > 1.0:
            raw = 1.0

        self._target_activity = raw

    # ----------------------------------------------------------------------
    # Timer callback
    # ----------------------------------------------------------------------
    def _timer_cb(self) -> None:
        if self.mode == "pcm":
            self._update_from_pcm()
        else:
            self._update_from_activity()

        # Publish current_level
        out_msg = Float32()
        out_msg.data = float(self._current_level)
        self.pub.publish(out_msg)

        if self.debug_logging:
            self.get_logger().debug(
                f"MouthAnim update: level={self._current_level:.3f}, "
                f"target={self._target_activity:.3f}"
            )

    # ----------------------------------------------------------------------
    # PCM mode: compute target_activity from audio buffer
    # ----------------------------------------------------------------------
    def _update_from_pcm(self) -> None:
        """Step through PCM buffer, compute RMS, map to target_activity."""
        if self._pcm_buffer is None or self._pcm_total <= 0:
            # No audio → target = 0
            self._target_activity = 0.0
            self._smooth_step()
            return

        start = self._pcm_index
        if start >= self._pcm_total:
            # Buffer fully consumed
            self._pcm_buffer = None
            self._pcm_total = 0
            self._pcm_index = 0
            self._target_activity = 0.0
            self._smooth_step()
            return

        end = min(start + self._window_samples, self._pcm_total)
        segment = self._pcm_buffer[start:end]

        if segment.size == 0:
            # Nothing to analyse this frame
            self._target_activity = 0.0
        else:
            rms = float(np.sqrt(np.mean(segment * segment)))

            # Map RMS → [0, 1] using floor/ceiling
            if rms <= self.rms_floor:
                level = 0.0
            elif rms >= self.rms_ceiling:
                level = 1.0
            else:
                level = (rms - self.rms_floor) / (self.rms_ceiling - self.rms_floor)

            self._target_activity = max(0.0, min(1.0, level))

            if self.debug_logging:
                self.get_logger().debug(
                    f"MouthAnim PCM: rms={rms:.4f}, level={self._target_activity:.3f}, "
                    f"idx={start}->{end}/{self._pcm_total}"
                )

        # Advance playback index by hop_samples
        self._pcm_index = min(self._pcm_total, start + self._hop_samples)

        # Smooth towards the new target_activity
        self._smooth_step()

    # ----------------------------------------------------------------------
    # Activity mode: just smooth whatever we last received
    # ----------------------------------------------------------------------
    def _update_from_activity(self) -> None:
        self._smooth_step()

    # ----------------------------------------------------------------------
    # Smoothing logic (attack/decay) shared by both modes
    # ----------------------------------------------------------------------
    def _smooth_step(self) -> None:
        target = self._target_activity
        current = self._current_level

        delta = target - current

        if delta > 0.0:
            # Mouth opening → attack
            gain = self.attack_gain
        else:
            # Mouth closing → decay (often slower)
            gain = self.decay_gain

        # Basic exponential smoothing step
        current += delta * gain

        # Clamp to [0, max_level]
        if current < 0.0:
            current = 0.0
        if current > self.max_level:
            current = self.max_level

        # Optional minimum visible level (for tiny noise wiggles)
        if current < self.min_level:
            current = 0.0

        self._current_level = current


def main(args: Optional[list[str]] = None) -> None:
    """Entry point for ROS 2."""
    rclpy.init(args=args)
    node = MouthAnimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down MouthAnimNode")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
