#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert TTS PCM or activity into a smooth UI mouth level."""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16MultiArray


class MouthAnimNode(Node):
    """Publish mouth level for the face UI."""

    def __init__(self) -> None:
        super().__init__("mouth_anim")
        self.declare_parameter("mode", "pcm")

        self.declare_parameter("pcm_topic", "/savo_speech/tts_pcm")
        self.declare_parameter("pcm_sample_rate", 16000.0)
        self.declare_parameter("window_duration_s", 0.04)
        self.declare_parameter("rms_floor", 0.02)
        self.declare_parameter("rms_ceiling", 0.30)

        self.declare_parameter("activity_topic", "/savo_speech/mouth_open")

        self.declare_parameter("output_topic", "/savo_ui/mouth_level")
        self.declare_parameter("update_rate_hz", 30.0)

        self.declare_parameter("attack_gain", 0.5)
        self.declare_parameter("decay_gain", 0.15)

        self.declare_parameter("min_level", 0.0)
        self.declare_parameter("max_level", 1.0)

        self.declare_parameter("debug_logging", False)
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

        self._window_samples: int = max(
            1, int(round(self.window_duration_s * self.pcm_sample_rate))
        )
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
        self._target_activity: float = 0.0
        self._current_level: float = 0.0

        self._pcm_buffer: Optional[np.ndarray] = None  # float32 in [-1, 1]
        self._pcm_index: int = 0
        self._pcm_total: int = 0
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
    def _pcm_cb(self, msg: Int16MultiArray) -> None:
        """Store a full utterance PCM buffer for timer-driven playback."""
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
    def _activity_cb(self, msg: Float32) -> None:
        raw = float(msg.data)
        if raw < 0.0:
            raw = 0.0
        elif raw > 1.0:
            raw = 1.0

        self._target_activity = raw
    def _timer_cb(self) -> None:
        if self.mode == "pcm":
            self._update_from_pcm()
        else:
            self._update_from_activity()

        out_msg = Float32()
        out_msg.data = float(self._current_level)
        self.pub.publish(out_msg)

        if self.debug_logging:
            self.get_logger().debug(
                f"MouthAnim update: level={self._current_level:.3f}, "
                f"target={self._target_activity:.3f}"
            )
    def _update_from_pcm(self) -> None:
        """Step through PCM buffer, compute RMS, map to target_activity."""
        if self._pcm_buffer is None or self._pcm_total <= 0:
            self._target_activity = 0.0
            self._smooth_step()
            return

        start = self._pcm_index
        if start >= self._pcm_total:
            self._pcm_buffer = None
            self._pcm_total = 0
            self._pcm_index = 0
            self._target_activity = 0.0
            self._smooth_step()
            return

        end = min(start + self._window_samples, self._pcm_total)
        segment = self._pcm_buffer[start:end]

        if segment.size == 0:
            self._target_activity = 0.0
        else:
            rms = float(np.sqrt(np.mean(segment * segment)))

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

        self._pcm_index = min(self._pcm_total, start + self._hop_samples)

        self._smooth_step()

    def _update_from_activity(self) -> None:
        self._smooth_step()

    def _smooth_step(self) -> None:
        target = self._target_activity
        current = self._current_level

        delta = target - current

        if delta > 0.0:
            gain = self.attack_gain
        else:
            gain = self.decay_gain

        current += delta * gain

        if current < 0.0:
            current = 0.0
        if current > self.max_level:
            current = self.max_level

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
