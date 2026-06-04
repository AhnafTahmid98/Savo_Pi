"""
Robot Savo — Remote STT Client Node (Pi side)

This node runs on the Pi and:
  - Captures audio from the ReSpeaker mic using sounddevice
  - Performs simple VAD + utterance detection
  - Sends each utterance as a WAV file to a remote STT server
  - Publishes the returned text to /savo_speech/stt_text

The STT server is your PC/Mac STT microservice (FastAPI + faster-whisper),
listening e.g. on http://robot-llm.local:9000/transcribe.

Design goals:
  - Do NOT modify existing stt_node / speech_bringup.
  - This is an ALTERNATIVE STT path, selected via a different launch file.
  - Keep parameters similar to your existing stt config.
"""

from __future__ import annotations

import io
import time
import wave
from typing import Optional

import numpy as np
import requests
import sounddevice as sd

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class RemoteSTTClientNode(Node):
    def __init__(self) -> None:
        super().__init__("remote_stt_client_node")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("stt_server_url", "http://robot-llm.local:9000")
        self.declare_parameter("endpoint_path", "/transcribe")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("block_duration_s", 1.0)
        self.declare_parameter("energy_threshold", 0.0003)
        self.declare_parameter("input_device_index", 0)
        self.declare_parameter("max_utterance_duration_s", 10.0)
        self.declare_parameter("min_transcript_chars", 3)

        # TTS gate (so robot doesn't transcribe its own voice)
        self.declare_parameter("tts_gate_enable", True)
        self.declare_parameter("tts_speaking_topic", "/savo_speech/tts_speaking")
        self.declare_parameter("tts_gate_cooldown_s", 0.8)

        # Resolve parameter values
        self.stt_server_url: str = (
            self.get_parameter("stt_server_url").get_parameter_value().string_value
        )
        endpoint_path: str = (
            self.get_parameter("endpoint_path").get_parameter_value().string_value
        )
        if not endpoint_path.startswith("/"):
            endpoint_path = "/" + endpoint_path
        self.endpoint = self.stt_server_url.rstrip("/") + endpoint_path

        self.sample_rate: int = (
            self.get_parameter("sample_rate").get_parameter_value().integer_value
        )
        self.block_duration_s: float = (
            self.get_parameter("block_duration_s").get_parameter_value().double_value
        )
        self.energy_threshold: float = (
            self.get_parameter("energy_threshold").get_parameter_value().double_value
        )
        self.input_device_index: int = (
            self.get_parameter("input_device_index").get_parameter_value().integer_value
        )
        self.max_utterance_duration_s: float = (
            self.get_parameter("max_utterance_duration_s")
            .get_parameter_value()
            .double_value
        )
        self.min_transcript_chars: int = (
            self.get_parameter("min_transcript_chars")
            .get_parameter_value()
            .integer_value
        )

        self.tts_gate_enable: bool = (
            self.get_parameter("tts_gate_enable").get_parameter_value().bool_value
        )
        self.tts_speaking_topic: str = (
            self.get_parameter("tts_speaking_topic")
            .get_parameter_value()
            .string_value
        )
        self.tts_gate_cooldown_s: float = (
            self.get_parameter("tts_gate_cooldown_s")
            .get_parameter_value()
            .double_value
        )

        self.get_logger().info(
            f"Remote STT client node starting\n"
            f"  STT endpoint       : {self.endpoint}\n"
            f"  sample_rate        : {self.sample_rate}\n"
            f"  block_duration_s   : {self.block_duration_s}\n"
            f"  energy_threshold   : {self.energy_threshold}\n"
            f"  input_device_index : {self.input_device_index}\n"
            f"  max_utt_dur_s      : {self.max_utterance_duration_s}\n"
            f"  tts_gate_enable    : {self.tts_gate_enable}\n"
            f"  tts_gate_topic     : {self.tts_speaking_topic}\n"
            f"  tts_gate_cooldown  : {self.tts_gate_cooldown_s}"
        )

        # ------------------------------------------------------------------
        # ROS pubs/subs
        # ------------------------------------------------------------------
        self.pub_text = self.create_publisher(String, "/savo_speech/stt_text", 10)

        self._tts_speaking: bool = False
        self._tts_last_speaking_time: float = 0.0

        if self.tts_gate_enable:
            self.create_subscription(
                Bool,
                self.tts_speaking_topic,
                self._on_tts_speaking,
                10,
            )

        # ------------------------------------------------------------------
        # Audio stream state
        # ------------------------------------------------------------------
        self._buffer_blocks: list[np.ndarray] = []
        self._utterance_started: bool = False
        self._utterance_start_time: float = 0.0

        blocksize = int(self.sample_rate * self.block_duration_s)

        try:
            self._stream = sd.InputStream(
                samplerate=self.sample_rate,
                channels=1,
                device=self.input_device_index,
                blocksize=blocksize,
                callback=self._audio_callback,
            )
            self._stream.start()
        except Exception as e:
            self.get_logger().error(f"Failed to open audio input device: {e}")
            raise

        self.get_logger().info("Remote STT client node is running.")

    # ----------------------------------------------------------------------
    # TTS speaking callback (gate)
    # ----------------------------------------------------------------------
    def _on_tts_speaking(self, msg: Bool) -> None:
        self._tts_speaking = msg.data
        if msg.data:
            self._tts_last_speaking_time = time.time()

    def _tts_gate_active(self) -> bool:
        if not self.tts_gate_enable:
            return False
        now = time.time()
        if self._tts_speaking:
            return True
        # Cooldown after TTS stops
        if now - self._tts_last_speaking_time < self.tts_gate_cooldown_s:
            return True
        return False

    # ----------------------------------------------------------------------
    # Audio callback / utterance logic
    # ----------------------------------------------------------------------
    def _audio_callback(self, indata, frames, time_info, status) -> None:
        if status:
            self.get_logger().warn(f"Audio callback status: {status}")

        # Always copy to avoid sounddevice buffer reuse issues
        block = indata.copy().astype(np.float32)
        energy = float(np.mean(block**2))
        now = time.time()

        # If robot is speaking (TTS gate), do not build utterances
        if self._tts_gate_active():
            # reset current utterance
            if self._utterance_started:
                self._buffer_blocks.clear()
                self._utterance_started = False
            return

        voiced = energy > self.energy_threshold

        if voiced:
            if not self._utterance_started:
                self._utterance_started = True
                self._utterance_start_time = now
                self._buffer_blocks = []
                # Optional: short log for debugging
                self.get_logger().debug("Utterance started")
            self._buffer_blocks.append(block)

            # Safety: cut off extremely long continuous speech
            if now - self._utterance_start_time > self.max_utterance_duration_s:
                self._finish_utterance()
        else:
            # Silence
            if self._utterance_started:
                # One silent block after speech → finalize
                self._finish_utterance()

    def _finish_utterance(self) -> None:
        """Finalize current utterance: send to remote STT, publish text."""
        if not self._buffer_blocks:
            self._utterance_started = False
            return

        self._utterance_started = False

        # Concatenate blocks into one 1D float32 array
        audio = np.concatenate(self._buffer_blocks, axis=0).flatten()
        self._buffer_blocks = []

        # Convert to WAV bytes (16 kHz mono, PCM16)
        wav_bytes = self._to_wav_bytes(audio)

        # Send to STT server
        try:
            self.get_logger().info(
                f"Sending utterance to remote STT (len={len(wav_bytes)} bytes)..."
            )
            files = {
                "audio": ("audio.wav", wav_bytes, "audio/wav"),
            }
            resp = requests.post(self.endpoint, files=files, timeout=15.0)
            resp.raise_for_status()
            data = resp.json()
        except Exception as e:
            self.get_logger().error(f"Error calling remote STT: {e}")
            return

        text = str(data.get("text", "")).strip()
        if len(text) < self.min_transcript_chars:
            self.get_logger().info(
                f"Remote STT text too short or empty: {repr(text)} (ignored)"
            )
            return

        msg = String()
        msg.data = text
        self.pub_text.publish(msg)
        self.get_logger().info(f"Remote STT transcript: {text!r}")

    def _to_wav_bytes(self, audio: np.ndarray) -> bytes:
        """Convert float32 audio [-1.0, 1.0] to 16-bit mono WAV bytes."""
        # Clip and scale
        audio = np.clip(audio, -1.0, 1.0)
        int16 = np.int16(audio * 32767.0)

        buf = io.BytesIO()
        with wave.open(buf, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # 16-bit
            wf.setframerate(self.sample_rate)
            wf.writeframes(int16.tobytes())
        return buf.getvalue()

    # ----------------------------------------------------------------------
    # Shutdown
    # ----------------------------------------------------------------------
    def destroy_node(self) -> None:
        try:
            if hasattr(self, "_stream"):
                self._stream.stop()
                self._stream.close()
        except Exception as e:
            self.get_logger().warn(f"Error closing audio stream: {e}")
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RemoteSTTClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Remote STT client node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
