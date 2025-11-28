#!/usr/bin/env python3
"""
Robot Savo — Remote Speech Client Node

This node runs on the Pi and connects the microphone to the Robot_Savo_Server
"speech gateway" (/speech endpoint), which performs STT + LLM in one step.

Data flow:
  mic audio  -->  /speech (Robot_Savo_Server)  -->  JSON reply
                                                    |
                                                    v
  /savo_speech/stt_text        (std_msgs/String: transcript)
  /savo_speech/tts_text        (std_msgs/String: reply_text)
  /savo_intent/intent_result   (savo_msgs/IntentResult: structured result)

Key features:
  - Chunked audio recording with simple energy-based VAD.
  - HTTP POST to SPEECH_SERVER_URL with in-memory WAV.
  - Robust error handling and logging.
  - Respects /savo_speech/tts_speaking gate to avoid self-listening.
  - Skips "no-op" turns (very short transcripts with no reply/error).
"""

import io
import json
import threading
import time
import wave
from typing import Optional, Tuple

import numpy as np
import requests
import sounddevice as sd

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import String, Bool
from savo_msgs.msg import IntentResult


class RemoteSpeechClientNode(Node):
    """
    RemoteSpeechClientNode

    Continuously records short audio segments from the Pi's microphone,
    sends them to the Robot_Savo_Server /speech endpoint, and publishes:

      - /savo_speech/stt_text        (transcript)
      - /savo_speech/tts_text        (reply_text)
      - /savo_intent/intent_result   (IntentResult)

    The node is designed to be resilient against network issues and
    server errors, and to avoid recording while the robot is speaking.
    """

    def __init__(self) -> None:
        super().__init__("remote_speech_client_node")

        # Parameters (with environment variable fallbacks for URLs/IDs)
        self.declare_parameters(
            "",
            [
                ("speech_server_url", Parameter.Type.STRING),
                ("robot_id", Parameter.Type.STRING),

                # Audio capture
                ("sample_rate_hz", 16000),
                ("chunk_duration_s", 2.0),
                ("energy_threshold", 0.0003),

                # Utterance-level behaviour (future)
                ("utterance_mode", True),
                ("max_utterance_duration_s", 15.0),

                # HTTP client
                ("request_timeout_s", 20.0),
                ("max_retries", 2),

                # TTS gate
                ("tts_gate_enable", True),
                ("tts_speaking_topic", "/savo_speech/tts_speaking"),
                ("tts_gate_cooldown_s", 0.8),

                # Loop behaviour
                ("idle_sleep_s", 0.1),
                ("input_device_index", 0),

                # Minimum transcript length for a "real" utterance
                ("min_transcript_chars", 3),

                # Logging
                ("debug_logging", False),
            ],
        )

        # URLs / identity with env fallbacks
        self.speech_server_url = self._get_param_with_env(
            "speech_server_url", env_name="SPEECH_SERVER_URL", default_value=""
        )
        self.robot_id = self._get_param_with_env(
            "robot_id", env_name="ROBOT_ID", default_value="robot_savo_pi"
        )

        # Other parameters
        self.sample_rate_hz = int(self.get_parameter("sample_rate_hz").value)
        self.chunk_duration_s = float(self.get_parameter("chunk_duration_s").value)
        self.energy_threshold = float(self.get_parameter("energy_threshold").value)

        self.utterance_mode = bool(self.get_parameter("utterance_mode").value)
        self.max_utterance_duration_s = float(
            self.get_parameter("max_utterance_duration_s").value
        )

        self.request_timeout_s = float(self.get_parameter("request_timeout_s").value)
        self.max_retries = int(self.get_parameter("max_retries").value)

        self.tts_gate_enable = bool(self.get_parameter("tts_gate_enable").value)
        self.tts_speaking_topic = str(
            self.get_parameter("tts_speaking_topic").value
        )
        self.tts_gate_cooldown_s = float(
            self.get_parameter("tts_gate_cooldown_s").value
        )

        self.idle_sleep_s = float(self.get_parameter("idle_sleep_s").value)
        self.input_device_index = int(self.get_parameter("input_device_index").value)

        self.min_transcript_chars = int(
            self.get_parameter("min_transcript_chars").value
        )

        self.debug_logging = bool(self.get_parameter("debug_logging").value)

        # Basic validation
        if not self.speech_server_url or not self.speech_server_url.startswith("http"):
            self.get_logger().error(
                "speech_server_url is not set or invalid. "
                "Set it via ROS param 'speech_server_url' or SPEECH_SERVER_URL env."
            )
            raise RuntimeError("Invalid speech_server_url")

        self.get_logger().info(
            "RemoteSpeechClientNode starting with:\n"
            f"  speech_server_url       = {self.speech_server_url}\n"
            f"  robot_id                = {self.robot_id}\n"
            f"  sample_rate_hz          = {self.sample_rate_hz}\n"
            f"  chunk_duration_s        = {self.chunk_duration_s}\n"
            f"  energy_threshold        = {self.energy_threshold}\n"
            f"  input_device_index      = {self.input_device_index}\n"
            f"  utterance_mode          = {self.utterance_mode}\n"
            f"  max_utterance_duration  = {self.max_utterance_duration_s}\n"
            f"  request_timeout_s       = {self.request_timeout_s}\n"
            f"  max_retries             = {self.max_retries}\n"
            f"  tts_gate_enable         = {self.tts_gate_enable}\n"
            f"  tts_speaking_topic      = {self.tts_speaking_topic}\n"
            f"  tts_gate_cooldown_s     = {self.tts_gate_cooldown_s}\n"
            f"  idle_sleep_s            = {self.idle_sleep_s}\n"
            f"  min_transcript_chars    = {self.min_transcript_chars}\n"
            f"  debug_logging           = {self.debug_logging}"
        )

        # Publishers
        self.stt_text_pub = self.create_publisher(String, "/savo_speech/stt_text", 10)
        self.tts_text_pub = self.create_publisher(String, "/savo_speech/tts_text", 10)
        self.intent_result_pub = self.create_publisher(
            IntentResult, "/savo_intent/intent_result", 10
        )

        # Subscriber: TTS speaking gate
        self.tts_speaking = False
        self.create_subscription(
            Bool,
            self.tts_speaking_topic,
            self._tts_speaking_cb,
            10,
        )

        # Internal state
        self._shutdown_flag = False
        self._worker_thread = threading.Thread(
            target=self._main_loop, name="remote_speech_worker", daemon=True
        )
        self._worker_thread.start()

    # -------------------------------------------------------------------------
    # Parameter helpers
    # -------------------------------------------------------------------------

    def _get_param_with_env(
        self, name: str, env_name: Optional[str] = None, default_value: str = ""
    ) -> str:
        """
        Helper to get a parameter, with an optional environment variable
        as a fallback, and finally a default value.
        """
        param = self.get_parameter(name)
        if param.type_ != Parameter.Type.NOT_SET and param.value not in ("", None):
            return str(param.value)

        if env_name:
            import os

            env_val = os.environ.get(env_name)
            if env_val:
                # Reflect env into ROS parameter as well for debugging
                self.set_parameters(
                    [Parameter(name=name, value=env_val)]
                )
                return env_val

        # Use default
        if default_value:
            self.set_parameters([Parameter(name=name, value=default_value)])
        return default_value

    # -------------------------------------------------------------------------
    # Subscribers
    # -------------------------------------------------------------------------

    def _tts_speaking_cb(self, msg: Bool) -> None:
        self.tts_speaking = bool(msg.data)
        if self.debug_logging:
            self.get_logger().debug(f"TTS speaking state: {self.tts_speaking}")

    # -------------------------------------------------------------------------
    # Main loop
    # -------------------------------------------------------------------------

    def _main_loop(self) -> None:
        """
        Background worker thread:

        - Waits until TTS is not speaking (if gate enabled).
        - Records a chunk of audio.
        - Skips if silence (RMS below threshold).
        - Sends to /speech with retries.
        - Publishes transcript / reply / IntentResult.
        """
        self.get_logger().info("Remote speech worker thread started.")

        last_tts_off_time: float = 0.0

        while rclpy.ok() and not self._shutdown_flag:
            # Avoid listening to ourselves (basic gate)
            if self.tts_gate_enable and self.tts_speaking:
                last_tts_off_time = time.time()
                time.sleep(self.idle_sleep_s)
                continue

            # Cooldown after TTS stops
            if self.tts_gate_enable and not self.tts_speaking:
                now = time.time()
                if now - last_tts_off_time < self.tts_gate_cooldown_s:
                    time.sleep(self.idle_sleep_s)
                    continue

            # Record a chunk
            try:
                audio_data = self._record_chunk()
            except Exception as exc:
                self.get_logger().error(f"Audio capture error: {exc}")
                time.sleep(1.0)
                continue

            if audio_data is None or len(audio_data) == 0:
                time.sleep(self.idle_sleep_s)
                continue

            # Simple energy-based VAD
            rms = self._compute_rms(audio_data)
            if self.debug_logging:
                self.get_logger().debug(f"Chunk RMS={rms:.6f}")

            if rms < self.energy_threshold:
                if self.debug_logging:
                    self.get_logger().debug(
                        f"Chunk skipped due to low energy (rms={rms:.6f})."
                    )
                continue

            if self.debug_logging:
                self.get_logger().debug(
                    f"Sending chunk to /speech (rms={rms:.6f}, "
                    f"len={len(audio_data)} samples)..."
                )

            # Convert to WAV bytes
            wav_bytes = self._encode_wav_bytes(audio_data, self.sample_rate_hz)

            # Call /speech with retries
            (
                transcript,
                reply_text,
                intent,
                nav_goal,
                tier_used,
                llm_ok,
                error_msg,
            ) = self._call_speech_with_retries(wav_bytes)

            # Publish outputs
            self._publish_results(
                transcript=transcript,
                reply_text=reply_text,
                intent=intent,
                nav_goal=nav_goal,
                tier_used=tier_used,
                llm_ok=llm_ok,
                error_msg=error_msg,
            )

        self.get_logger().info("Remote speech worker thread exiting.")

    # -------------------------------------------------------------------------
    # Audio capture helpers
    # -------------------------------------------------------------------------

    def _record_chunk(self) -> Optional[np.ndarray]:
        """
        Records a single chunk of audio from the default or selected input device.

        Returns:
            np.ndarray of shape (N,) with dtype int16, or None on error.
        """
        frames = int(self.sample_rate_hz * self.chunk_duration_s)
        if frames <= 0:
            self.get_logger().warn("chunk_duration_s <= 0, skipping record.")
            time.sleep(0.5)
            return None

        try:
            # device=None uses default input; otherwise use specified index
            device = None if self.input_device_index < 0 else self.input_device_index
            audio = sd.rec(
                frames,
                samplerate=self.sample_rate_hz,
                channels=1,
                dtype="int16",
                device=device,
            )
            sd.wait()
        except Exception as exc:
            self.get_logger().error(f"sounddevice.rec failed: {exc}")
            return None

        # Flatten to 1D int16 array
        return audio.reshape(-1)

    @staticmethod
    def _compute_rms(audio: np.ndarray) -> float:
        """
        Compute RMS of int16 audio and normalize to [0, 1].
        """
        if audio.size == 0:
            return 0.0
        float_audio = audio.astype(np.float32) / 32768.0
        return float(np.sqrt(np.mean(float_audio * float_audio)))

    @staticmethod
    def _encode_wav_bytes(audio: np.ndarray, sample_rate_hz: int) -> bytes:
        """
        Encode int16 mono audio as a WAV file in memory.
        """
        buffer = io.BytesIO()
        with wave.open(buffer, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # 16-bit
            wf.setframerate(sample_rate_hz)
            wf.writeframes(audio.tobytes())
        return buffer.getvalue()

    # -------------------------------------------------------------------------
    # HTTP /speech call
    # -------------------------------------------------------------------------

    def _call_speech_with_retries(
        self, wav_bytes: bytes
    ) -> Tuple[str, str, str, Optional[str], str, bool, str]:
        """
        Calls the /speech endpoint with a WAV file payload, using retries.

        Returns:
            (transcript, reply_text, intent, nav_goal, tier_used, llm_ok, error_msg)
        """
        last_error = ""
        for attempt in range(1, self.max_retries + 1):
            try:
                return self._call_speech_once(wav_bytes)
            except Exception as exc:
                last_error = str(exc)
                self.get_logger().warn(
                    f"/speech request failed (attempt {attempt}/{self.max_retries}): "
                    f"{exc}"
                )
                time.sleep(1.0)

        # All attempts failed → fallback reply
        fallback_reply = (
            "I am having a connection problem with my server. "
            "Please try again in a moment."
        )
        return (
            "",
            fallback_reply,
            "STATUS",
            None,
            "unknown",
            False,
            last_error or "All /speech attempts failed.",
        )

    def _call_speech_once(
        self, wav_bytes: bytes
    ) -> Tuple[str, str, str, Optional[str], str, bool, str]:
        """
        Single HTTP POST to /speech.

        Returns:
            (transcript, reply_text, intent, nav_goal, tier_used, llm_ok, error_msg)
        """
        files = {
            "file": ("audio.wav", wav_bytes, "audio/wav"),
        }

        resp = requests.post(
            self.speech_server_url,
            files=files,
            timeout=self.request_timeout_s,
        )

        if resp.status_code != 200:
            raise RuntimeError(
                f"/speech HTTP {resp.status_code}: {resp.text[:200]}"
            )

        try:
            data = resp.json()
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"/speech invalid JSON: {exc}")

        if self.debug_logging:
            self.get_logger().debug(f"/speech raw JSON: {data}")

        transcript = str(data.get("transcript", "") or "")
        reply_text = str(data.get("reply_text", "") or "")
        intent = str(data.get("intent", "") or "CHATBOT")
        nav_goal = data.get("nav_goal", None)
        tier_used = str(data.get("tier_used", "") or "unknown")
        llm_ok = bool(data.get("llm_ok", True))

        # Normalize nav_goal: None or non-empty string
        if isinstance(nav_goal, str) and nav_goal.strip() == "":
            nav_goal = None

        return transcript, reply_text, intent, nav_goal, tier_used, llm_ok, ""

    # -------------------------------------------------------------------------
    # Publishing results
    # -------------------------------------------------------------------------

    def _publish_results(
        self,
        transcript: str,
        reply_text: str,
        intent: str,
        nav_goal: Optional[str],
        tier_used: str,
        llm_ok: bool,
        error_msg: str,
    ) -> None:
        """
        Publish transcript, reply_text, and IntentResult to ROS.

        Also logs a concise summary for debugging:
          [STT] transcript: '...'
          [LLM] intent=..., nav_goal=..., tier_used=..., success=..., error='...'
          [TTS] reply_text: '...'
        """

        # Normalise strings
        t_clean = (transcript or "").strip()
        r_clean = (reply_text or "").strip()
        e_clean = (error_msg or "").strip()

        # Skip pure "no-op" turns: no meaningful transcript, no reply, no error
        if len(t_clean) < self.min_transcript_chars and not r_clean and not e_clean:
            if self.debug_logging:
                self.get_logger().debug(
                    "Skipping publish: transcript too short "
                    f"(len={len(t_clean)}, min={self.min_transcript_chars}) "
                    "and no reply/error."
                )
            return

        # Log summary
        self.get_logger().info(f"[STT] transcript: '{t_clean}'")
        self.get_logger().info(
            f"[LLM] intent={intent}, nav_goal={nav_goal or ''}, "
            f"tier_used={tier_used}, success={llm_ok}, error='{e_clean}'"
        )
        self.get_logger().info(f"[TTS] reply_text: '{r_clean}'")

        # Publish STT text (we still publish the raw transcript, even if short,
        # as long as this is not a "no-op" turn).
        stt_msg = String()
        stt_msg.data = transcript
        self.stt_text_pub.publish(stt_msg)

        # Publish TTS text (we always publish, even if llm_ok is False,
        # since we might have a human-friendly error message).
        tts_msg = String()
        tts_msg.data = reply_text
        self.tts_text_pub.publish(tts_msg)

        # IntentResult
        ir = IntentResult()
        ir.source = "remote_speech_client"
        ir.robot_id = self.robot_id
        ir.user_text = transcript
        ir.reply_text = reply_text
        ir.intent = intent
        ir.nav_goal = nav_goal if nav_goal is not None else ""
        ir.tier_used = tier_used
        ir.success = bool(llm_ok)
        ir.error = error_msg or ""

        self.intent_result_pub.publish(ir)

        if self.debug_logging:
            self.get_logger().debug(
                f"Published IntentResult: intent={intent}, "
                f"nav_goal={ir.nav_goal}, tier_used={tier_used}, "
                f"success={ir.success}, error='{ir.error}'"
            )

    # -------------------------------------------------------------------------
    # Shutdown
    # -------------------------------------------------------------------------

    def destroy_node(self) -> bool:
        self._shutdown_flag = True
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RemoteSpeechClientNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("KeyboardInterrupt, shutting down.")
    except Exception as exc:
        # If initialization failed, we may not have a logger yet
        if node is not None:
            node.get_logger().error(f"Unhandled exception: {exc}")
        else:
            print(f"remote_speech_client_node: unhandled exception: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
