#!/usr/bin/env python3
"""
Robot Savo — Remote Speech Client Node (utterance-aware)

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
  - Utterance-level mode:
      * capture small audio blocks (e.g. 2s)
      * buffer while voice is present
      * finalize utterance on silence or max duration
      * send ONE HTTP request to /speech per utterance
  - Simple energy-based VAD for block/utterance detection.
  - HTTP POST to SPEECH_SERVER_URL with in-memory WAV.
  - Respects /savo_speech/tts_speaking gate with cooldown to avoid self-listening.
  - Robust error handling and logging.
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

    Continuously records audio from the Pi's microphone, groups it into
    utterances (if enabled), sends them to the Robot_Savo_Server /speech
    endpoint, and publishes:

      - /savo_speech/stt_text        (transcript)
      - /savo_speech/tts_text        (reply_text)
      - /savo_intent/intent_result   (IntentResult)

    The node is designed to be resilient against network issues and
    server errors, and to avoid recording while the robot is speaking.
    """

    def __init__(self) -> None:
        super().__init__("remote_speech_client_node")

        # Parameters (with environment variable fallbacks where relevant)
        self.declare_parameters(
            "",
            [
                ("speech_server_url", Parameter.Type.STRING),
                ("robot_id", Parameter.Type.STRING),
                # Audio capture
                ("sample_rate_hz", 16000),
                ("chunk_duration_s", 2.0),
                ("energy_threshold", 0.0003),
                ("input_device_index", 0),
                # Utterance behavior
                ("utterance_mode", True),
                ("max_utterance_duration_s", 15.0),
                # HTTP /speech
                ("request_timeout_s", 15.0),
                ("max_retries", 2),
                # TTS gate
                ("tts_gate_enable", True),
                ("tts_speaking_topic", "/savo_speech/tts_speaking"),
                ("tts_gate_cooldown_s", 0.8),
                # Node behavior
                ("idle_sleep_s", 0.1),
                ("debug_logging", False),
            ],
        )

        # Endpoint + identity
        self.speech_server_url = self._get_param_with_env(
            "speech_server_url", env_name="SPEECH_SERVER_URL", default_value=""
        )
        self.robot_id = self._get_param_with_env(
            "robot_id", env_name="ROBOT_ID", default_value="robot_savo_pi"
        )

        # Audio params
        self.sample_rate_hz = int(self.get_parameter("sample_rate_hz").value)
        self.chunk_duration_s = float(self.get_parameter("chunk_duration_s").value)
        self.energy_threshold = float(self.get_parameter("energy_threshold").value)
        self.input_device_index = int(self.get_parameter("input_device_index").value)

        # Utterance params
        self.utterance_mode = bool(self.get_parameter("utterance_mode").value)
        self.max_utterance_duration_s = float(
            self.get_parameter("max_utterance_duration_s").value
        )

        # HTTP params
        self.request_timeout_s = float(self.get_parameter("request_timeout_s").value)
        self.max_retries = int(self.get_parameter("max_retries").value)

        # TTS gate params
        self.tts_gate_enable = bool(self.get_parameter("tts_gate_enable").value)
        self.tts_speaking_topic = str(
            self.get_parameter("tts_speaking_topic").value or "/savo_speech/tts_speaking"
        )
        self.tts_gate_cooldown_s = float(
            self.get_parameter("tts_gate_cooldown_s").value
        )

        # Node behavior
        self.idle_sleep_s = float(self.get_parameter("idle_sleep_s").value)
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
        self._last_tts_state_change = time.monotonic()
        self.create_subscription(
            Bool,
            self.tts_speaking_topic,
            self._tts_speaking_cb,
            10,
        )

        # Utterance buffer state
        self._utt_active = False
        self._utt_start_time = 0.0
        self._utt_last_voice_time = 0.0
        self._utt_buffers = []  # list of np.ndarray blocks

        # Internal worker
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
                self.set_parameters([Parameter(name=name, value=env_val)])
                return env_val

        # Use default
        if default_value:
            self.set_parameters([Parameter(name=name, value=default_value)])
        return default_value

    # -------------------------------------------------------------------------
    # Subscribers
    # -------------------------------------------------------------------------

    def _tts_speaking_cb(self, msg: Bool) -> None:
        new_state = bool(msg.data)
        if new_state != self.tts_speaking:
            self.tts_speaking = new_state
            self._last_tts_state_change = time.monotonic()
            if self.debug_logging:
                self.get_logger().debug(
                    f"TTS speaking changed to {self.tts_speaking}"
                )
        else:
            self.tts_speaking = new_state

    # -------------------------------------------------------------------------
    # Main loop
    # -------------------------------------------------------------------------

    def _main_loop(self) -> None:
        """
        Background worker thread:

        - Applies TTS gate (and cooldown).
        - In utterance_mode:
            * records blocks
            * appends voiced blocks to utterance buffer
            * finalizes on silence or max duration
        - In non-utterance mode:
            * treats each voiced block as a separate request
        - Sends to /speech with retries.
        - Publishes transcript / reply / IntentResult.
        """
        self.get_logger().info("Remote speech worker thread started.")

        while rclpy.ok() and not self._shutdown_flag:
            # Respect TTS gate if enabled
            if self.tts_gate_enable:
                now = time.monotonic()
                if self.tts_speaking:
                    # Robot is speaking → do not listen
                    if self.debug_logging:
                        self.get_logger().debug("TTS speaking; skipping capture.")
                    time.sleep(self.idle_sleep_s)
                    continue
                # Cooldown after TTS stops
                time_since_change = now - self._last_tts_state_change
                if time_since_change < self.tts_gate_cooldown_s:
                    if self.debug_logging:
                        self.get_logger().debug(
                            f"TTS cooldown ({time_since_change:.3f}s < "
                            f"{self.tts_gate_cooldown_s}s); skipping capture."
                        )
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

            rms = self._compute_rms(audio_data)

            if self.debug_logging:
                self.get_logger().debug(
                    f"Recorded chunk: samples={len(audio_data)}, rms={rms:.6f}"
                )

            if self.utterance_mode:
                self._handle_chunk_utterance_mode(audio_data, rms)
            else:
                self._handle_chunk_simple_mode(audio_data, rms)

        self.get_logger().info("Remote speech worker thread exiting.")

    # -------------------------------------------------------------------------
    # Utterance logic
    # -------------------------------------------------------------------------

    def _handle_chunk_utterance_mode(self, audio_data: np.ndarray, rms: float) -> None:
        """
        Utterance-aware handling:
          - Start utterance when voiced block arrives.
          - Continue appending while voiced or until max duration.
          - Finalize on sustained silence or duration limit.
        """
        now = time.monotonic()
        voiced = rms >= self.energy_threshold

        # Case 1: voiced block
        if voiced:
            if not self._utt_active:
                # Start new utterance
                self._utt_active = True
                self._utt_start_time = now
                self._utt_last_voice_time = now
                self._utt_buffers = [audio_data]
                if self.debug_logging:
                    self.get_logger().debug(
                        "Utterance started (first voiced block)."
                    )
            else:
                # Continue existing utterance
                self._utt_last_voice_time = now
                self._utt_buffers.append(audio_data)
                if self.debug_logging:
                    duration = now - self._utt_start_time
                    self.get_logger().debug(
                        f"Utterance extended: blocks={len(self._utt_buffers)}, "
                        f"duration={duration:.3f}s"
                    )

            # Check duration limit
            if self._utt_active:
                utt_duration = now - self._utt_start_time
                if utt_duration >= self.max_utterance_duration_s:
                    if self.debug_logging:
                        self.get_logger().debug(
                            "Utterance max duration reached; finalizing."
                        )
                    self._finalize_utterance_send_and_publish()
            return

        # Case 2: silent block
        if not self._utt_active:
            # No utterance in progress; ignore silence
            if self.debug_logging:
                self.get_logger().debug("Silent block, no active utterance.")
            return

        # We have an active utterance, and got a silent block -> end-of-utterance
        if self.debug_logging:
            duration = now - self._utt_start_time
            silence_since_voice = now - self._utt_last_voice_time
            self.get_logger().debug(
                "Silent block with active utterance; finalizing. "
                f"utt_duration={duration:.3f}s, "
                f"silence_since_voice={silence_since_voice:.3f}s"
            )

        self._finalize_utterance_send_and_publish()

    def _finalize_utterance_send_and_publish(self) -> None:
        """
        Concatenate buffered blocks, send to /speech, publish results,
        then reset utterance state.
        """
        if not self._utt_buffers:
            # Nothing to send
            self._reset_utterance()
            return

        audio = np.concatenate(self._utt_buffers)
        if self.debug_logging:
            self.get_logger().debug(
                f"Finalizing utterance: total_samples={len(audio)}"
            )

        wav_bytes = self._encode_wav_bytes(audio, self.sample_rate_hz)

        (
            transcript,
            reply_text,
            intent,
            nav_goal,
            tier_used,
            llm_ok,
            error_msg,
        ) = self._call_speech_with_retries(wav_bytes)

        self._publish_results(
            transcript=transcript,
            reply_text=reply_text,
            intent=intent,
            nav_goal=nav_goal,
            tier_used=tier_used,
            llm_ok=llm_ok,
            error_msg=error_msg,
        )

        self._reset_utterance()

    def _reset_utterance(self) -> None:
        """
        Reset utterance buffer state.
        """
        self._utt_active = False
        self._utt_start_time = 0.0
        self._utt_last_voice_time = 0.0
        self._utt_buffers = []

    def _handle_chunk_simple_mode(self, audio_data: np.ndarray, rms: float) -> None:
        """
        Simple per-block mode:
          - If voiced, send this block directly to /speech.
          - If silent, ignore.
        """
        if rms < self.energy_threshold:
            if self.debug_logging:
                self.get_logger().debug(
                    f"Chunk skipped due to low energy (rms={rms:.6f})."
                )
            return

        if self.debug_logging:
            self.get_logger().debug(
                "Simple mode: sending voiced block as a single request."
            )

        wav_bytes = self._encode_wav_bytes(audio_data, self.sample_rate_hz)

        (
            transcript,
            reply_text,
            intent,
            nav_goal,
            tier_used,
            llm_ok,
            error_msg,
        ) = self._call_speech_with_retries(wav_bytes)

        self._publish_results(
            transcript=transcript,
            reply_text=reply_text,
            intent=intent,
            nav_goal=nav_goal,
            tier_used=tier_used,
            llm_ok=llm_ok,
            error_msg=error_msg,
        )

    # -------------------------------------------------------------------------
    # Audio capture helpers
    # -------------------------------------------------------------------------

    def _record_chunk(self) -> Optional[np.ndarray]:
        """
        Records a single chunk of audio from the selected input device.

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

        # All attempts failed
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
        """
        # STT text
        stt_msg = String()
        stt_msg.data = transcript
        self.stt_text_pub.publish(stt_msg)

        # TTS text
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
                "Published IntentResult: "
                f"intent={intent}, nav_goal={ir.nav_goal}, "
                f"tier_used={tier_used}, success={ir.success}, "
                f"error='{ir.error}'"
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
