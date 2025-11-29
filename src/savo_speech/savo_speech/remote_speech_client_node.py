#!/usr/bin/env python3
"""
Robot Savo — Remote Speech Client Node (Wake Words + Face State + Auto-Sleep)

This node runs on the Pi and connects the microphone to the Robot_Savo_Server
"speech gateway" (/speech endpoint), which performs STT + LLM in one step.

Data flow:
  mic audio  -->  /speech (Robot_Savo_Server)  -->  JSON reply
                                                    |
                                                    v
  /savo_speech/stt_text        (std_msgs/String: transcript)
  /savo_speech/tts_text        (std_msgs/String: reply_text)
  /savo_intent/intent_result   (savo_msgs/IntentResult: structured result)
  /savo_ui/face_state          (std_msgs/String: "idle|listening|thinking|speaking")

Key features:
  - Audio capture from ReSpeaker via sounddevice.
  - Simple energy-based VAD.
  - Optional "utterance mode":
      * buffer multiple blocks while speech is present,
      * when silence (or max duration) happens → send ONE WAV to /speech.
  - HTTP POST to SPEECH_SERVER_URL with in-memory WAV.
  - Robust error handling and logging.
  - Respects /savo_speech/tts_speaking gate to avoid self-listening.
  - Skips "no-op" turns (very short transcripts with no reply/error).
  - Wake-word logic:
      * robot stays "asleep" until it hears phrases like "hei robot", "hei savo",
        "sabo", "robo savo", etc.
      * when asleep, utterances without a wake phrase are ignored for TTS/intent.
      * when awake, special "sleep" phrases can put it back to idle.
  - Face state publishing for UI:
      * "idle"      = resting, not actively in a conversation
      * "listening" = human is speaking (we are buffering an utterance)
      * "thinking"  = we are waiting for /speech (LLM) reply
      * "speaking"  = TTS is playing (driven via /tts_speaking)
  - Auto-sleep:
      * if robot is awake but there is no speech and no TTS for
        awake_idle_timeout_s seconds (default 30.0 s), it goes back to sleep.
      * the silence countdown effectively starts after TTS is done
        (we do not auto-sleep while tts_speaking = True).
"""

from __future__ import annotations

import io
import json
import threading
import time
import wave
from typing import Optional, Tuple, List

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

    Continuously records audio from the Pi's microphone,
    sends it to the Robot_Savo_Server /speech endpoint, and publishes:

      - /savo_speech/stt_text        (transcript)
      - /savo_speech/tts_text        (reply_text)
      - /savo_intent/intent_result   (IntentResult)
      - /savo_ui/face_state          ("idle|listening|thinking|speaking")

    The node is designed to be resilient against network issues and
    server errors, and to avoid recording while the robot is speaking.

    New in this upgraded version:
      - Local wake-word gating ("hei robot", "hei savo", "sabo", etc.)
      - Simple awake/asleep state (wake_active flag)
      - Face state publishing for the display UI
      - Auto-sleep if user is silent for too long (30s by default),
        only counting after TTS has finished.
    """

    def __init__(self) -> None:
        super().__init__("remote_speech_client_node")

        # ---------------------------------------------------------------------
        # Parameters (with environment variable fallbacks for URLs/IDs)
        # ---------------------------------------------------------------------
        self.declare_parameters(
            "",
            [
                ("speech_server_url", Parameter.Type.STRING),
                ("robot_id", Parameter.Type.STRING),

                # Audio capture
                ("sample_rate_hz", 16000),
                ("chunk_duration_s", 2.0),
                ("energy_threshold", 0.0003),

                # Utterance-level behaviour
                ("utterance_mode", True),
                ("max_utterance_duration_s", 15.0),

                # HTTP client
                ("request_timeout_s", 15.0),
                ("max_retries", 2),

                # TTS gate (self-listening protection + speaking state)
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

                # Face / UI integration
                ("face_state_topic", "/savo_ui/face_state"),

                # Wake-word configuration
                ("wake_word_enable", True),
                (
                    "wake_words",
                    [
                        "hei robot",
                        "hei robo",
                        "hei savo",
                        "savo",
                        "sabo",
                        "robo savo",
                        "robot savo",
                        "robot sabo",
                    ],
                ),
                (
                    "sleep_phrases",
                    [
                        "go to sleep",
                        "you can rest",
                        "that is all",
                        "that’s all",
                        "kiitos",
                        "thanks robot",
                    ],
                ),

                # Auto-sleep if user is silent for too long (seconds)
                # Countdown effectively starts after TTS has finished.
                ("awake_idle_timeout_s", 30.0),
            ],
        )

        # URLs / identity with env fallbacks
        self.speech_server_url: str = self._get_param_with_env(
            "speech_server_url", env_name="SPEECH_SERVER_URL", default_value=""
        )
        self.robot_id: str = self._get_param_with_env(
            "robot_id", env_name="ROBOT_ID", default_value="robot_savo_pi"
        )

        # Other parameters
        self.sample_rate_hz: int = int(self.get_parameter("sample_rate_hz").value)
        self.chunk_duration_s: float = float(self.get_parameter("chunk_duration_s").value)
        self.energy_threshold: float = float(self.get_parameter("energy_threshold").value)

        self.utterance_mode: bool = bool(self.get_parameter("utterance_mode").value)
        self.max_utterance_duration_s: float = float(
            self.get_parameter("max_utterance_duration_s").value
        )

        self.request_timeout_s: float = float(self.get_parameter("request_timeout_s").value)
        self.max_retries: int = int(self.get_parameter("max_retries").value)

        self.tts_gate_enable: bool = bool(self.get_parameter("tts_gate_enable").value)
        self.tts_speaking_topic: str = str(
            self.get_parameter("tts_speaking_topic").value
        )
        self.tts_gate_cooldown_s: float = float(
            self.get_parameter("tts_gate_cooldown_s").value
        )

        self.idle_sleep_s: float = float(self.get_parameter("idle_sleep_s").value)
        self.input_device_index: int = int(self.get_parameter("input_device_index").value)

        self.min_transcript_chars: int = int(
            self.get_parameter("min_transcript_chars").value
        )

        self.debug_logging: bool = bool(self.get_parameter("debug_logging").value)

        # Face / UI
        self.face_state_topic: str = str(self.get_parameter("face_state_topic").value)

        # Wake-word configuration
        self.wake_word_enable: bool = bool(
            self.get_parameter("wake_word_enable").value
        )
        raw_wake_words = self.get_parameter("wake_words").value
        raw_sleep_phrases = self.get_parameter("sleep_phrases").value

        # Normalise to lowercase lists of strings
        self.wake_words: List[str] = [
            str(w).strip().lower()
            for w in (raw_wake_words or [])
            if str(w).strip()
        ]
        self.sleep_phrases: List[str] = [
            str(w).strip().lower()
            for w in (raw_sleep_phrases or [])
            if str(w).strip()
        ]

        # Auto-sleep configuration
        self.awake_idle_timeout_s: float = float(
            self.get_parameter("awake_idle_timeout_s").value
        )

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
            f"  debug_logging           = {self.debug_logging}\n"
            f"  face_state_topic        = {self.face_state_topic}\n"
            f"  wake_word_enable        = {self.wake_word_enable}\n"
            f"  wake_words              = {self.wake_words}\n"
            f"  sleep_phrases           = {self.sleep_phrases}\n"
            f"  awake_idle_timeout_s    = {self.awake_idle_timeout_s}"
        )

        # ---------------------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------------------
        self.stt_text_pub = self.create_publisher(String, "/savo_speech/stt_text", 10)
        self.tts_text_pub = self.create_publisher(String, "/savo_speech/tts_text", 10)
        self.intent_result_pub = self.create_publisher(
            IntentResult, "/savo_intent/intent_result", 10
        )
        self.face_state_pub = self.create_publisher(
            String, self.face_state_topic, 10
        )

        # ---------------------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------------------
        # TTS speaking gate (also drives "speaking"/"idle|listening" face state)
        self.tts_speaking: bool = False
        self.create_subscription(
            Bool,
            self.tts_speaking_topic,
            self._tts_speaking_cb,
            10,
        )

        # ---------------------------------------------------------------------
        # Internal conversation state
        # ---------------------------------------------------------------------
        self._shutdown_flag: bool = False

        # Utterance buffering
        self._in_utterance: bool = False
        self._utterance_start_time: float = 0.0
        self._utterance_buffers: List[np.ndarray] = []

        # Wake / sleep state
        self.wake_active: bool = False
        self._current_face_state: str = ""  # ensure we always go through setter
        self._set_face_state("idle")

        # Track last time we had user activity (utterance / reply / wake/sleep)
        self._last_user_activity_time: float = time.time()

        # Background worker thread (audio capture + /speech calls)
        self._worker_thread = threading.Thread(
            target=self._main_loop,
            name="remote_speech_worker",
            daemon=True,
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
    # Face state helpers
    # -------------------------------------------------------------------------

    def _set_face_state(self, state: str) -> None:
        """
        Update and publish the current face state if it changed.

        Valid states (by convention):
          "idle", "listening", "thinking", "speaking"
        """
        state = (state or "").strip().lower()
        if not state:
            return

        if state == self._current_face_state:
            return

        self._current_face_state = state

        msg = String()
        msg.data = state
        self.face_state_pub.publish(msg)

        if self.debug_logging:
            self.get_logger().debug(f"[Face] state -> {state}")

    # -------------------------------------------------------------------------
    # Subscribers
    # -------------------------------------------------------------------------

    def _tts_speaking_cb(self, msg: Bool) -> None:
        """
        Track whether TTS is currently speaking.

        This is used both as a simple gate to avoid self-listening,
        and to drive the "speaking"/"idle|listening" face state.
        """
        was_speaking = self.tts_speaking
        self.tts_speaking = bool(msg.data)

        if self.debug_logging:
            self.get_logger().debug(
                f"TTS speaking state: {self.tts_speaking} (was {was_speaking})"
            )

        # When TTS starts, we are clearly "speaking".
        if self.tts_speaking:
            self._set_face_state("speaking")
            # Also count this as activity; we don't want to sleep while mid-sentence.
            self._last_user_activity_time = time.time()
            return

        # TTS just stopped
        if was_speaking and not self.tts_speaking:
            # Reset activity time so auto-sleep countdown starts AFTER TTS done.
            self._last_user_activity_time = time.time()
            if self.wake_active:
                self._set_face_state("listening")
            else:
                self._set_face_state("idle")

    # -------------------------------------------------------------------------
    # Main loop
    # -------------------------------------------------------------------------

    def _main_loop(self) -> None:
        """
        Background worker thread:

        - Waits until TTS is not speaking (if gate enabled).
        - Records chunks of audio.
        - In utterance_mode:
            * while chunks have energy → buffer them,
            * on silence or max duration → send full utterance to /speech.
        - In non-utterance mode:
            * every voiced chunk is sent as its own request.

        Also periodically checks if the robot has been awake but idle
        (no utterance, no TTS) for too long and auto-sleeps.
        """
        self.get_logger().info("Remote speech worker thread started.")

        last_tts_off_time: float = 0.0

        while rclpy.ok() and not self._shutdown_flag:
            # Avoid listening to ourselves (basic gate)
            if self.tts_gate_enable and self.tts_speaking:
                last_tts_off_time = time.time()
                time.sleep(self.idle_sleep_s)
                self._check_idle_timeout()
                continue

            # Cooldown after TTS stops
            if self.tts_gate_enable and not self.tts_speaking:
                now = time.time()
                if now - last_tts_off_time < self.tts_gate_cooldown_s:
                    time.sleep(self.idle_sleep_s)
                    self._check_idle_timeout()
                    continue

            # Record a chunk
            try:
                audio_data = self._record_chunk()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Audio capture error: {exc}")
                time.sleep(1.0)
                self._check_idle_timeout()
                continue

            if audio_data is None or len(audio_data) == 0:
                time.sleep(self.idle_sleep_s)
                self._check_idle_timeout()
                continue

            # Simple energy-based VAD
            rms = self._compute_rms(audio_data)
            if self.debug_logging:
                self.get_logger().debug(
                    f"Chunk RMS={rms:.6f}, threshold={self.energy_threshold:.6f}, "
                    f"in_utterance={self._in_utterance}"
                )

            # Silence
            if rms < self.energy_threshold:
                if self.utterance_mode and self._in_utterance:
                    # End-of-utterance detected
                    self._finalize_utterance()
                # If not in utterance, just idle (face state driven elsewhere)
                self._check_idle_timeout()
                continue

            # Voiced chunk
            if not self.utterance_mode:
                # Simple per-chunk mode: send immediately
                if self.debug_logging:
                    self.get_logger().debug(
                        f"Sending single chunk to /speech (len={len(audio_data)} samples)"
                    )
                # User activity: reset idle timer
                self._last_user_activity_time = time.time()

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
                self._check_idle_timeout()
                continue

            # Utterance mode: buffer while we have speech
            now = time.time()
            if not self._in_utterance:
                # Start new utterance
                self._in_utterance = True
                self._utterance_start_time = now
                self._utterance_buffers = [audio_data]
                # As soon as we detect voice, we consider ourselves "listening".
                self._set_face_state("listening")
                # User activity: reset idle timer
                self._last_user_activity_time = now
                if self.debug_logging:
                    self.get_logger().debug("Started new utterance buffer.")
            else:
                # Continue existing utterance
                self._utterance_buffers.append(audio_data)
                # Continuous speech counts as activity
                self._last_user_activity_time = now

            # Check max utterance duration
            if now - self._utterance_start_time >= self.max_utterance_duration_s:
                if self.debug_logging:
                    self.get_logger().debug(
                        "Max utterance duration reached. Finalizing utterance."
                    )
                self._finalize_utterance()

            self._check_idle_timeout()

        # If we exit the loop while still in an utterance, flush it
        if self.utterance_mode and self._in_utterance:
            self._finalize_utterance()

        self.get_logger().info("Remote speech worker thread exiting.")

    def _finalize_utterance(self) -> None:
        """
        Concatenate buffered chunks into one utterance, send to /speech,
        and publish the results.
        """
        if not self._utterance_buffers:
            # Nothing to do
            self._in_utterance = False
            self._utterance_start_time = 0.0
            # If we had started listening but got no data, fall back to idle/listening.
            if not self.tts_speaking:
                if self.wake_active:
                    self._set_face_state("listening")
                else:
                    self._set_face_state("idle")
            return

        audio_all = np.concatenate(self._utterance_buffers)
        num_samples = len(audio_all)

        if self.debug_logging:
            duration = num_samples / float(self.sample_rate_hz)
            self.get_logger().debug(
                f"Finalizing utterance with {len(self._utterance_buffers)} chunks, "
                f"{num_samples} samples (~{duration:.2f} s)."
            )

        # Reset utterance state BEFORE doing network I/O
        self._utterance_buffers = []
        self._in_utterance = False
        self._utterance_start_time = 0.0

        # We are now waiting for the LLM → "thinking" state (unless TTS is already speaking).
        if not self.tts_speaking:
            self._set_face_state("thinking")

        # Full utterance = user activity; reset idle timer
        self._last_user_activity_time = time.time()

        wav_bytes = self._encode_wav_bytes(audio_all, self.sample_rate_hz)
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
    # Idle auto-sleep helper
    # -------------------------------------------------------------------------

    def _check_idle_timeout(self) -> None:
        """
        Auto-sleep if robot is awake but user is silent for too long.

        Conditions:
          - wake_active == True
          - not currently speaking (tts_speaking == False)
          - not currently buffering an utterance (_in_utterance == False)
          - awake_idle_timeout_s > 0
          - now - last_user_activity_time >= awake_idle_timeout_s

        This effectively starts counting after:
          - the last user utterance was finalized, OR
          - TTS finished (we reset last_user_activity_time in _tts_speaking_cb).
        """
        if not self.wake_active:
            return
        if self.tts_speaking:
            # Never auto-sleep while speaking.
            return
        if self._in_utterance:
            # We are still in an utterance, don't sleep.
            return
        if self.awake_idle_timeout_s <= 0.0:
            return

        now = time.time()
        if now - self._last_user_activity_time >= self.awake_idle_timeout_s:
            self.get_logger().info(
                "[Wake] Idle timeout reached. Going back to sleep."
            )
            self.wake_active = False
            self._set_face_state("idle")

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
        except Exception as exc:  # noqa: BLE001
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
        self,
        wav_bytes: bytes,
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
            except Exception as exc:  # noqa: BLE001
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
        self,
        wav_bytes: bytes,
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
        except json.JSONDecodeError as exc:  # noqa: BLE001
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
    # Wake-word helpers
    # -------------------------------------------------------------------------

    def _contains_any_phrase(self, text_lc: str, phrases: List[str]) -> bool:
        """Return True if any phrase from list is a substring of text_lc."""
        if not text_lc or not phrases:
            return False
        return any(p in text_lc for p in phrases)

    def _apply_wake_word_policy(
        self,
        transcript: str,
        reply_text: str,
        intent: str,
        nav_goal: Optional[str],
        llm_ok: bool,  # kept for future policy tweaks
    ) -> Tuple[str, str, str, Optional[str], bool, bool]:
        """
        Apply local wake-word / sleep-phrase policy.

        Returns:
            (final_transcript, final_reply_text, final_intent,
             final_nav_goal, allow_intent_result, allow_tts)
        """
        # Defaults: publish everything
        allow_intent = True
        allow_tts = True

        t_clean = (transcript or "").strip()
        r_clean = (reply_text or "").strip()
        text_lc = t_clean.lower()

        if not self.wake_word_enable:
            # Wake-word disabled → just stay "awake" always
            self.wake_active = True
            self._last_user_activity_time = time.time()
            return transcript, reply_text, intent, nav_goal, allow_intent, allow_tts

        # ---- Robot currently asleep: look for wake words ----
        if not self.wake_active:
            if self._contains_any_phrase(text_lc, self.wake_words):
                # Wake up
                self.wake_active = True
                self._last_user_activity_time = time.time()
                self.get_logger().info("[Wake] Wake phrase detected. Robot is now awake.")
                # First turn after wake: treat as greeting, not a full command.
                allow_intent = False  # do not drive navigation/status logic yet
                allow_tts = True      # but we can still speak a short reply

                if not r_clean:
                    reply_text = "Yes, I am here. How can I help you?"
                # Face is now ready to listen after this short acknowledgement.
                # Actual "speaking" visual is driven by /tts_speaking.
                self._set_face_state("listening")
            else:
                # No wake phrase → ignore for TTS/intent
                self.get_logger().info(
                    "[Wake] Ignoring utterance while asleep (no wake word)."
                )
                allow_intent = False
                allow_tts = False
                # We stay idle visually
                self._set_face_state("idle")

            return transcript, reply_text, intent, nav_goal, allow_intent, allow_tts

        # ---- Robot is already awake: maybe handle sleep phrases ----
        if self._contains_any_phrase(text_lc, self.sleep_phrases):
            # User wants us to rest; we answer once and then go idle.
            self.get_logger().info("[Wake] Sleep phrase detected. Robot will go idle.")
            self._last_user_activity_time = time.time()
            self.wake_active = False
            allow_intent = False  # do not feed this into nav/status logic
            allow_tts = True

            if not r_clean:
                reply_text = "Okay, I will rest now. Call me again when you need help."
            # After TTS finishes, /tts_speaking callback will move face to "idle".
            return transcript, reply_text, intent, nav_goal, allow_intent, allow_tts

        # Otherwise: normal awake conversation
        self._last_user_activity_time = time.time()
        return transcript, reply_text, intent, nav_goal, allow_intent, allow_tts

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

        Wake-word logic is applied here to decide whether we actually
        publish TTS and IntentResult (STT is still published for debugging).
        """
        # Normalise strings
        t_clean = (transcript or "").strip()
        r_clean = (reply_text or "").strip()
        e_clean = (error_msg or "").strip()

        # Apply wake-word / sleep-phrase policy
        (
            transcript,
            reply_text,
            intent,
            nav_goal,
            allow_intent,
            allow_tts,
        ) = self._apply_wake_word_policy(
            transcript=transcript,
            reply_text=reply_text,
            intent=intent,
            nav_goal=nav_goal,
            llm_ok=llm_ok,
        )

        # Recompute cleaned values after possible modifications
        t_clean = (transcript or "").strip()
        r_clean = (reply_text or "").strip()

        # Skip pure "no-op" turns: no meaningful transcript, no reply, no error
        if len(t_clean) < self.min_transcript_chars and not r_clean and not e_clean:
            if self.debug_logging:
                self.get_logger().debug(
                    "Skipping publish: transcript too short "
                    f"(len={len(t_clean)}, min={self.min_transcript_chars}) "
                    "and no reply/error."
                )
            # When nothing is going on, if we're not speaking, keep UI in idle/listening.
            if not self.tts_speaking:
                if self.wake_active:
                    self._set_face_state("listening")
                else:
                    self._set_face_state("idle")
            return

        # Log summary
        self.get_logger().info(f"[STT] transcript: '{t_clean}'")
        self.get_logger().info(
            f"[LLM] intent={intent}, nav_goal={nav_goal or ''}, "
            f"tier_used={tier_used}, success={llm_ok}, error='{e_clean}'"
        )
        self.get_logger().info(f"[TTS] reply_text: '{r_clean}' (allow_tts={allow_tts})")

        # ---------------------------------------------------------------------
        # STT text — always publish for debugging / logging
        # ---------------------------------------------------------------------
        stt_msg = String()
        stt_msg.data = transcript
        self.stt_text_pub.publish(stt_msg)

        # ---------------------------------------------------------------------
        # TTS text — only publish if allowed by wake-word policy
        # ---------------------------------------------------------------------
        if allow_tts and r_clean:
            tts_msg = String()
            tts_msg.data = reply_text
            self.tts_text_pub.publish(tts_msg)
        else:
            if self.debug_logging:
                self.get_logger().debug(
                    "[TTS] Suppressed TTS publish due to wake/sleep policy "
                    f"(allow_tts={allow_tts}, reply_len={len(r_clean)})"
                )

        # ---------------------------------------------------------------------
        # IntentResult — only publish when allowed
        # ---------------------------------------------------------------------
        if allow_intent:
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
        else:
            if self.debug_logging:
                self.get_logger().debug(
                    "[Intent] Suppressed IntentResult publish due to wake/sleep policy."
                )

        # If we are awake and not currently speaking, default visual is "listening".
        if not self.tts_speaking:
            if self.wake_active:
                self._set_face_state("listening")
            else:
                self._set_face_state("idle")

    # -------------------------------------------------------------------------
    # Shutdown
    # -------------------------------------------------------------------------

    def destroy_node(self) -> bool:
        self._shutdown_flag = True
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[RemoteSpeechClientNode] = None
    try:
        node = RemoteSpeechClientNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("KeyboardInterrupt, shutting down.")
    except Exception as exc:  # noqa: BLE001
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
