#!/usr/bin/env python3
"""Remote /speech client with wake-word gating and face-state updates."""

from __future__ import annotations

import io
import json
import threading
import time
import wave
from typing import List, Optional, Tuple

import numpy as np
import requests
import sounddevice as sd

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import Bool, String
from savo_msgs.msg import IntentResult


class RemoteSpeechClientNode(Node):
    """Capture mic audio, call /speech, and publish speech/intent outputs."""

    def __init__(self) -> None:
        super().__init__("remote_speech_client_node")
        self.declare_parameters(
            "",
            [
                ("speech_server_url", Parameter.Type.STRING),
                ("robot_id", Parameter.Type.STRING),

                ("sample_rate_hz", 16000),
                ("chunk_duration_s", 2.0),
                ("energy_threshold", 0.0003),

                ("utterance_mode", True),
                ("max_utterance_duration_s", 15.0),

                ("request_timeout_s", 15.0),
                ("max_retries", 2),

                # Prevent Robot Savo from transcribing its own speaker output.
                ("tts_gate_enable", True),
                ("tts_speaking_topic", "/savo_speech/tts_speaking"),
                ("tts_gate_cooldown_s", 0.8),

                ("idle_sleep_s", 0.1),
                ("input_device_index", 0),

                ("min_transcript_chars", 3),

                ("debug_logging", False),

                ("face_state_topic", "/savo_ui/face_state"),

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

                # Starts counting after TTS finishes.
                ("awake_idle_timeout_s", 30.0),
            ],
        )

        self.speech_server_url: str = self._get_param_with_env(
            "speech_server_url", env_name="SPEECH_SERVER_URL", default_value=""
        )
        self.robot_id: str = self._get_param_with_env(
            "robot_id", env_name="ROBOT_ID", default_value="robot_savo_pi"
        )

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

        self.face_state_topic: str = str(self.get_parameter("face_state_topic").value)

        self.wake_word_enable: bool = bool(
            self.get_parameter("wake_word_enable").value
        )
        raw_wake_words = self.get_parameter("wake_words").value
        raw_sleep_phrases = self.get_parameter("sleep_phrases").value

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

        self.awake_idle_timeout_s: float = float(
            self.get_parameter("awake_idle_timeout_s").value
        )

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
        self.stt_text_pub = self.create_publisher(String, "/savo_speech/stt_text", 10)
        self.tts_text_pub = self.create_publisher(String, "/savo_speech/tts_text", 10)
        self.intent_result_pub = self.create_publisher(
            IntentResult, "/savo_intent/intent_result", 10
        )
        self.face_state_pub = self.create_publisher(
            String, self.face_state_topic, 10
        )
        self.tts_speaking: bool = False
        self._tts_last_end_time: float = 0.0
        self.create_subscription(
            Bool,
            self.tts_speaking_topic,
            self._tts_speaking_cb,
            10,
      )
        self._shutdown_flag: bool = False

        self._in_utterance: bool = False
        self._utterance_start_time: float = 0.0
        self._utterance_buffers: List[np.ndarray] = []

        # Blocks mic capture while /speech is in flight.
        self._thinking: bool = False

        self.wake_active: bool = False
        self._current_face_state: str = ""  # ensure we always go through setter
        self._set_face_state("idle")

        self._last_user_activity_time: float = time.time()

        self._worker_thread = threading.Thread(
            target=self._main_loop,
            name="remote_speech_worker",
            daemon=True,
        )
        self._worker_thread.start()
    def _get_param_with_env(
        self, name: str, env_name: Optional[str] = None, default_value: str = ""
    ) -> str:
        """Read a ROS parameter, then env var, then fallback default."""
        param = self.get_parameter(name)
        if param.type_ != Parameter.Type.NOT_SET and param.value not in ("", None):
            return str(param.value)

        if env_name:
            import os

            env_val = os.environ.get(env_name)
            if env_val:
                self.set_parameters(
                    [Parameter(name=name, value=env_val)]
                )
                return env_val

        if default_value:
            self.set_parameters([Parameter(name=name, value=default_value)])
        return default_value

    def _set_face_state(self, state: str) -> None:
        """Publish a face state only when it changes."""
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

    def _tts_speaking_cb(self, msg: Bool) -> None:
        """Track TTS state for self-listening gate and face state."""
        was_speaking = self.tts_speaking
        self.tts_speaking = bool(msg.data)

        if self.debug_logging:
            self.get_logger().debug(
                f"TTS speaking state: {self.tts_speaking} (was {was_speaking})"
            )

        if self.tts_speaking:
            self._in_utterance = False
            self._utterance_buffers = []
            self._utterance_start_time = 0.0

            self._set_face_state("speaking")
            self._last_user_activity_time = time.time()
            return

        if was_speaking and not self.tts_speaking:
            self._tts_last_end_time = time.time()

            # Auto-sleep countdown starts after TTS finishes.
            self._last_user_activity_time = self._tts_last_end_time
            if self.wake_active:
                self._set_face_state("listening")
            else:
                self._set_face_state("idle")

    def _main_loop(self) -> None:
        """Capture audio chunks and send finalized utterances to /speech."""
        self.get_logger().info("Remote speech worker thread started.")

        while rclpy.ok() and not self._shutdown_flag:
            # Avoid self-listening while Piper is active.
            if self.tts_gate_enable and self.tts_speaking:
                time.sleep(self.idle_sleep_s)
                self._check_idle_timeout()
                continue

            # Let speaker output decay before reopening the mic.
            if self.tts_gate_enable and not self.tts_speaking:
                now_gate = time.time()
                if (now_gate - self._tts_last_end_time) < self.tts_gate_cooldown_s:
                    time.sleep(self.idle_sleep_s)
                    self._check_idle_timeout()
                    continue

            # Avoid overlapping questions while /speech is still replying.
            if self._thinking:
                time.sleep(self.idle_sleep_s)
                self._check_idle_timeout()
                continue

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

            rms = self._compute_rms(audio_data)
            if self.debug_logging:
                self.get_logger().debug(
                    f"Chunk RMS={rms:.6f}, threshold={self.energy_threshold:.6f}, "
                    f"in_utterance={self._in_utterance}"
                )

            if rms < self.energy_threshold:
                if self.utterance_mode and self._in_utterance:
                    self._finalize_utterance()
                self._check_idle_timeout()
                continue

            if not self.utterance_mode:
                if self.debug_logging:
                    self.get_logger().debug(
                        f"Sending single chunk to /speech (len={len(audio_data)} samples)"
                    )
                self._last_user_activity_time = time.time()

                self._thinking = True
                self._set_face_state("thinking")

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

                self._thinking = False

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

            now = time.time()
            if not self._in_utterance:
                self._in_utterance = True
                self._utterance_start_time = now
                self._utterance_buffers = [audio_data]
                self._set_face_state("listening")
                self._last_user_activity_time = now
                if self.debug_logging:
                    self.get_logger().debug("Started new utterance buffer.")
            else:
                self._utterance_buffers.append(audio_data)
                self._last_user_activity_time = now

            if now - self._utterance_start_time >= self.max_utterance_duration_s:
                if self.debug_logging:
                    self.get_logger().debug(
                        "Max utterance duration reached. Finalizing utterance."
                    )
                self._finalize_utterance()

            self._check_idle_timeout()

        if self.utterance_mode and self._in_utterance:
            self._finalize_utterance()

        self.get_logger().info("Remote speech worker thread exiting.")

    def _finalize_utterance(self) -> None:
        """Send the buffered utterance to /speech and publish the result."""
        if not self._utterance_buffers:
            self._in_utterance = False
            self._utterance_start_time = 0.0
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

        # Reset before network I/O so capture can recover cleanly on errors.
        self._utterance_buffers = []
        self._in_utterance = False
        self._utterance_start_time = 0.0

        if not self.tts_speaking:
            self._set_face_state("thinking")

        self._thinking = True

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

        self._thinking = False

        self._publish_results(
            transcript=transcript,
            reply_text=reply_text,
            intent=intent,
            nav_goal=nav_goal,
            tier_used=tier_used,
            llm_ok=llm_ok,
            error_msg=error_msg,
        )
    def _check_idle_timeout(self) -> None:
        """Return to idle after wake mode has been quiet long enough."""
        if not self.wake_active:
            return
        if self.tts_speaking:
            return
        if self._in_utterance:
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
    def _record_chunk(self) -> Optional[np.ndarray]:
        """Record one int16 mono chunk from sounddevice."""
        frames = int(self.sample_rate_hz * self.chunk_duration_s)
        if frames <= 0:
            self.get_logger().warn("chunk_duration_s <= 0, skipping record.")
            time.sleep(0.5)
            return None

        try:
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

        return audio.reshape(-1)

    @staticmethod
    def _compute_rms(audio: np.ndarray) -> float:
        """Compute normalized RMS for int16 audio."""
        if audio.size == 0:
            return 0.0
        float_audio = audio.astype(np.float32) / 32768.0
        return float(np.sqrt(np.mean(float_audio * float_audio)))

    @staticmethod
    def _encode_wav_bytes(audio: np.ndarray, sample_rate_hz: int) -> bytes:
        """Encode int16 mono audio as an in-memory WAV."""
        buffer = io.BytesIO()
        with wave.open(buffer, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # 16-bit
            wf.setframerate(sample_rate_hz)
            wf.writeframes(audio.tobytes())
        return buffer.getvalue()
    def _call_speech_with_retries(
        self,
        wav_bytes: bytes,
    ) -> Tuple[str, str, str, Optional[str], str, bool, str]:
        """Call /speech with retries and return a fallback on failure."""
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
        """Send one HTTP request to /speech."""
        files = {
            "file": ("audio.wav", wav_bytes, "audio/wav"),
        }

        start_time = time.time()
        resp = requests.post(
            self.speech_server_url,
            files=files,
            timeout=self.request_timeout_s,
        )
        elapsed_ms = (time.time() - start_time) * 1000.0

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

        if isinstance(nav_goal, str) and nav_goal.strip() == "":
            nav_goal = None

        self.get_logger().info(
            f"[Timing] /speech total={elapsed_ms:.0f} ms, "
            f"tier_used={tier_used}, llm_ok={llm_ok}"
        )

        return transcript, reply_text, intent, nav_goal, tier_used, llm_ok, ""
    def _contains_any_phrase(self, text_lc: str, phrases: List[str]) -> bool:
        """Return true when text contains any configured phrase."""
        if not text_lc or not phrases:
            return False
        return any(p in text_lc for p in phrases)

    def _apply_wake_word_policy(
        self,
        transcript: str,
        reply_text: str,
        intent: str,
        nav_goal: Optional[str],
        llm_ok: bool,
    ) -> Tuple[str, str, str, Optional[str], bool, bool]:
        """Apply local wake/sleep policy before publishing outputs."""
        allow_intent = True
        allow_tts = True

        t_clean = (transcript or "").strip()
        r_clean = (reply_text or "").strip()
        text_lc = t_clean.lower()

        if not self.wake_word_enable:
            self.wake_active = True
            self._last_user_activity_time = time.time()
            return transcript, reply_text, intent, nav_goal, allow_intent, allow_tts

        if not self.wake_active:
            if self._contains_any_phrase(text_lc, self.wake_words):
                self.wake_active = True
                self._last_user_activity_time = time.time()
                self.get_logger().info("[Wake] Wake phrase detected. Robot is now awake.")
                # First wake phrase is acknowledged but not sent into nav/status logic.
                allow_intent = False
                allow_tts = True

                if not r_clean:
                    reply_text = "Yes, I am here. How can I help you?"
                self._set_face_state("listening")
            else:
                self.get_logger().info(
                    "[Wake] Ignoring utterance while asleep (no wake word)."
                )
                allow_intent = False
                allow_tts = False
                self._set_face_state("idle")

            return transcript, reply_text, intent, nav_goal, allow_intent, allow_tts

        if self._contains_any_phrase(text_lc, self.sleep_phrases):
            self.get_logger().info("[Wake] Sleep phrase detected. Robot will go idle.")
            self._last_user_activity_time = time.time()
            self.wake_active = False
            allow_intent = False
            allow_tts = True

            if not r_clean:
                reply_text = "Okay, I will rest now. Call me again when you need help."
            return transcript, reply_text, intent, nav_goal, allow_intent, allow_tts

        self._last_user_activity_time = time.time()
        return transcript, reply_text, intent, nav_goal, allow_intent, allow_tts

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
        """Publish speech outputs after local wake/sleep filtering."""
        t_clean = (transcript or "").strip()
        r_clean = (reply_text or "").strip()
        e_clean = (error_msg or "").strip()

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

        t_clean = (transcript or "").strip()
        r_clean = (reply_text or "").strip()

        if len(t_clean) < self.min_transcript_chars and not r_clean and not e_clean:
            if self.debug_logging:
                self.get_logger().debug(
                    "Skipping publish: transcript too short "
                    f"(len={len(t_clean)}, min={self.min_transcript_chars}) "
                    "and no reply/error."
                )
            if not self.tts_speaking:
                if self.wake_active:
                    self._set_face_state("listening")
                else:
                    self._set_face_state("idle")
            return

        self.get_logger().info(f"[STT] transcript: '{t_clean}'")
        self.get_logger().info(
            f"[LLM] intent={intent}, nav_goal={nav_goal or ''}, "
            f"tier_used={tier_used}, success={llm_ok}, error='{e_clean}'"
        )
        self.get_logger().info(f"[TTS] reply_text: '{r_clean}' (allow_tts={allow_tts})")
        stt_msg = String()
        stt_msg.data = transcript
        self.stt_text_pub.publish(stt_msg)
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

        if not self.tts_speaking:
            if self.wake_active:
                self._set_face_state("listening")
            else:
                self._set_face_state("idle")
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
