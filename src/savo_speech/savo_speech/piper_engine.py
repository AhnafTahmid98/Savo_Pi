"""
Robot Savo — Piper TTS engine wrapper.

This module wraps the Piper TTS library (piper-tts) so that ROS 2 nodes
(like tts_node.py) can synthesize speech from text using local ONNX
models (e.g. en_US-ryan-high, en_US-hfc_female-medium) without dealing
with Piper internals.

Design goals:
- Very small and stable public API (PiperEngine).
- Model loaded once per node, reused for all utterances.
- Return audio as numpy arrays so higher-level code can play it using
  savo_speech.audio_io.play_pcm().
- Safe behaviour on errors (log + return empty audio).
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
from piper.voice import PiperVoice  # provided by the piper-tts package

logger = logging.getLogger(__name__)


class PiperEngine:
    """
    High-level wrapper around PiperVoice for a single voice model.

    Typical usage (outside ROS, for testing):

        from savo_speech.piper_engine import PiperEngine
        from savo_speech.audio_io import play_pcm

        engine = PiperEngine(
            model_path="/home/savo/Savo_Pi/models/piper/en_US-ryan-high.onnx"
        )
        audio, sr = engine.synthesize_to_float32("Hello, I am Robot Savo.")
        play_pcm(audio, sample_rate=sr)

    In ROS (inside tts_node), you typically construct *one* PiperEngine
    instance per voice (male/female) and reuse it.

    Parameters
    ----------
    model_path : str | Path
        Path to the .onnx model file for the selected Piper voice.
        The matching .onnx.json file must be in the same directory.
    speaker_id : Optional[int]
        Speaker index for multi-speaker models (most English voices use 0).
    max_chars : int
        Hard limit on input text length (characters). Text longer than this
        will be truncated with a log warning, to avoid unbounded latency.
    """

    def __init__(
        self,
        model_path: str | Path,
        speaker_id: Optional[int] = None,
        max_chars: int = 500,
    ) -> None:
        self.model_path = str(model_path)
        self.speaker_id = speaker_id
        self.max_chars = max_chars

        path = Path(self.model_path)
        if not path.is_file():
            raise FileNotFoundError(
                f"PiperEngine: model file not found: {self.model_path}"
            )

        logger.info(
            "PiperEngine: loading voice model:\n"
            "  model_path = %s\n"
            "  speaker_id = %s\n"
            "  max_chars  = %d",
            self.model_path,
            str(self.speaker_id),
            self.max_chars,
        )

        # Piper will automatically use the matching .onnx.json that lives
        # next to the .onnx file.
        # NOTE: PiperVoice.load(...) may take ~1–2 seconds on the Pi.
        self._voice = PiperVoice.load(self.model_path)
        self._sample_rate: int = int(self._voice.config.sample_rate)

        logger.info(
            "PiperEngine: model loaded OK (sample_rate=%d Hz)", self._sample_rate
        )

    # ------------------------------------------------------------------ #
    # Public API                                                         #
    # ------------------------------------------------------------------ #

    @property
    def sample_rate(self) -> int:
        """Return the native sample rate of the Piper model."""
        return self._sample_rate

    def synthesize_to_pcm16(self, text: str) -> Tuple[np.ndarray, int]:
        """
        Synthesize text to 16-bit PCM audio (numpy int16) and return it.

        This is the "raw" representation that Piper produces internally.

        Parameters
        ----------
        text : str
            Input text to speak.

        Returns
        -------
        audio : np.ndarray
            1D int16 array containing the synthesized waveform.
            Empty array if synthesis failed or text was empty.
        sample_rate : int
            Model's native sample rate in Hz (e.g. 22050 or 16000).
        """
        clean = (text or "").strip()
        if not clean:
            logger.debug("PiperEngine.synthesize_to_pcm16(): empty text, skipping")
            return np.zeros(0, dtype=np.int16), self._sample_rate

        if len(clean) > self.max_chars:
            logger.warning(
                "PiperEngine: input text length %d > max_chars=%d, truncating",
                len(clean),
                self.max_chars,
            )
            clean = clean[: self.max_chars]

        logger.debug(
            "PiperEngine: synthesizing text (%d chars, sr=%d, speaker_id=%s)",
            len(clean),
            self._sample_rate,
            str(self.speaker_id),
        )

        chunks: list[np.ndarray] = []

        try:
            # synthesize_stream_raw yields successive byte chunks of int16 PCM.
            # We simply accumulate them into one array.
            for audio_bytes in self._voice.synthesize_stream_raw(
                clean,
                speaker_id=self.speaker_id,
            ):
                if not audio_bytes:
                    continue
                chunk = np.frombuffer(audio_bytes, dtype=np.int16)
                if chunk.size > 0:
                    chunks.append(chunk)

        except Exception as exc:  # noqa: BLE001
            logger.error("PiperEngine.synthesize_to_pcm16() failed: %s", exc)
            return np.zeros(0, dtype=np.int16), self._sample_rate

        if not chunks:
            logger.warning(
                "PiperEngine.synthesize_to_pcm16(): no audio chunks produced"
            )
            return np.zeros(0, dtype=np.int16), self._sample_rate

        audio = np.concatenate(chunks, axis=0)
        logger.debug(
            "PiperEngine: produced %d samples (int16) at %d Hz",
            audio.size,
            self._sample_rate,
        )
        return audio, self._sample_rate

    def synthesize_to_float32(self, text: str) -> Tuple[np.ndarray, int]:
        """
        Synthesize text to mono float32 PCM in [-1.0, 1.0] for playback.

        This is the format expected by our audio_io.play_pcm() helper.

        Parameters
        ----------
        text : str
            Input text to speak.

        Returns
        -------
        audio : np.ndarray
            1D float32 array with values in [-1.0, 1.0].
            Empty array if synthesis failed or text was empty.
        sample_rate : int
            Model's native sample rate in Hz.
        """
        pcm16, sr = self.synthesize_to_pcm16(text)
        if pcm16.size == 0:
            return np.zeros(0, dtype=np.float32), sr

        audio_f32 = pcm16.astype(np.float32) / 32768.0
        # Guard against any weird clipping
        audio_f32 = np.clip(audio_f32, -1.0, 1.0)

        logger.debug(
            "PiperEngine: converted int16 → float32, %d samples at %d Hz",
            audio_f32.size,
            sr,
        )
        return audio_f32, sr
