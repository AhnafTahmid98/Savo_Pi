#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Piper TTS engine wrapper.

This module hides the Piper/PiperVoice internals behind a small, stable API
so that the ROS 2 TTS node (tts_node.py) can stay focused on ROS logic.

Design goals:
- Simple API: initialize once, then call synthesize_to_pcm16(text).
- Robust to different Piper versions (with/without synthesize_stream_raw).
- Return raw int16 PCM + sample rate so the node can play or post-process.
"""

from __future__ import annotations

import logging
from typing import Iterable, List, Optional, Tuple

import numpy as np

from piper.voice import PiperVoice  # provided by the piper-tts package

logger = logging.getLogger(__name__)


class PiperEngine:
    """
    Thin wrapper around PiperVoice.

    This class is designed to be created once per voice (e.g. "male" / "female")
    and reused for many utterances.

    Typical usage from tts_node.py:

        engine = PiperEngine(
            model_path="/home/savo/Savo_Pi/models/piper/en_US-ryan-high.onnx",
            config_path="/home/savo/Savo_Pi/models/piper/en_US-ryan-high.onnx.json",
            sample_rate=22050,
            gain=1.0,
            length_scale=0.95,
            noise_scale=0.667,
            noise_w=0.8,
        )

        pcm_i16, sr = engine.synthesize_to_pcm16("Hello, I am Robot Savo.")
    """

    def __init__(
        self,
        *,
        model_path: str,
        config_path: Optional[str] = None,
        sample_rate: Optional[int] = None,
        gain: float = 1.0,
        length_scale: float = 1.0,
        noise_scale: float = 0.667,
        noise_w: float = 0.8,
    ) -> None:
        """
        Initialize a PiperEngine from a model + config.

        Parameters
        ----------
        model_path : str
            Full path to the .onnx model file.
        config_path : Optional[str]
            Full path to the .onnx.json config file.
            If None, PiperVoice.load() will try "<model_path>.json".
        sample_rate : Optional[int]
            Desired output sample rate. If None, use voice.config.sample_rate.
        gain : float
            Linear gain applied to the int16 PCM (e.g. 1.0, 0.8, 1.2).
        length_scale : float
            Controls speaking rate (lower = slower, higher = faster).
        noise_scale : float
            Controls prosody / variation.
        noise_w : float
            Controls noise on the waveform.
        """
        self.model_path = model_path
        self.config_path = config_path
        self.gain = float(gain)
        self.length_scale = float(length_scale)
        self.noise_scale = float(noise_scale)
        self.noise_w = float(noise_w)

        logger.info(
            "PiperEngine: loading model\n"
            "  model_path  = %s\n"
            "  config_path = %s\n"
            "  gain        = %.3f\n"
            "  length_scale= %.3f\n"
            "  noise_scale = %.3f\n"
            "  noise_w     = %.3f",
            self.model_path,
            self.config_path,
            self.gain,
            self.length_scale,
            self.noise_scale,
            self.noise_w,
        )

        # Use CPU (Pi 5). If later you have a GPU on PC, you could flip use_cuda=True.
        self._voice: PiperVoice = PiperVoice.load(
            model_path=self.model_path,
            config_path=self.config_path,
            use_cuda=False,
        )

        # Decide final sample rate: param overrides config, else use config.
        if sample_rate is not None and sample_rate > 0:
            self.sample_rate: int = int(sample_rate)
        else:
            # Piper config knows its own training sample rate (e.g. 22050)
            self.sample_rate = int(getattr(self._voice.config, "sample_rate", 22050))

        # Detect which API surface is available on this PiperVoice implementation.
        self._has_stream_raw: bool = hasattr(self._voice, "synthesize_stream_raw")
        self._has_ids_to_raw: bool = hasattr(self._voice, "synthesize_ids_to_raw")
        self._has_phonemize: bool = hasattr(self._voice, "phonemize")
        self._has_phonemes_to_ids: bool = hasattr(self._voice, "phonemes_to_ids")

        logger.info(
            "PiperEngine: voice loaded (sample_rate=%d Hz, stream_raw=%s, ids_to_raw=%s)",
            self.sample_rate,
            self._has_stream_raw,
            self._has_ids_to_raw,
        )

    # ------------------------------------------------------------------ #
    # Public API                                                         #
    # ------------------------------------------------------------------ #

    def synthesize_to_pcm16(self, text: str) -> Tuple[np.ndarray, int]:
        """
        Synthesize speech for `text` and return (pcm_int16, sample_rate).

        Returns
        -------
        pcm : np.ndarray
            1D numpy array, dtype=int16. Empty array if synthesis fails.
        sample_rate : int
            Sample rate in Hz (usually 22050).
        """
        if not text:
            logger.debug("PiperEngine.synthesize_to_pcm16() called with empty text")
            return np.zeros((0,), dtype=np.int16), self.sample_rate

        text_clean = text.strip()
        if not text_clean:
            logger.debug("PiperEngine.synthesize_to_pcm16(): text only whitespace")
            return np.zeros((0,), dtype=np.int16), self.sample_rate

        try:
            if self._has_stream_raw:
                pcm = self._synthesize_via_stream_raw(text_clean)
            elif self._has_ids_to_raw and self._has_phonemize and self._has_phonemes_to_ids:
                pcm = self._synthesize_via_ids_path(text_clean)
            else:
                logger.error(
                    "PiperEngine: PiperVoice lacks both synthesize_stream_raw() "
                    "and synthesize_ids_to_raw()/phonemize(). Cannot synthesize."
                )
                return np.zeros((0,), dtype=np.int16), self.sample_rate

        except Exception as exc:  # noqa: BLE001
            logger.error("PiperEngine.synthesize_to_pcm16() failed: %s", exc)
            return np.zeros((0,), dtype=np.int16), self.sample_rate

        if pcm.size == 0:
            return pcm, self.sample_rate

        # Apply gain and clip to int16 range.
        if self.gain != 1.0:
            pcm = np.clip(pcm.astype(np.float32) * self.gain, -32768, 32767).astype(
                np.int16
            )

        return pcm, self.sample_rate

    # ------------------------------------------------------------------ #
    # Internal helpers                                                   #
    # ------------------------------------------------------------------ #

    def _synthesize_via_stream_raw(self, text: str) -> np.ndarray:
        """
        Use PiperVoice.synthesize_stream_raw() if available.

        This is the preferred path on modern Piper versions.
        """
        assert self._has_stream_raw, "synthesize_stream_raw must exist here"

        chunks: List[bytes] = []

        logger.debug(
            "PiperEngine._synthesize_via_stream_raw(): text='%s'",
            text,
        )

        # sentence_silence = 0.0 → we control pauses at higher level if needed.
        for audio_bytes in self._voice.synthesize_stream_raw(
            text,
            speaker_id=None,
            length_scale=self.length_scale,
            noise_scale=self.noise_scale,
            noise_w=self.noise_w,
            sentence_silence=0.0,
        ):
            if audio_bytes:
                chunks.append(audio_bytes)

        if not chunks:
            logger.warning(
                "PiperEngine._synthesize_via_stream_raw(): no audio chunks produced"
            )
            return np.zeros((0,), dtype=np.int16)

        raw = b"".join(chunks)
        pcm = np.frombuffer(raw, dtype=np.int16)

        logger.debug(
            "PiperEngine._synthesize_via_stream_raw(): produced %d samples", pcm.size
        )
        return pcm

    def _synthesize_via_ids_path(self, text: str) -> np.ndarray:
        """
        Fallback path for older Piper implementations without synthesize_stream_raw().

        We manually:
          1) phonemize text → per-sentence phoneme lists
          2) convert phonemes → ids
          3) call synthesize_ids_to_raw()
        """
        assert self._has_ids_to_raw and self._has_phonemize and self._has_phonemes_to_ids

        logger.debug("PiperEngine._synthesize_via_ids_path(): text='%s'", text)

        # sentence_phonemes: List[List[str]]
        sentence_phonemes: Iterable[List[str]] = self._voice.phonemize(text)
        raw_chunks: List[bytes] = []

        for phonemes in sentence_phonemes:
            phoneme_ids = self._voice.phonemes_to_ids(phonemes)
            raw_bytes = self._voice.synthesize_ids_to_raw(
                phoneme_ids,
                speaker_id=None,
                length_scale=self.length_scale,
                noise_scale=self.noise_scale,
                noise_w=self.noise_w,
            )
            if raw_bytes:
                raw_chunks.append(raw_bytes)

        if not raw_chunks:
            logger.warning(
                "PiperEngine._synthesize_via_ids_path(): no audio produced for text"
            )
            return np.zeros((0,), dtype=np.int16)

        raw = b"".join(raw_chunks)
        pcm = np.frombuffer(raw, dtype=np.int16)

        logger.debug(
            "PiperEngine._synthesize_via_ids_path(): produced %d samples", pcm.size
        )
        return pcm
