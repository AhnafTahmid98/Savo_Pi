#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — PiperEngine wrapper

Thin wrapper around the piper-tts Python API so that the TTS ROS node
doesn't have to deal with model loading, JSON configs, or raw PCM bytes.

We use:
  - piper.voice.PiperVoice
  - local .onnx + .onnx.json files

Typical usage (see tts_node.py):

    engine = PiperEngine(
        model_path="/home/savo/Savo_Pi/models/piper/en_US-ryan-high.onnx",
        config_path="/home/savo/Savo_Pi/models/piper/en_US-ryan-high.onnx.json",
    )

    audio_i16 = engine.synthesize_to_pcm16("Hello, I am Robot Savo.")
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Union, Iterable, Any, Dict, Optional

import numpy as np
from piper.voice import PiperVoice  # provided by the piper-tts package

logger = logging.getLogger(__name__)

PathLike = Union[str, Path]


class PiperEngine:
    """
    Small wrapper around PiperVoice for a single voice/model.

    Responsibilities:
    - Load ONNX model + JSON config using PiperVoice.load().
    - Expose a simple synthesize_to_pcm16() method that returns
      a mono int16 NumPy array (PCM).
    - Expose model sample_rate if available.
    """

    def __init__(self, model_path: PathLike, config_path: PathLike) -> None:
        """
        Load a Piper ONNX model + JSON config from disk.

        Parameters
        ----------
        model_path : PathLike
            Path to .onnx file, e.g. en_US-ryan-high.onnx
        config_path : PathLike
            Path to .onnx.json file, e.g. en_US-ryan-high.onnx.json
        """
        self.model_path = Path(model_path)
        self.config_path = Path(config_path)

        if not self.model_path.is_file():
            raise FileNotFoundError(f"PiperEngine: model file not found: {self.model_path}")
        if not self.config_path.is_file():
            raise FileNotFoundError(f"PiperEngine: config file not found: {self.config_path}")

        logger.info(
            "PiperEngine: loading model via PiperVoice.load():\n"
            f"  model  = {self.model_path}\n"
            f"  config = {self.config_path}"
        )

        # IMPORTANT: use PiperVoice.load(model_path, config_path) exactly as
        # the library expects. Do NOT pass a dict or raw bytes here.
        self._voice: PiperVoice = PiperVoice.load(
            str(self.model_path),
            str(self.config_path),
        )

        # Try to get sample_rate from the voice config if exposed.
        self.sample_rate: int = 22050  # sensible default for English voices
        sr = self._extract_sample_rate_from_voice(self._voice)
        if sr is not None:
            self.sample_rate = sr

        logger.info(
            "PiperEngine: model loaded successfully "
            f"(sample_rate={self.sample_rate} Hz)"
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _extract_sample_rate_from_voice(voice: PiperVoice) -> Optional[int]:
        """
        Try to extract sample_rate from PiperVoice.config if available.

        We keep this defensive so changes in piper-tts internals
        won't crash Robot Savo.
        """
        cfg: Any = getattr(voice, "config", None)

        if isinstance(cfg, dict):
            audio_cfg = cfg.get("audio", {})
            if isinstance(audio_cfg, dict):
                sr = audio_cfg.get("sample_rate")
                if isinstance(sr, (int, float)):
                    return int(sr)

        # Fallback: some versions might expose a 'sample_rate' attribute
        raw_sr = getattr(voice, "sample_rate", None)
        if isinstance(raw_sr, (int, float)):
            return int(raw_sr)

        return None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def synthesize_to_pcm16(self, text: str) -> np.ndarray:
        """
        Synthesize speech from text to a mono int16 NumPy array.

        Parameters
        ----------
        text : str
            Input text (one utterance).

        Returns
        -------
        audio_i16 : np.ndarray
            1D int16 array: PCM audio at the model's native sample rate
            (self.sample_rate, typically 22050 Hz for English voices).
            Empty array if synthesis fails or text is empty.
        """
        clean = (text or "").strip()
        if not clean:
            logger.debug("PiperEngine.synthesize_to_pcm16() called with empty text")
            return np.zeros(0, dtype=np.int16)

        # PiperVoice.synthesize() normally returns an iterator/generator of
        # raw PCM byte chunks. We support both iterator and plain-bytes cases.
        try:
            result = self._voice.synthesize(clean)
        except Exception as exc:  # noqa: BLE001
            logger.error("PiperEngine.synthesize_to_pcm16() error: %s", exc)
            return np.zeros(0, dtype=np.int16)

        audio_bytes: bytes

        # If result is already bytes, wrap it; if iterable, concatenate.
        if isinstance(result, (bytes, bytearray)):
            audio_bytes = bytes(result)
        elif isinstance(result, Iterable):
            chunks = []
            for chunk in result:
                if isinstance(chunk, (bytes, bytearray)):
                    chunks.append(bytes(chunk))
                else:
                    logger.warning(
                        "PiperEngine: unexpected synthesize() chunk type: %r",
                        type(chunk),
                    )
            audio_bytes = b"".join(chunks)
        else:
            logger.error(
                "PiperEngine: unexpected synthesize() return type: %r",
                type(result),
            )
            return np.zeros(0, dtype=np.int16)

        if not audio_bytes:
            logger.warning("PiperEngine: synthesize() produced no audio bytes")
            return np.zeros(0, dtype=np.int16)

        # Convert bytes -> int16 NumPy array (little endian, signed 16-bit)
        audio_i16 = np.frombuffer(audio_bytes, dtype="<i2")
        if audio_i16.ndim != 1:
            audio_i16 = audio_i16.flatten()

        return audio_i16.copy()  # copy so we own the memory buffer
