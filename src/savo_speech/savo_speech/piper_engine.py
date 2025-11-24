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

import json
import logging
from pathlib import Path
from typing import Union, Iterable, Any, Dict

import numpy as np
from piper.voice import PiperVoice  # provided by the piper-tts package

logger = logging.getLogger(__name__)

PathLike = Union[str, Path]


class PiperEngine:
    """
    Small wrapper around PiperVoice for a single voice/model.

    Responsibilities:
    - Load ONNX model + JSON config from disk.
    - Expose a simple synthesize_to_pcm16() method that returns
      a mono int16 NumPy array (PCM).
    - Optionally expose model metadata such as sample_rate.
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
            "PiperEngine: loading model:\n"
            f"  model  = {self.model_path}\n"
            f"  config = {self.config_path}"
        )

        # Load binary model
        with self.model_path.open("rb") as f:
            model_bytes = f.read()

        # Load JSON config
        with self.config_path.open("r", encoding="utf-8") as f:
            self._config: Dict[str, Any] = json.load(f)

        # Create PiperVoice instance
        self._voice: PiperVoice = PiperVoice.load(model_bytes, self._config)

        # Cache sample rate from config if available
        audio_cfg = self._config.get("audio", {}) if isinstance(self._config, dict) else {}
        self.sample_rate: int = int(audio_cfg.get("sample_rate", 22050))

        logger.info(
            "PiperEngine: model loaded successfully "
            f"(sample_rate={self.sample_rate} Hz)"
        )

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

        # If result is already bytes, wrap it; if iterable, concatenate.
        audio_bytes: bytes
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
