#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — PiperEngine wrapper (AudioChunk-aware)

Thin wrapper around the piper-tts Python API so that the TTS ROS node
doesn't have to deal with model loading, JSON configs, or raw PCM bytes.

We use:
  - piper.voice.PiperVoice
  - piper.voice.AudioChunk  (newer piper-tts versions)
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
from typing import Union, Iterable, Any, Dict, Optional, List

import numpy as np
from piper.voice import PiperVoice

try:
    # Newer piper-tts exposes AudioChunk (streaming API)
    from piper.voice import AudioChunk  # type: ignore[attr-defined]
except Exception:  # noqa: BLE001
    AudioChunk = None  # type: ignore[assignment]

logger = logging.getLogger(__name__)

PathLike = Union[str, Path]


class PiperEngine:
    """
    Small wrapper around PiperVoice for a single voice/model.

    Responsibilities:
    - Load ONNX model + JSON config from disk.
    - Expose a simple synthesize_to_pcm16() method that returns
      a mono int16 NumPy array (PCM).
    - Track model sample_rate via config or AudioChunk metadata.
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

        # Cache sample rate from config if available; may be overridden
        audio_cfg = self._config.get("audio", {}) if isinstance(self._config, dict) else {}
        self.sample_rate: int = int(audio_cfg.get("sample_rate", 22050))

        logger.info(
            "PiperEngine: model loaded successfully "
            f"(sample_rate={self.sample_rate} Hz)"
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _collect_audio_bytes_from_result(self, result: Any) -> bytes:
        """
        Normalize the output of PiperVoice.synthesize(...) into a single
        bytes object containing raw int16 PCM.

        Supports:
        - bytes / bytearray
        - Iterable[bytes/bytearray]
        - Iterable[AudioChunk] (new piper-tts streaming API)
        """
        # Direct bytes
        if isinstance(result, (bytes, bytearray)):
            return bytes(result)

        # Stream / generator
        if isinstance(result, Iterable):
            chunks: List[bytes] = []
            detected_rate: Optional[int] = None

            for chunk in result:
                # Case 1: raw bytes chunk
                if isinstance(chunk, (bytes, bytearray)):
                    chunks.append(bytes(chunk))
                    continue

                # Case 2: AudioChunk from newer piper-tts
                if AudioChunk is not None and isinstance(chunk, AudioChunk):  # type: ignore[arg-type]
                    # AudioChunk typically has:
                    #   chunk.audio -> bytes
                    #   chunk.sample_rate -> int
                    #   chunk.num_channels -> int
                    audio_bytes = bytes(chunk.audio)
                    chunks.append(audio_bytes)

                    # Track sample rate from the stream if available
                    try:
                        sr = int(getattr(chunk, "sample_rate", 0))
                        if sr > 0:
                            if detected_rate is None:
                                detected_rate = sr
                            elif detected_rate != sr:
                                logger.warning(
                                    "PiperEngine: AudioChunk sample_rate changed "
                                    f"from {detected_rate} to {sr} mid-stream"
                                )
                    except Exception:  # noqa: BLE001
                        pass
                    continue

                # Fallback: unknown chunk type
                logger.warning(
                    "PiperEngine: unexpected synthesize() chunk type: %r",
                    type(chunk),
                )

            audio_bytes = b"".join(chunks)

            # If we discovered a sample_rate from the chunks, prefer that
            if detected_rate is not None and detected_rate != self.sample_rate:
                logger.info(
                    "PiperEngine: overriding config sample_rate %d -> %d "
                    "based on AudioChunk metadata",
                    self.sample_rate,
                    detected_rate,
                )
                self.sample_rate = detected_rate

            return audio_bytes

        # Unexpected top-level type
        logger.error(
            "PiperEngine: unexpected synthesize() return type: %r",
            type(result),
        )
        return b""

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
        # AudioChunk objects (new API) or raw PCM bytes (older API).
        try:
            result = self._voice.synthesize(clean)
        except Exception as exc:  # noqa: BLE001
            logger.error("PiperEngine.synthesize_to_pcm16() error: %s", exc)
            return np.zeros(0, dtype=np.int16)

        audio_bytes = self._collect_audio_bytes_from_result(result)

        if not audio_bytes:
            logger.warning("PiperEngine: synthesize() produced no audio bytes")
            return np.zeros(0, dtype=np.int16)

        # Convert bytes -> int16 NumPy array (little endian, signed 16-bit)
        audio_i16 = np.frombuffer(audio_bytes, dtype="<i2")
        if audio_i16.ndim != 1:
            audio_i16 = audio_i16.flatten()

        return audio_i16.copy()  # copy so we own the memory buffer
