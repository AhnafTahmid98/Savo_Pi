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
from typing import Union, Iterable, Any, Dict, Optional, List

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

    @staticmethod
    def _float_to_int16(arr: np.ndarray) -> np.ndarray:
        """Convert float audio [-1, 1] or larger to int16 PCM safely."""
        if arr.size == 0:
            return np.zeros(0, dtype=np.int16)
        # Clip to [-1, 1] then scale
        arr_clipped = np.clip(arr, -1.0, 1.0)
        return (arr_clipped * 32767.0).astype(np.int16)

    @staticmethod
    def _bytes_to_int16(buf: bytes) -> np.ndarray:
        """Convert little-endian signed 16-bit PCM bytes to int16 NumPy array."""
        if not buf:
            return np.zeros(0, dtype=np.int16)
        audio_i16 = np.frombuffer(buf, dtype="<i2")
        if audio_i16.ndim != 1:
            audio_i16 = audio_i16.flatten()
        return audio_i16.copy()

    def _chunk_to_int16(self, chunk: Any, idx: int = 0) -> np.ndarray:
        """
        Try to extract int16 audio data from an AudioChunk-like object.

        We do NOT assume attribute names; instead we:
        - Look for attributes containing 'audio' or 'pcm'
        - Accept bytes/bytearray/memoryview or NumPy arrays
        """
        # If the chunk is already bytes, just convert
        if isinstance(chunk, (bytes, bytearray, memoryview)):
            return self._bytes_to_int16(bytes(chunk))

        # Try to get attributes that might hold audio
        try:
            attr_names: List[str] = [
                name
                for name in dir(chunk)
                if not name.startswith("_")
                and any(key in name.lower() for key in ("audio", "pcm", "samples", "data"))
            ]
        except Exception:  # noqa: BLE001
            attr_names = []

        for name in attr_names:
            try:
                value = getattr(chunk, name)
            except Exception:  # noqa: BLE001
                continue

            # bytes-like
            if isinstance(value, (bytes, bytearray, memoryview)):
                return self._bytes_to_int16(bytes(value))

            # NumPy array
            if isinstance(value, np.ndarray):
                arr = value
                if arr.ndim > 1:
                    arr = arr.flatten()

                if arr.dtype == np.int16:
                    return arr.astype(np.int16, copy=True)

                if np.issubdtype(arr.dtype, np.floating):
                    return self._float_to_int16(arr.astype(np.float32))

                # If it's some other numeric type, cast to float32 then convert
                if np.issubdtype(arr.dtype, np.number):
                    return self._float_to_int16(arr.astype(np.float32))

            # Python list/tuple of numbers
            if isinstance(value, (list, tuple)):
                try:
                    arr = np.asarray(value)
                    if arr.ndim > 1:
                        arr = arr.flatten()
                    if np.issubdtype(arr.dtype, np.floating):
                        return self._float_to_int16(arr.astype(np.float32))
                    if np.issubdtype(arr.dtype, np.integer):
                        return arr.astype(np.int16, copy=True)
                except Exception:  # noqa: BLE001
                    continue

        # Some versions use NamedTuple-like interface: _asdict()
        try:
            asdict = getattr(chunk, "_asdict", None)
            if callable(asdict):
                mapping = asdict()
                if isinstance(mapping, dict):
                    for key, value in mapping.items():
                        if not isinstance(key, str):
                            continue
                        if not any(k in key.lower() for k in ("audio", "pcm", "samples", "data")):
                            continue

                        if isinstance(value, (bytes, bytearray, memoryview)):
                            return self._bytes_to_int16(bytes(value))
                        if isinstance(value, np.ndarray):
                            arr = value
                            if arr.ndim > 1:
                                arr = arr.flatten()
                            if np.issubdtype(arr.dtype, np.floating):
                                return self._float_to_int16(arr.astype(np.float32))
                            if np.issubdtype(arr.dtype, np.integer):
                                return arr.astype(np.int16, copy=True)
        except Exception:  # noqa: BLE001
            pass

        # If we reach here, we failed to extract audio from this chunk
        logger.warning(
            "PiperEngine: could not extract audio from chunk #%d of type %r; "
            "candidate attrs=%r",
            idx,
            type(chunk),
            attr_names,
        )
        return np.zeros(0, dtype=np.int16)

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

        try:
            result = self._voice.synthesize(clean)
        except Exception as exc:  # noqa: BLE001
            logger.error("PiperEngine.synthesize_to_pcm16() error in synthesize(): %s", exc)
            return np.zeros(0, dtype=np.int16)

        # Case 1: direct bytes/bytearray
        if isinstance(result, (bytes, bytearray, memoryview)):
            return self._bytes_to_int16(bytes(result))

        # Case 2: iterable of chunks (bytes or AudioChunk-like objects)
        if isinstance(result, Iterable):
            chunks: List[np.ndarray] = []
            for idx, chunk in enumerate(result):
                # bytes-like
                if isinstance(chunk, (bytes, bytearray, memoryview)):
                    arr = self._bytes_to_int16(bytes(chunk))
                    if arr.size:
                        chunks.append(arr)
                    continue

                # AudioChunk or other object
                arr = self._chunk_to_int16(chunk, idx=idx)
                if arr.size:
                    chunks.append(arr)

            if not chunks:
                logger.warning("PiperEngine: synthesize() iterable produced no usable audio")
                return np.zeros(0, dtype=np.int16)

            audio_i16 = np.concatenate(chunks, axis=0)
            return audio_i16.astype(np.int16, copy=False)

        # Unknown return type
        logger.error(
            "PiperEngine: unexpected synthesize() return type: %r",
            type(result),
        )
        return np.zeros(0, dtype=np.int16)
