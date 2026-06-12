#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Piper model wrapper that returns mono int16 PCM."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Union, Iterable, Any, Dict, Optional, List

import numpy as np
from piper.voice import PiperVoice  # provided by the piper-tts package

logger = logging.getLogger(__name__)

PathLike = Union[str, Path]


class PiperEngine:
    """Single-voice Piper wrapper."""

    def __init__(self, model_path: PathLike, config_path: PathLike) -> None:
        """Load a Piper ONNX model and config."""
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

        # PiperVoice.load expects paths here, not parsed config data.
        self._voice: PiperVoice = PiperVoice.load(
            str(self.model_path),
            str(self.config_path),
        )

        # piper-tts exposes sample_rate differently by version.
        self.sample_rate: int = 22050  # common English voice default
        sr = self._extract_sample_rate_from_voice(self._voice)
        if sr is not None:
            self.sample_rate = sr

        logger.info(
            "PiperEngine: model loaded successfully "
            f"(sample_rate={self.sample_rate} Hz)"
        )
    @staticmethod
    def _extract_sample_rate_from_voice(voice: PiperVoice) -> Optional[int]:
        """Read sample_rate defensively across piper-tts versions."""
        cfg: Any = getattr(voice, "config", None)

        if isinstance(cfg, dict):
            audio_cfg = cfg.get("audio", {})
            if isinstance(audio_cfg, dict):
                sr = audio_cfg.get("sample_rate")
                if isinstance(sr, (int, float)):
                    return int(sr)

        raw_sr = getattr(voice, "sample_rate", None)
        if isinstance(raw_sr, (int, float)):
            return int(raw_sr)

        return None

    @staticmethod
    def _float_to_int16(arr: np.ndarray) -> np.ndarray:
        """Convert float audio [-1, 1] or larger to int16 PCM safely."""
        if arr.size == 0:
            return np.zeros(0, dtype=np.int16)
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
        """Extract int16 audio from Piper chunk variants across piper-tts versions."""
        if isinstance(chunk, (bytes, bytearray, memoryview)):
            return self._bytes_to_int16(bytes(chunk))

        # Attribute names vary between piper-tts releases.
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

            if isinstance(value, (bytes, bytearray, memoryview)):
                return self._bytes_to_int16(bytes(value))

            if isinstance(value, np.ndarray):
                arr = value
                if arr.ndim > 1:
                    arr = arr.flatten()

                if arr.dtype == np.int16:
                    return arr.astype(np.int16, copy=True)

                if np.issubdtype(arr.dtype, np.floating):
                    return self._float_to_int16(arr.astype(np.float32))

                if np.issubdtype(arr.dtype, np.number):
                    return self._float_to_int16(arr.astype(np.float32))

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

        # Some versions expose chunks as NamedTuple-like objects.
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

        logger.warning(
            "PiperEngine: could not extract audio from chunk #%d of type %r; "
            "candidate attrs=%r",
            idx,
            type(chunk),
            attr_names,
        )
        return np.zeros(0, dtype=np.int16)

    def synthesize_to_pcm16(self, text: str) -> np.ndarray:
        """Synthesize one utterance to mono int16 PCM at the model sample rate."""
        clean = (text or "").strip()
        if not clean:
            logger.debug("PiperEngine.synthesize_to_pcm16() called with empty text")
            return np.zeros(0, dtype=np.int16)

        try:
            result = self._voice.synthesize(clean)
        except Exception as exc:  # noqa: BLE001
            logger.error("PiperEngine.synthesize_to_pcm16() error in synthesize(): %s", exc)
            return np.zeros(0, dtype=np.int16)

        if isinstance(result, (bytes, bytearray, memoryview)):
            return self._bytes_to_int16(bytes(result))

        if isinstance(result, Iterable):
            chunks: List[np.ndarray] = []
            for idx, chunk in enumerate(result):
                if isinstance(chunk, (bytes, bytearray, memoryview)):
                    arr = self._bytes_to_int16(bytes(chunk))
                    if arr.size:
                        chunks.append(arr)
                    continue

                arr = self._chunk_to_int16(chunk, idx=idx)
                if arr.size:
                    chunks.append(arr)

            if not chunks:
                logger.warning("PiperEngine: synthesize() iterable produced no usable audio")
                return np.zeros(0, dtype=np.int16)

            audio_i16 = np.concatenate(chunks, axis=0)
            return audio_i16.astype(np.int16, copy=False)

        logger.error(
            "PiperEngine: unexpected synthesize() return type: %r",
            type(result),
        )
        return np.zeros(0, dtype=np.int16)
