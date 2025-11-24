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

    audio_i16, sr = engine.synthesize_to_pcm16("Hello, I am Robot Savo.")
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Union, Iterable, Any, Dict, Optional, Tuple

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
      a mono int16 NumPy array (PCM) and the model sample rate.
    - Optionally use default length_scale / noise params from config.
    """

    def __init__(
        self,
        model_path: Union[PathLike, Dict[str, PathLike]],
        config_path: Optional[PathLike] = None,
        *,
        default_length_scale: float = 1.0,
        default_noise_scale: float = 0.667,
        default_noise_w: float = 0.8,
    ) -> None:
        """
        Load a Piper ONNX model + JSON config from disk.

        Parameters
        ----------
        model_path : PathLike or dict
            - If str/Path: path to .onnx file, e.g. "en_US-ryan-high.onnx"
              (then config_path must also be provided).
            - If dict: must contain keys "model_path"/"config_path" OR
              "model"/"config", each pointing to a str/Path.
        config_path : Optional[PathLike]
            Path to .onnx.json file, e.g. "en_US-ryan-high.onnx.json".
            Ignored if model_path is given as a dict.
        default_length_scale, default_noise_scale, default_noise_w :
            Default prosody settings used when calling synthesize().
        """
        # Normalize input into two concrete paths
        model_p, config_p = self._normalize_paths(model_path, config_path)

        self.model_path = model_p
        self.config_path = config_p

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

        # Cache sample rate and default prosody from config if available
        audio_cfg = self._config.get("audio", {}) if isinstance(self._config, dict) else {}
        self.sample_rate: int = int(audio_cfg.get("sample_rate", 22050))

        self.length_scale_default: float = float(
            self._config.get("length_scale", default_length_scale)
        )
        self.noise_scale_default: float = float(
            self._config.get("noise_scale", default_noise_scale)
        )
        self.noise_w_default: float = float(
            self._config.get("noise_w", default_noise_w)
        )

        logger.info(
            "PiperEngine: model loaded successfully "
            f"(sample_rate={self.sample_rate} Hz, "
            f"length_scale={self.length_scale_default:.3f}, "
            f"noise_scale={self.noise_scale_default:.3f}, "
            f"noise_w={self.noise_w_default:.3f})"
        )

    # ------------------------------------------------------------------
    # Path normalization helper
    # ------------------------------------------------------------------

    @staticmethod
    def _normalize_paths(
        model_path: Union[PathLike, Dict[str, PathLike]],
        config_path: Optional[PathLike],
    ) -> Tuple[Path, Path]:
        """
        Normalize model/config inputs into Path objects.

        Supports:
        - PiperEngine("foo.onnx", "foo.onnx.json")
        - PiperEngine({"model_path": "foo.onnx", "config_path": "foo.onnx.json"})
        - PiperEngine({"model": "foo.onnx", "config": "foo.onnx.json"})
        """
        if isinstance(model_path, dict):
            # Dict style
            mp = model_path.get("model_path") or model_path.get("model")
            cp = model_path.get("config_path") or model_path.get("config")

            if mp is None or cp is None:
                raise ValueError(
                    "PiperEngine: model_path dict must contain "
                    "'model_path'/'config_path' or 'model'/'config' keys"
                )

            return Path(mp), Path(cp)

        # Positional style: model_path + config_path
        if config_path is None:
            raise ValueError(
                "PiperEngine: config_path must be provided when model_path is not a dict"
            )

        return Path(model_path), Path(config_path)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def synthesize_to_pcm16(
        self,
        text: str,
        *,
        length_scale: Optional[float] = None,
        noise_scale: Optional[float] = None,
        noise_w: Optional[float] = None,
    ) -> Tuple[np.ndarray, int]:
        """
        Synthesize speech from text to a mono int16 NumPy array.

        Parameters
        ----------
        text : str
            Input text (one utterance).
        length_scale, noise_scale, noise_w : optional
            If provided, override the default prosody parameters for this call.

        Returns
        -------
        audio_i16 : np.ndarray
            1D int16 array: PCM audio at the model's native sample rate.
        sample_rate : int
            The model sample rate (Hz), usually from the JSON config.
        """
        clean = (text or "").strip()
        if not clean:
            logger.debug("PiperEngine.synthesize_to_pcm16() called with empty text")
            return np.zeros(0, dtype=np.int16), self.sample_rate

        ls = float(self.length_scale_default if length_scale is None else length_scale)
        ns = float(self.noise_scale_default if noise_scale is None else noise_scale)
        nw = float(self.noise_w_default if noise_w is None else noise_w)

        try:
            # PiperVoice.synthesize returns an iterator of raw PCM byte chunks.
            result = self._voice.synthesize(
                clean,
                length_scale=ls,
                noise_scale=ns,
                noise_w=nw,
            )
        except Exception as exc:  # noqa: BLE001
            logger.error("PiperEngine.synthesize_to_pcm16() error: %s", exc)
            return np.zeros(0, dtype=np.int16), self.sample_rate

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
            return np.zeros(0, dtype=np.int16), self.sample_rate

        if not audio_bytes:
            logger.warning("PiperEngine: synthesize() produced no audio bytes")
            return np.zeros(0, dtype=np.int16), self.sample_rate

        # Convert bytes -> int16 NumPy array (little endian, signed 16-bit)
        audio_i16 = np.frombuffer(audio_bytes, dtype="<i2")
        if audio_i16.ndim != 1:
            audio_i16 = audio_i16.flatten()

        # Return a copy so the buffer is owned by NumPy (safe to modify)
        return audio_i16.copy(), self.sample_rate
