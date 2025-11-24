#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Piper TTS engine wrapper.

This module wraps the piper-tts Python API so that ROS 2 nodes (tts_node)
can synthesize speech from text using local .onnx models.

Design goals:
- Hide Piper internals behind a small, stable API (PiperEngine).
- Support multiple "profiles" (male / female) by creating separate engines.
- Return numpy int16 PCM at a known sample rate for playback with sounddevice.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

import numpy as np
from piper.voice import PiperVoice  # provided by piper-tts

logger = logging.getLogger(__name__)


class PiperEngine:
    """
    Thin wrapper around piper.voice.PiperVoice.

    Typical usage:

        engine = PiperEngine(
            model_path=Path("/home/savo/Savo_Pi/models/piper/en_US-ryan-high.onnx"),
            config_path=Path("/home/savo/Savo_Pi/models/piper/en_US-ryan-high.onnx.json"),
            sample_rate=22050,
            length_scale=0.95,
            noise_scale=0.667,
            noise_w=0.8,
            gain=1.0,
        )

        audio_int16 = engine.synthesize_to_pcm16("Hello, I am Robot Savo.")
    """

    def __init__(
        self,
        model_path: Path,
        config_path: Path,
        *,
        sample_rate: int = 22050,
        length_scale: float = 1.0,
        noise_scale: float = 0.667,
        noise_w: float = 0.8,
        gain: float = 1.0,
        speaker_id: Optional[int] = None,
    ) -> None:
        """
        Initialize the Piper voice.

        Parameters
        ----------
        model_path : Path
            Path to the .onnx voice model file.
        config_path : Path
            Path to the .onnx.json voice config file.
        sample_rate : int
            Target playback sample rate in Hz (usually 22050).
        length_scale : float
            Speech rate. >1 = faster, <1 = slower.
        noise_scale : float
            Controls prosody / variation.
        noise_w : float
            Another noise parameter (see Piper docs).
        gain : float
            Linear gain applied to synthesized audio.
        speaker_id : Optional[int]
            For multi-speaker models. Our voices are single-speaker, so this
            is usually None, but we accept it to keep the API future-proof.
        """
        self.model_path = Path(model_path)
        self.config_path = Path(config_path)
        self.sample_rate = int(sample_rate)
        self.length_scale = float(length_scale)
        self.noise_scale = float(noise_scale)
        self.noise_w = float(noise_w)
        self.gain = float(gain)
        self.speaker_id = speaker_id

        logger.info(
            "PiperEngine: loading voice\n"
            "  model_path  = %s\n"
            "  config_path = %s\n"
            "  sample_rate = %d\n"
            "  length_scale= %.3f\n"
            "  noise_scale = %.3f\n"
            "  noise_w     = %.3f\n"
            "  gain        = %.3f\n"
            "  speaker_id  = %s",
            self.model_path,
            self.config_path,
            self.sample_rate,
            self.length_scale,
            self.noise_scale,
            self.noise_w,
            self.gain,
        )

        if not self.model_path.is_file():
            raise FileNotFoundError(f"Piper model file not found: {self.model_path}")

        if not self.config_path.is_file():
            raise FileNotFoundError(f"Piper config file not found: {self.config_path}")

        # Load Piper voice into memory. This can take a second on the Pi.
        # PiperVoice.load() returns a PiperVoice instance with a .synthesize()
        # generator method.
        self.voice: PiperVoice = PiperVoice.load(
            str(self.model_path),
            str(self.config_path),
        )

        logger.info("PiperEngine: voice loaded successfully")

    # ------------------------------------------------------------------ #
    # Public API                                                         #
    # ------------------------------------------------------------------ #

    def synthesize_to_pcm16(self, text: str) -> np.ndarray:
        """
        Synthesize text to a mono int16 PCM numpy array.

        Parameters
        ----------
        text : str
            Text to speak.

        Returns
        -------
        audio : np.ndarray
            1D numpy array, dtype=int16, mono PCM at self.sample_rate Hz.
            Empty array if synthesis fails.
        """
        text = (text or "").strip()
        if not text:
            logger.warning("PiperEngine.synthesize_to_pcm16() called with empty text")
            return np.zeros(0, dtype=np.int16)

        logger.debug(
            "PiperEngine.synthesize_to_pcm16(): text='%s'",
            text if len(text) < 120 else text[:117] + "...",
        )

        # Piper's synthesize() yields chunks of raw int16 PCM bytes.
        try:
            chunks: list[np.ndarray] = []

            for chunk in self.voice.synthesize(
                text,
                length_scale=self.length_scale,
                noise_scale=self.noise_scale,
                noise_w=self.noise_w,
                speaker_id=self.speaker_id,
            ):
                # Each chunk is a bytes object containing little-endian int16 PCM
                arr = np.frombuffer(chunk, dtype=np.int16)
                if arr.size > 0:
                    chunks.append(arr)

            if not chunks:
                logger.error("PiperEngine: synthesize() returned no audio chunks")
                return np.zeros(0, dtype=np.int16)

            audio = np.concatenate(chunks).astype(np.int16)

        except Exception as exc:  # noqa: BLE001
            logger.error("PiperEngine.synthesize_to_pcm16() failed: %s", exc)
            return np.zeros(0, dtype=np.int16)

        # Apply linear gain, with clipping to int16 range.
        if self.gain != 1.0:
            audio_f32 = audio.astype(np.float32) * self.gain
            audio = np.clip(audio_f32, -32768, 32767).astype(np.int16)

        logger.debug(
            "PiperEngine: synthesized %d samples at %d Hz",
            audio.size,
            self.sample_rate,
        )
        return audio
