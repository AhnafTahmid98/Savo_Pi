#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Piper TTS engine wrapper.

This module hides the details of the piper-tts Python API behind a small,
stable interface that TTSNode can use:

    engine = PiperEngine(
        model_dir="/home/savo/Savo_Pi/models/piper",
        default_profile="male",
        length_scale=0.95,
        noise_scale=0.667,
        noise_w=0.8,
    )

    pcm16, sample_rate = engine.synthesize_to_pcm16("Hello, world!")

Design goals:
- Load a small set of known voice profiles (male/female) for Robot Savo.
- Use the piper-tts streaming API (synthesize with chunk_size) so we can
  generate audio in chunks, but still return a single NumPy array to TTSNode.
- Keep all TTS logic out of ROS nodes; TTSNode just calls this engine.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from piper.voice import PiperVoice  # provided by the piper-tts package

logger = logging.getLogger(__name__)


@dataclass
class VoiceConfig:
    """Configuration for a single Piper voice profile."""

    profile_name: str          # "male" / "female" / etc.
    base_name: str             # e.g. "en_US-ryan-high"
    model_path: Path           # .onnx
    config_path: Path          # .onnx.json
    voice: PiperVoice          # loaded PiperVoice instance


class PiperEngine:
    """
    Wrapper around piper-tts's PiperVoice for Robot Savo.

    Main methods:
        - list_profiles() -> List[str]
        - set_default_profile(name: str) -> None
        - synthesize_to_pcm16(text: str, profile: Optional[str] = None)
            -> Tuple[np.ndarray, int]
    """

    # Map high-level profiles to concrete Piper model base names
    PROFILE_MAP: Dict[str, str] = {
        "male": "en_US-ryan-high",
        "female": "en_US-hfc_female-medium",
    }

    def __init__(
        self,
        model_dir: str,
        default_profile: str = "male",
        length_scale: float = 1.0,
        noise_scale: float = 0.667,
        noise_w: float = 0.8,
        chunk_size: int = 2048,
    ) -> None:
        """
        Initialize the PiperEngine.

        Parameters
        ----------
        model_dir : str
            Directory containing the .onnx and .onnx.json voice files.
        default_profile : str
            Human-readable profile key ("male", "female").
        length_scale : float
            Speaking rate (not yet wired directly to piper; left for future).
        noise_scale : float
            Noise scale (not yet wired directly; future tuning).
        noise_w : float
            Noise_w parameter (not yet wired directly; future tuning).
        chunk_size : int
            Number of samples per synthesized chunk when streaming.
        """
        self.model_dir = Path(model_dir).expanduser()
        self.length_scale = float(length_scale)
        self.noise_scale = float(noise_scale)
        self.noise_w = float(noise_w)
        self.chunk_size = int(chunk_size)

        if not self.model_dir.is_dir():
            raise FileNotFoundError(f"PiperEngine: model_dir does not exist: {self.model_dir}")

        logger.info(
            "PiperEngine: initializing from %s (default_profile=%s)",
            self.model_dir,
            default_profile,
        )

        # Loaded voice configs keyed by profile ("male", "female", ...)
        self._voices: Dict[str, VoiceConfig] = {}
        self._default_profile: Optional[str] = None
        self.sample_rate: int = 22050  # will be overwritten by first loaded voice

        self._load_all_profiles()
        self.set_default_profile(default_profile)

        logger.info(
            "PiperEngine: loaded profiles: %s (sample_rate=%d Hz)",
            ", ".join(sorted(self._voices.keys())),
            self.sample_rate,
        )

    # ------------------------------------------------------------------ #
    # Public API                                                         #
    # ------------------------------------------------------------------ #

    def list_profiles(self) -> List[str]:
        """Return the list of available profiles (e.g. ['female', 'male'])."""
        return sorted(self._voices.keys())

    def set_default_profile(self, profile: str) -> None:
        """
        Set the default voice profile.

        If the requested profile is not available, we keep the existing default
        and log a warning.
        """
        if profile not in self._voices:
            logger.warning(
                "PiperEngine.set_default_profile(): profile '%s' not available. "
                "Available: %s",
                profile,
                ", ".join(self.list_profiles()),
            )
            if self._default_profile is None and self._voices:
                # fall back to the first known voice
                self._default_profile = self.list_profiles()[0]
            return

        self._default_profile = profile
        logger.info("PiperEngine: default profile set to '%s'", profile)

    def synthesize_to_pcm16(
        self,
        text: str,
        profile: Optional[str] = None,
    ) -> Tuple[np.ndarray, int]:
        """
        Synthesize speech to a NumPy int16 PCM array.

        Parameters
        ----------
        text : str
            Text to speak.
        profile : Optional[str]
            Optional profile override ("male", "female"). If None, use the
            current default profile.

        Returns
        -------
        pcm16 : np.ndarray
            1D NumPy array of dtype=int16. May be empty on failure.
        sample_rate : int
            Sample rate in Hz (from Piper config).
        """
        text = self._normalize_text(text)

        if not text:
            logger.debug("PiperEngine.synthesize_to_pcm16() called with empty text")
            return np.zeros(0, dtype=np.int16), self.sample_rate

        voice_cfg = self._select_voice(profile)
        if voice_cfg is None:
            logger.error("PiperEngine.synthesize_to_pcm16(): no voice available")
            return np.zeros(0, dtype=np.int16), self.sample_rate

        logger.debug(
            "PiperEngine: synthesizing text (%d chars) with profile '%s'",
            len(text),
            voice_cfg.profile_name,
        )

        # piper-tts streaming API:
        #   for audio_chunk in voice.synthesize(text, chunk_size=...):
        #       audio_chunk is a NumPy int16 array of PCM samples
        pcm_chunks: List[bytes] = []

        try:
            for chunk in voice_cfg.voice.synthesize(text, chunk_size=self.chunk_size):
                # chunk may be a NumPy array or something array-like
                if hasattr(chunk, "tobytes"):
                    pcm_chunks.append(chunk.tobytes())
                else:
                    arr = np.asarray(chunk, dtype=np.int16)
                    pcm_chunks.append(arr.tobytes())

        except Exception as exc:  # noqa: BLE001
            logger.error("PiperEngine.synthesize_to_pcm16() failed: %s", exc)
            return np.zeros(0, dtype=np.int16), self.sample_rate

        if not pcm_chunks:
            logger.warning(
                "PiperEngine.synthesize_to_pcm16(): no audio chunks produced for text '%s'",
                text,
            )
            return np.zeros(0, dtype=np.int16), self.sample_rate

        pcm_bytes = b"".join(pcm_chunks)
        pcm16 = np.frombuffer(pcm_bytes, dtype=np.int16)

        logger.debug(
            "PiperEngine: synthesized %d samples at %d Hz for text '%s'",
            pcm16.size,
            self.sample_rate,
            text,
        )
        return pcm16, self.sample_rate

    # ------------------------------------------------------------------ #
    # Internal helpers                                                   #
    # ------------------------------------------------------------------ #

    def _select_voice(self, profile: Optional[str]) -> Optional[VoiceConfig]:
        """Return the VoiceConfig for the requested or default profile."""
        if profile and profile in self._voices:
            return self._voices[profile]

        if self._default_profile and self._default_profile in self._voices:
            return self._voices[self._default_profile]

        # Fallback: any available voice
        if self._voices:
            return self._voices[self.list_profiles()[0]]

        return None

    def _load_all_profiles(self) -> None:
        """Load all known profiles (male/female) that have model files."""
        for profile_name, base_name in self.PROFILE_MAP.items():
            try:
                model_path, config_path = self._resolve_voice_paths(base_name)
            except FileNotFoundError as exc:
                logger.warning(
                    "PiperEngine: missing files for profile '%s' (%s): %s",
                    profile_name,
                    base_name,
                    exc,
                )
                continue

            try:
                voice = PiperVoice.load(str(model_path), str(config_path))
            except Exception as exc:  # noqa: BLE001
                logger.error(
                    "PiperEngine: failed to load PiperVoice for profile '%s' "
                    "(model=%s, config=%s): %s",
                    profile_name,
                    model_path,
                    config_path,
                    exc,
                )
                continue

            cfg = VoiceConfig(
                profile_name=profile_name,
                base_name=base_name,
                model_path=model_path,
                config_path=config_path,
                voice=voice,
            )
            self._voices[profile_name] = cfg

            # Use the first successfully loaded voice to set sample_rate
            if len(self._voices) == 1:
                try:
                    self.sample_rate = int(voice.config.sample_rate)
                except Exception:  # noqa: BLE001
                    # Fallback to a common Piper default if config missing
                    self.sample_rate = 22050

        if not self._voices:
            raise RuntimeError(
                f"PiperEngine: no usable voices found in {self.model_dir}. "
                "Expected files like 'en_US-ryan-high.onnx' and "
                "'en_US-ryan-high.onnx.json'."
            )

    def _resolve_voice_paths(self, base_name: str) -> Tuple[Path, Path]:
        """
        Return (model_path, config_path) for a given base name.

        base_name="en_US-ryan-high" -> model_path=".../en_US-ryan-high.onnx",
                                       config_path=".../en_US-ryan-high.onnx.json"
        """
        model_path = self.model_dir / f"{base_name}.onnx"
        config_path = self.model_dir / f"{base_name}.onnx.json"

        if not model_path.is_file():
            raise FileNotFoundError(f"Model file not found: {model_path}")
        if not config_path.is_file():
            raise FileNotFoundError(f"Config file not found: {config_path}")

        return model_path, config_path

    @staticmethod
    def _normalize_text(text: str) -> str:
        """Simple normalization for input text."""
        if text is None:
            return ""
        # Collapse whitespace and strip
        normalized = " ".join(str(text).split())
        return normalized.strip()
