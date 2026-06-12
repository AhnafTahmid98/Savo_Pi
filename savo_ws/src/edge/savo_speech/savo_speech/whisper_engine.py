#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Small faster-whisper wrapper used by the STT node."""

from __future__ import annotations

import logging
from typing import Iterable, Optional

import numpy as np
from faster_whisper import WhisperModel

logger = logging.getLogger(__name__)


class FasterWhisperEngine:
    """Wrapper around faster-whisper's WhisperModel."""

    def __init__(
        self,
        model_size_or_path: str,
        device: str = "cpu",
        compute_type: str = "int8",
        language: str = "en",
        beam_size: int = 5,
    ) -> None:
        """Load the Whisper model once."""
        self.model_size_or_path = model_size_or_path
        self.device = device
        self.compute_type = compute_type
        self.language = language
        self.beam_size = beam_size

        logger.info(
            "Initializing FasterWhisperEngine:\n"
            "  model_size_or_path = %s\n"
            "  device             = %s\n"
            "  compute_type       = %s\n"
            "  language           = %s\n"
            "  beam_size          = %d",
            self.model_size_or_path,
            self.device,
            self.compute_type,
            self.language,
            self.beam_size,
        )

        # This may take a few seconds on the Pi, so we do it once in __init__.
        self._model = WhisperModel(
            self.model_size_or_path,
            device=self.device,
            compute_type=self.compute_type,
        )

        logger.info("FasterWhisperEngine model loaded successfully")
    def transcribe_block(
        self,
        audio: np.ndarray,
        sample_rate: Optional[int] = None,
        language: Optional[str] = None,
        beam_size: Optional[int] = None,
    ) -> str:
        """Transcribe one mono float32 audio block."""
        if audio is None or audio.size == 0:
            logger.debug("transcribe_block() called with empty audio array")
            return ""

        audio_f32 = np.asarray(audio, dtype=np.float32)
        if audio_f32.ndim != 1:
            audio_f32 = audio_f32.flatten()

        lang = language or self.language
        bs = int(beam_size or self.beam_size)

        if sample_rate is not None and sample_rate != 16000:
            # Not a fatal error, but worth logging so we remember.
            logger.warning(
                "FasterWhisperEngine.transcribe_block called with "
                "sample_rate=%d (expected 16000). Audio will still be "
                "passed to faster-whisper, but quality may be affected.",
                sample_rate,
            )

        logger.debug(
            "Transcribing audio block: samples=%d, dtype=%s, "
            "sample_rate=%s, language=%s, beam_size=%d",
            audio_f32.size,
            audio_f32.dtype,
            str(sample_rate) if sample_rate is not None else "unknown",
            lang,
            bs,
        )

        # STT nodes already do energy gating, so faster-whisper VAD stays off.
        try:
            segments, info = self._model.transcribe(
                audio_f32,
                language=lang,
                beam_size=bs,
                vad_filter=False,
                task="transcribe",
            )
        except Exception as exc:  # noqa: BLE001
            logger.error("FasterWhisperEngine.transcribe_block error: %s", exc)
            return ""

        text = self._segments_to_text(segments)
        logger.debug(
            "FasterWhisperEngine: duration=%.3f s, text='%s'",
            getattr(info, "duration", 0.0),
            text,
        )
        return text
    @staticmethod
    def _segments_to_text(segments: Iterable) -> str:
        """Combine faster-whisper segments into one clean string."""
        parts = []
        for seg in segments:
            seg_text = getattr(seg, "text", "")
            if not seg_text:
                continue
            seg_text = seg_text.strip()
            if seg_text:
                parts.append(seg_text)

        return " ".join(parts).strip()
