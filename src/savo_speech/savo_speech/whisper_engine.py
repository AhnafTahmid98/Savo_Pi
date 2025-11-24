"""
Robot Savo — Faster-Whisper engine wrapper.

This module provides a thin, robust wrapper around faster-whisper so that
ROS 2 nodes (like stt_node.py) can use a simple Python class instead of
dealing with model loading and transcription options directly.

Design goals:
- Hide faster-whisper internals behind a small, stable API.
- Keep configuration minimal but explicit (model path, device, compute_type).
- Be safe to call in a loop from STTNode's timer callback.
"""

from __future__ import annotations

import logging
from typing import Iterable, Tuple

import numpy as np
from faster_whisper import WhisperModel

logger = logging.getLogger(__name__)


class FasterWhisperEngine:
    """
    Wrapper around faster-whisper's WhisperModel.

    Typical usage:

        engine = FasterWhisperEngine(
            model_size_or_path="small.en",
            device="cpu",
            compute_type="int8",
            language="en",
            beam_size=5,
        )

        text = engine.transcribe_block(audio_np_float32_16k)
    """

    def __init__(
        self,
        model_size_or_path: str,
        device: str = "cpu",
        compute_type: str = "int8",
        language: str = "en",
        beam_size: int = 5,
    ) -> None:
        """
        Initialize the WhisperModel.

        Parameters
        ----------
        model_size_or_path : str
            e.g. "tiny", "small.en", "/path/to/local/model".
        device : str
            "cpu", "cuda", or "auto". On the Pi, use "cpu".
        compute_type : str
            e.g. "int8", "int8_float16", "float16", "float32".
            "int8" is recommended on ARM for speed & memory.
        language : str
            Language code, e.g. "en" for English.
        beam_size : int
            Beam size for beam search. 5 is a good trade-off.
        """
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

        # Load the underlying WhisperModel.
        # This may take a few seconds on the Pi, so we do it once in __init__.
        self._model = WhisperModel(
            self.model_size_or_path,
            device=self.device,
            compute_type=self.compute_type,
        )

        logger.info("FasterWhisperEngine model loaded successfully")

    # ------------------------------------------------------------------ #
    # Public API                                                         #
    # ------------------------------------------------------------------ #

    def transcribe_block(self, audio: np.ndarray) -> str:
        """
        Transcribe a single audio block (mono float32, 16 kHz recommended).

        Parameters
        ----------
        audio : np.ndarray
            1D numpy array, float32, representing PCM audio at ~16 kHz.
            The STTNode is responsible for ensuring the sample rate.

        Returns
        -------
        text : str
            The concatenated transcription for the block. Empty string
            if nothing intelligible was recognized.
        """
        if audio is None or audio.size == 0:
            logger.debug("transcribe_block() called with empty audio array")
            return ""

        # Ensure we pass a float32 mono array to faster-whisper
        audio_f32 = np.asarray(audio, dtype=np.float32)
        if audio_f32.ndim != 1:
            audio_f32 = audio_f32.flatten()

        logger.debug(
            "Transcribing audio block: samples=%d, dtype=%s",
            audio_f32.size,
            audio_f32.dtype,
        )

        # NOTE:
        # - We use a simple configuration here: language, beam_size.
        # - We do NOT enable built-in VAD (vad_filter=False) because the
        #   STT node already uses energy-based gating before calling us.
        try:
            segments, info = self._model.transcribe(
                audio_f32,
                language=self.language,
                beam_size=self.beam_size,
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

    # ------------------------------------------------------------------ #
    # Internal helpers                                                   #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _segments_to_text(segments: Iterable) -> str:
        """
        Combine segments into a single clean string.

        Parameters
        ----------
        segments : Iterable
            Iterable of faster-whisper segment objects with .text attributes.

        Returns
        -------
        text : str
            Concatenated and stripped transcript.
        """
        parts = []
        for seg in segments:
            seg_text = getattr(seg, "text", "")
            if seg_text:
                seg_text = seg_text.strip()
                if seg_text:
                    parts.append(seg_text)

        return " ".join(parts).strip()
