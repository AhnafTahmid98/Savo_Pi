#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Savo — Audio I/O utilities for savo_speech.

This module centralizes low-level audio capture and playback so that the ROS 2
nodes (stt_node, tts_node) can stay focused on ROS logic and STT/TTS behavior.

Design goals:
- Simple, blocking APIs (good enough for a single STT/TTS node).
- Safe defaults for Robot Savo on the Pi (mono, float32, 16 kHz / 22.05 kHz).
- Clear logging and exceptions when devices or parameters are incorrect.

Dependencies:
- sounddevice
- numpy
"""

from __future__ import annotations

import logging
from typing import Optional

import numpy as np
import sounddevice as sd

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Helpers for device selection / introspection
# ---------------------------------------------------------------------------


def list_audio_devices() -> None:
    """
    Log all available audio devices.

    Useful for debugging on the Pi to ensure ReSpeaker (input) and speaker
    (output) are configured and visible to sounddevice.
    """
    try:
        devices = sd.query_devices()
    except Exception as exc:  # noqa: BLE001
        logger.error("Failed to query audio devices: %s", exc)
        return

    logger.info("Available audio devices:")
    for idx, dev in enumerate(devices):
        logger.info(
            "  [%d] %s (in=%d ch, out=%d ch)",
            idx,
            dev.get("name", "UNKNOWN"),
            dev.get("max_input_channels", 0),
            dev.get("max_output_channels", 0),
        )


def get_default_input_device() -> Optional[int]:
    """
    Return the default input device index (or None if not set/available).

    If sounddevice is correctly configured (e.g. via ALSA), this will usually
    be the ReSpeaker mic on Robot Savo.
    """
    try:
        default_in, _ = sd.default.device
        devices = sd.query_devices()
    except Exception as exc:  # noqa: BLE001
        logger.error("Failed to query default audio devices: %s", exc)
        return None

    if default_in is None or default_in < 0 or default_in >= len(devices):
        logger.warning(
            "No valid default input device configured. "
            "sounddevice will choose a backend-dependent default."
        )
        return None

    dev = devices[default_in]
    logger.info(
        "Default input device: [%d] %s (in=%d ch)",
        default_in,
        dev.get("name", "UNKNOWN"),
        dev.get("max_input_channels", 0),
    )
    return default_in


def get_default_output_device() -> Optional[int]:
    """
    Return the default output device index (or None if not set/available).

    On Robot Savo, this should typically be the external speaker device
    configured at the ALSA/system level.
    """
    try:
        _, default_out = sd.default.device
        devices = sd.query_devices()
    except Exception as exc:  # noqa: BLE001
        logger.error("Failed to query default audio devices: %s", exc)
        return None

    if default_out is None or default_out < 0 or default_out >= len(devices):
        logger.warning(
            "No valid default output device configured. "
            "sounddevice will choose a backend-dependent default."
        )
        return None

    dev = devices[default_out]
    logger.info(
        "Default output device: [%d] %s (out=%d ch)",
        default_out,
        dev.get("name", "UNKNOWN"),
        dev.get("max_output_channels", 0),
    )
    return default_out


# ---------------------------------------------------------------------------
# Core APIs: record_block() and play_pcm()
# ---------------------------------------------------------------------------


def record_block(
    sample_rate: int,
    duration_s: float,
    input_device_index: Optional[int] = None,
    channels: int = 1,
    dtype: str = "float32",
) -> np.ndarray:
    """
    Record a fixed-duration block of audio.

    Parameters
    ----------
    sample_rate : int
        Target sampling rate in Hz (e.g. 16000 for faster-whisper).
    duration_s : float
        Duration of the recording in seconds.
    input_device_index : Optional[int]
        Explicit sounddevice input device index. On Robot Savo, we usually
        pass the ReSpeaker index (e.g. 0). If None, use default input device.
    channels : int
        Number of input channels to record. For STT, we typically want mono.
    dtype : str
        Numpy dtype string for the buffer. "float32" is recommended.

    Returns
    -------
    audio : np.ndarray
        1D numpy array of shape (num_samples,), dtype=float32.
    """
    if duration_s <= 0.0:
        raise ValueError(f"record_block() duration_s must be > 0 (got {duration_s})")

    num_frames = int(round(duration_s * sample_rate))
    if num_frames <= 0:
        raise ValueError(
            f"record_block() duration_s={duration_s} and sample_rate={sample_rate} "
            "result in zero frames"
        )

    # Decide which device to use
    device = input_device_index
    if device is not None and device < 0:
        logger.warning(
            "record_block() called with invalid input_device_index=%s, "
            "falling back to default input device.",
            device,
        )
        device = None

    if device is None:
        device = get_default_input_device()

    logger.debug(
        "Recording audio: sample_rate=%d Hz, duration=%.3f s, frames=%d, "
        "channels=%d, device=%s",
        sample_rate,
        duration_s,
        num_frames,
        channels,
        "default" if device is None else str(device),
    )

    try:
        audio = sd.rec(
            frames=num_frames,
            samplerate=sample_rate,
            channels=channels,
            dtype=dtype,
            device=device,
        )
        sd.wait()
    except Exception as exc:  # noqa: BLE001
        logger.error("Audio recording failed: %s", exc)
        raise

    # Ensure we always return a 1D float32 mono array.
    arr = np.asarray(audio, dtype=np.float32)

    if arr.ndim == 2:
        # Collapse multi-channel -> mono by averaging channels.
        if arr.shape[1] > 1:
            arr = arr.mean(axis=1)
        else:
            arr = arr[:, 0]

    elif arr.ndim != 1:
        logger.warning(
            "Unexpected audio array shape %s, trying to flatten", arr.shape
        )
        arr = arr.flatten()

    return arr


def play_pcm(
    audio: np.ndarray,
    sample_rate: int,
    device: Optional[int] = None,
) -> None:
    """
    Play back a mono PCM audio buffer.

    Parameters
    ----------
    audio : np.ndarray
        1D numpy array (float32 recommended) representing the audio signal.
        Values outside [-1.0, 1.0] will be clipped.
    sample_rate : int
        Playback sample rate in Hz.
    device : Optional[int]
        Optional sounddevice device index. If None, use default output.
    """
    if audio is None or audio.size == 0:
        logger.warning("play_pcm() called with empty audio buffer")
        return

    # Clip to [-1, 1] for safety.
    audio_f32 = np.asarray(audio, dtype=np.float32)
    max_abs = float(np.max(np.abs(audio_f32)))
    if max_abs > 1.0:
        logger.debug(
            "play_pcm() clipping audio with max abs amplitude %.4f to [-1, 1]",
            max_abs,
        )
        audio_f32 = np.clip(audio_f32, -1.0, 1.0)

    # Ensure correct shape for sounddevice: (num_frames, channels)
    audio_2d = audio_f32.reshape(-1, 1)

    # If no device specified, we try to use the default output.
    if device is None:
        device = get_default_output_device()

    logger.debug(
        "Playing audio: sample_rate=%d Hz, frames=%d, device=%s",
        sample_rate,
        audio_2d.shape[0],
        "default" if device is None else str(device),
    )

    try:
        sd.play(audio_2d, samplerate=sample_rate, device=device)
        sd.wait()
    except Exception as exc:  # noqa: BLE001
        logger.error("Audio playback failed: %s", exc)
        raise
