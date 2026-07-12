# -*- coding: utf-8 -*-

"""Speech runtime states and events shared by Python AI-facing components."""

from __future__ import annotations

from enum import Enum
from typing import TypeVar


class SpeechState(str, Enum):
    STARTING = "STARTING"
    SLEEPING = "SLEEPING"
    WAKE_DETECTED = "WAKE_DETECTED"
    AWAKE_IDLE = "AWAKE_IDLE"
    LISTENING = "LISTENING"
    UTTERANCE_READY = "UTTERANCE_READY"
    TRANSCRIBING = "TRANSCRIBING"
    THINKING = "THINKING"
    SYNTHESIZING = "SYNTHESIZING"
    SPEAKING = "SPEAKING"
    ERROR = "ERROR"

    @classmethod
    def values(cls) -> tuple[str, ...]:
        return tuple(item.value for item in cls)


class SpeechEvent(str, Enum):
    STARTUP_COMPLETE = "STARTUP_COMPLETE"
    WAKE_DETECTED = "WAKE_DETECTED"
    LISTEN_REQUESTED = "LISTEN_REQUESTED"
    SPEECH_STARTED = "SPEECH_STARTED"
    UTTERANCE_READY = "UTTERANCE_READY"
    STT_STARTED = "STT_STARTED"
    TRANSCRIPT_READY = "TRANSCRIPT_READY"
    THINKING_STARTED = "THINKING_STARTED"
    REPLY_READY = "REPLY_READY"
    TTS_STARTED = "TTS_STARTED"
    AUDIO_READY = "AUDIO_READY"
    PLAYBACK_STARTED = "PLAYBACK_STARTED"
    PLAYBACK_FINISHED = "PLAYBACK_FINISHED"
    CANCEL_REQUESTED = "CANCEL_REQUESTED"
    IDLE_TIMEOUT = "IDLE_TIMEOUT"
    MUTE_ENABLED = "MUTE_ENABLED"
    MUTE_DISABLED = "MUTE_DISABLED"
    FAILURE = "FAILURE"
    RESET = "RESET"

    @classmethod
    def values(cls) -> tuple[str, ...]:
        return tuple(item.value for item in cls)


class WakeState(str, Enum):
    DISABLED = "DISABLED"
    SLEEPING = "SLEEPING"
    AWAKE = "AWAKE"
    COOLDOWN = "COOLDOWN"
    ERROR = "ERROR"

    @classmethod
    def values(cls) -> tuple[str, ...]:
        return tuple(item.value for item in cls)


class ProviderState(str, Enum):
    DISABLED = "DISABLED"
    UNAVAILABLE = "UNAVAILABLE"
    STARTING = "STARTING"
    READY = "READY"
    BUSY = "BUSY"
    DEGRADED = "DEGRADED"
    ERROR = "ERROR"

    @classmethod
    def values(cls) -> tuple[str, ...]:
        return tuple(item.value for item in cls)


EnumT = TypeVar("EnumT", bound=Enum)


def normalize_enum_token(value: object) -> str:
    if value is None:
        return ""

    return str(value).strip().upper().replace("-", "_").replace(" ", "_")


def _parse_enum(
    value: object,
    enum_type: type[EnumT],
    default: EnumT,
) -> EnumT:
    if isinstance(value, enum_type):
        return value

    normalized = normalize_enum_token(value)

    for item in enum_type:
        if item.value == normalized:
            return item

    return default


def parse_speech_state(
    value: object,
    default: SpeechState = SpeechState.ERROR,
) -> SpeechState:
    return _parse_enum(value, SpeechState, default)


def parse_speech_event(
    value: object,
    default: SpeechEvent = SpeechEvent.FAILURE,
) -> SpeechEvent:
    return _parse_enum(value, SpeechEvent, default)


def parse_wake_state(
    value: object,
    default: WakeState = WakeState.ERROR,
) -> WakeState:
    return _parse_enum(value, WakeState, default)


def parse_provider_state(
    value: object,
    default: ProviderState = ProviderState.ERROR,
) -> ProviderState:
    return _parse_enum(value, ProviderState, default)


def is_busy_state(value: object) -> bool:
    state = parse_speech_state(value)

    return state in {
        SpeechState.WAKE_DETECTED,
        SpeechState.LISTENING,
        SpeechState.UTTERANCE_READY,
        SpeechState.TRANSCRIBING,
        SpeechState.THINKING,
        SpeechState.SYNTHESIZING,
        SpeechState.SPEAKING,
    }


def is_capture_state(value: object) -> bool:
    state = parse_speech_state(value)

    return state in {
        SpeechState.LISTENING,
        SpeechState.UTTERANCE_READY,
    }


def is_output_state(value: object) -> bool:
    state = parse_speech_state(value)

    return state in {
        SpeechState.SYNTHESIZING,
        SpeechState.SPEAKING,
    }


__all__ = [
    "ProviderState",
    "SpeechEvent",
    "SpeechState",
    "WakeState",
    "is_busy_state",
    "is_capture_state",
    "is_output_state",
    "normalize_enum_token",
    "parse_provider_state",
    "parse_speech_event",
    "parse_speech_state",
    "parse_wake_state",
]
