#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Package identity, ownership, node, topic, and service constants."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final, Tuple

from savo_speech.version import PACKAGE_NAME, ROBOT_NAME


# =============================================================================
# Deployment identity
# =============================================================================
RUNTIME_HOST: Final[str] = "savo-edge"
PACKAGE_ROLE: Final[str] = "physical_speech_interface"
PACKAGE_NAMESPACE: Final[str] = "/savo_speech"
PRODUCTION_LANGUAGE: Final[str] = "cpp"
AI_SERVICE_LANGUAGE: Final[str] = "python"


# =============================================================================
# Production components
# =============================================================================
NODE_RESPEAKER_AUDIO: Final[str] = "respeaker_audio_node"
NODE_SPEECH_MANAGER: Final[str] = "speech_manager_node"
NODE_SAVOMIND_BRIDGE: Final[str] = "savomind_bridge_node"
NODE_SPEECH_DASHBOARD: Final[str] = "speech_dashboard_node"

SERVICE_WAKE_WORD: Final[str] = "wake_word_service"
SERVICE_LOCAL_STT: Final[str] = "local_stt_service"
SERVICE_LOCAL_TTS: Final[str] = "local_tts_service"

CPP_PRODUCTION_COMPONENTS: Final[Tuple[str, ...]] = (
    NODE_RESPEAKER_AUDIO,
    NODE_SPEECH_MANAGER,
)

PYTHON_AI_COMPONENTS: Final[Tuple[str, ...]] = (
    SERVICE_WAKE_WORD,
    SERVICE_LOCAL_STT,
    SERVICE_LOCAL_TTS,
    NODE_SAVOMIND_BRIDGE,
)


# =============================================================================
# Public ROS topics
# =============================================================================
TOPIC_STATE: Final[str] = "/savo_speech/state"
TOPIC_STATUS: Final[str] = "/savo_speech/status"
TOPIC_HEALTH: Final[str] = "/savo_speech/health"
TOPIC_HEARTBEAT: Final[str] = "/savo_speech/heartbeat"

TOPIC_WAKE_STATE: Final[str] = "/savo_speech/wake_state"
TOPIC_WAKE_WORD_DETECTED: Final[str] = "/savo_speech/wake_word/detected"
TOPIC_LISTENING: Final[str] = "/savo_speech/listening"
TOPIC_INPUT_MUTED: Final[str] = "/savo_speech/input_muted"
TOPIC_OUTPUT_MUTED: Final[str] = "/savo_speech/output_muted"
TOPIC_TRANSCRIPT: Final[str] = "/savo_speech/transcript"

TOPIC_TTS_GATE: Final[str] = "/savo_speech/tts_gate"
TOPIC_TTS_STARTED: Final[str] = "/savo_speech/tts/started"
TOPIC_TTS_FINISHED: Final[str] = "/savo_speech/tts/finished"
TOPIC_TTS_CANCELLED: Final[str] = "/savo_speech/tts/cancelled"

TOPIC_FACE_STATE: Final[str] = "/savo_speech/face_state"
TOPIC_LAST_ERROR: Final[str] = "/savo_speech/last_error"

PUBLIC_TOPICS: Final[Tuple[str, ...]] = (
    TOPIC_STATE,
    TOPIC_STATUS,
    TOPIC_HEALTH,
    TOPIC_HEARTBEAT,
    TOPIC_WAKE_STATE,
    TOPIC_WAKE_WORD_DETECTED,
    TOPIC_LISTENING,
    TOPIC_INPUT_MUTED,
    TOPIC_OUTPUT_MUTED,
    TOPIC_TRANSCRIPT,
    TOPIC_TTS_GATE,
    TOPIC_TTS_STARTED,
    TOPIC_TTS_FINISHED,
    TOPIC_TTS_CANCELLED,
    TOPIC_FACE_STATE,
    TOPIC_LAST_ERROR,
)


# =============================================================================
# Public ROS services
# =============================================================================
ROS_SERVICE_WAKE: Final[str] = "/savo_speech/wake"
ROS_SERVICE_SLEEP: Final[str] = "/savo_speech/sleep"
ROS_SERVICE_START_LISTENING: Final[str] = "/savo_speech/start_listening"
ROS_SERVICE_STOP_LISTENING: Final[str] = "/savo_speech/stop_listening"
ROS_SERVICE_CANCEL: Final[str] = "/savo_speech/cancel"
ROS_SERVICE_MUTE_INPUT: Final[str] = "/savo_speech/mute_input"
ROS_SERVICE_MUTE_OUTPUT: Final[str] = "/savo_speech/mute_output"
ROS_SERVICE_RELOAD_AUDIO_DEVICE: Final[str] = (
    "/savo_speech/reload_audio_device"
)

PUBLIC_SERVICES: Final[Tuple[str, ...]] = (
    ROS_SERVICE_WAKE,
    ROS_SERVICE_SLEEP,
    ROS_SERVICE_START_LISTENING,
    ROS_SERVICE_STOP_LISTENING,
    ROS_SERVICE_CANCEL,
    ROS_SERVICE_MUTE_INPUT,
    ROS_SERVICE_MUTE_OUTPUT,
    ROS_SERVICE_RELOAD_AUDIO_DEVICE,
)


# =============================================================================
# Ownership boundaries
# =============================================================================
OWNED_HARDWARE: Final[Tuple[str, ...]] = (
    "respeaker_usb_microphone_array",
    "respeaker_alsa_capture",
    "respeaker_alsa_playback",
    "respeaker_3_5_mm_audio_output",
    "adafruit_powered_speakers",
)

FORBIDDEN_MOTION_TOPICS: Final[Tuple[str, ...]] = (
    "/cmd_vel",
    "/cmd_vel_manual",
    "/cmd_vel_auto",
    "/cmd_vel_nav",
    "/cmd_vel_recovery",
    "/cmd_vel_safe",
)


# =============================================================================
# Status labels
# =============================================================================
STATUS_OK: Final[str] = "OK"
STATUS_WARN: Final[str] = "WARN"
STATUS_ERROR: Final[str] = "ERROR"
STATUS_STALE: Final[str] = "STALE"
STATUS_DISABLED: Final[str] = "DISABLED"
STATUS_UNKNOWN: Final[str] = "UNKNOWN"


@dataclass(frozen=True)
class SpeechPackageIdentity:
    package_name: str = PACKAGE_NAME
    robot_name: str = ROBOT_NAME
    runtime_host: str = RUNTIME_HOST
    package_role: str = PACKAGE_ROLE
    production_language: str = PRODUCTION_LANGUAGE
    ai_service_language: str = AI_SERVICE_LANGUAGE

    def to_dict(self) -> dict:
        return {
            "package_name": self.package_name,
            "robot_name": self.robot_name,
            "runtime_host": self.runtime_host,
            "package_role": self.package_role,
            "production_language": self.production_language,
            "ai_service_language": self.ai_service_language,
            "cpp_production_components": list(CPP_PRODUCTION_COMPONENTS),
            "python_ai_components": list(PYTHON_AI_COMPONENTS),
            "owned_hardware": list(OWNED_HARDWARE),
            "public_topics": list(PUBLIC_TOPICS),
            "public_services": list(PUBLIC_SERVICES),
            "forbidden_motion_topics": list(FORBIDDEN_MOTION_TOPICS),
        }


IDENTITY: Final[SpeechPackageIdentity] = SpeechPackageIdentity()
