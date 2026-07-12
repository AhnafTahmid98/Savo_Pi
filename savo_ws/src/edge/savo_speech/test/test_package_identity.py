#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for package identity, version, and ownership boundaries."""

from savo_speech import VERSION, __version__, get_package_version_info
from savo_speech.constants import (
    CPP_PRODUCTION_COMPONENTS,
    FORBIDDEN_MOTION_TOPICS,
    IDENTITY,
    NODE_RESPEAKER_AUDIO,
    NODE_SPEECH_MANAGER,
    OWNED_HARDWARE,
    PACKAGE_NAME,
    PYTHON_AI_COMPONENTS,
    ROBOT_NAME,
    RUNTIME_HOST,
    SERVICE_LOCAL_STT,
    SERVICE_LOCAL_TTS,
    SERVICE_WAKE_WORD,
)


def test_version_metadata_matches_package_identity():
    info = get_package_version_info()

    assert PACKAGE_NAME == "savo_speech"
    assert ROBOT_NAME == "Robot Savo"
    assert VERSION == __version__ == "0.1.0"

    assert info.package_name == PACKAGE_NAME
    assert info.robot_name == ROBOT_NAME
    assert info.ros_distro == "jazzy"

    assert (
        info.banner()
        == "Robot Savo | savo_speech 0.1.0 (ROS 2 jazzy)"
    )


def test_runtime_placement_and_language_split_are_locked():
    assert RUNTIME_HOST == "savo-edge"

    assert CPP_PRODUCTION_COMPONENTS == (
        NODE_RESPEAKER_AUDIO,
        NODE_SPEECH_MANAGER,
    )

    assert SERVICE_WAKE_WORD in PYTHON_AI_COMPONENTS
    assert SERVICE_LOCAL_STT in PYTHON_AI_COMPONENTS
    assert SERVICE_LOCAL_TTS in PYTHON_AI_COMPONENTS


def test_physical_audio_ownership_is_explicit():
    assert "respeaker_usb_microphone_array" in OWNED_HARDWARE
    assert "respeaker_alsa_capture" in OWNED_HARDWARE
    assert "respeaker_alsa_playback" in OWNED_HARDWARE
    assert "adafruit_powered_speakers" in OWNED_HARDWARE


def test_speech_package_cannot_own_motion_topics():
    assert "/cmd_vel" in FORBIDDEN_MOTION_TOPICS
    assert "/cmd_vel_nav" in FORBIDDEN_MOTION_TOPICS
    assert "/cmd_vel_safe" in FORBIDDEN_MOTION_TOPICS


def test_identity_exports_machine_readable_contract():
    data = IDENTITY.to_dict()

    assert data["package_name"] == "savo_speech"
    assert data["runtime_host"] == "savo-edge"
    assert data["production_language"] == "cpp"
    assert data["ai_service_language"] == "python"
