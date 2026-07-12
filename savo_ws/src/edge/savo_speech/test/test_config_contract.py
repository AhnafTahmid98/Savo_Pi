from pathlib import Path

import pytest

from savo_speech.ros.param_utils import (
    BASE_CONFIG_NODES,
    ConfigError,
    extract_ros_parameters,
    load_base_parameters,
    validate_node_parameters,
)


ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "config"


def test_all_base_configuration_files_are_valid():
    parameters = load_base_parameters(CONFIG_DIR)

    assert set(parameters) == set(BASE_CONFIG_NODES.values())


def test_audio_defaults_are_pc_safe():
    parameters = extract_ros_parameters(
        CONFIG_DIR / "audio.yaml",
        "respeaker_audio_node",
    )

    assert parameters["enabled"] is False
    assert parameters["backend"] == "dryrun"
    assert parameters["sample_rate_hz"] == 16000
    assert parameters["capture.device_channels"] == 0
    assert parameters["capture.processed_channel"] == 0


def test_ai_and_external_services_are_disabled_by_default():
    parameters = load_base_parameters(CONFIG_DIR)

    assert parameters["wake_word_service"]["enabled"] is False
    assert parameters["local_stt_service"]["enabled"] is False
    assert parameters["local_tts_service"]["enabled"] is False
    assert parameters["savomind_bridge_node"]["enabled"] is False


def test_speech_manager_owns_utterance_timing():
    parameters = load_base_parameters(CONFIG_DIR)
    manager = parameters["speech_manager_node"]

    assert manager["utterance.min_duration_s"] > 0
    assert (
        manager["utterance.min_duration_s"]
        < manager["utterance.max_duration_s"]
    )
    assert manager["tts_gate.enabled"] is True


def test_invalid_processed_channel_is_rejected():
    parameters = extract_ros_parameters(
        CONFIG_DIR / "audio.yaml",
        "respeaker_audio_node",
    )

    parameters["capture.device_channels"] = 2
    parameters["capture.processed_channel"] = 2

    with pytest.raises(ConfigError):
        validate_node_parameters(
            "respeaker_audio_node",
            parameters,
        )


def test_invalid_utterance_duration_is_rejected():
    parameters = extract_ros_parameters(
        CONFIG_DIR / "speech_manager.yaml",
        "speech_manager_node",
    )

    parameters["utterance.min_duration_s"] = 10.0
    parameters["utterance.max_duration_s"] = 2.0

    with pytest.raises(ConfigError):
        validate_node_parameters(
            "speech_manager_node",
            parameters,
        )


def test_invalid_savomind_url_is_rejected():
    parameters = extract_ros_parameters(
        CONFIG_DIR / "savomind.yaml",
        "savomind_bridge_node",
    )

    parameters["base_url"] = "localhost:8000"

    with pytest.raises(ConfigError):
        validate_node_parameters(
            "savomind_bridge_node",
            parameters,
        )
