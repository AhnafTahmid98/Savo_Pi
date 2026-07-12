# -*- coding: utf-8 -*-

"""ROS parameter-file and deployment-profile validation utilities."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any, Callable, Mapping

import yaml


BASE_CONFIG_NODES: dict[str, str] = {
    "audio.yaml": "respeaker_audio_node",
    "speech_manager.yaml": "speech_manager_node",
    "wake_word.yaml": "wake_word_service",
    "local_stt.yaml": "local_stt_service",
    "local_tts.yaml": "local_tts_service",
    "savomind.yaml": "savomind_bridge_node",
}

ALLOWED_PROFILE_NODES = frozenset(BASE_CONFIG_NODES.values())


class ConfigError(ValueError):
    """Raised when a savo_speech configuration contract is invalid."""


def load_yaml_mapping(path: str | Path) -> dict[str, Any]:
    config_path = Path(path)

    if not config_path.is_file():
        raise ConfigError(f"configuration file does not exist: {config_path}")

    with config_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    if not isinstance(data, dict):
        raise ConfigError(
            f"configuration root must be a mapping: {config_path}"
        )

    return data


def extract_ros_parameters(
    path: str | Path,
    expected_node: str | None = None,
) -> dict[str, Any]:
    config_path = Path(path)
    node_name = expected_node or BASE_CONFIG_NODES.get(config_path.name)

    if not node_name:
        raise ConfigError(
            f"no expected node registered for configuration: {config_path.name}"
        )

    data = load_yaml_mapping(config_path)

    if set(data) != {node_name}:
        raise ConfigError(
            f"{config_path.name} must contain only node {node_name!r}"
        )

    node_entry = data[node_name]

    if not isinstance(node_entry, dict):
        raise ConfigError(
            f"node entry must be a mapping in {config_path.name}"
        )

    parameters = node_entry.get("ros__parameters")

    if not isinstance(parameters, dict):
        raise ConfigError(
            f"missing ros__parameters mapping in {config_path.name}"
        )

    return dict(parameters)


def _require_keys(
    parameters: Mapping[str, Any],
    required: set[str],
    context: str,
) -> None:
    missing = sorted(required - set(parameters))

    if missing:
        raise ConfigError(
            f"{context} is missing required parameters: {missing}"
        )


def _require_bool(
    parameters: Mapping[str, Any],
    key: str,
    context: str,
) -> bool:
    value = parameters[key]

    if type(value) is not bool:
        raise ConfigError(f"{context}.{key} must be a boolean")

    return value


def _require_string(
    parameters: Mapping[str, Any],
    key: str,
    context: str,
    *,
    allow_empty: bool = False,
) -> str:
    value = parameters[key]

    if not isinstance(value, str):
        raise ConfigError(f"{context}.{key} must be a string")

    if not allow_empty and not value.strip():
        raise ConfigError(f"{context}.{key} must not be empty")

    return value


def _require_integer(
    parameters: Mapping[str, Any],
    key: str,
    context: str,
    *,
    minimum: int | None = None,
    maximum: int | None = None,
) -> int:
    value = parameters[key]

    if type(value) is not int:
        raise ConfigError(f"{context}.{key} must be an integer")

    if minimum is not None and value < minimum:
        raise ConfigError(
            f"{context}.{key} must be at least {minimum}"
        )

    if maximum is not None and value > maximum:
        raise ConfigError(
            f"{context}.{key} must be at most {maximum}"
        )

    return value


def _require_number(
    parameters: Mapping[str, Any],
    key: str,
    context: str,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
) -> float:
    value = parameters[key]

    if type(value) not in {int, float}:
        raise ConfigError(f"{context}.{key} must be numeric")

    numeric = float(value)

    if minimum is not None and numeric < minimum:
        raise ConfigError(
            f"{context}.{key} must be at least {minimum}"
        )

    if maximum is not None and numeric > maximum:
        raise ConfigError(
            f"{context}.{key} must be at most {maximum}"
        )

    return numeric


def _require_choice(
    parameters: Mapping[str, Any],
    key: str,
    context: str,
    choices: set[str],
) -> str:
    value = _require_string(parameters, key, context)

    if value not in choices:
        raise ConfigError(
            f"{context}.{key} must be one of {sorted(choices)}"
        )

    return value


def _validate_audio(parameters: Mapping[str, Any]) -> None:
    context = "respeaker_audio_node"

    required = {
        "enabled",
        "backend",
        "sample_rate_hz",
        "sample_format",
        "frame_ms",
        "capture.device_hint",
        "capture.alsa_device",
        "capture.device_channels",
        "capture.processed_channel",
        "capture.output_channels",
        "playback.device_hint",
        "playback.alsa_device",
        "playback.channels",
        "buffering.ring_buffer_ms",
        "buffering.pre_roll_ms",
        "reconnect.enabled",
        "reconnect.interval_s",
        "reconnect.max_attempts",
        "dryrun.capture_enabled",
        "dryrun.playback_enabled",
    }

    _require_keys(parameters, required, context)
    _require_bool(parameters, "enabled", context)
    _require_choice(parameters, "backend", context, {"alsa", "dryrun"})

    _require_integer(
        parameters,
        "sample_rate_hz",
        context,
        minimum=8000,
        maximum=48000,
    )

    _require_choice(
        parameters,
        "sample_format",
        context,
        {"S16_LE"},
    )

    frame_ms = _require_integer(
        parameters,
        "frame_ms",
        context,
        minimum=10,
        maximum=30,
    )

    if frame_ms not in {10, 20, 30}:
        raise ConfigError(
            f"{context}.frame_ms must be 10, 20, or 30"
        )

    _require_string(parameters, "capture.device_hint", context)
    _require_string(
        parameters,
        "capture.alsa_device",
        context,
        allow_empty=True,
    )

    device_channels = _require_integer(
        parameters,
        "capture.device_channels",
        context,
        minimum=0,
        maximum=16,
    )

    processed_channel = _require_integer(
        parameters,
        "capture.processed_channel",
        context,
        minimum=0,
        maximum=15,
    )

    if device_channels > 0 and processed_channel >= device_channels:
        raise ConfigError(
            f"{context}.capture.processed_channel must be below "
            "capture.device_channels"
        )

    _require_integer(
        parameters,
        "capture.output_channels",
        context,
        minimum=1,
        maximum=2,
    )

    _require_string(parameters, "playback.device_hint", context)
    _require_string(
        parameters,
        "playback.alsa_device",
        context,
        allow_empty=True,
    )

    _require_integer(
        parameters,
        "playback.channels",
        context,
        minimum=1,
        maximum=2,
    )

    ring_buffer_ms = _require_integer(
        parameters,
        "buffering.ring_buffer_ms",
        context,
        minimum=500,
        maximum=30000,
    )

    pre_roll_ms = _require_integer(
        parameters,
        "buffering.pre_roll_ms",
        context,
        minimum=0,
        maximum=5000,
    )

    if pre_roll_ms >= ring_buffer_ms:
        raise ConfigError(
            f"{context}.buffering.pre_roll_ms must be below "
            "buffering.ring_buffer_ms"
        )

    _require_bool(parameters, "reconnect.enabled", context)

    _require_number(
        parameters,
        "reconnect.interval_s",
        context,
        minimum=0.1,
        maximum=60.0,
    )

    _require_integer(
        parameters,
        "reconnect.max_attempts",
        context,
        minimum=0,
        maximum=1000,
    )

    _require_bool(parameters, "dryrun.capture_enabled", context)
    _require_bool(parameters, "dryrun.playback_enabled", context)


def _validate_speech_manager(parameters: Mapping[str, Any]) -> None:
    context = "speech_manager_node"

    required = {
        "enabled",
        "start_awake",
        "session.idle_timeout_s",
        "session.wake_cooldown_s",
        "utterance.min_duration_s",
        "utterance.max_duration_s",
        "utterance.speech_start_ms",
        "utterance.silence_end_ms",
        "utterance.post_roll_ms",
        "timeout.stt_s",
        "timeout.thinking_s",
        "timeout.tts_s",
        "timeout.playback_s",
        "tts_gate.enabled",
        "tts_gate.release_delay_ms",
        "barge_in.enabled",
        "status.publish_period_s",
        "heartbeat.publish_period_s",
    }

    _require_keys(parameters, required, context)
    _require_bool(parameters, "enabled", context)
    _require_bool(parameters, "start_awake", context)

    _require_number(
        parameters,
        "session.idle_timeout_s",
        context,
        minimum=1.0,
        maximum=3600.0,
    )

    _require_number(
        parameters,
        "session.wake_cooldown_s",
        context,
        minimum=0.0,
        maximum=60.0,
    )

    minimum_duration = _require_number(
        parameters,
        "utterance.min_duration_s",
        context,
        minimum=0.05,
        maximum=10.0,
    )

    maximum_duration = _require_number(
        parameters,
        "utterance.max_duration_s",
        context,
        minimum=0.1,
        maximum=120.0,
    )

    if minimum_duration >= maximum_duration:
        raise ConfigError(
            f"{context} utterance minimum duration must be below maximum"
        )

    _require_integer(
        parameters,
        "utterance.speech_start_ms",
        context,
        minimum=20,
        maximum=3000,
    )

    _require_integer(
        parameters,
        "utterance.silence_end_ms",
        context,
        minimum=100,
        maximum=10000,
    )

    _require_integer(
        parameters,
        "utterance.post_roll_ms",
        context,
        minimum=0,
        maximum=3000,
    )

    for key in (
        "timeout.stt_s",
        "timeout.thinking_s",
        "timeout.tts_s",
        "timeout.playback_s",
        "status.publish_period_s",
        "heartbeat.publish_period_s",
    ):
        _require_number(
            parameters,
            key,
            context,
            minimum=0.1,
            maximum=600.0,
        )

    _require_bool(parameters, "tts_gate.enabled", context)

    _require_integer(
        parameters,
        "tts_gate.release_delay_ms",
        context,
        minimum=0,
        maximum=5000,
    )

    _require_bool(parameters, "barge_in.enabled", context)


def _validate_wake_word(parameters: Mapping[str, Any]) -> None:
    context = "wake_word_service"

    required = {
        "enabled",
        "backend",
        "device",
        "sample_rate_hz",
        "frame_ms",
        "model_root",
        "model_names",
        "wake_phrases",
        "detection.threshold",
        "detection.cooldown_s",
        "detection.max_pending_frames",
        "worker_count",
        "preload_model",
    }

    _require_keys(parameters, required, context)
    _require_bool(parameters, "enabled", context)
    _require_choice(
        parameters,
        "backend",
        context,
        {"openwakeword"},
    )
    _require_choice(parameters, "device", context, {"cpu"})

    _require_integer(
        parameters,
        "sample_rate_hz",
        context,
        minimum=8000,
        maximum=48000,
    )

    _require_integer(
        parameters,
        "frame_ms",
        context,
        minimum=10,
        maximum=100,
    )

    _require_string(parameters, "model_root", context)

    if not isinstance(parameters["model_names"], list):
        raise ConfigError(f"{context}.model_names must be a list")

    phrases = parameters["wake_phrases"]

    if not isinstance(phrases, list) or not phrases:
        raise ConfigError(
            f"{context}.wake_phrases must be a non-empty list"
        )

    normalized_phrases = [
        str(phrase).strip().lower()
        for phrase in phrases
        if str(phrase).strip()
    ]

    if len(normalized_phrases) != len(phrases):
        raise ConfigError(
            f"{context}.wake_phrases cannot contain empty entries"
        )

    if len(normalized_phrases) != len(set(normalized_phrases)):
        raise ConfigError(
            f"{context}.wake_phrases must be unique"
        )

    _require_number(
        parameters,
        "detection.threshold",
        context,
        minimum=0.0,
        maximum=1.0,
    )

    _require_number(
        parameters,
        "detection.cooldown_s",
        context,
        minimum=0.0,
        maximum=60.0,
    )

    _require_integer(
        parameters,
        "detection.max_pending_frames",
        context,
        minimum=1,
        maximum=100,
    )

    _require_integer(
        parameters,
        "worker_count",
        context,
        minimum=1,
        maximum=2,
    )

    _require_bool(parameters, "preload_model", context)


def _validate_local_stt(parameters: Mapping[str, Any]) -> None:
    context = "local_stt_service"

    required = {
        "enabled",
        "backend",
        "model_root",
        "model_id",
        "device",
        "compute_type",
        "cpu_threads",
        "worker_count",
        "preload_model",
        "language",
        "beam_size",
        "vad_filter",
        "max_audio_duration_s",
    }

    _require_keys(parameters, required, context)
    _require_bool(parameters, "enabled", context)
    _require_choice(
        parameters,
        "backend",
        context,
        {"faster_whisper"},
    )
    _require_string(parameters, "model_root", context)
    _require_string(parameters, "model_id", context)
    _require_choice(parameters, "device", context, {"cpu"})

    _require_choice(
        parameters,
        "compute_type",
        context,
        {"int8", "int8_float32", "float32"},
    )

    _require_integer(
        parameters,
        "cpu_threads",
        context,
        minimum=1,
        maximum=16,
    )

    _require_integer(
        parameters,
        "worker_count",
        context,
        minimum=1,
        maximum=2,
    )

    _require_bool(parameters, "preload_model", context)
    _require_string(parameters, "language", context, allow_empty=True)

    _require_integer(
        parameters,
        "beam_size",
        context,
        minimum=1,
        maximum=10,
    )

    _require_bool(parameters, "vad_filter", context)

    _require_number(
        parameters,
        "max_audio_duration_s",
        context,
        minimum=1.0,
        maximum=120.0,
    )


def _validate_local_tts(parameters: Mapping[str, Any]) -> None:
    context = "local_tts_service"

    required = {
        "enabled",
        "backend",
        "executable",
        "model_root",
        "voice_model",
        "voice_config",
        "speaker_id",
        "length_scale",
        "noise_scale",
        "noise_width",
        "output_sample_rate_hz",
        "worker_count",
        "preload_model",
    }

    _require_keys(parameters, required, context)
    _require_bool(parameters, "enabled", context)
    _require_choice(parameters, "backend", context, {"piper"})
    _require_string(parameters, "executable", context)
    _require_string(parameters, "model_root", context)
    _require_string(parameters, "voice_model", context, allow_empty=True)
    _require_string(parameters, "voice_config", context, allow_empty=True)

    _require_integer(
        parameters,
        "speaker_id",
        context,
        minimum=-1,
        maximum=1000,
    )

    _require_number(
        parameters,
        "length_scale",
        context,
        minimum=0.1,
        maximum=5.0,
    )

    _require_number(
        parameters,
        "noise_scale",
        context,
        minimum=0.0,
        maximum=5.0,
    )

    _require_number(
        parameters,
        "noise_width",
        context,
        minimum=0.0,
        maximum=5.0,
    )

    output_rate = _require_integer(
        parameters,
        "output_sample_rate_hz",
        context,
        minimum=0,
        maximum=96000,
    )

    if output_rate not in {0} and output_rate < 8000:
        raise ConfigError(
            f"{context}.output_sample_rate_hz must be 0 or at least 8000"
        )

    _require_integer(
        parameters,
        "worker_count",
        context,
        minimum=1,
        maximum=2,
    )

    _require_bool(parameters, "preload_model", context)


def _validate_savomind(parameters: Mapping[str, Any]) -> None:
    context = "savomind_bridge_node"

    required = {
        "enabled",
        "base_url",
        "provider_policy",
        "allow_local_fallback",
        "source",
        "default_language",
        "timeout.connect_s",
        "timeout.stt_s",
        "timeout.chat_s",
        "timeout.tts_s",
        "retry.max_attempts",
        "retry.backoff_s",
        "session.require_matching_id",
    }

    _require_keys(parameters, required, context)
    _require_bool(parameters, "enabled", context)

    base_url = _require_string(parameters, "base_url", context)

    if not base_url.startswith(("http://", "https://")):
        raise ConfigError(
            f"{context}.base_url must start with http:// or https://"
        )

    _require_choice(
        parameters,
        "provider_policy",
        context,
        {"local_only", "online_first", "online_only"},
    )

    _require_bool(parameters, "allow_local_fallback", context)
    _require_string(parameters, "source", context)
    _require_string(
        parameters,
        "default_language",
        context,
        allow_empty=True,
    )

    for key in (
        "timeout.connect_s",
        "timeout.stt_s",
        "timeout.chat_s",
        "timeout.tts_s",
    ):
        _require_number(
            parameters,
            key,
            context,
            minimum=0.1,
            maximum=600.0,
        )

    _require_integer(
        parameters,
        "retry.max_attempts",
        context,
        minimum=0,
        maximum=10,
    )

    _require_number(
        parameters,
        "retry.backoff_s",
        context,
        minimum=0.0,
        maximum=60.0,
    )

    _require_bool(
        parameters,
        "session.require_matching_id",
        context,
    )


VALIDATORS: dict[str, Callable[[Mapping[str, Any]], None]] = {
    "respeaker_audio_node": _validate_audio,
    "speech_manager_node": _validate_speech_manager,
    "wake_word_service": _validate_wake_word,
    "local_stt_service": _validate_local_stt,
    "local_tts_service": _validate_local_tts,
    "savomind_bridge_node": _validate_savomind,
}


def validate_node_parameters(
    node_name: str,
    parameters: Mapping[str, Any],
) -> None:
    validator = VALIDATORS.get(node_name)

    if validator is None:
        raise ConfigError(f"unknown configuration node: {node_name}")

    validator(parameters)


def load_base_parameters(
    config_dir: str | Path,
) -> dict[str, dict[str, Any]]:
    directory = Path(config_dir)
    result: dict[str, dict[str, Any]] = {}

    for filename, node_name in BASE_CONFIG_NODES.items():
        parameters = extract_ros_parameters(
            directory / filename,
            node_name,
        )

        validate_node_parameters(node_name, parameters)
        result[node_name] = parameters

    return result


def load_profile(
    path: str | Path,
) -> dict[str, Any]:
    profile_path = Path(path)
    data = load_yaml_mapping(profile_path)

    required_keys = {
        "schema_version",
        "profile",
        "description",
        "overrides",
    }

    if set(data) != required_keys:
        raise ConfigError(
            f"{profile_path.name} must contain exactly "
            f"{sorted(required_keys)}"
        )

    if data["schema_version"] != 1:
        raise ConfigError(
            f"{profile_path.name} has unsupported schema version"
        )

    if data["profile"] != profile_path.stem:
        raise ConfigError(
            f"{profile_path.name} profile name must match its filename"
        )

    if not isinstance(data["description"], str):
        raise ConfigError(
            f"{profile_path.name} description must be a string"
        )

    overrides = data["overrides"]

    if not isinstance(overrides, dict):
        raise ConfigError(
            f"{profile_path.name} overrides must be a mapping"
        )

    unknown_nodes = set(overrides) - ALLOWED_PROFILE_NODES

    if unknown_nodes:
        raise ConfigError(
            f"{profile_path.name} contains unknown nodes: "
            f"{sorted(unknown_nodes)}"
        )

    for node_name, node_overrides in overrides.items():
        if not isinstance(node_overrides, dict):
            raise ConfigError(
                f"{profile_path.name} override for {node_name} "
                "must be a mapping"
            )

    return data


def apply_profile(
    base_parameters: Mapping[str, Mapping[str, Any]],
    profile: Mapping[str, Any],
) -> dict[str, dict[str, Any]]:
    merged = deepcopy(dict(base_parameters))
    overrides = profile["overrides"]

    for node_name, node_overrides in overrides.items():
        if node_name not in merged:
            raise ConfigError(
                f"profile overrides unknown base node: {node_name}"
            )

        unknown_keys = set(node_overrides) - set(merged[node_name])

        if unknown_keys:
            raise ConfigError(
                f"profile contains unknown parameters for {node_name}: "
                f"{sorted(unknown_keys)}"
            )

        merged[node_name].update(node_overrides)

    for node_name, parameters in merged.items():
        validate_node_parameters(node_name, parameters)

    return merged


def load_and_apply_profile(
    config_dir: str | Path,
    profile_path: str | Path,
) -> dict[str, dict[str, Any]]:
    base = load_base_parameters(config_dir)
    profile = load_profile(profile_path)
    return apply_profile(base, profile)
