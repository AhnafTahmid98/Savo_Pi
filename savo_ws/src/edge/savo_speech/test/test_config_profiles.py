from pathlib import Path

import pytest
import yaml

from savo_speech.ros.param_utils import (
    ConfigError,
    apply_profile,
    load_and_apply_profile,
    load_base_parameters,
    load_profile,
)


ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "config"
PROFILE_DIR = CONFIG_DIR / "profiles"


def test_all_deployment_profiles_are_valid():
    profile_paths = sorted(PROFILE_DIR.glob("*.yaml"))

    assert profile_paths

    for path in profile_paths:
        merged = load_and_apply_profile(CONFIG_DIR, path)

        assert "respeaker_audio_node" in merged
        assert "speech_manager_node" in merged


def test_dryrun_profile_never_enables_external_ai():
    merged = load_and_apply_profile(
        CONFIG_DIR,
        PROFILE_DIR / "dryrun_no_hardware.yaml",
    )

    assert merged["respeaker_audio_node"]["backend"] == "dryrun"
    assert merged["respeaker_audio_node"]["enabled"] is True

    assert merged["wake_word_service"]["enabled"] is False
    assert merged["local_stt_service"]["enabled"] is False
    assert merged["local_tts_service"]["enabled"] is False
    assert merged["savomind_bridge_node"]["enabled"] is False


def test_bench_profile_owns_only_real_audio():
    merged = load_and_apply_profile(
        CONFIG_DIR,
        PROFILE_DIR / "bench_respeaker_audio.yaml",
    )

    assert merged["respeaker_audio_node"]["backend"] == "alsa"
    assert merged["respeaker_audio_node"]["enabled"] is True
    assert merged["speech_manager_node"]["enabled"] is False
    assert merged["savomind_bridge_node"]["enabled"] is False


def test_online_profile_does_not_require_local_models():
    merged = load_and_apply_profile(
        CONFIG_DIR,
        PROFILE_DIR / "robot_savo_online_first.yaml",
    )

    assert merged["savomind_bridge_node"]["enabled"] is True
    assert (
        merged["savomind_bridge_node"]["provider_policy"]
        == "online_first"
    )

    assert merged["local_stt_service"]["enabled"] is False
    assert merged["local_tts_service"]["enabled"] is False


def test_full_hybrid_profile_enables_local_fallback():
    merged = load_and_apply_profile(
        CONFIG_DIR,
        PROFILE_DIR / "robot_savo_full_hybrid.yaml",
    )

    assert merged["local_stt_service"]["enabled"] is True
    assert merged["local_tts_service"]["enabled"] is True
    assert (
        merged["savomind_bridge_node"]["allow_local_fallback"]
        is True
    )


def test_unknown_profile_parameter_is_rejected():
    base = load_base_parameters(CONFIG_DIR)
    profile = load_profile(
        PROFILE_DIR / "dryrun_no_hardware.yaml"
    )

    profile["overrides"]["respeaker_audio_node"][
        "unknown.parameter"
    ] = True

    with pytest.raises(ConfigError):
        apply_profile(base, profile)


def test_profile_name_must_match_filename(tmp_path):
    source = PROFILE_DIR / "dryrun_no_hardware.yaml"

    with source.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    data["profile"] = "wrong_name"

    target = tmp_path / "temporary_profile.yaml"

    with target.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream)

    with pytest.raises(ConfigError):
        load_profile(target)
