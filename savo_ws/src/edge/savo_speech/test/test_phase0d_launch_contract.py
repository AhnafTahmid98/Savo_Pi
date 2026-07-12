from pathlib import Path

import pytest

from savo_speech.ros.launch_config import (
    LaunchConfigError,
    PHASE0D_SUPPORTED_PROFILES,
    build_phase0d_runtime_config,
    normalize_profile_name,
)


ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "config"


def read_text(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_phase0d_supports_only_safe_dryrun_profile():
    assert PHASE0D_SUPPORTED_PROFILES == {
        "dryrun_no_hardware",
    }


def test_dryrun_runtime_configuration_is_safe():
    runtime = build_phase0d_runtime_config(
        CONFIG_DIR,
        "dryrun_no_hardware",
    )

    assert runtime["respeaker_audio_node"]["enabled"] is True
    assert runtime["respeaker_audio_node"]["backend"] == "dryrun"
    assert runtime["speech_manager_node"]["enabled"] is True

    assert runtime["wake_word_service"]["enabled"] is False
    assert runtime["local_stt_service"]["enabled"] is False
    assert runtime["local_tts_service"]["enabled"] is False
    assert runtime["savomind_bridge_node"]["enabled"] is False


def test_hardware_and_ai_profiles_are_rejected_in_phase0d():
    for profile in (
        "bench_respeaker_audio",
        "robot_savo_online_first",
        "robot_savo_full_hybrid",
    ):
        with pytest.raises(LaunchConfigError):
            build_phase0d_runtime_config(
                CONFIG_DIR,
                profile,
            )


def test_profile_path_traversal_is_rejected():
    for invalid in (
        "../dryrun_no_hardware",
        "profiles/dryrun_no_hardware",
        r"profiles\dryrun_no_hardware",
        "",
    ):
        with pytest.raises(LaunchConfigError):
            normalize_profile_name(invalid)


def test_yaml_suffix_is_normalized():
    assert (
        normalize_profile_name("dryrun_no_hardware.yaml")
        == "dryrun_no_hardware"
    )


def test_bringup_launch_starts_only_cpp_runtime_nodes():
    text = read_text("launch/speech_bringup.launch.py")

    assert 'executable="respeaker_audio_node"' in text
    assert 'executable="speech_manager_node"' in text
    assert "build_phase0d_runtime_config" in text
    assert "OnProcessExit" in text
    assert "Shutdown" in text

    assert "wake_word_service" not in text
    assert "local_stt_service" not in text
    assert "local_tts_service" not in text
    assert "savomind_bridge_node" not in text


def test_dryrun_launch_delegates_to_bringup():
    text = read_text("launch/speech_dryrun.launch.py")

    assert "IncludeLaunchDescription" in text
    assert "speech_bringup.launch.py" in text
    assert '"profile": "dryrun_no_hardware"' in text


def test_runtime_checker_covers_every_public_service():
    text = read_text("scripts/speech_dryrun_check.py")

    expected = (
        "/savo_speech/wake",
        "/savo_speech/sleep",
        "/savo_speech/start_listening",
        "/savo_speech/stop_listening",
        "/savo_speech/cancel",
        "/savo_speech/mute_input",
        "/savo_speech/mute_output",
        "/savo_speech/reload_audio_device",
    )

    for service in expected:
        assert service in text


def test_cmake_installs_runtime_checker():
    text = read_text("CMakeLists.txt")

    assert "scripts/speech_dryrun_check.py" in text
    assert "RENAME speech_dryrun_check" in text
