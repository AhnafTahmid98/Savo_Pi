from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_dryrun_cpp_nodes_exist():
    required = [
        "src/respeaker_audio_node.cpp",
        "src/speech_manager_node.cpp",
        "include/savo_speech/qos_profiles.hpp",
    ]

    for relative_path in required:
        assert (ROOT / relative_path).is_file()


def test_cmake_registers_cpp_nodes():
    text = read_text("CMakeLists.txt")

    assert "add_executable(respeaker_audio_node" in text
    assert "add_executable(speech_manager_node" in text

    assert "src/respeaker_audio_node.cpp" in text
    assert "src/speech_manager_node.cpp" in text

    assert "DESTINATION lib/${PROJECT_NAME}" in text


def test_audio_node_owns_only_audio_services():
    text = read_text("src/respeaker_audio_node.cpp")

    assert "SERVICE_MUTE_INPUT" in text
    assert "SERVICE_MUTE_OUTPUT" in text
    assert "SERVICE_RELOAD_AUDIO_DEVICE" in text

    assert "SERVICE_WAKE" not in text
    assert "SERVICE_START_LISTENING" not in text
    assert "SERVICE_CANCEL" not in text


def test_manager_owns_session_services():
    text = read_text("src/speech_manager_node.cpp")

    assert "SERVICE_WAKE" in text
    assert "SERVICE_SLEEP" in text
    assert "SERVICE_START_LISTENING" in text
    assert "SERVICE_STOP_LISTENING" in text
    assert "SERVICE_CANCEL" in text

    assert "SERVICE_MUTE_INPUT" not in text
    assert "SERVICE_MUTE_OUTPUT" not in text
    assert "SERVICE_RELOAD_AUDIO_DEVICE" not in text


def test_phase_zero_contains_no_real_audio_or_ai_runtime():
    combined = "\n".join(
        [
            read_text("src/respeaker_audio_node.cpp"),
            read_text("src/speech_manager_node.cpp"),
        ]
    ).lower()

    forbidden = [
        "alsa/asoundlib.h",
        "sounddevice",
        "faster_whisper",
        "piper",
        "openwakeword",
        "requests.post",
        "httpx",
    ]

    for token in forbidden:
        assert token not in combined


def test_cpp_nodes_never_publish_motion_commands():
    combined = "\n".join(
        [
            read_text("src/respeaker_audio_node.cpp"),
            read_text("src/speech_manager_node.cpp"),
        ]
    )

    forbidden = [
        '"/cmd_vel"',
        '"/cmd_vel_manual"',
        '"/cmd_vel_auto"',
        '"/cmd_vel_nav"',
        '"/cmd_vel_recovery"',
        '"/cmd_vel_safe"',
    ]

    for topic in forbidden:
        assert topic not in combined
