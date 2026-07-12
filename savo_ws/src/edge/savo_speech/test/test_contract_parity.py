from pathlib import Path
import re

from savo_speech import constants as c
from savo_speech.interfaces.speech_states import (
    ProviderState,
    SpeechEvent,
    SpeechState,
    WakeState,
)


ROOT = Path(__file__).resolve().parents[1]


def read_text(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def extract_cpp_array(text: str, array_name: str) -> tuple[str, ...]:
    pattern = rf"{array_name}\s*=\s*\{{(?P<body>.*?)\}};"
    match = re.search(pattern, text, flags=re.DOTALL)

    assert match is not None
    return tuple(re.findall(r'"([A-Z0-9_]+)"', match.group("body")))


def extract_cpp_ros_names(
    text: str,
) -> tuple[tuple[str, ...], tuple[str, ...]]:
    entries = dict(
        re.findall(
            (
                r"inline constexpr std::string_view\s+"
                r"([A-Z0-9_]+)\s*=\s*\"([^\"]+)\";"
            ),
            text,
        )
    )

    topics = tuple(
        value
        for key, value in entries.items()
        if not key.startswith("SERVICE_")
    )

    services = tuple(
        value
        for key, value in entries.items()
        if key.startswith("SERVICE_")
    )

    return topics, services


def test_cpp_and_python_state_values_match():
    text = read_text("include/savo_speech/speech_types.hpp")

    assert (
        extract_cpp_array(text, "SPEECH_STATE_NAMES")
        == SpeechState.values()
    )

    assert (
        extract_cpp_array(text, "SPEECH_EVENT_NAMES")
        == SpeechEvent.values()
    )

    assert (
        extract_cpp_array(text, "WAKE_STATE_NAMES")
        == WakeState.values()
    )

    assert (
        extract_cpp_array(text, "PROVIDER_STATE_NAMES")
        == ProviderState.values()
    )


def test_cpp_and_python_ros_names_match():
    text = read_text("include/savo_speech/topic_names.hpp")
    cpp_topics, cpp_services = extract_cpp_ros_names(text)

    assert cpp_topics == c.PUBLIC_TOPICS
    assert cpp_services == c.PUBLIC_SERVICES
