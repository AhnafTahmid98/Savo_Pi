from savo_speech.interfaces.speech_states import (
    ProviderState,
    SpeechEvent,
    SpeechState,
    WakeState,
    is_busy_state,
    is_capture_state,
    is_output_state,
    parse_provider_state,
    parse_speech_event,
    parse_speech_state,
    parse_wake_state,
)


def test_speech_state_values_are_stable():
    assert SpeechState.values() == (
        "STARTING",
        "SLEEPING",
        "WAKE_DETECTED",
        "AWAKE_IDLE",
        "LISTENING",
        "UTTERANCE_READY",
        "TRANSCRIBING",
        "THINKING",
        "SYNTHESIZING",
        "SPEAKING",
        "ERROR",
    )


def test_event_wake_and_provider_values_are_unique():
    for enum_type in (SpeechEvent, WakeState, ProviderState):
        values = enum_type.values()
        assert len(values) == len(set(values))


def test_parsers_normalize_tokens():
    assert parse_speech_state("awake idle") == SpeechState.AWAKE_IDLE
    assert (
        parse_speech_event("playback-finished")
        == SpeechEvent.PLAYBACK_FINISHED
    )
    assert parse_wake_state("cooldown") == WakeState.COOLDOWN
    assert parse_provider_state("degraded") == ProviderState.DEGRADED


def test_parsers_use_safe_fallbacks():
    assert parse_speech_state("invalid") == SpeechState.ERROR
    assert parse_speech_event("invalid") == SpeechEvent.FAILURE
    assert parse_wake_state("invalid") == WakeState.ERROR
    assert parse_provider_state("invalid") == ProviderState.ERROR


def test_state_categories():
    assert is_busy_state(SpeechState.LISTENING)
    assert is_busy_state(SpeechState.SPEAKING)
    assert not is_busy_state(SpeechState.SLEEPING)

    assert is_capture_state(SpeechState.LISTENING)
    assert is_capture_state(SpeechState.UTTERANCE_READY)
    assert not is_capture_state(SpeechState.TRANSCRIBING)

    assert is_output_state(SpeechState.SYNTHESIZING)
    assert is_output_state(SpeechState.SPEAKING)
    assert not is_output_state(SpeechState.THINKING)
