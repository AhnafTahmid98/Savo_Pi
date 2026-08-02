# savo_speech

ROS 2 Jazzy C++ package responsible for Robot Savo's physical speech interface.

## Ownership

`savo_speech` owns:

- ReSpeaker microphone capture
- ReSpeaker playback through the 3.5 mm speaker output
- wake-word detection
- voice activity detection
- utterance recording
- PCM and WAV handling
- audio transport to and from SavoMind
- microphone gating during playback
- playback completion events
- speech runtime state
- audio-device and SavoMind connection health
- ROS diagnostics

`savo_speech` does not own:

- STT inference or provider routing
- LLM inference or provider routing
- transcript reasoning
- intent classification
- agent routing
- TTS inference or provider routing
- navigation or motor commands
- safety-policy decisions
- UI rendering

## Runtime

- ROS 2 Jazzy
- C++20
- `ament_cmake`
- native execution on `savo-edge`
- ALSA audio backend
- HTTP communication with SavoMind

## Current implementation status

The C++ runtime implements ALSA capture/playback workers, wake-word and VAD
processing, bounded utterance sessions, WAV serialization, playback queueing,
microphone gating, diagnostics, and privacy-safe transcript/response topics.

The inspected `~/SavoMind` API is not yet a complete robot round-trip contract:
`POST /speech/stt` accepts only a shared-host `audio_path`; `POST /chat` returns
correlated response text; `POST /speech/tts` deliberately serializes with
`include_audio=False` and sets `audio_path=None`. Therefore Robot SAVO cannot
retrieve validated TTS bytes for the C++ playback queue. The node continues to
report `savomind_initialized=false` and must not claim end-to-end readiness.

`savo_speech_transport_core` now defines the bounded/cancelable transport
boundary and validates request IDs plus PCM WAV sample rate, channel count, and
sample width. It is intentionally not wired to an invented HTTP protocol. The
external SavoMind contract must add a bounded audio response (or an authenticated
temporary-file handoff) before the runtime can safely complete the round trip.
