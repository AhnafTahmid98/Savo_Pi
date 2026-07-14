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

## Current implementation phase

Phase 1 provides the package foundation, startup configuration,
readiness, dashboard, heartbeat, and diagnostics.

Audio capture and playback are not implemented yet, so the node
intentionally reports:

```text
waiting_for_audio
```
