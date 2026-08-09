# savo_speech

## Purpose

Edge physical speech I/O: microphone capture, wake/VAD/utterance handling, bounded SavoMind transport, validated returned WAV playback, and speech health/state.

## Deployment

Edge only and optional at startup (`start_speech=false` in deploy wrapper).

## Responsibilities

ALSA capture/playback, PocketSphinx wake word, adaptive VAD, pre-roll/session bounds, PCM/WAV serialization, protocol-v2 Unix socket, returned 16 kHz mono PCM16 validation, queued/cancelable playback, mic gating, device recovery, and privacy-safe state.

## Non-responsibilities and authority boundaries

Does not perform STT, LLM/agent reasoning, TTS inference, navigation dispatch, UI rendering, safety policy, or motion. SavoMind owns inference.

## Package structure

C++ runtime/library, XML launch, protocol documentation, bench fake server, and audio/device/runtime tests.

## Runtime components

### `savo_speech_node`

Production C++ node owning full capture/session/socket/playback state machine.

### Diagnostic executables

`savo_speech_audio_device_probe`, capture/playback/worker/audio-runtime test binaries, and `fake_savomind_speech_server.py` are bench tools, not inference services.

## Runtime data flow

`ReSpeaker -> wake/VAD -> bounded WAV -> /run/savomind/speech.sock -> transcript/response/TTS WAV -> validation/playback -> completion acknowledgement`.

## ROS interfaces

### Published topics

`/savo_speech/readiness`, `/dashboard`, `/state`, `/transcript`, `/response`, `/playback/state`, `/playback/finished`, `/result` (String), `/heartbeat` (`UInt64`), `/diagnostics` (DiagnosticArray). Raw continuous microphone audio is not published by default.

### Subscribed topics

No ROS command topic is required by the production speech session; cancellation/gating are internal/protocol/configured runtime controls.

### Services

No public ROS service.

### Actions

No ROS action.

## TF ownership

None.

## Parameters and configuration

Important bounds include SavoMind endpoint `/run/savomind/speech.sock`, connect 1500 ms, I/O 30000 ms, playback 60000 ms, maximum text 8192 bytes, optional peer-UID verification (default off, UID 10001), plus configured ALSA devices, sample format, wake/VAD/session/queue limits.

## Launch files

`speech_bringup.launch.xml` starts production node/config.

## Persistent state and runtime files

Unix socket is SavoMind-owned runtime endpoint; utterance/TTS data is bounded in memory and not a persistent transcript database.

## Hardware ownership

Edge ReSpeaker microphone array/audio input and speaker playback device.

## Dependencies

### Internal Robot Savo dependencies

Observed by bringup, supervisor, UI, bridge; no direct control/nav dependency.

### External ROS/system dependencies

ALSA, PocketSphinx/SphinxBase, Unix sockets/peer credentials, diagnostic/std messages.

## Safety behavior

Missing/wrong server, malformed/oversized/mismatched response, invalid WAV, timeout, or cancellation fails closed. Mic is gated during playback. Speech cannot directly dispatch motors or bypass bridge/Core authority.

## Failure and degraded behavior

Audio/socket failure makes speech unavailable while robot STOP and other control remain usable. Device recovery is bounded and state-visible.

## Startup and shutdown behavior

Optional start; probes/opens devices and endpoint, reports readiness, cancels I/O/playback and closes devices on shutdown.

## Build

`bash deploy/edge/build_edge.sh --clean --test`.

## Run

`ros2 launch savo_speech speech_bringup.launch.xml`.

## Validation and testing

Protocol, WAV, queues, framing/correlation, credentials, timeout/cancel, mic gating, audio runtime, and dated cross-repository smoke tests.

## Current validation status

Implemented/source-validated with protocol-v2 smoke evidence; ReSpeaker/speaker wake/VAD/playback/recovery/latency require current Edge hardware validation.

## Known limitations and remaining validation

Device names, acoustics, wake/VAD thresholds, echo/feedback, and peer-UID deployment policy remain hardware/environment dependent.

## Change-control considerations

Protocol/schema, endpoint/credentials, audio format, limits, privacy outputs, or navigation acknowledgement semantics require SavoMind compatibility tests.

## Related documentation

- [Implementation README](../../savo_ws/src/edge/savo_speech/README.md)
- [Speech flow](../architecture/speech_intent_flow.md)
- [Speech test plan](../testing/speech_test_plan.md)
- [Audio setup](../setup/audio_setup.md)
