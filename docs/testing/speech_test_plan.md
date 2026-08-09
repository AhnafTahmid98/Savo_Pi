# Speech Test Plan

## Objective

Verify Edge physical speech I/O, wake/VAD/session bounding, WAV serialization, correlated SavoMind protocol-v2 transport, validated/cancelable playback, microphone gating and honest health. STT/LLM/TTS inference belongs to SavoMind, not `savo_speech`.

## Scope

ALSA discovery/capture/playback, wake word, VAD/segmentation, request/session identity, Unix socket bounds/credentials/timeouts, malformed responses, WAV validation, playback queue/cancel/mic gate/echo prevention, SavoMind/socket loss and recovery.

## Test ownership

Speech maintainer owns SPH-001–006; Edge audio/operator/privacy reviewer owns SPH-007–010; SavoMind owner participates in SPH-006/010.

## Safety classification

All tests are `STATIC`, `UNIT`, `PC`, `TARGET-NON-HARDWARE`, `HARDWARE-NON-ACTUATING`, `INTEGRATION`, `FAULT-INJECTION`, or `RECOVERY` / `NO-MOTION`.

## Preconditions

Edge safe-idle and robot STOP; approved ALSA aliases/devices; low initial speaker volume; representative rooms; privacy approval; production `/run/savomind/speech.sock` owner/group/peer policy recorded.

## Required hardware

ReSpeaker capture device, speaker/output path and Edge Pi. No drivetrain actuation is required.

## Required software / configuration

ALSA/PocketSphinx, speech real-robot profile, protocol-v2 docs, fake server for bench only, and production SavoMind endpoint for final integration.

## Interfaces under test

Speech readiness/dashboard/state/transcript/response/playback/result/heartbeat/diagnostics; protocol-v2 Unix socket and PCM/WAV payloads. No ROS command service/action or inference ownership.

## Test stages

| Test ID | Stage / class | Verification and acceptance |
| --- | --- | --- |
| SPH-001 | T0 `STATIC` | Verify audio format/devices, wake/VAD/session/queue/protocol bounds, endpoint/timeouts/optional UID policy, privacy outputs, mic-gate and no motion/inference dispatch. |
| SPH-002 | T1 `UNIT`/`PC` | Build/tests pass for ALSA configs, buffers/queues, wake/VAD/utterance, WAV, framing/correlation/credentials/timeouts, transport, playback/cancel/mic gate, health and round-trip contract. |
| SPH-003 | T2 `TARGET-NON-HARDWARE` | Use the package fake server at an explicitly overridden test socket; test lifecycle, request/session correlation, valid mono PCM16 16 kHz return, playback token/ack and cleanup. Fake-server PASS is not production inference evidence. |
| SPH-004 | T6 `FAULT-INJECTION` | Bench-test missing socket/server, wrong peer where enabled, malformed/oversized/mismatched/late response, invalid WAV/text, timeout, disconnect and unbounded-retry prevention; all fail closed. |
| SPH-005 | T2/T3 `HARDWARE-NON-ACTUATING` | Enumerate/open intended capture/playback devices; record sample format/channels/rate, levels/clipping, clean close and device contention. |
| SPH-006 | T5 `INTEGRATION` | Against the exact production SavoMind protocol/version, verify bounded request/session identity, transcript/response correlation, returned WAV validation, playback acknowledgement and that inference remains external. |
| SPH-007 | T4 `HARDWARE-NON-ACTUATING` | Validate wake word, VAD, pre-roll/utterance segmentation/session timeout in approved quiet/noisy/accent/distance cases. If no formal threshold exists, record measurements and leave acceptance BLOCKED. |
| SPH-008 | T4 `HARDWARE-NON-ACTUATING` | Validate playback queue/order, cancel at each state, volume bounds, mic gated during playback, no echo/feedback loop and privacy-safe ROS output. |
| SPH-009 | T6/T7 `FAULT-INJECTION`/`RECOVERY` | Disconnect/reconnect mic/speaker/socket/SavoMind and restart speech; state/readiness becomes unavailable, I/O/playback cancels boundedly, fresh devices/session are required after recovery. |
| SPH-010 | T5 `INTEGRATION` | UI/bridge/supervisor observe fresh speech state; pending navigation acknowledgement, if selected, remains correlated and still passes bridge/Core admission. Speech never dispatches motor/nav commands directly. |

## Pass criteria

Intended devices are stable; wake/VAD/session are bounded; every response matches request/session and valid WAV constraints; playback/cancel/mic gate prevent feedback; failures/restart are honest and bounded; inference/authority remain external.

## Blocked criteria

Missing ALSA alias/device/privacy approval, SavoMind protocol endpoint/version unavailable, peer policy unresolved, or wake/VAD/latency/accuracy acceptance thresholds not approved.

## Failure criteria

Wrong device, unbounded capture/session/retry, mismatched/invalid response played, cancellation ignored, mic active causing feedback, raw/private audio exposed, false readiness, or direct robot command path.

## Abort criteria

Mute/cancel/stop on acoustic feedback, clipping, unsafe volume, privacy breach, runaway retry, playback that cannot stop, device heat/electrical issue, or unexpected robot command.

## Evidence to retain

Commit/profile/device list/audio metadata (not content unless approved), protocol/server version, correlation IDs/state/latency/reason codes, malformed/timeout cases, wake/VAD measurement table, playback/cancel/mic-gate evidence, restart logs and reviewer.

## Regression triggers

Audio hardware/alias/format, wake/VAD/session/queue/volume, endpoint/credentials/protocol/schema/limits/timeouts, WAV validation/playback/cancel/mic gate, SavoMind model/provider behavior, UI/bridge acknowledgement.

## Current validation status

Automated protocol/audio-runtime coverage and prior protocol-v2 smoke evidence exist. Current ALSA aliases/devices, acoustics, wake/VAD, feedback, latency, recovery and production SavoMind round-trip require Edge validation.

## Related documentation

- [Speech package](../packages/savo_speech.md)
- [Speech flow](../architecture/speech_intent_flow.md)
- [Audio setup](../setup/audio_setup.md)
- [Bridge plan](bridge_test_plan.md)
