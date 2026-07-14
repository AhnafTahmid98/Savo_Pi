# Changelog

## 0.1.0 - 2026-07-14

- Rebuilt `savo_speech` as a clean C++20 ROS 2 package.
- Locked physical audio and SavoMind transport ownership.
- Added package version and runtime constants.
- Added speech-phase and speech-error contracts.
- Added readiness, dashboard, heartbeat, and diagnostics topics.
- Added the `edge_real_robot_v1` profile.
- Added the XML production launch file.

### Phase 2A

- Added native ALSA card and PCM enumeration.
- Added capture and playback endpoint detection.
- Added channel, sample-rate, and PCM-format capability inspection.
- Added stable card-ID-based ALSA device names.
- Added `savo_speech_audio_device_probe`.

### Phase 3B

- Added a thread-safe fixed-capacity PCM ring buffer.
- Added chronological pre-roll snapshots.
- Added overwrite and lifetime sample statistics.
- Added a thread-safe bounded audio-frame queue.
- Added reject-newest and drop-oldest overflow policies.
- Added queue closure, draining, timeout, and runtime statistics.

### Phase 3C

- Added the complete in-memory PCM audio-buffer contract.
- Added strict RIFF/WAVE S16_LE encoding.
- Added strict RIFF/WAVE decoding and validation.
- Added mono and multichannel WAV support.
- Added file-size and audio-data-size safety limits.
- Added malformed, truncated, unsupported, and misaligned WAV rejection.
- Added optional diagnostic WAV file reading and writing.

### Phase 4A

- Added the native ALSA persistent capture-stream implementation.
- Added exact sample-rate and channel negotiation.
- Added configurable ALSA period and buffer sizing.
- Added bounded XRUN and suspended-stream recovery.
- Added capture statistics and frame sequencing.
- Added the real PCM-to-WAV capture diagnostic executable.

### Phase 4B

- Added the native persistent ALSA playback stream.
- Added exact WAV-to-device format negotiation.
- Added partial-write handling and playback completion draining.
- Added bounded XRUN and suspended-stream recovery.
- Added reusable stream preparation after drain.
- Added immediate playback cancellation through ALSA drop.
- Added the C++ WAV playback diagnostic executable.

### Phase 4C

- Added deterministic microphone gating during playback.
- Added configurable post-playback echo suppression hold.
- Added manual and shutdown microphone-gate overrides.
- Added period-sized playback chunking.
- Added bounded-latency playback cancellation between chunks.
- Added automatic ALSA stop on cancellation or playback failure.
- Integrated chunked playback into the native playback diagnostic.

### Phase 4D

- Added echo-safe captured-frame routing.
- Added configured multichannel-to-mono channel selection.
- Connected microphone gating to captured audio processing.
- Added automatic queue and pre-roll flushing on gate entry.
- Added bounded processing-queue overflow handling.
- Added mono pre-roll snapshots for future wake-word and VAD use.
- Added capture-pipeline statistics and deterministic tests.

### Phase 4E

- Added the continuous C++20 capture worker.
- Added clean start, stop, restart, and destructor shutdown.
- Added stop-token-aware continuous frame acquisition.
- Added capture-pipeline result accounting.
- Added capture fault state and last-error reporting.
- Added automatic source closure after runtime failure.
- Added deterministic fake-source lifecycle and fault tests.

### Phase 4F

- Added the asynchronous playback worker.
- Added bounded FIFO playback-request queuing.
- Added unique playback request identifiers.
- Added current, pending, and all-request cancellation.
- Added completed, cancelled, and failed completion events.
- Added playback-worker lifecycle, statistics, and fault state.
- Added automatic pending-request cancellation after playback faults.
- Added the real asynchronous ALSA playback diagnostic.
