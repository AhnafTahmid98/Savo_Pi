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

### Phase 4G

- Added the unified physical audio runtime coordinator.
- Added coordinated capture and playback worker startup.
- Added playback-first startup with capture-failure rollback.
- Added shutdown microphone gating and deterministic shutdown order.
- Added unified captured-frame and playback-completion routing.
- Added worker-fault propagation to runtime health.
- Added complete runtime health and statistics snapshots.
- Added runtime restart support.
- Added a real simultaneous ALSA capture/playback diagnostic.

### Phase 4H

- Integrated the unified AudioRuntime into the production ROS node.
- Added production ALSA capture and playback parameter loading.
- Replaced the Phase 1 simulated waiting-for-audio state.
- Added real audio readiness and worker-fault propagation.
- Added clean node-owned audio startup and shutdown.
- Added runtime capture, playback, queue, gate, and fault diagnostics.
- Added real audio-runtime data to the dashboard topic.
- Preserved the existing readiness, dashboard, heartbeat, and diagnostics contracts.

### Phase 4I-A

- Added the captured-frame source abstraction.
- Made AudioRuntime a captured-frame source.
- Added the captured-audio processor interface.
- Added the real-time audio activity monitor.
- Added the continuous capture-processing dispatcher.
- Added processing freshness and sequence continuity monitoring.
- Added source, processor, timeout, and fault statistics.
- Added deterministic processing and stale-frame tests.

### Phase 4I-B

- Integrated the capture-processing dispatcher into SpeechNode.
- Added node-owned audio activity monitoring.
- Added processing readiness and freshness to package readiness.
- Added processing-aware microphone-gate handling.
- Added deterministic processing-before-audio shutdown ordering.
- Added live RMS, peak, clipping, sequence, and freshness dashboard data.
- Added the savo_speech/processing diagnostic entry.
- Eliminated the permanently undrained production capture queue.

### Phase 4J-A

- Added the deterministic captured-audio processor chain.
- Added immutable processor registration through chain sealing.
- Added unique processor-name validation.
- Added required and optional processor failure policies.
- Added ordered same-frame delivery to every registered processor.
- Added chain-level and per-processor runtime statistics.
- Added per-processor failure attribution and last-error reporting.
- Added processor-chain statistics reset without registration loss.

### Phase 4J-B

- Integrated CapturedAudioProcessorChain into SpeechNode.
- Registered AudioActivityMonitor as the first required processor.
- Sealed the production processor chain before dispatcher startup.
- Routed all captured mono frames through the processor chain.
- Added processor-chain statistics to the dashboard.
- Added chain-level and per-processor diagnostic values.
- Added deterministic dispatcher, chain, processor, runtime, and ALSA shutdown ordering.

### Phase 4J-B

- Integrated CapturedAudioProcessorChain into SpeechNode.
- Registered AudioActivityMonitor as the first required processor.
- Sealed the processor chain before capture-dispatcher startup.
- Routed captured mono audio through the production processor chain.
- Added chain-level and per-processor dashboard and diagnostic statistics.
- Added deterministic processor-chain shutdown ownership.

### Phase 4K-A

- Added the backend-independent wake-word processor.
- Added the wake-word backend interface and result contract.
- Added bounded wake-word event delivery.
- Added confidence thresholding and consecutive-frame debounce.
- Added phrase-consistency validation.
- Added configurable detection cooldown.
- Added queue-overflow and dropped-event accounting.
- Added backend failure propagation and diagnostics.

### Phase 4K-B

- Added the ARM64-compatible PocketSphinx wake-word backend.
- Added pkg-config architecture-independent dependency discovery.
- Added configurable acoustic-model, dictionary, and keyword-file paths.
- Added continuous 16 kHz mono S16_LE decoder processing.
- Added named keyword-search activation.
- Added hypothesis normalization and decoder restart after detection.
- Added decoder state, score, failure, reset, and detection statistics.
- Added deterministic configuration and filesystem validation tests.

### Phase 4K-C

- Added packaged PocketSphinx wake-word pronunciation dictionary.
- Added default English and Finnish Savo wake phrases.
- Added extended Savo, Sabo, Robo, and robot-address aliases.
- Added alternative pronunciations for accent tolerance.
- Added independent thresholds for each keyword phrase.
- Added stricter initial thresholds for short single-word aliases.
- Added install rules for wake-word runtime assets.
- Added deterministic dictionary and keyword-file validation tests.
- Added a real PocketSphinx decoder initialization test using packaged assets.

### Phase 4K-D

- Added architecture-independent wake-word asset resolution.
- Added default, extended, and custom wake-word profile selection.
- Added installed ROS package-share asset discovery.
- Added PocketSphinx and WakeWordProcessor ownership to SpeechNode.
- Registered wake-word processing before sealing the captured-audio chain.
- Added required and optional wake-word initialization policies.
- Added configurable confidence, debounce, cooldown, and event queue settings.
- Added wake-word dashboard state and dedicated diagnostics.
- Added safe processor, backend, chain, dispatcher, and audio shutdown ordering.
- Enabled the safer default wake-word profile in the production edge profile.
- Added deterministic wake-word asset resolver tests.
