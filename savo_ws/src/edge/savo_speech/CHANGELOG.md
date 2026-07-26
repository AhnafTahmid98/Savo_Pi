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

### Phase 4L-A

- Added a hardware-independent VAD backend contract.
- Added normalized speech-score backend results.
- Added silence and speech processor states.
- Added independent speech-start and speech-end thresholds.
- Added threshold hysteresis and transition debounce.
- Added stable speech-segment identifiers.
- Added speech-start and speech-end transition events.
- Added a bounded drop-oldest VAD event queue.
- Added backend exception and invalid-result handling.
- Added VAD state, transition, queue, sequence, and failure statistics.
- Added deterministic reset behavior.
- Added hardware-independent VAD processor unit tests.

### Phase 4L-B

- Added a lightweight adaptive-energy production VAD backend.
- Reused the existing normalized RMS, peak, and clipping calculations.
- Added configurable startup noise-floor calibration.
- Added adaptive exponential background-noise tracking.
- Added minimum absolute speech-energy gating.
- Added SNR-based normalized speech scoring.
- Added noise-floor freezing during probable speech.
- Added mono and sample-rate validation.
- Added calibration, noise update, freeze, clipping, sequence, and failure statistics.
- Added deterministic reset and state snapshots.
- Added architecture-independent adaptive-energy VAD tests.

### Phase 4L-C

- Integrated the adaptive-energy VAD backend into SpeechNode.
- Added VAD backend and processor ROS parameters.
- Added centralized backend and processor configuration validation.
- Added required and optional VAD initialization behavior.
- Registered VAD in the captured-audio processor chain.
- Added explicit processor-before-backend shutdown ordering.
- Added VAD dashboard state, levels, scoring, and event metrics.
- Added dedicated savo_speech/vad diagnostics.
- Added configurable Raspberry Pi and ReSpeaker production defaults.

### Phase 4M-A

- Added a hardware-independent utterance-session state machine.
- Added idle, armed, and recording states.
- Added wake-word event arming and replay rejection.
- Added VAD speech-start and speech-end transition handling.
- Added bounded chronological pre-roll capture.
- Added stable utterance identifiers and wake/VAD metadata.
- Added delayed speech-end handling until matching audio arrives.
- Added speech-start and maximum-duration timeouts.
- Added explicit cancellation and no-audio cancellation.
- Added audio sequence, timestamp, format, and discontinuity validation.
- Added completed utterance gap and missing-frame metadata.
- Added a bounded drop-oldest completed-utterance queue.
- Added deterministic reset, snapshot, polling, and timed-wait behavior.
- Added hardware-independent utterance-session unit tests.

### Phase 4M-B

- Added a hardware-independent utterance-session processor bridge.
- Connected WakeWordProcessor and VadProcessor event queues to UtteranceSessionCore.
- Locked per-frame ordering as audio, wake events, VAD events, then time advancement.
- Added same-frame wake-word and VAD speech-start handling.
- Added same-frame speech-end audio inclusion before finalization.
- Added deterministic timeout-boundary event precedence.
- Added FIFO draining of all queued wake-word and VAD events.
- Added accepted and rejected event statistics.
- Added frame success, failure, and last-error statistics.
- Added completed-utterance polling and timed-wait forwarding.
- Added explicit cancellation forwarding.
- Added reset behavior that discards pre-reset queued events.
- Added upstream pending-event and session-state snapshots.
- Added hardware-independent processor-bridge unit tests.

### Phase 4M-C

- Integrated UtteranceSessionProcessor into SpeechNode.
- Added utterance-session enable and required parameters.
- Added pre-roll, speech-start timeout, maximum-duration, and completed-queue parameters.
- Added dependency validation requiring audio, wake word, and VAD.
- Required sessions now require required wake-word and VAD processors.
- Registered utterance_session as the final captured-audio processor.
- Added required and optional session initialization behavior.
- Added session-before-VAD shutdown ordering.
- Mapped armed sessions to Listening and active sessions to Recording.
- Added utterance-session dashboard and diagnostics integration.
- Added Raspberry Pi and ReSpeaker production profile defaults.

### Phase 4M-E

- Integrated CompletedUtteranceWorker into SpeechNode.
- Added bounded serialized-output queue configuration.
- Added completed-source wait timeout and WAV byte limit parameters.
- Added dependency validation on the utterance-session processor.
- Added required and optional serialization-worker startup behavior.
- Added dependency-safe worker shutdown before session destruction.
- Added dashboard and dedicated diagnostics visibility.
- Enabled serialization in the Raspberry Pi production profile.
