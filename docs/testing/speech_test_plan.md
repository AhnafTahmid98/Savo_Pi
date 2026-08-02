# Speech test plan

## Preconditions

- edge safe-idle is running and robot motion remains `STOP`;
- ReSpeaker capture and speaker playback ALSA devices are confirmed;
- the room is quiet enough for wake/VAD testing;
- speaker volume starts low;
- privacy approval is established;
- `/run/savomind/speech.sock` is owned by the expected deployment user/group
  for production testing.

Required hardware is the ReSpeaker microphone array and the 3.5 mm speaker.

## Source and protocol validation

Build and test `savo_speech` first:

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to savo_speech --symlink-install
source install/setup.bash
colcon test --packages-select savo_speech --ctest-args --output-on-failure
colcon test-result --verbose
```

The bounded Unix-socket contract is documented in:

```text
savo_speech/docs/savomind_speech_transport_v2.md
```

## Bench fake-server validation

The fake server validates framing, request correlation, returned-WAV checks,
cancellation, playback integration, and restart behavior. It is not STT, LLM,
or TTS inference and must never be presented as production SavoMind.

Terminal 1:

```bash
source ~/Savo_Pi/savo_ws/install/setup.bash
ros2 run savo_speech fake_savomind_speech_server.py \
  --socket /tmp/savomind-speech-test.sock
```

Launch speech in a bench configuration that explicitly overrides the socket to
`/tmp/savomind-speech-test.sock` and keeps `savomind.required:=true`. Speak one
bounded test utterance after the wake word.

Expected lifecycle:

```text
listening → recording → sending → transcribing → thinking
→ synthesizing → playing → completed
```

The transcript and reply must carry the same request/session correlation and the
returned audio must be mono PCM16 at 16 kHz. A missing endpoint, wrong request
ID, malformed frame, oversized response, invalid WAV, timeout, or late response
must fail closed.

## Physical diagnostics

Microphone readiness:

```bash
cd ~/Savo_Pi
python3 tools/diag/voice/audio_mic_test.py \
  --timeout 5 \
  --output log/diag/audio_mic.json
```

Observe one privacy-filtered transcript during an approved utterance:

```bash
python3 tools/diag/voice/asr_topic_test.py \
  --timeout 20 \
  --output log/diag/speech_transcript.json
```

Observe one response:

```bash
python3 tools/diag/voice/tts_topic_test.py \
  --timeout 20 \
  --output log/diag/speech_response.json
```

Observe playback completion while another approved component requests the
utterance; this diagnostic never requests speech itself:

```bash
python3 tools/diag/voice/audio_speaker_test.py \
  --expect-playback \
  --timeout 30 \
  --output log/diag/audio_playback.json
```

## Production SavoMind gate

Production speech remains `BLOCKED` until the separate SavoMind application
runs the exact authenticated v2 Unix-socket server and returns:

- matching request ID;
- bounded transcript;
- bounded reply;
- valid mono PCM16 16 kHz TTS WAV bytes;
- a correlated playback token when pending navigation requires acknowledgement.

After physical playback, `savo_speech` must send the v2 acknowledgement frame.
SavoMind may revalidate and dispatch pending navigation only after accepting it.

Do not substitute an undocumented HTTP response, filesystem path, or raw Docker
mount for the agreed contract.

## Expected result, abort, and cleanup

Expected result: capture, wake, VAD, utterance bounds, request correlation,
returned audio, mic gate, playback, cancellation, and restart behavior all pass.
Raw continuous microphone audio is not published.

Abort on acoustic feedback, clipping, privacy breach, runaway volume, unbounded
retry, stale cancellation, or playback that cannot be stopped. Cleanup cancels
active transport/playback, mutes output, restores mic state, removes temporary
audio according to policy, and records states, latencies, reason codes, and JSON
results—not speech content unless explicitly approved.
