# savo_speech

ROS 2 Jazzy C++ package responsible for Robot SAVO's physical speech interface
on `savo-edge`.

## Ownership

`savo_speech` owns microphone capture, wake-word detection, VAD, utterance
segmentation, PCM/WAV serialization, bounded transport of completed utterances,
returned-audio validation, speaker playback, microphone gating, cancellation,
completion events, runtime state, freshness, and diagnostics.

SavoMind owns STT, agent/LLM reasoning, and TTS inference. `savo_speech` does not
own navigation, motion, safety policy, agent routing, or UI rendering.

## Implemented runtime

The production C++ path provides:

- ALSA capture and playback streams;
- PocketSphinx wake-word detection;
- adaptive energy VAD;
- bounded utterance sessions and pre-roll;
- PCM WAV encoding and strict WAV decoding;
- bounded completed-utterance queues;
- versioned Unix-socket speech and physical-playback acknowledgement framing;
- request/session correlation;
- connect, I/O, and playback-completion timeouts;
- optional SavoMind server-UID verification with `SO_PEERCRED`;
- cancellation using socket shutdown and playback cancellation;
- returned 16 kHz mono PCM16 TTS validation;
- playback through the existing asynchronous audio runtime;
- microphone gating during playback;
- privacy-safe transcript, response, playback, result, state, heartbeat, and
  diagnostic topics.

The documented socket contract is in
`docs/savomind_speech_transport_v2.md`. The production endpoint is
`/run/savomind/speech.sock`.

## SavoMind production boundary

The companion SavoMind brain application implements the authenticated protocol
v2 server. It returns correlated transcript, response text, validated TTS WAV,
and a playback token when pending navigation requires physical playback
acknowledgement. `savo_speech` acknowledges completion or failure on a second
connection; SavoMind may dispatch pending navigation only after accepting that
event. A missing or wrong-UID server remains fail-closed with
`savomind_endpoint_unavailable`.

A deterministic bench-only fake server is installed as:

```bash
ros2 run savo_speech fake_savomind_speech_server.py \
  --socket /tmp/savomind-speech-test.sock
```

It is not an STT/LLM/TTS implementation. It exists only to validate framing,
correlation, playback, cancellation, microphone gating, and restart behavior.

## ROS outputs

```text
/savo_speech/readiness
/savo_speech/dashboard
/savo_speech/heartbeat
/savo_speech/state
/savo_speech/transcript
/savo_speech/response
/savo_speech/playback/state
/savo_speech/playback/finished
/savo_speech/result
/savo_speech/diagnostics
```

Raw continuous microphone audio is not published by default.

## Build and test

```bash
cd ~/Savo_Pi/savo_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to savo_speech --symlink-install
source install/setup.bash
colcon test --packages-select savo_speech --ctest-args --output-on-failure
colcon test-result --verbose
```

The first real edge validation must use the ReSpeaker and speaker with movement
kept in `STOP`.
