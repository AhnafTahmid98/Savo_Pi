# Edge audio setup

## Purpose and hardware

This guide commissions the ReSpeaker USB capture/playback path used by `savo_speech`. It verifies stable ALSA identity, capture, cautious playback, and the inputs needed for later wake/VAD tuning. Speech is disabled by default in Edge bringup.

## Current audio contract

The production profile requires stable ALSA PCM alias `savo_respeaker` for both capture and playback and intentionally forbids numeric-card fallback. Current parameters are:

| Setting | Value |
| --- | --- |
| Encoding | signed 16-bit little-endian PCM |
| Sample rate | exactly 16000 Hz |
| Capture | 6 interleaved channels, select channel 0 |
| Playback | mono |
| Period/chunk | 320 frames, 4 periods |
| SavoMind socket | `/run/savomind/speech.sock` |

The repository does not ship the host ALSA configuration that defines `savo_respeaker`. Speech commissioning is blocked until a site-managed ALSA configuration supplies that stable alias and it survives reboot. Do not replace it with a changing numeric card ID in production.

## Detection and access

After the Edge build:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/Savo_Pi/savo_ws/install/setup.bash"
id
ls -l /dev/snd/* 2>/dev/null || true
ros2 run savo_speech savo_speech_audio_device_probe --match ReSpeaker
```

If the standard `alsa-utils` tools are already installed on the target, `arecord -l` and `aplay -l` provide an additional inventory; the role installer does not explicitly install `alsa-utils`, so they are not the primary Robot Savo check.

Use the probe's stable hardware/plugin names to verify the intended physical unit and the host alias configuration. Re-run after reboot/USB re-enumeration.

## Capture test

With other audio owners stopped:

```bash
ros2 run savo_speech savo_speech_capture_test \
  --device savo_respeaker \
  --rate 16000 \
  --channels 6 \
  --channel 0 \
  --output /tmp/savo_speech_capture.wav
```

Expected result is an exact 16 kHz negotiated format, six-channel capture reduced to a plausible mono WAV, no clipping/continuous silence, and no device-open error. The temporary WAV may contain speech; delete or move it to approved protected evidence storage after review.

## Playback test

Warn nearby people, set a safe physical volume, and use the captured or an approved mono S16_LE WAV:

```bash
ros2 run savo_speech savo_speech_playback_test \
  --device savo_respeaker \
  --input /tmp/savo_speech_capture.wav
```

Expected result is intelligible, low-volume playback without underrun/device errors. This is an audible-output test, not a motion test.

## Speech and SavoMind commissioning

Prepare `/run/savomind` and start the SavoMind speech server as described in [SavoMind setup](savomind_edge_setup.md). Only then enable speech for a controlled Edge safe-idle run. Verify readiness/diagnostics and that the expected server UID owns the socket.

Wake-word, VAD noise-floor, confidence, and utterance timing values are explicitly temporary in the production profile. Detection during setup does not validate far-field recognition, false-wake rate, noisy-room behavior, or provider round trips; complete the [speech test plan](../testing/speech_test_plan.md).

## Failure handling, security, and evidence

For a missing device, compare probe/device listings before and after reconnect. For alias failure, fix the protected host ALSA configuration; do not enable numeric fallback. For busy devices, stop duplicate speech/audio owners. For permission denial, inspect `/dev/snd` ownership and the actual host policy. Do not store recorded voices or provider credentials in Git/logs.

Retain device probe output, alias configuration reference (without secrets), negotiated formats, reboot check, capture/playback result, socket metadata, and redacted diagnostics.
