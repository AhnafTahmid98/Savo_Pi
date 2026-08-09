# Speech and UI Operation

## Purpose and authority

Speech and UI are optional Edge features. UI is read-only presentation.
SavoMind inference is external and can request only the typed bridge operations
that still pass Core authority, operation admission, control, and safety.

Map and semantic-location approval are not voice operations. The supported
location-review CLI requires an identified operator; map release uses its typed
review contract through an approved operator client.

## Preconditions and normal use

- Edge service, bridge readiness, audio/display hardware, Unix-socket ownership,
  and time/network state have been validated.
- The deployed Edge environment explicitly enables speech/UI as required;
  defaults are off.
- SavoMind owns `/run/savomind/speech.sock` and the allowed bridge identity can
  access `/run/savo_bridge/command.sock`.

Check current state:

```bash
systemctl status savo_edge.service --no-pager
ros2 topic echo --once /savo_bridge/readiness
ros2 topic echo --once /savo_speech/readiness
ros2 topic echo --once /savo_speech/state
```

1. Confirm the UI freshness indicators match current Core/Edge state.
2. Use the configured wake word; observe listening, thinking, speaking, and
   playback states.
3. Speak one request and wait for acknowledgement. Voice navigation still
   requires bridge freshness, map context, supervisor permission, and the
   public navigation gateway.
4. Cancel through the supported speech/client interaction. Use normal STOP or
   the emergency procedure for motion risk; speech cancellation is not an
   emergency stop.
5. During playback the microphone is intentionally gated to limit feedback.

## Troubleshooting

| Symptom | Safe check | Action |
| --- | --- | --- |
| No microphone/wake | Edge logs; ALSA device; speech readiness | Keep voice unavailable; maintainer checks configured device |
| STT/TTS unavailable | Socket existence/permissions; SavoMind process | Use non-voice approved control; do not retry rapidly |
| Speaker silent | Playback state and Edge logs | Maintainer validates device/volume with bench procedure |
| UI blank | Edge service, framebuffer/input permissions | Use observer; UI loss does not remove authority |
| UI stale | Compare timestamps with readiness topics | Treat displayed state as unavailable; do not operate from it |
| Bridge disconnected | `/savo_bridge/readiness`, socket, Edge logs | Voice commands unavailable; Core remains authoritative |

Do not retain raw private speech or full SavoMind environment files in routine
logs. Escalate protocol mismatch, repeated audio-runtime crashes, stale state
shown as current, UI mutation behavior, or any request that bypasses admission.
