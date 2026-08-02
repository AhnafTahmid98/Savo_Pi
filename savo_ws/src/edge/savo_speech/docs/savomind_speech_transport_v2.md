# SavoMind speech transport v2

This is the production boundary between native `savo_speech` on `savo-edge`
and the SavoMind brain container. It uses an authenticated Unix-domain stream
socket at `/run/savomind/speech.sock`. It is never exposed over TCP.

All integers are unsigned big-endian. Text is strict UTF-8 without terminators.
Every length is validated before allocation. Each connection carries exactly
one request and one response.

The server authenticates the native client with `SO_PEERCRED`; the client may
also require the container server UID. The socket is mode `0660` inside a
setgid runtime directory shared only by the Robot SAVO edge runtime group.

## Speech request

| Field | Bytes |
| --- | ---: |
| Magic `SAVOSPRQ` | 8 |
| Protocol version (`2`) | 4 |
| Request ID length | 4 |
| Session ID length | 4 |
| WAV length | 8 |
| Request ID | variable |
| Session ID | variable |
| PCM WAV | variable |

The WAV must be mono signed 16-bit PCM at 16 kHz.

## Speech response

| Field | Bytes |
| --- | ---: |
| Magic `SAVOSPRS` | 8 |
| Protocol version (`2`) | 4 |
| Status (`0` means success) | 4 |
| Flags | 4 |
| Request ID length | 4 |
| Reason length | 4 |
| Transcript length | 4 |
| Reply length | 4 |
| Playback-token length | 4 |
| TTS WAV length | 8 |
| Request ID | variable |
| Reason | variable |
| Transcript | variable |
| Reply | variable |
| Playback token | variable |
| PCM TTS WAV | variable |

Flag bit 0 means physical playback acknowledgement is required. When it is set,
the playback token must be nonempty. A successful response contains the exact
request ID, transcript, reply, and a valid mono PCM16 16 kHz TTS WAV.

## Physical playback acknowledgement

When acknowledgement is required, native `savo_speech` plays the TTS audio and
opens a second connection only after playback reaches a terminal state.

### Acknowledgement request

| Field | Bytes |
| --- | ---: |
| Magic `SAVOSPAK` | 8 |
| Protocol version (`2`) | 4 |
| Playback status (`0` success, `1` failure) | 4 |
| Request ID length | 4 |
| Session ID length | 4 |
| Playback-token length | 4 |
| Reason length | 4 |
| Request ID | variable |
| Session ID | variable |
| Playback token | variable |
| Reason | variable |

### Acknowledgement response

| Field | Bytes |
| --- | ---: |
| Magic `SAVOSPAR` | 8 |
| Protocol version (`2`) | 4 |
| Status (`0` means event accepted) | 4 |
| Request ID length | 4 |
| Reason length | 4 |
| Request ID | variable |
| Reason | variable |

SavoMind correlates the token and session, records playback success or failure,
and only then revalidates and dispatches a pending navigation goal. Transport
acceptance is not a claim that navigation succeeded; navigation continues
through the guarded `savo_bridge` path.

## Cancellation and failure behavior

Connect, I/O, processing, and playback completion are bounded. Cancellation
shuts down the active socket. If TTS playback cannot start, fails, times out, or
the speech worker stops after receiving an acknowledgement token,
`savo_speech` makes a best-effort failure acknowledgement so pending navigation
cannot remain indefinitely armed.

## Ownership

SavoMind owns STT, agent/LLM processing, TTS inference, and pending-navigation
lifecycle. `savo_speech` owns capture, microphone gating, WAV validation,
physical playback, completion detection, and ROS speech state. Robot commands
never use this protocol; typed command dispatch remains owned by `savo_bridge`.
