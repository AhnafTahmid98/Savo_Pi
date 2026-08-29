# Robot playback socket protocol v1

`savo_speech` listens on `/run/savo_speech/playback.sock` using an `AF_UNIX`,
`SOCK_STREAM` socket. One connection carries one request. Multibyte integers use
network byte order (big endian).

Deployment must create the runtime directory before starting `savo_speech`;
the application does not invoke `sudo`:

```bash
sudo install -d -o savo -g savo -m 0755 /run/savo_speech
```

The server retains `create_directories()` for environments where its process
already has permission to create the directory. The socket itself uses mode
`0666` so the containerized SavoMind process can connect without sharing the
host `savo` user's group. This mode is connectivity, not authorization:
production enables `SO_PEERCRED` validation and accepts only peer UID `10001`.

## Request

The 20-byte header is followed immediately by `wav_bytes` bytes.

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 4 | Magic ASCII `SVPW` |
| 4 | 2 | Version, `1` |
| 6 | 2 | Reserved, must be `0` |
| 8 | 8 | Nonzero playback request ID |
| 16 | 4 | WAV payload length |

The payload must be a RIFF/WAVE file containing uncompressed signed 16-bit
little-endian PCM at 16 kHz with one channel. The configured
`robot_playback.maximum_wav_bytes` is a hard limit over the complete WAV.

## Acknowledgement

The server returns a 16-byte acknowledgement after validating and attempting
to enqueue the request. Acceptance means the existing playback worker owns the
request; it does not mean playback has completed.

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 4 | Magic ASCII `SVPA` |
| 4 | 2 | Version, `1` |
| 6 | 2 | Status |
| 8 | 8 | Request ID (zero when unavailable) |

Status values are: `0` accepted, `1` invalid header, `2` invalid payload
length, `3` unauthorized peer, `4` invalid WAV, `5` invalid audio format,
`6` playback queue rejection, and `7` internal error.

When `robot_playback.require_peer_uid` is true, the server checks Linux
`SO_PEERCRED` before reading the request and accepts only
`robot_playback.peer_uid`.
