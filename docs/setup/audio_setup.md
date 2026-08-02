# Edge audio setup

Connect ReSpeaker USB and the powered speaker to the edge Pi. Use `arecord -l`
and `aplay -l` to identify stable ALSA hardware names, then enter them in the
edge speech profile. Start with amplifier volume low. The robot process, not
SavoMind, owns capture/playback; SavoMind owns STT/agent/TTS inference.

Run microphone observation first. Actual speaker output needs an explicit local
operator and must verify mic gating, bounded playback, cancellation, no feedback,
and safe volume. Never store `.env` credentials or private recordings in Git.


## SavoMind speech socket

Provision the authenticated edge runtime directory before starting native
`savo_speech` or the SavoMind container:

```bash
cd ~/Savo_Pi
sudo deploy/edge/prepare_runtime_sockets.sh \
  --user "$USER" \
  --install-tmpfiles
```

The directory is `root:savomind-bridge`, mode `2770`. The native edge runtime
user and SavoMind container GID `10001` share access. The socket remains local
to the edge host and must not be exposed over TCP.
