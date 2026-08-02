# Edge audio setup

Connect ReSpeaker USB and the powered speaker to the edge Pi. Use `arecord -l`
and `aplay -l` to identify stable ALSA hardware names, then enter them in the
edge speech profile. Start with amplifier volume low. The robot process, not
SavoMind, owns capture/playback; SavoMind owns STT/agent/TTS inference.

Run microphone observation first. Actual speaker output needs an explicit local
operator and must verify mic gating, bounded playback, cancellation, no feedback,
and safe volume. Never store `.env` credentials or private recordings in Git.
