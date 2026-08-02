# Speech test plan

Prerequisites are edge safe-idle, verified ALSA device names, a quiet room,
privacy approval, and a reachable SavoMind instance. Required hardware is the
ReSpeaker and 3.5 mm speaker.

Exact commands: run `python3 tools/diag/voice/audio_mic_test.py` first. Speaker
observation is `python3 tools/diag/voice/audio_speaker_test.py`; actual audio
output requires the tool’s explicit operator option. Test wake, VAD, utterance
WAV bounds, request correlation, mic gate, playback, cancellation, and restart.

Expected result: lifecycle advances listening→recording→sending→thinking→playing
with matching IDs, bounded files/responses, gated microphone during playback,
and no raw continuous audio topic. Missing device/SavoMind, malformed response,
wrong ID, invalid WAV, or feedback is FAIL/BLOCKED. Abort on feedback, privacy
breach, runaway volume, or unbounded retry. Cleanup mutes/stops playback and
deletes temporary audio according to policy. Record metadata, states, latencies,
provider reason, and JSON results—not speech content unless explicitly approved.
