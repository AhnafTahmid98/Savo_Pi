# Speech and intent flow

```text
ReSpeaker → C++ capture/wake/VAD/session/WAV → bounded SavoMind transport
→ SavoMind STT → agent/LLM policy → typed savo_bridge command (when needed)
→ existing ROS authority → correlated result
→ SavoMind TTS → validated WAV → C++ playback queue → completion
```

`savo_speech` owns physical audio, mic gating, barge-in policy, temporary-file
safety, and lifecycle topics. SavoMind owns STT, reasoning, intent, and TTS.
`savo_bridge` accepts only strict allow-listed typed commands with peer
credentials, IDs, deadlines, replay protection, acknowledgements, and safety
gates. It is not a generic ROS proxy.
