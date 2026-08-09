# SavoMind–ROS Boundary

SavoMind is outside the ROS 2 trust and execution boundary. It communicates on Edge through two local Unix-socket protocols.

| Boundary | Path | Robot owner | Purpose |
| --- | --- | --- | --- |
| Command/observation | `/run/savo_bridge/command.sock`, `snapshot.json` | `savo_bridge` | Typed robot requests and read-only snapshot |
| Speech | `/run/savomind/speech.sock` | `savo_speech` client | Bounded utterance/response/audio exchange |

The bridge allows only compiled adapters for STOP, cancellation, bounded teleoperation, named-location navigation, mapping mission/control/query, Scan360, save/verification/review/release observation, and supervisor queries. Every adapter validates schema, bounds, state freshness, service/action availability, timeout, and applicable authority. The configured maximum bridge request/response is `65536` bytes; peer UID allow-listing is configured for the SavoMind service identity.

Speech protocol v2 validates request/session correlation, text size, timeouts, audio metadata/data, WAV safety, playback, and acknowledgement. The configured production socket is not a network listener.

SavoMind cannot obtain arbitrary ROS publish/call access, shell access, motor access, supervisor mutation, navigation-readiness bypass, or operator map/location approval. On invalid credentials, malformed/oversized input, stale snapshot, timeout, disconnect, or authority loss, Robot Savo rejects or cancels and fails closed.

Socket directories/modes are prepared by deployment and vanish across reboot. Source and protocol tests exist; deployed UID/GID, systemd sandboxing, SavoMind version compatibility, disconnect/replay behavior, and authority revocation require target integration evidence.
