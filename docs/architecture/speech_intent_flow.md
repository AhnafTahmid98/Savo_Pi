# Speech and Intent Flow

There is no `savo_intent` ROS package. Language understanding and reasoning belong to external SavoMind; Robot Savo accepts only typed, bounded requests.

## Flow

```text
ReSpeaker -> savo_speech -> bounded protocol v2 -> /run/savomind/speech.sock
                                             SavoMind
                                                |
                                                | approved command schema
                                                v
                           /run/savo_bridge/command.sock -> savo_bridge
                                                |
                               typed ROS services/actions only
                                                v
                       supervisor + operation-owner admission -> control/safety

Robot ROS state -> bridge snapshot /run/savo_bridge/snapshot.json -> SavoMind
SavoMind WAV response -> validation -> playback -> acknowledgement
```

Speech owns wake/VAD/utterance capture, request/session correlation, timeouts, WAV validation, playback, and microphone gating. Bridge owns peer checks, schema/bounds, freshness, typed adapters, and ROS result translation. Supervisor owns permission; navigation/mapping/location/control owners execute and can reject.

The bridge cannot offer arbitrary topics, generic services/actions, shell execution, direct `/cmd_vel_safe`, motor access, approval actions, or readiness bypass. Socket loss, malformed or oversized payload, bad peer credentials, stale robot state, timeout, unavailable operation, or revoked authority fails closed. Speech failure may degrade interaction but cannot change motion safety.

`/run` artifacts are volatile and recreated at boot with deployment-managed ownership/modes. Source-contract and cross-repository protocol tests exist. Target validation remains for ReSpeaker selection, wake/VAD behavior, noisy-room capture, socket identity/permissions, SavoMind compatibility, playback format/device, interruption, timeouts, and authority revocation during an utterance-driven operation.
