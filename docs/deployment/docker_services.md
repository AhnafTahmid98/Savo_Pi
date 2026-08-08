# SavoMind and Container Services

## Scope

The Savo_Pi repository contains the ROS 2 side of Robot Savo. It does not contain an authoritative SavoMind Docker Compose stack in the inspected snapshot. SavoMind is therefore treated as a separately versioned companion deployment.

This document defines the integration boundary that any container deployment must preserve. Container image names, provider credentials, model volumes, and update commands must come from the SavoMind repository or its approved release record.

## Responsibility split

| Area | Responsibility |
| --- | --- |
| Robot Savo ROS 2 | Microphone capture and playback hardware through `savo_speech` |
| SavoMind companion | Speech-to-text inference |
| Typed bridge observations and commands through `savo_bridge` | Language-model planning and response generation |
| Core-side safety, readiness, supervisor authority, mapping, and navigation | Text-to-speech generation |
| Local validation of returned WAV data and physical-playback acknowledgement | Provider selection and inference fallback |
| Command rejection on stale or invalid state | High-level request interpretation |

SavoMind must not receive direct device access to motors, encoder GPIO, motor PWM, emergency-stop hardware, Core persistent state, or unrestricted ROS graph control.

## Speech socket contract

The native Edge speech process and SavoMind communicate through:

```text
/run/savomind/speech.sock
```

The approved transport is a local Unix-domain socket. Runtime provisioning is performed with:

```bash
sudo bash deploy/edge/prepare_runtime_sockets.sh --install-tmpfiles
```

The runtime directory policy is:

- owner: `root`
- group: `savomind-bridge`
- mode: `2770`
- default group ID: `10001`, unless an approved cross-deployment value is documented

The native speech client and the container/service user must share only the required group membership. The socket must not be published as a TCP port.

The speech protocol requires bounded messages, request/session correlation, timeouts, validated returned audio, cancellation handling, and physical-playback acknowledgement. A successful TTS generation is not equivalent to successful playback on the robot.

## ROS bridge boundary

`savo_bridge` exposes robot observations and a constrained command socket. Its production paths are:

```text
/run/savo_bridge/command.sock
/run/savo_bridge/snapshot.json
```

Only typed operations implemented by the bridge are allowed. The container must not mount the ROS workspace and invoke arbitrary `ros2 topic pub`, `service`, `action`, or shell commands as an integration shortcut.

The bridge must reject malformed, stale, unauthorized, timed-out, map-incompatible, or unavailable operations. Operator-only map/location approval is not exposed to SavoMind.

## Container hardening requirements

An approved SavoMind container deployment should:

- Run as a non-root user.
- Use a read-only root filesystem where practical.
- Mount only the required model, cache, and Unix-socket paths.
- Avoid `--privileged`, host PID, host device, and unrestricted host networking.
- Drop Linux capabilities not required by the runtime.
- Set CPU, memory, and log limits appropriate to the Edge Pi.
- Use a restart policy that does not create an uncontrolled crash loop.
- Keep API keys in protected secrets or environment files outside Git.
- Pin image versions or immutable digests for releases.
- Record model/provider configuration with the test evidence.
- Expose a health signal separate from “container is running.”

## Startup order

1. Prepare `/run/savomind` and group membership.
2. Start Core and verify `STOP` and readiness.
3. Start Edge and verify native speech/bridge owners.
4. Start SavoMind.
5. Verify socket ownership and protocol compatibility.
6. Perform a non-motion speech round trip.
7. Verify cancellation, timeout, and provider-failure behavior.
8. Enable high-level command use only after the bridge boundary passes.

## Validation

Minimum validation includes:

- Container starts without privileged access.
- Expected image version/digest is recorded.
- Required provider/model configuration is loaded.
- Speech socket appears with approved ownership and mode.
- UID/peer checks accept the intended client and reject an unintended user.
- Wake/VAD utterance reaches STT.
- LLM response is correlated with the correct session.
- TTS WAV is validated before playback.
- Physical playback acknowledgement is returned.
- Cancellation stops pending inference/playback.
- Socket loss and container restart produce bounded failure.
- Bridge snapshot remains read-only to SavoMind.
- Typed `STOP` is fail-closed.
- No generic ROS or shell path exists.

## Logs and secrets

Container logs must not include:

- Provider API keys.
- Raw authorization headers.
- Complete private user utterances unless explicitly required by a controlled test.
- Persistent-state database contents.
- Unrestricted snapshots containing sensitive environment data.

Set retention and rotation. Record enough identifiers to correlate speech session, bridge request, and ROS event without exposing secrets.

## Updates and rollback

Treat a SavoMind image, model, provider, or protocol change as a release change. Before updating:

- Back up the current configuration.
- Record the active image digest and model set.
- Verify compatibility with the Robot Savo speech and bridge contracts.
- Stage the new image without deleting the previous image.
- Run the non-motion integration suite.
- Retain a documented rollback command.

Do not update SavoMind automatically during a robot mission.
