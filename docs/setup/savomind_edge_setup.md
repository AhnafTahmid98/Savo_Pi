# SavoMind Edge integration setup

## Scope

The authoritative SavoMind installation procedure is maintained in the SavoMind repository. This page documents only the Robot Savo Edge-side contracts provided here; it does not invent images, Compose files, model downloads, or provider configuration.

SavoMind is outside the ROS workspace. It exchanges local speech frames with `savo_speech` and approved typed robot operations with `savo_bridge`; it has no direct drivetrain, supervisor, map-release, or generic ROS authority.

## Runtime identities and paths

| Contract | Current value |
| --- | --- |
| Shared group | `savomind-bridge`, GID `10001` |
| Expected SavoMind peer UID | `10001` |
| Speech socket | `/run/savomind/speech.sock` |
| Bridge command socket | `/run/savo_bridge/command.sock` |
| Bridge snapshot | `/run/savo_bridge/snapshot.json` |
| Command socket mode | `0660` |
| Command socket group | GID `10001` |

The bridge configuration permits peer UID `10001`; a container or native service must preserve that identity and shared-group access without running privileged or mounting arbitrary host devices. Do not expose either Unix socket through TCP or a network share.

## Prepare the Robot Savo side

```bash
cd "$HOME/Savo_Pi"
sudo bash deploy/edge/prepare_runtime_sockets.sh \
  --user "$USER" \
  --install-tmpfiles
getent group savomind-bridge
stat -c '%A %U:%G %n' /run/savomind
```

This prepares the volatile speech parent directory, not the socket. The SavoMind speech server creates `speech.sock`.

For distributed Edge bringup, current source also requires explicit `/run/savo_bridge` provisioning:

```bash
sudo install -d -m 0770 -o "$USER" -g savomind-bridge /run/savo_bridge
```

This known service gap is described in [Edge setup](savo_edge_setup.md). If standalone bridge ownership is selected instead, first build the Edge workspace and install its unit without starting it:

```bash
sudo "$HOME/Savo_Pi/savo_ws/install/savo_bridge/lib/savo_bridge/install_edge_runtime.sh" \
  --user "$USER" \
  --workspace "$HOME/Savo_Pi/savo_ws"
```

Then set distributed `SAVO_START_BRIDGE=false`. Do not enable/start both bridge owners.

## Configuration and secrets

Record the SavoMind revision/version and speech protocol compatibility. Provider credentials belong to the SavoMind repository's protected secret mechanism, never Robot Savo YAML, unit files, logs, snapshots, or Git. Use a non-root service/container with only the required socket mounts and no host device or Docker-socket access.

## Startup order and smoke checks

1. Prepare `/run/savomind` and the selected bridge parent mechanism.
2. Start Core safe idle and verify `STOP`.
3. Start Edge/bridge using exactly one owner.
4. Start the SavoMind speech server so it creates `/run/savomind/speech.sock`.
5. Enable/start `savo_speech` only after the audio guide passes.
6. Start the external SavoMind client/planner through its authoritative procedure.

```bash
stat -c '%A %U:%G %n' \
  /run/savomind /run/savomind/speech.sock \
  /run/savo_bridge /run/savo_bridge/command.sock
ss -xl | grep -E '/run/(savomind|savo_bridge)/'
```

Expected result is two local sockets with controlled identity/mode, current speech readiness and bridge observations, and typed requests that remain constrained by Core state. Do not use commissioning to request motion.

## Failure handling and evidence

If a parent or socket is absent, identify which owner should create it; do not make it world-writable. On `permission denied`, compare UID/GID/mode and re-login after group changes. On protocol rejection, record both revisions and logs without payload credentials. Retain versions, identity listings, socket metadata, selected owner model, and redacted smoke-test output. See the [SavoMind–ROS boundary](../architecture/savomind_ros_boundary.md) and [speech test plan](../testing/speech_test_plan.md).
