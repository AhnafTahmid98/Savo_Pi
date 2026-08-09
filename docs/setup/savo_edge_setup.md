# Savo Edge fresh installation

## Purpose and target

This procedure prepares the Raspberry Pi 5 that owns RealSense acquisition, visual odometry, speech hardware, touchscreen presentation, the constrained SavoMind bridge, and Edge power monitoring. The supported target is Ubuntu 24.04 LTS ARM64 with ROS 2 Jazzy and hostname `edge` or `savo-edge`.

## Prerequisites

- Core is available or scheduled for later discovery testing and remains in `STOP`.
- ROS Jazzy exists at `/opt/ros/jazzy`.
- The authorized repository is at `$HOME/Savo_Pi`, or `SAVO_ROOT` identifies its absolute location.
- RealSense, ReSpeaker/audio, display/touch, and Edge UPS wiring are reviewed.

```bash
cd "$HOME/Savo_Pi"
hostname
uname -m
grep '^VERSION_ID=' /etc/os-release
git status --short
git rev-parse HEAD
```

## Install dependencies and sockets

```bash
bash deploy/common/install_role_deps.sh --role edge
```

The installer enforces platform, disk, ROS, and the exact 10-package Edge array; installs role dependencies; runs `rosdep`; builds the role; and calls the Edge runtime-socket preparation. Re-run the latter explicitly when auditing its identity:

```bash
sudo bash deploy/edge/prepare_runtime_sockets.sh \
  --user "$USER" \
  --install-tmpfiles
getent group savomind-bridge
stat -c '%A %U:%G %n' /run/savomind
```

The script establishes group `savomind-bridge` with GID `10001`, adds the selected user, creates `/run/savomind` at mode `2770`, and installs a tmpfiles rule. Log out and in after membership changes. It creates no speech socket and starts no service.

## Bridge runtime-directory decision

The standalone bridge unit creates `/run/savo_bridge` using `RuntimeDirectory=savo_bridge`. The distributed `savo_edge.service` currently starts the bridge but does not create that directory. For the repository-documented distributed path, provision it before every boot/start until the service defect is fixed:

```bash
sudo install -d -m 0770 -o "$USER" -g savomind-bridge /run/savo_bridge
stat -c '%A %U:%G %n' /run/savo_bridge
```

The directory must be a real absolute directory, not a symlink. `/run` is volatile. Alternatively, install the standalone bridge unit after the workspace build and set `SAVO_START_BRIDGE=false` for distributed Edge bringup; never allow both owners. See [SavoMind setup](savomind_edge_setup.md).

## Hardware access

```bash
id
ls -l /dev/video* /dev/snd/* /dev/i2c-1 /dev/fb0 2>/dev/null || true
ls -l /dev/input/by-path/* 2>/dev/null || true
```

Use [device permissions](device_permissions_and_udev.md), [RealSense setup](realsense_setup.md), [audio setup](audio_setup.md), and [UPS setup](ups_hat_setup.md). The repository ships no universal udev policy for video, audio, framebuffer, touch, I2C, or GPIO; record the target's least-privilege host policy.

## Build and validate

```bash
cd "$HOME/Savo_Pi"
bash deploy/edge/build_edge.sh --clean --test
bash deploy/common/validate_full_bringup.sh
```

Require all 10 Edge packages and zero test failures/errors.

## Environment and service

Prepare `/etc/robot-savo/robot-savo.env` from the repository example, set `SAVO_ROLE=edge`, and match the Core ROS domain/RMW. Render and install only the role unit:

```bash
sudo bash deploy/systemd/render_units.sh \
  --user "$USER" \
  --group "$USER" \
  --root "$PWD" \
  --output-dir /tmp/robot-savo-units
systemd-analyze verify /tmp/robot-savo-units/savo_edge.service
sudo install -m 0644 /tmp/robot-savo-units/savo_edge.service \
  /etc/systemd/system/savo_edge.service
sudo systemctl daemon-reload
```

Do not also enable generic `savo.service`, standalone UI, or standalone bridge when distributed bringup owns the same component.

The current `savo_edge.service` also declares `Wants=`/`After=` on
`savo-ui-runtime.service`, while its distributed launch environment keeps
`SAVO_START_UI=false`. The renderer emits both units, but this fresh-install
procedure intentionally does not install the UI runtime unit automatically.
Resolve and validate the intended UI ownership before installing that companion
unit; do not install it merely to silence a dependency warning.

## Safe-idle commissioning

The current Edge defaults are:

| Setting | Default |
| --- | --- |
| Robot/control | `safe_idle` / `STOP` |
| Bringup profile | `lidar_only` |
| RealSense / VO / bridge | enabled |
| Speech / UI | disabled |
| D435 obstacle cloud | disabled |
| D435 voxel validation | false |

Start interactively first:

```bash
cd "$HOME/Savo_Pi"
bash deploy/edge/run_edge.sh
```

From another configured shell, use `ros2 node list` and `ros2 topic list` to verify one RealSense owner, one VO producer, one bridge owner, current health/readiness, Core discovery, and no unexpected command publisher. Do not enable speech, UI, or obstacle cloud until their own component setup/validation gates pass.

After safe idle passes:

```bash
sudo systemctl enable savo_edge.service
sudo systemctl start savo_edge.service
systemctl status savo_edge.service --no-pager
journalctl -u savo_edge.service -b --no-pager
```

## Failure handling, evidence, and next step

Stop on bridge directory/group mismatch, missing camera/audio/display access, duplicate nodes, discovery loss, or restart loops. Preserve revision, device listings, environment, build/test output, and journals. Detection does not validate VO accuracy, obstacle behavior, speech quality, or UI usability. Continue with the [commissioning checklist](commissioning_checklist.md) and [Edge component validation](../testing/savo_edge_component_validation.md).
