# Deploy Robot Savo Edge

## Purpose

This procedure deploys the Edge role to the Raspberry Pi that owns RealSense acquisition, visual odometry, speech hardware, touchscreen presentation, the constrained SavoMind bridge, and Edge-side power monitoring.

The Edge computer can request robot behavior only through typed and bounded interfaces. It must not acquire direct drivetrain, supervisor, map-release, or operator-approval authority.

## Production assumptions

| Item | Required value or behavior |
| --- | --- |
| Operating system | Ubuntu 24.04 LTS |
| ROS distribution | ROS 2 Jazzy |
| Hostname | `edge` or `savo-edge` |
| Repository | Complete Git checkout of `Savo_Pi` |
| Core–Edge address | Normally `192.168.50.2/24` on the dedicated link |
| Default role mode | `safe_idle` |
| RealSense | Enabled by default |
| Visual odometry | Enabled by default |
| D435 obstacle cloud | Disabled until hardware-validated |
| Speech and UI | Disabled by default; enable only after component validation |
| SavoMind bridge | Enabled by default in distributed Edge bringup |

## Safety and authority prerequisites

Before deployment:

- Verify the matching Core role remains in `STOP`.
- Confirm ROS domain and middleware settings match Core.
- Confirm SavoMind cannot directly publish motor commands or bypass Core readiness.
- Stop any duplicate standalone bridge or UI service before starting distributed Edge bringup.
- Keep runtime sockets local to the Edge host.
- Do not expose Unix-domain command sockets over a network share or TCP proxy.

## 1. Verify the checkout

```bash
cd ~/Savo_Pi
git status --short
git rev-parse --verify HEAD
git diff --check
```

Record the exact revision. Production deployment should use a Git checkout rather than an unidentified exported archive.

## 2. Install Edge dependencies

```bash
cd ~/Savo_Pi
bash deploy/common/install_role_deps.sh --role edge
```

In addition to common tools, the Edge installer provides the audio development/runtime packages required by `savo_speech`, resolves role-scoped ROS dependencies, and prepares the controlled runtime socket directory.

Resolve all `rosdep`, APT, architecture, or disk-space failures before continuing.

## 3. Prepare runtime sockets

The speech transport uses the local runtime directory `/run/savomind/`. Prepare it with:

```bash
cd ~/Savo_Pi
sudo bash deploy/edge/prepare_runtime_sockets.sh --install-tmpfiles
```

The script creates the `savomind-bridge` group if needed, enforces the configured group identity, creates `/run/savomind` with controlled group access, and can install a `tmpfiles.d` policy so the directory is recreated after boot. It creates no sockets and starts no services.

The bridge uses `/run/savo_bridge/` for its command socket and snapshot. The standalone bridge systemd unit creates it through `RuntimeDirectory=savo_bridge`. When the bridge is started inside distributed Edge bringup, provision the directory explicitly before launch:

```bash
sudo install -d -m 0770 -o "$USER" -g savomind-bridge /run/savo_bridge
```

The parent directory must be a real absolute directory, not a symbolic link. The bridge intentionally rejects unsafe or missing parent-directory paths. A future role-service revision may move this ownership into systemd; until then, treat this command as a required Edge preflight when `SAVO_START_BRIDGE=true`.

### Verify the runtime policy

```bash
getent group savomind-bridge
stat -c '%A %U:%G %n' /run/savomind /run/savo_bridge
```

Add only the required service users to the bridge group. Log out and back in after changing an interactive user's group membership.

## 4. Build and test the Edge role

```bash
cd ~/Savo_Pi
bash deploy/edge/build_edge.sh --clean --test
```

### Required result

- All 10 Edge packages are present.
- Build completes successfully.
- Test result reports zero failures and zero errors.

The build does not start cameras, audio, display output, SavoMind, the bridge, or the distributed robot.

## 5. Run validators

```bash
cd ~/Savo_Pi
bash deploy/common/validate_full_bringup.sh
bash deploy/common/validate_pre_real_test_readiness.sh
```

There must be no source-contract failure. Hardware-dependent blockers remain valid until the corresponding component tests pass.

## 6. Verify Edge hardware permissions

Before first runtime launch, confirm the deployment user can access the intended devices:

```bash
id
ls -l /dev/video* 2>/dev/null || true
ls -l /dev/snd/* 2>/dev/null || true
ls -l /dev/fb0 2>/dev/null || true
```

Verify the actual RealSense, ReSpeaker, speaker, framebuffer, touch, and Edge UPS devices using their package-specific setup and test documents. Do not loosen permissions globally to bypass a configuration issue.

## 7. Review production environment

Install and edit the shared environment template:

```bash
sudo install -d -m 0750 /etc/robot-savo
sudo install -m 0640 deploy/systemd/robot-savo.env.example \
  /etc/robot-savo/robot-savo.env
sudoedit /etc/robot-savo/robot-savo.env
```

Verify repository paths, ROS domain, middleware configuration, network interface assumptions, state/log directories, and any explicitly enabled Edge features.

Do not place provider credentials in Git. Use the SavoMind companion system's protected environment mechanism.

## 8. Render and install the Edge role service

```bash
cd ~/Savo_Pi
sudo bash deploy/systemd/render_units.sh \
  --user "$USER" \
  --group "$USER" \
  --root "$PWD" \
  --output-dir /tmp/robot-savo-units
```

```bash
systemd-analyze verify /tmp/robot-savo-units/savo_edge.service
sudo install -m 0644 /tmp/robot-savo-units/savo_edge.service \
  /etc/systemd/system/savo_edge.service
sudo systemctl daemon-reload
```

Do not also enable:

- The generic `savo.service` in Edge mode.
- Standalone `savo_bridge.service` while distributed Edge bringup starts the bridge.
- Standalone `savo-ui.service` while distributed Edge bringup starts the UI.

Each runtime component must have exactly one owner.

## 9. First interactive safe-idle launch

Start Edge interactively before enabling automatic startup:

```bash
cd ~/Savo_Pi
bash deploy/edge/run_edge.sh
```

Default behavior must preserve:

- `robot_mode=safe_idle`
- `bringup_profile=lidar_only`
- Locked geometry required
- Provisional geometry rejected for motion-capable paths
- D435 voxel validation false
- RealSense enabled
- VO enabled
- D435 obstacle cloud disabled
- Speech disabled
- UI disabled
- Bridge enabled

Validate device health and Core connectivity without authorizing motion.

## 10. Enable optional Edge components deliberately

Speech, UI, and D435 obstacle-cloud integration should be enabled one at a time only after their component test plans pass.

For every feature change:

- Record the configuration change.
- Confirm the feature has one runtime owner.
- Start in safe idle.
- Verify health and stale-state handling.
- Verify shutdown/restart behavior.
- Preserve Core-side `STOP` and authority gates.

The D435 obstacle cloud must remain disabled until self-filter bounds, frame transforms, point filtering, dropout behavior, and Nav2 interaction are physically validated.

## 11. Enable the Edge service

After the interactive launch passes:

```bash
sudo systemctl enable savo_edge.service
sudo systemctl start savo_edge.service
```

### Verify

```bash
systemctl status savo_edge.service --no-pager
journalctl -u savo_edge.service -b --no-pager
```

Then verify ROS discovery and Edge package state from a correctly configured shell:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Savo_Pi/savo_ws/install/setup.bash
ros2 node list
ros2 topic list
```

Confirm there is one bridge node, one RealSense ownership path, one VO producer, and no unexpected command publishers.

## 12. Release update workflow

For later updates:

```bash
cd ~/Savo_Pi
bash deploy/common/update_role.sh --role edge --pull
```

The updater stages, builds, and tests the new role before switching the active install. The previous install is retained for rollback.

## Deployment acceptance criteria

Edge deployment passes when:

- The exact source revision is recorded.
- Dependencies and runtime directories are correct.
- All Edge packages build and test successfully.
- Source validators report no failure.
- The service unit passes verification.
- Safe-idle starts without duplicate component owners.
- Core and Edge discover each other reliably.
- RealSense and VO health are visible.
- Bridge socket ownership and peer restrictions are correct.
- Optional speech/UI/cloud features remain disabled until validated.
- Restart and rollback procedures are available.

Continue with the component and full-robot test plans before enabling autonomous behavior.
