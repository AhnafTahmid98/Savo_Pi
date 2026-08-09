# Deploy Robot Savo Core

## Purpose

This procedure deploys the Core role to the Raspberry Pi that owns drivetrain execution, near-field safety, localization, mapping, navigation, semantic locations, head control, power aggregation, and robot-wide supervision.

The Core computer is a safety authority. Deployment must preserve fail-closed startup and must never start physical motion as an incidental side effect of installation, building, or service activation.

## Production assumptions

| Item | Required value or behavior |
| --- | --- |
| Operating system | Ubuntu 24.04 LTS |
| ROS distribution | ROS 2 Jazzy |
| Hostname | `core` or `savo-core` |
| Repository | Complete Git checkout of `Savo_Pi` |
| Core–Edge address | Normally `192.168.50.1/24` on the dedicated link |
| Startup mode | `safe_idle` |
| Control startup mode | `STOP` |
| Geometry | Locked for motion-capable operation |
| D435 voxel integration | Disabled until independently hardware-validated |

## Safety prerequisites

Before deployment:

- Place the robot on a stable surface.
- Keep the physical emergency stop accessible.
- Disconnect or raise the drivetrain if any later step could reach actuator testing.
- Confirm no unapproved ROS command publishers are running.
- Confirm the active source revision is known and the working tree is clean.
- Back up persistent maps, locations, and supervisor state before replacing a working deployment.

Installation and build steps do not authorize motion.

## 1. Verify the checkout

```bash
cd ~/Savo_Pi
git status --short
git rev-parse --verify HEAD
git diff --check
```

### Pass criteria

- The intended branch and commit are checked out.
- The working tree is clean, or all intentional differences are documented.
- `git diff --check` reports no whitespace errors.

An exported ZIP without `.git` metadata may be inspected or built, but it is not a production release identity. Use a Git checkout for target deployment.

## 2. Install Core dependencies

```bash
cd ~/Savo_Pi
bash deploy/common/install_role_deps.sh --role core
```

The installer verifies Ubuntu, architecture, disk space, package-manager dependencies, and the role-scoped ROS dependency set. Resolve every `rosdep` failure before continuing.

## 3. Prepare persistent storage

Run once on a new Core installation, and again only when ownership or directory policy changes:

```bash
cd ~/Savo_Pi
sudo bash deploy/core/prepare_runtime_storage.sh \
  --owner "$USER" \
  --group "$USER"
```

The script creates the controlled runtime structure under:

```text
/var/lib/robot_savo/
/var/log/robot_savo/
```

Persistent state includes maps, release transactions, semantic locations, backups, supervisor state, and ROS-owned runtime data. Do not replace these directories with `/tmp` paths.

### Verify storage preparation

```bash
find /var/lib/robot_savo -maxdepth 3 -type d -printf '%M %u:%g %p\n'
find /var/log/robot_savo -maxdepth 2 -type d -printf '%M %u:%g %p\n'
```

## 4. Build and test the Core role

```bash
cd ~/Savo_Pi
bash deploy/core/build_core.sh --clean --test
```

### Required result

- All 14 Core packages are found.
- Build completes without error.
- Test result reports zero failures and zero errors.
- No package is skipped through `--allow-missing`.

## 5. Run source and readiness validators

```bash
cd ~/Savo_Pi
bash deploy/common/validate_full_bringup.sh
bash deploy/common/validate_pre_real_test_readiness.sh
```

`validate_full_bringup.sh` must pass. The aggregate validator may report a hardware or measurement `BLOCKED` condition, but it must not report a source-contract `FAIL`.

A geometry block is expected while the active geometry profile remains provisional. Do not remove the gate to make the validator green.

## 6. Review production environment

The systemd environment template is:

```text
deploy/systemd/robot-savo.env.example
```

Create the target environment file:

```bash
sudo install -d -m 0750 /etc/robot-savo
sudo install -m 0640 deploy/systemd/robot-savo.env.example \
  /etc/robot-savo/robot-savo.env
sudoedit /etc/robot-savo/robot-savo.env
```

### Verify at minimum

- `SAVO_ROOT` points to the deployed repository.
- `SAVO_WS` points to its `savo_ws` workspace.
- `ROS_DISTRO=jazzy`.
- ROS domain and middleware settings match Edge and observer hosts.
- State and log roots use the prepared persistent directories.
- No secrets are stored in a world-readable file.

## 7. Render Core systemd units

Render into a staging directory first:

```bash
cd ~/Savo_Pi
sudo bash deploy/systemd/render_units.sh \
  --user "$USER" \
  --group "$USER" \
  --root "$PWD" \
  --output-dir /tmp/robot-savo-units
```

Inspect the rendered Core unit and verify it before installation:

```bash
systemd-analyze verify /tmp/robot-savo-units/savo_core.service
sed -n '1,240p' /tmp/robot-savo-units/savo_core.service
```

Install only the selected production unit:

```bash
sudo install -m 0644 /tmp/robot-savo-units/savo_core.service \
  /etc/systemd/system/savo_core.service
sudo systemctl daemon-reload
```

Do not simultaneously enable the generic `savo.service` in Core mode. One role must have one service owner.

## 8. First safe-idle launch

For the first launch, use an interactive terminal before enabling automatic startup:

```bash
cd ~/Savo_Pi
bash deploy/core/run_core.sh
```

The default launch must remain:

- `robot_mode=safe_idle`
- `bringup_profile=lidar_only`
- `control_startup_mode=STOP`
- Locked geometry required
- Provisional geometry not allowed
- Visual odometry fusion disabled
- D435 voxel validation false

If geometry is still provisional, a motion-capable production path should fail closed. Record that block rather than bypassing it.

## 9. Enable the service after verification

After the interactive safe-idle launch and target checks pass:

```bash
sudo systemctl enable savo_core.service
sudo systemctl start savo_core.service
```

### Verify

```bash
systemctl status savo_core.service --no-pager
journalctl -u savo_core.service -b --no-pager
```

Then verify ROS state from a correctly configured shell:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Savo_Pi/savo_ws/install/setup.bash
ros2 node list
ros2 topic list
```

The presence of nodes is not enough. Confirm Core readiness, supervisor state, control mode `STOP`, sensor health, TF, and the absence of unauthorized command publishers.

## 10. Release update workflow

For later updates, prefer the staged updater:

```bash
cd ~/Savo_Pi
bash deploy/common/update_role.sh --role core --pull
```

This builds and tests in an isolated release directory and activates the install only after success. The previous install is retained for rollback.

Use `--no-restart` when the new install must be inspected before service restart. Do not combine `--pull` with `--allow-dirty`.

## Deployment acceptance criteria

Core deployment passes when:

- The exact source revision is recorded.
- Dependency installation succeeds.
- Persistent state paths exist with approved ownership.
- All Core packages build and test successfully.
- Validators show no source-contract failure.
- The rendered service passes systemd verification.
- Safe-idle starts without unintended motion.
- Control remains in `STOP`.
- Supervisor and package readiness are visible.
- Service restart behavior is verified.
- A backup and rollback path are available.

Physical motion remains a separate test authorization. Continue with the staged procedures in `docs/testing/full_robot_test_plan.md` and `docs/testing/real_robot_acceptance_checklist.md`.
