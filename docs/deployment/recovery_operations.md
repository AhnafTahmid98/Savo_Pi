# Recovery Operations

## Purpose

This document defines safe recovery for failed builds, failed service starts, bad releases, corrupted persistent state, network loss, and component crashes. Recovery must preserve physical safety and evidence before attempting restoration.

## First response

For any unexpected behavior:

1. Stop or revoke motion authority.
2. Use the physical emergency stop when movement is unsafe or cannot be confirmed stopped.
3. Cancel active navigation or mapping goals when the interface remains available.
4. Stop the affected role service.
5. Record time, host, source revision, active mode/profile, and visible symptoms.
6. Preserve logs before restarting repeatedly.
7. Identify whether the failure is source, build, service, state, hardware, network, or configuration related.

Do not begin by deleting build, persistent state, or logs. Those actions can remove evidence and make recovery harder.

## Service recovery

Inspect the unit:

```bash
systemctl status savo_core.service --no-pager
journalctl -u savo_core.service -b --no-pager
```

Use the Edge service name on Edge.

Before restart, confirm:

- No duplicate service owns the same role/component.
- Required runtime directories exist and are writable.
- The environment file points to the intended repository and workspace.
- The active install link resolves.
- Core startup remains `STOP`.
- Geometry and map gates were not weakened.

Then restart once:

```bash
sudo systemctl restart savo_core.service
```

If it fails again, stop the unit and diagnose instead of relying on an indefinite restart loop.

## Roll back a staged role release

`deploy/common/update_role.sh` activates a completed release only after its build and tests pass. The prior install is retained as:

```text
savo_ws/install.previous.<UTC timestamp>
```

To roll back:

```bash
sudo systemctl stop savo_core.service
cd ~/Savo_Pi/savo_ws
ls -ld install install.previous.* .releases/*/install
```

Select the exact retained install from the release record. Preserve the failed active link/path for evidence, then switch atomically where possible:

```bash
mv install install.failed.$(date -u +%Y%m%dT%H%M%SZ)
ln -sfn "$PWD/install.previous.<timestamp>" install.next
mv -Tf install.next install
```

Verify:

```bash
readlink -f install
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg prefix savo_bringup
```

Start in safe idle:

```bash
sudo systemctl start savo_core.service
```

Use the Edge service name for an Edge rollback. Record the failed and restored releases. Do not delete either until the incident is closed.

## Failed staged update

If `update_role.sh` fails during build or tests, the active install should remain unchanged and a previously active service should be restarted by the script's exit trap unless `--no-restart` was used.

Verify:

```bash
systemctl status savo_core.service --no-pager
readlink -f ~/Savo_Pi/savo_ws/install
find ~/Savo_Pi/savo_ws/.releases -maxdepth 2 -type d -printf '%TY-%Tm-%Td %TT %p\n' | sort
```

Retain the failed release directory until logs and test results are collected.

## Persistent-state backup

Create a backup before planned deployment or destructive maintenance:

```bash
cd ~/Savo_Pi
sudo bash deploy/common/backup_robot_state.sh
```

Review the script options when using a non-default state root or backup destination:

```bash
bash deploy/common/backup_robot_state.sh --help
```

The backup workflow includes metadata and SHA-256 integrity information for maps, locations, supervisor state, and optional configuration.

A backup is accepted only when:

- The archive was created successfully.
- Its integrity manifest verifies.
- The source revision and active release are recorded.
- The storage destination is separate from the state being protected.

## Restore persistent state

Stop all Robot Savo services before restore. Inspect the restore command:

```bash
bash deploy/common/restore_robot_state.sh --help
```

The restore workflow rejects path traversal, symbolic-link payloads, hash mismatch, invalid SQLite state, and accidental overwrite of non-empty state. Use `--overwrite` only after reviewing the existing state and confirming the backup identity.

The restore process preserves the previous state root as a timestamped pre-restore copy when overwrite is authorized.

After restore:

1. Verify file ownership and permissions.
2. Verify map release hashes and active-map contract.
3. Verify the locations database and release identity.
4. Verify supervisor state parses and matches the expected map context.
5. Start Core in safe idle.
6. Confirm `STOP` and readiness before any mission.

## Corrupted map or release state

If production map verification fails:

- Do not bypass AM-8 release checks.
- Do not edit hashes manually to match corrupted content.
- Stop navigation activation.
- Restore a known approved map release or repeat the mapping/review/release workflow.
- Confirm location releases reference the correct map identity and geometry digest.

## Corrupted location database

Stop Core and back up the damaged database before repair. Use the package-provided validation/export/restore paths where available. Do not replace the database with an empty file while a production release still references it.

After recovery, test candidate, approval, release, restart persistence, and named-location lookup before navigation.

## Network or time loss

When Core–Edge discovery or timestamps fail:

```bash
ip -brief address
ping -c 3 192.168.50.1   # from Edge, adjust as configured
chronyc tracking
chronyc sources -v
```

Confirm ROS domain and middleware settings on both hosts. Keep VO fusion, speech command use, and autonomous missions blocked until discovery and clock stability return.

## Device recovery

For LiDAR, RealSense, audio, UI, power, or sensor-device loss:

- Return to safe idle.
- Stop the single owning node/service.
- Inspect kernel and service logs.
- Verify cable, power, USB/I2C/GPIO ownership, and permissions.
- Restart only the approved owner.
- Rerun the component validation before restoring mission capability.

Repeated unplug/replug cycles are not a substitute for diagnosing power or bus instability.

## Health scripts

The repository provides health utilities under `deploy/common/`, including:

- `check_service_health.sh`
- `robot_savo_health_check.sh`

Review their arguments before use. A health script is evidence for the checks it implements, not a general declaration that the robot is safe to move.

## Recovery acceptance criteria

Recovery is complete when:

- Physical motion is controlled and no uncommanded motion occurred.
- The intended source/install/state identity is known.
- The affected service starts in safe idle.
- Core control is `STOP`.
- Persistent state passes integrity checks.
- Duplicate owners are absent.
- The original failure is understood or formally contained.
- Required component/regression tests pass.
- Logs and incident notes are retained.
- Motion is reauthorized only through the normal gate.
