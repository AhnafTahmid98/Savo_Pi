# Backup, Restore, and Rollback

## Purpose and classification

Maintainer-only controlled recovery for Core persistent state and staged role
installs. Stop robot services before backup/restore so maps, SQLite, supervisor,
and release journals are not changing. Keep physical motion disabled.

## State backup

1. Cancel missions, command `STOP`, stop Edge then Core, and confirm inactive.
2. Choose an access-controlled archive path on storage separate from
   `/var/lib/robot_savo`.
3. Run the required command:

   ```bash
   cd ~/Savo_Pi
   sudo bash deploy/common/backup_robot_state.sh \
     --output <backup-directory>/robot-savo-state-<UTC-timestamp>.tar.gz
   ```

The archive contains `maps`, `locations`, `supervisor`, backup metadata,
SHA-256 manifests, and a copy of `/etc/robot-savo` when present. Treat it as
sensitive because configuration may contain deployment details.

Verify the outer checksum:

```bash
cd <backup-directory>
sha256sum -c robot-savo-state-<UTC-timestamp>.tar.gz.sha256
```

## Restore

1. Preserve logs, commits, active releases, and the damaged state.
2. Confirm all Robot Savo services are stopped.
3. Verify archive identity/checksum and available disk space.
4. Restore to an empty target first when practical:

   ```bash
   sudo bash deploy/common/restore_robot_state.sh \
     --archive <backup-archive>
   ```

The script rejects unsafe archive paths, symlinks, hash failures, invalid
SQLite databases, and a non-empty state root. For an authorized overwrite:

```bash
sudo bash deploy/common/restore_robot_state.sh \
  --archive <backup-archive> --overwrite
```

With `--overwrite`, the existing state is copied to
`/var/lib/robot_savo.pre-restore.<UTC timestamp>` before restoration. Do not
remove that copy until recovery is accepted. The current implementation then
copies the archive over the non-empty state/config roots; it does not remove
unrelated stale files. Treat this as an overlay restore, inspect for stale map,
release, location, supervisor, and configuration artifacts, and escalate if an
exact replacement is required. Do not improvise by deleting production state.

## Software rollback

`update_role.sh` retains `savo_ws/install.previous.<UTC timestamp>`. There is no
one-command rollback script. Follow the maintainer procedure in
[recovery operations](../deployment/recovery_operations.md): stop the role,
record current/previous targets, preserve the failed install, atomically point
`install` to the selected retained install, verify `ros2 pkg prefix
savo_bringup`, then start the role in safe idle. Never guess the timestamp or
delete the failed release before incident closure.

## Post-restore or rollback acceptance

- [ ] File ownership/permissions and storage preflight pass.
- [ ] Active map contract, referenced artifacts, hashes, and joint release agree.
- [ ] Locations SQLite integrity and release/map association pass.
- [ ] Supervisor state parses, is expected, and starts unarmed/controlled.
- [ ] Core starts in `safe_idle`; control reports `STOP`.
- [ ] Required readiness, TF, safety, power, and diagnostics pass.
- [ ] A fresh backup and recovery evidence are retained.
- [ ] Applicable regression and explicit return-to-service approval complete.
