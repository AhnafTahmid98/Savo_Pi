# Recovery operations

Create an offline state archive with `deploy/common/backup_robot_state.sh
--output /safe/path/state.tar.gz`. Restore only while services are stopped; first
test without `--overwrite`, then use it explicitly. The restore preserves the
previous root, rejects traversal/symlinks, checks hashes and SQLite integrity.

- Core reboot: safe-idle/STOP first; run storage preflight before enabling roles.
- Edge reboot/loss: core must continue safely; cancel voice/UI sessions and wait
  for fresh edge health after return.
- DDS loss: STOP/cancel active actions; verify domain, link, firewall, and time.
- Camera loss: keep D435/voxel disabled; LiDAR safety authority remains intact.
- Database failure: stop location services, back up, validate/restore SQLite, then
  verify map/release correlation.
- Map storage full: cancel mapping, rotate logs/archive state, never delete active
  release artifacts ad hoc.
- Service crash: inspect journal and health; restart only after cause and STOP are
  confirmed. Restart limits prevent loops.
