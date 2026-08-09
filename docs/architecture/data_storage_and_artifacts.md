# Data Storage and Artifacts

Core owns authoritative persistent robot state. Edge runtime IPC is volatile.

| Path | Owner | Contents / lifecycle |
| --- | --- | --- |
| `/var/lib/robot_savo/maps/sessions` | Mapping | Session maps, mission and quality artifacts |
| `/var/lib/robot_savo/maps/production` | Mapping/Nav | Immutable releases and `active_map.yaml` |
| `/var/lib/robot_savo/maps/release_transactions` | Mapping | Promotion journals and recovery evidence |
| `/var/lib/robot_savo/locations/locations.db` | Locations | Authoritative SQLite registry |
| `/var/lib/robot_savo/locations/releases` | Locations | Location release artifacts |
| `/var/lib/robot_savo/locations/backups` | Locations/operations | Registry backups |
| `/var/lib/robot_savo/supervisor/system_state.json` | Supervisor | Persistent authority/system state |
| `/var/lib/robot_savo/ros` | Deployment | Core ROS home |
| `/var/log/robot_savo` | Deployment/packages | Core production logs |
| `/run/savo_bridge` | Bridge | Volatile socket and observation snapshot |
| `/run/savomind` | Speech/deployment | Volatile speech socket directory |

Map promotion is transactional and hash/manifest driven. Navigation must verify the active release before admission. Locations have a separate candidate/review/release lifecycle. Supervisor state must be parsed and validated; file presence alone does not grant authority.

Deployment creates directories with service-specific ownership and systemd write restrictions. Backup/restore tooling targets Core state, validates integrity, and protects against accidental overwrite. Runtime sockets and snapshots are intentionally excluded from backups.

Disk-full, corrupt/missing manifest, incomplete transaction, invalid active release, SQLite error, or state parse failure must block the dependent operation rather than silently create authority. Validate free space, ownership/modes, atomic replacement/recovery, backup/restore, release rollback, log rotation, and power-loss behavior on the target filesystem.
