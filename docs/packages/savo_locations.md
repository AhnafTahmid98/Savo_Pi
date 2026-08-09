# savo_locations

## Purpose

Authoritative Core semantic-location registry, candidate review lifecycle, SQLite store, and map-associated location-release transaction.

## Deployment

Core only. `locations_bringup.launch.py` starts `savo_locations_node`; production paths are provisioned by `prepare_runtime_storage.sh`.

## Responsibilities

Normalize/resolve names and aliases; enforce unique identity/tag/map constraints; persist candidates, approved records, and event journal; expose typed read/write services; prepare/verify/commit/rollback location snapshots tied to a map release.

## Non-responsibilities and authority boundaries

Does not detect tags, create map poses, navigate, approve on behalf of an operator, or authorize operations. Mapping/head create evidence; mapping review gateway + supervisor + operator control mutation; nav uses only approved `approach_pose`.

## Package structure

C++ domain/catalog/validation, SQLite repository/store, release repository/services, ROS conversions/node, config, launch, and unit/ROS runtime tests.

## Runtime components

### `savo_locations_node` / node name `savo_locations`

Single production C++ registry. Opens/integrity-checks/migrates SQLite, bootstraps an in-memory read view, publishes status/snapshot/events, and serves typed operations. On write failure it retains last committed reads but latches writes unavailable until explicit recovery succeeds.

## Runtime data flow

`mapped tag candidate -> register -> pending review -> operator-authorized approve/reject -> approved record -> resolve -> nav`; AM-8 wraps catalog snapshot in prepare/verify/commit or rollback.

## ROS interfaces

### Published topics

| Topic | Type | Purpose |
| --- | --- | --- |
| `/savo_locations/status` | `std_msgs/msg/String` | Storage/write/readiness state, transient local |
| `/savo_locations/snapshot` | `std_msgs/msg/String` | Diagnostic catalog snapshot, transient local |
| `/savo_locations/heartbeat` | `std_msgs/msg/UInt64` | Liveness |
| `/savo_locations/events` | `savo_msgs/msg/LocationEvent` | Committed mutation event |

### Subscribed topics

None; mutations/queries are typed services.

### Services

| Service | Type | Purpose |
| --- | --- | --- |
| `/savo_locations/resolve` | `ResolveLocation` | Deterministic ID/name/alias lookup with optional map context |
| `/savo_locations/get`, `/list` | `GetLocation`, `ListLocations` | Approved catalog reads |
| `/candidates/get`, `/candidates/list` | `GetLocationCandidate`, `ListLocationCandidates` | Candidate reads |
| `/candidates/register` | `RegisterLocationCandidate` | Persist pending evidence |
| `/candidates/approve` | `ApproveLocation` | Atomic candidate-to-location approval |
| `/candidates/reject` | `RejectLocationCandidate` | Versioned reason-required rejection |
| `/set_enabled` | `SetLocationEnabled` | Versioned enable/disable |
| `/storage/recover` | `RecoverLocationStorage` | Explicit fresh-connection integrity recovery |
| `/releases/{prepare,verify,commit,rollback}` | Corresponding `savo_msgs` services | Joint map/location release transaction |

### Actions

No action server. `NavigateToLocation` is owned by `savo_nav`; `RegisterMappedLocation` by `savo_mapping`.

## TF ownership

None. Stored tag, approach, and confirmation poses use `map`; the registry does not transform them.

## Parameters and configuration

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `database_path` | `/var/lib/robot_savo/locations/locations.db` | SQLite authority |
| `releases_root` | `/var/lib/robot_savo/locations/releases` | Release snapshots |
| `auto_migrate` | `true` | Supported forward schema migration |
| `create_parent_directories` | `false` | Deployment owns permissions |
| `enable_write_services` | `true` | Maintenance read-only switch |
| status/heartbeat | `1/2 Hz` | Observability |

Policy requires map ID/revision, unique tag per map revision, pending revision match, approach pose for approval, ambiguous lookup failure, and separate tag/approach poses. SQLite uses WAL, foreign keys, 5 s busy timeout, schema/integrity checks, and fail-closed newer/corrupt schema behavior.

## Launch files

`locations_bringup.launch.py` loads node/storage policy and production paths.

## Persistent state and runtime files

SQLite database/WAL plus releases and backups under `/var/lib/robot_savo/locations`. Release artifacts include digests/context and active identity. Permissions are deployment-managed (`0750` directory, `0640` database policy).

## Hardware ownership

None.

## Dependencies

### Internal Robot Savo dependencies

`savo_msgs`; mapping registration/review/release clients; supervisor authorization; nav resolver; bridge/UI/observer reads.

### External ROS/system dependencies

SQLite3, filesystem/crypto digest utilities, ROS C++.

## Safety behavior

Duplicate IDs/tags, map mismatch, stale revisions, ambiguous names, missing approach pose, pending/unapproved/disabled records, corrupt storage, or release digest mismatch fail closed. SavoMind cannot approve locations.

## Failure and degraded behavior

Startup integrity failure prevents writable authority. A runtime write fault latches `write_ready=false`; last committed in-memory reads remain available. Recovery validates a fresh connection/catalog before swap.

## Startup and shutdown behavior

Validates schema/integrity, migrates supported older schema transactionally, bootstraps catalog, then advertises services. SQLite transactions roll back on close/failure.

## Build

`bash deploy/core/build_core.sh --clean --test`.

## Run

`ros2 launch savo_locations locations_bringup.launch.py`.

## Validation and testing

Domain normalization/validation, duplicates, candidates, approval atomicity, SQLite schema/integrity/hardening, persistence/restart, services/events, recovery, and release transaction tests.

## Current validation status

Implemented/source-tested; current physical candidate capture, operator review, release, restart persistence, duplicate handling, and named navigation require integration validation.

## Known limitations and remaining validation

Unicode case folding is deliberately disabled; ASCII normalization rules apply. Release correctness depends on locked map/geometry and operator process.

## Change-control considerations

Schema, normalization/identity, uniqueness, pose/map rules, service names, write recovery, or release digest changes require migration and rollback testing.

## Related documentation

- [Implementation README](../../savo_ws/src/core/savo_locations/README.md)
- [Mapping/navigation architecture](../architecture/mapping_navigation_architecture.md)
- [Recovery operations](../deployment/recovery_operations.md)
- [Ownership matrix](package_ownership_matrix.md)

