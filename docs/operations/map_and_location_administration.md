# Map and Location Administration

## Purpose and classification

This operator/maintainer guide covers controlled persistent state. It is
non-motion unless a separate navigation or head-confirmation operation is
started. Direct SQLite, release-directory, and active-manifest edits are
prohibited.

## Maps

Mapping session data lives under `/var/lib/robot_savo/maps/sessions`.
Production releases, immutable artifacts, and `active_map.yaml` live under
`maps/production`; transaction journals live under `maps/release_transactions`.

The lifecycle is:

```text
session -> save -> verify -> quality -> operator review
  -> transactional joint release -> active release -> navigation verification
```

Use read-only status/catalog topics and the approved mapping control surface to
inspect map identity, revision, quality, review generation, release ID, hashes,
and active state. A saved or quality-passing session is not active production
state. The current repository ships no standalone operator catalog/promotion,
deactivation, or rollback CLI; those mutations remain in the guarded autonomous
release workflow and maintainer rollback tooling.

Back up state immediately after an approved release and before rollback.

## Semantic locations

Candidates are registered from mapping/head evidence and persisted in
`/var/lib/robot_savo/locations/locations.db`. Review through the supported Core
CLI, which calls the supervisor-authorized mapping review gateway:

```bash
ros2 run savo_mapping location_review_cli list
ros2 run savo_mapping location_review_cli inspect <candidate-id>
ros2 run savo_mapping location_review_cli approve <candidate-id> \
  --actor <operator-id>
ros2 run savo_mapping location_review_cli reject <candidate-id> \
  --actor <operator-id> --reason "<review-reason>"
```

The CLI loads the current revision unless `--revision` is supplied. The gateway
rechecks pending state, revision, map context, and supervisor authorization.
Use `--json` when retaining machine-readable evidence.

Approval creates an approved location record; it does not itself create or
activate a location release. Joint release preparation, verification, commit,
and rollback remain controlled by the map release transaction. Named navigation
uses only enabled, unambiguous records compatible with the active map.

There is no supported deletion service or routine delete CLI. For duplicates,
reject a pending duplicate. For an already approved duplicate, stop use and
escalate for controlled enablement/release correction; never delete rows in
SQLite. SavoMind cannot approve or reject candidates, maps, or releases.

## Restart persistence and controlled state

Maps, map sessions/releases and release history, semantic locations, supervisor
state, and deployed configuration persist across restart. Their identities and
integrity must remain associated; deleting a file is not routine cleanup.

ROS nodes, active subscriptions, Unix sockets, volatile bridge/UI snapshots,
temporary mission state, and non-retained temporary logs/caches are transient.
Transient disappearance after shutdown is expected, but operators must not
delete runtime paths to recover a live system. Stop the owning service, preserve
evidence, and use the documented maintainer preparation or recovery path.

Retain actor, request/candidate/release IDs, expected/current revisions, map
identity, decision reason, result, database/release backup, and logs.
