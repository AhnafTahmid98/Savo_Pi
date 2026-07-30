# Robot Savo — `savo_locations`

`savo_locations` is the authoritative semantic-location registry for
Robot Savo.

## Ownership

This package owns:

- canonical location IDs;
- human-readable display names;
- deterministic aliases;
- semantic location types;
- active-map and map-revision binding;
- released-map provenance;
- safe navigation approach poses;
- optional confirmation poses;
- expected AprilTag family and ID;
- candidate and approved-location lifecycle;
- location enable/disable state;
- registry persistence and audit history.

This package does not own:

- AprilTag image detection;
- camera-to-map transformations;
- occupancy-grid validation;
- path planning;
- motor commands;
- natural-language intent classification;
- final robot-wide safety authorization.

## Package relationships

- `savo_head` visually detects and confirms AprilTags.
- `savo_mapping` georeferences observations and proposes candidates.
- `savo_locations` stores and resolves approved locations.
- `savo_nav` consumes a resolved approach pose and confirms arrival.
- SavoMind or `savo_bridge` resolves natural-language destination intent.
- `savo_supervisor` observes readiness and safety state.

## Locked LOC-0 terminology

Map context uses:

- `map_id`
- `map_revision`
- `map_release_id`

`map_revision` is the active numeric map revision used by runtime
contracts.

`map_release_id` records released-map provenance and is not a substitute
for `map_revision`.

## Pose separation

The AprilTag pose is not automatically a safe navigation goal.

A location may contain:

- `tag_pose_map`: actual AprilTag pose in the map frame;
- `approach_pose`: safe free-space robot navigation target;
- `confirmation_pose`: optional robot pose/orientation for viewing the tag.

Navigation must use the approved approach pose.

## Lifecycle

Candidate states:

- unknown
- pending review
- approved
- rejected

Location states:

- unknown
- approved
- retired

Only approved and enabled records may resolve for navigation.

## Storage policy

SQLite will become the authoritative runtime store in LOC-2.

YAML under `config/` is for schema policy and empty seed/import data.
Installed package YAML must not be modified as a runtime database.

## Current phase

LOC-0 provides:

- valid C++17 `ament_cmake` package structure;
- package ownership contract;
- stable service and topic names;
- lifecycle and semantic-type enums;
- schema-versioned empty seed data;
- static and C++ unit tests.

LOC-0 intentionally does not provide:

- a ROS node;
- SQLite persistence;
- candidate approval;
- alias normalization;
- location resolution;
- mapping integration;
- navigation integration;
- AprilTag runtime integration.

Those capabilities are introduced in later phases.
## LOC-1A normalization and validation

LOC-1A adds a ROS-independent deterministic C++ domain core.

It provides:

- ASCII whitespace trimming and collapsing;
- deterministic lookup-key normalization;
- canonical location-ID conversion and validation;
- pure C++ map, pose, tag, location and candidate models;
- finite numeric-value validation;
- mandatory `map` frame validation;
- normalized quaternion validation;
- alias collision detection;
- semantic-type validation;
- map-revision and AprilTag validation;
- stable machine-readable validation reason codes.

LOC-1A does not introduce a database, ROS node, service server,
launch file, mapping integration or navigation integration.

Lookup normalization performs ASCII case conversion only.
Non-ASCII UTF-8 bytes are preserved exactly so behavior does not
depend on the operating-system locale. Full Unicode case folding
may be added later through an explicitly selected dependency.
## LOC-1B in-memory registry

LOC-1B adds a deterministic, thread-safe, ROS-independent
in-memory location registry.

The registry supports:

- canonical location-ID insertion;
- globally unique location IDs;
- identity collision protection within a map revision;
- AprilTag collision protection within a map revision;
- exact ID, display-name and alias resolution;
- optional map-context enforcement;
- explicit ambiguous, disabled, retired and map-mismatch results;
- optimistic record revision checks;
- strict revision increment sequencing;
- revisioned enable/disable changes;
- deterministic location-ID-sorted listing.

A disabled location continues to reserve its identity and tag.
A retired location releases its identity and tag for future use.

A matching identity across different maps is allowed. Resolution
without map context must return ambiguous rather than selecting
one destination silently.

LOC-1B still does not add SQLite, a ROS node, ROS services,
launch files, mapping integration or navigation integration.
## LOC-1C candidate approval transaction

LOC-1C adds the deterministic candidate lifecycle and the
in-memory candidate-to-location approval transaction.

Candidate lifecycle:

- newly registered candidates begin as `pending_review`;
- candidate IDs are globally unique;
- pending candidate tags are unique within a map revision;
- updates require the expected candidate revision;
- updates are allowed only while pending;
- rejection requires a non-empty reason;
- approval and rejection increment the candidate revision.

Approval ownership:

- the operator supplies the semantic identity and safe approach
  pose;
- the candidate remains authoritative for `map_id`,
  `map_revision`, `map_release_id`, AprilTag family, AprilTag ID
  and `tag_pose_map`;
- the approved location stores the originating candidate ID;
- location records begin approved, enabled and at revision 1.

Approval is atomic from the catalog API. If location insertion
fails because of an ID, alias, identity or AprilTag conflict, the
candidate remains pending at its existing revision.

LOC-1C still does not introduce SQLite, a ROS node, ROS services,
launch files, mapping integration or navigation integration.
## LOC-2A SQLite foundation

LOC-2A introduces the persistent-store foundation without adding
a ROS node or record serialization.

Runtime database:

```text
/var/lib/robot_savo/locations/locations.db
```

Backup directory:

```text
/var/lib/robot_savo/locations/backups/
```

The store configures:

- WAL journaling for file-backed databases;
- foreign-key enforcement;
- `NORMAL` synchronous policy;
- a 5000 ms busy timeout;
- memory-backed SQLite temporary storage;
- full-mutex SQLite connection mode.

Schema migration is explicit and transactional. Databases with a
schema newer than the supported version fail closed. Migration 001
creates the location, alias, candidate, event, metadata and schema
history tables.

Active location tags, active normalized identity keys and pending
candidate tags use partial unique indexes. Retired locations and
terminal candidates therefore do not permanently reserve those
identities.

Integrity validation runs both SQLite `integrity_check` and
`foreign_key_check`.

LOC-2A does not yet serialize domain records, load the in-memory
catalog, create backups, expose ROS services or start a runtime
node. Those capabilities follow in later LOC-2 phases.
## LOC-2B typed SQLite snapshots

LOC-2B persists complete typed location and candidate snapshots.

Persisted locations include:

- approval/retirement state;
- enabled state and record revision;
- canonical identity, display name and aliases;
- semantic type;
- map ID, map revision and map release ID;
- approach, confirmation and tag poses;
- AprilTag binding;
- semantic area metadata;
- originating candidate ID.

Persisted candidates include:

- lifecycle state and candidate revision;
- mapping and AprilTag evidence;
- detection quality and observation statistics;
- optional approach and confirmation poses;
- suggested identity and semantic metadata;
- mapping session and source component;
- review reason and approved location binding.

Snapshot replacement uses one `BEGIN IMMEDIATE` transaction.
Existing locations and candidates are restored automatically if
any new row or identity index fails.

Domain validation runs before persistence and after loading.
Persisted data that no longer satisfies the domain contracts is
reported as corrupt and is not returned to runtime consumers.

LOC-2B still does not introduce a ROS node, runtime services,
automatic catalog rehydration, backup rotation or event writes.
## LOC-3A read-only ROS registry

LOC-3A adds the production C++ ROS registry node.

Startup behavior:

- opens the configured SQLite authority;
- applies supported schema migrations when enabled;
- performs SQLite integrity and foreign-key checks;
- loads and validates the complete persistent catalog;
- enters `ready` only after bootstrap succeeds;
- remains alive but fail-closed in `degraded` state when storage
  cannot be trusted.

Read services:

- `/savo_locations/resolve`
- `/savo_locations/get`
- `/savo_locations/list`

Runtime diagnostics:

- `/savo_locations/status`
- `/savo_locations/heartbeat`
- `/savo_locations/snapshot`

Status and snapshot use reliable transient-local JSON diagnostic
messages. Typed location access remains the three `savo_msgs`
read services.

LOC-3A deliberately does not expose candidate registration,
approval, rejection, enable/disable, import or other write paths.
Those operations require a separately validated write phase.


## LOC-3B2 persistent ROS write services

LOC-3B2 exposes the persistent mutation surface:

- `/savo_locations/candidates/register`;
- `/savo_locations/candidates/approve`;
- `/savo_locations/set_enabled`;
- `/savo_locations/events` for reliable post-commit events.

Every mutation is serialized, validated against a complete hydrated catalog,
and committed through the dedicated atomic SQLite transaction API introduced
in LOC-3B1. Service success is returned only after the snapshot and immutable
event row commit together. Validation, duplicate, conflict, stale-revision and
no-op requests do not modify persistent state.

A persistence or event-journal failure disables further writes until restart
while the last committed in-memory read view remains available. Status reports
separate `read_ready` and `write_ready` fields so supervision can distinguish a
readable registry from a write-degraded registry.


## LOC-3P persistent runtime hardening

A failed SQLite mutation disables further writes while preserving the
last committed in-memory catalog for read services. Write readiness is
restored only through `/savo_locations/storage/recover`. Recovery opens
a new connection, validates schema and SQLite integrity, bootstraps the
full catalog and event journal into temporary state, validates catalog
hydration, and swaps the active repository only after all checks pass.

Simultaneous mutation requests remain serialized. Concurrency tests
require exactly one commit when two operators attempt to approve the
same candidate revision.
