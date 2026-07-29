#ifndef SAVO_LOCATIONS__SQLITE_SCHEMA_HPP_
#define SAVO_LOCATIONS__SQLITE_SCHEMA_HPP_

#include <cstdint>
#include <string_view>

namespace savo_locations
{

inline constexpr std::uint32_t
  kSupportedSqliteSchemaVersion{2U};

inline constexpr std::string_view
  kMigration001Sql{R"SQL(
CREATE TABLE IF NOT EXISTS schema_migrations (
  schema_version INTEGER PRIMARY KEY
    CHECK(schema_version > 0),
  migration_name TEXT NOT NULL,
  applied_at_unix_ns INTEGER NOT NULL
    CHECK(applied_at_unix_ns > 0)
);

CREATE TABLE IF NOT EXISTS registry_metadata (
  metadata_key TEXT PRIMARY KEY,
  metadata_value TEXT NOT NULL,
  updated_at_unix_ns INTEGER NOT NULL
    CHECK(updated_at_unix_ns > 0)
);

CREATE TABLE IF NOT EXISTS locations (
  location_id TEXT PRIMARY KEY,

  state INTEGER NOT NULL
    CHECK(state IN (1, 2)),

  enabled INTEGER NOT NULL
    CHECK(enabled IN (0, 1)),

  record_revision INTEGER NOT NULL
    CHECK(record_revision > 0),

  display_name TEXT NOT NULL,
  semantic_type TEXT NOT NULL,

  map_id TEXT NOT NULL,

  map_revision INTEGER NOT NULL
    CHECK(map_revision > 0),

  map_release_id TEXT NOT NULL DEFAULT '',

  approach_frame_id TEXT NOT NULL
    CHECK(approach_frame_id = 'map'),

  approach_x REAL NOT NULL,
  approach_y REAL NOT NULL,
  approach_z REAL NOT NULL,

  approach_qx REAL NOT NULL,
  approach_qy REAL NOT NULL,
  approach_qz REAL NOT NULL,
  approach_qw REAL NOT NULL,

  confirmation_pose_valid INTEGER NOT NULL
    CHECK(confirmation_pose_valid IN (0, 1)),

  confirmation_frame_id TEXT,
  confirmation_x REAL,
  confirmation_y REAL,
  confirmation_z REAL,
  confirmation_qx REAL,
  confirmation_qy REAL,
  confirmation_qz REAL,
  confirmation_qw REAL,

  tag_family TEXT NOT NULL,

  tag_id INTEGER NOT NULL
    CHECK(tag_id >= 0),

  tag_pose_map_valid INTEGER NOT NULL
    CHECK(tag_pose_map_valid IN (0, 1)),

  tag_frame_id TEXT,
  tag_x REAL,
  tag_y REAL,
  tag_z REAL,
  tag_qx REAL,
  tag_qy REAL,
  tag_qz REAL,
  tag_qw REAL,

  arrival_confirmation_required INTEGER NOT NULL
    CHECK(arrival_confirmation_required IN (0, 1)),

  building TEXT NOT NULL DEFAULT '',
  floor TEXT NOT NULL DEFAULT '',
  area TEXT NOT NULL DEFAULT '',
  notes TEXT NOT NULL DEFAULT '',

  source_candidate_id TEXT NOT NULL DEFAULT '',

  created_at_unix_ns INTEGER NOT NULL
    CHECK(created_at_unix_ns > 0),

  updated_at_unix_ns INTEGER NOT NULL
    CHECK(updated_at_unix_ns > 0)
);

CREATE TABLE IF NOT EXISTS location_aliases (
  location_id TEXT NOT NULL,

  alias_order INTEGER NOT NULL
    CHECK(alias_order >= 0),

  alias_kind INTEGER NOT NULL
    CHECK(alias_kind IN (1, 2, 3)),

  alias_text TEXT NOT NULL,
  normalized_key TEXT NOT NULL,

  map_id TEXT NOT NULL,

  map_revision INTEGER NOT NULL
    CHECK(map_revision > 0),

  reserves_identity INTEGER NOT NULL
    CHECK(reserves_identity IN (0, 1)),

  PRIMARY KEY(location_id, alias_order),

  UNIQUE(location_id, normalized_key),

  FOREIGN KEY(location_id)
    REFERENCES locations(location_id)
    ON UPDATE CASCADE
    ON DELETE CASCADE
);

CREATE TABLE IF NOT EXISTS location_candidates (
  candidate_id TEXT PRIMARY KEY,

  state INTEGER NOT NULL
    CHECK(state IN (1, 2, 3)),

  candidate_revision INTEGER NOT NULL
    CHECK(candidate_revision > 0),

  map_id TEXT NOT NULL,

  map_revision INTEGER NOT NULL
    CHECK(map_revision > 0),

  map_release_id TEXT NOT NULL DEFAULT '',

  tag_family TEXT NOT NULL,

  tag_id INTEGER NOT NULL
    CHECK(tag_id >= 0),

  tag_frame_id TEXT NOT NULL
    CHECK(tag_frame_id = 'map'),

  tag_x REAL NOT NULL,
  tag_y REAL NOT NULL,
  tag_z REAL NOT NULL,

  tag_qx REAL NOT NULL,
  tag_qy REAL NOT NULL,
  tag_qz REAL NOT NULL,
  tag_qw REAL NOT NULL,

  detection_quality REAL NOT NULL
    CHECK(
      detection_quality >= 0.0 AND
      detection_quality <= 1.0
    ),

  accepted_observations INTEGER NOT NULL
    CHECK(accepted_observations > 0),

  position_stddev_m REAL NOT NULL
    CHECK(position_stddev_m >= 0.0),

  yaw_stddev_rad REAL NOT NULL
    CHECK(yaw_stddev_rad >= 0.0),

  approach_pose_valid INTEGER NOT NULL
    CHECK(approach_pose_valid IN (0, 1)),

  approach_frame_id TEXT,
  approach_x REAL,
  approach_y REAL,
  approach_z REAL,
  approach_qx REAL,
  approach_qy REAL,
  approach_qz REAL,
  approach_qw REAL,

  confirmation_pose_valid INTEGER NOT NULL
    CHECK(confirmation_pose_valid IN (0, 1)),

  confirmation_frame_id TEXT,
  confirmation_x REAL,
  confirmation_y REAL,
  confirmation_z REAL,
  confirmation_qx REAL,
  confirmation_qy REAL,
  confirmation_qz REAL,
  confirmation_qw REAL,

  suggested_location_id TEXT NOT NULL DEFAULT '',
  suggested_display_name TEXT NOT NULL DEFAULT '',
  suggested_semantic_type TEXT NOT NULL DEFAULT '',

  building TEXT NOT NULL DEFAULT '',
  floor TEXT NOT NULL DEFAULT '',
  area TEXT NOT NULL DEFAULT '',
  notes TEXT NOT NULL DEFAULT '',

  source_session_id TEXT NOT NULL DEFAULT '',
  source_component TEXT NOT NULL DEFAULT '',

  review_reason TEXT NOT NULL DEFAULT '',
  approved_location_id TEXT NOT NULL DEFAULT '',

  created_at_unix_ns INTEGER NOT NULL
    CHECK(created_at_unix_ns > 0),

  updated_at_unix_ns INTEGER NOT NULL
    CHECK(updated_at_unix_ns > 0)
);

CREATE TABLE IF NOT EXISTS candidate_aliases (
  candidate_id TEXT NOT NULL,

  alias_order INTEGER NOT NULL
    CHECK(alias_order >= 0),

  alias_text TEXT NOT NULL,
  normalized_key TEXT NOT NULL,

  PRIMARY KEY(candidate_id, alias_order),

  UNIQUE(candidate_id, normalized_key),

  FOREIGN KEY(candidate_id)
    REFERENCES location_candidates(candidate_id)
    ON UPDATE CASCADE
    ON DELETE CASCADE
);

CREATE TABLE IF NOT EXISTS location_events (
  event_sequence INTEGER PRIMARY KEY AUTOINCREMENT,

  event_time_unix_ns INTEGER NOT NULL
    CHECK(event_time_unix_ns > 0),

  event_type INTEGER NOT NULL
    CHECK(event_type >= 0),

  candidate_id TEXT NOT NULL DEFAULT '',
  location_id TEXT NOT NULL DEFAULT '',

  entity_revision INTEGER NOT NULL
    CHECK(entity_revision >= 0),

  actor_id TEXT NOT NULL DEFAULT '',
  reason TEXT NOT NULL DEFAULT '',

  event_payload_json TEXT NOT NULL DEFAULT '{}'
);

CREATE UNIQUE INDEX IF NOT EXISTS
  idx_locations_active_tag_unique
ON locations(
  map_id,
  map_revision,
  tag_family,
  tag_id
)
WHERE state != 2;

CREATE UNIQUE INDEX IF NOT EXISTS
  idx_location_aliases_active_identity_unique
ON location_aliases(
  map_id,
  map_revision,
  normalized_key
)
WHERE reserves_identity = 1;

CREATE UNIQUE INDEX IF NOT EXISTS
  idx_candidates_pending_tag_unique
ON location_candidates(
  map_id,
  map_revision,
  tag_family,
  tag_id
)
WHERE state = 1;

CREATE INDEX IF NOT EXISTS
  idx_locations_map_context
ON locations(
  map_id,
  map_revision,
  state,
  enabled
);

CREATE INDEX IF NOT EXISTS
  idx_candidates_state
ON location_candidates(
  state,
  map_id,
  map_revision
);

CREATE INDEX IF NOT EXISTS
  idx_location_events_location
ON location_events(
  location_id,
  event_sequence
);

CREATE INDEX IF NOT EXISTS
  idx_location_events_candidate
ON location_events(
  candidate_id,
  event_sequence
);
)SQL"};

inline constexpr std::string_view
  kMigration002Sql{R"SQL(
CREATE TRIGGER IF NOT EXISTS
  location_events_reject_update
BEFORE UPDATE ON location_events
BEGIN
  SELECT RAISE(
    ABORT,
    'location_events is append-only'
  );
END;

CREATE TRIGGER IF NOT EXISTS
  location_events_reject_delete
BEFORE DELETE ON location_events
BEGIN
  SELECT RAISE(
    ABORT,
    'location_events is append-only'
  );
END;
)SQL"};


}  // namespace savo_locations

#endif  // SAVO_LOCATIONS__SQLITE_SCHEMA_HPP_
