#ifndef SAVO_LOCATIONS__SQLITE_REPOSITORY_HPP_
#define SAVO_LOCATIONS__SQLITE_REPOSITORY_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "savo_locations/model.hpp"
#include "savo_locations/sqlite_store.hpp"
#include "savo_locations/validation.hpp"

namespace savo_locations
{

enum class SnapshotCode : std::uint8_t
{
  kOk = 0U,
  kInvalidArgument,
  kStoreNotOpen,
  kValidationFailed,
  kIdentityConflict,
  kTagConflict,
  kTransactionActive,
  kSqlError,
  kCorruptData,
  kStaleRevision,
  kCandidateRegistrationDeltaInvalid,
  kApprovalDeltaInvalid,
  kLocationEnabledDeltaInvalid,
  kEventJournalError,
};


struct SnapshotResult
{
  bool success{false};

  SnapshotCode code{
    SnapshotCode::kSqlError};

  int sqlite_code{0};
  std::string reason;

  std::vector<ValidationIssue>
    validation_issues;
};


struct CatalogSnapshot
{
  std::vector<LocationRecordData> locations;
  std::vector<CandidateRecordData> candidates;
};


[[nodiscard]]
std::string_view to_string(
  SnapshotCode code) noexcept;


enum class PersistenceEventType : std::uint8_t
{
  kUnknown = 0U,
  kSnapshotReplaced = 1U,
  kCandidateApproved = 2U,
  kCandidateRegistered = 3U,
  kCandidateRejected = 4U,
  kLocationEnabledChanged = 5U,
};


struct PersistenceEvent
{
  std::uint64_t sequence{0U};
  std::int64_t event_time_unix_ns{0};

  PersistenceEventType event_type{
    PersistenceEventType::kUnknown};

  std::string candidate_id;
  std::string location_id;

  std::uint64_t entity_revision{0U};

  std::string actor_id;
  std::string reason;
  std::string payload_json{"{}"};
};


struct BootstrapReport
{
  std::uint32_t schema_version{0U};
  bool integrity_healthy{false};

  std::size_t location_count{0U};
  std::size_t candidate_count{0U};

  std::uint64_t event_count{0U};
  std::uint64_t last_event_sequence{0U};
};


struct CandidateRegistrationCommit
{
  std::string candidate_id;

  std::string actor_id;
  std::string reason;
  std::string payload_json{"{}"};

  CatalogSnapshot post_registration_snapshot;
};


struct CandidateApprovalCommit
{
  std::string candidate_id;

  std::uint64_t
    expected_candidate_revision{0U};

  std::string approved_location_id;

  std::string actor_id;
  std::string reason;
  std::string payload_json{"{}"};

  CatalogSnapshot post_approval_snapshot;
};


struct LocationEnabledCommit
{
  std::string location_id;

  std::uint64_t
    expected_record_revision{0U};

  bool enabled{false};

  std::string actor_id;
  std::string reason;
  std::string payload_json{"{}"};

  CatalogSnapshot post_update_snapshot;
};


class SqliteRepository
{
public:
  explicit SqliteRepository(
    SqliteStore & store);

  [[nodiscard]]
  SnapshotResult save_snapshot(
    const CatalogSnapshot & snapshot);

  [[nodiscard]]
  SnapshotResult load_snapshot(
    CatalogSnapshot * snapshot) const;

[[nodiscard]]
SnapshotResult bootstrap(
  CatalogSnapshot * snapshot,
  BootstrapReport * report) const;

[[nodiscard]]
SnapshotResult append_event(
  const PersistenceEvent & event,
  std::uint64_t * sequence);

[[nodiscard]]
SnapshotResult list_events(
  std::uint64_t after_sequence,
  std::size_t limit,
  std::vector<PersistenceEvent> * events) const;

[[nodiscard]]
SnapshotResult commit_candidate_registration(
  const CandidateRegistrationCommit & request,
  std::uint64_t * event_sequence);

[[nodiscard]]
SnapshotResult commit_candidate_approval(
  const CandidateApprovalCommit & request,
  std::uint64_t * event_sequence);

[[nodiscard]]
SnapshotResult commit_location_enabled(
  const LocationEnabledCommit & request,
  std::uint64_t * event_sequence);

private:
  SqliteStore & store_;
};

}  // namespace savo_locations

#endif  // SAVO_LOCATIONS__SQLITE_REPOSITORY_HPP_
