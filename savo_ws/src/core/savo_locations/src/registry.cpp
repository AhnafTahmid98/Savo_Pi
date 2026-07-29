#include "savo_locations/registry.hpp"

#include <limits>
#include <mutex>
#include <set>
#include <utility>

#include "savo_locations/normalization.hpp"

namespace savo_locations
{
namespace
{

struct Match
{
  LocationRecordData record;
  MatchType type{MatchType::kNone};
};


bool same_map_context(
  const MapContext & left,
  const MapContext & right) noexcept
{
  return
    left.map_id == right.map_id &&
    left.map_revision == right.map_revision;
}


bool reserves_identity(
  const LocationRecordData & record) noexcept
{
  return record.state != LocationState::kRetired;
}


std::set<std::string> identity_keys(
  const LocationRecordData & record)
{
  std::set<std::string> keys;

  const auto id_key =
    normalize_lookup_key(
      record.location.location_id);

  const auto display_key =
    normalize_lookup_key(
      record.location.display_name);

  if (!id_key.empty()) {
    keys.insert(id_key);
  }

  if (!display_key.empty()) {
    keys.insert(display_key);
  }

  for (
    const auto & alias :
    record.location.aliases)
  {
    const auto alias_key =
      normalize_lookup_key(alias);

    if (!alias_key.empty()) {
      keys.insert(alias_key);
    }
  }

  return keys;
}


bool records_have_identity_overlap(
  const LocationRecordData & left,
  const LocationRecordData & right)
{
  const auto left_keys =
    identity_keys(left);

  const auto right_keys =
    identity_keys(right);

  for (const auto & key : left_keys) {
    if (right_keys.count(key) != 0U) {
      return true;
    }
  }

  return false;
}


bool valid_record_envelope(
  const LocationRecordData & record,
  std::string * reason)
{
  if (record.record_revision == 0U) {
    if (reason != nullptr) {
      *reason =
        "record revision must be greater than zero";
    }

    return false;
  }

  if (record.state == LocationState::kUnknown) {
    if (reason != nullptr) {
      *reason =
        "location state must not be unknown";
    }

    return false;
  }

  if (
    record.state == LocationState::kRetired &&
    record.enabled)
  {
    if (reason != nullptr) {
      *reason =
        "retired locations must be disabled";
    }

    return false;
  }

  return true;
}


RegistryMutationResult mutation_failure(
  const MutationCode code,
  std::string reason)
{
  RegistryMutationResult result;
  result.success = false;
  result.code = code;
  result.reason = std::move(reason);
  return result;
}


RegistryMutationResult validation_failure(
  const ValidationResult & validation,
  std::string reason)
{
  RegistryMutationResult result;
  result.success = false;
  result.code = MutationCode::kInvalidRecord;
  result.reason = std::move(reason);
  result.validation_issues = validation.issues();
  return result;
}


RegistryMutationResult mutation_success(
  const MutationCode code,
  std::string reason,
  const LocationRecordData & record)
{
  RegistryMutationResult result;
  result.success = true;
  result.code = code;
  result.reason = std::move(reason);
  result.record = record;
  return result;
}


ResolveResult resolve_status(
  const LocationRecordData & record,
  const MatchType match_type,
  const std::string & normalized_query)
{
  ResolveResult result;
  result.normalized_query = normalized_query;
  result.match_type = match_type;
  result.record = record;

  if (record.state == LocationState::kRetired) {
    result.resolved = false;
    result.code = ResolveCode::kRetired;
    result.reason = "location is retired";
    return result;
  }

  if (!record.enabled) {
    result.resolved = false;
    result.code = ResolveCode::kDisabled;
    result.reason = "location is disabled";
    return result;
  }

  result.resolved = true;
  result.code = ResolveCode::kResolved;
  result.reason = "location resolved";
  return result;
}


std::optional<MatchType> generic_match_type(
  const LocationRecordData & record,
  const std::string & normalized_query)
{
  if (
    normalize_lookup_key(
      record.location.location_id) ==
    normalized_query)
  {
    return MatchType::kLocationId;
  }

  if (
    normalize_lookup_key(
      record.location.display_name) ==
    normalized_query)
  {
    return MatchType::kDisplayName;
  }

  for (
    const auto & alias :
    record.location.aliases)
  {
    if (
      normalize_lookup_key(alias) ==
      normalized_query)
    {
      return MatchType::kAlias;
    }
  }

  return std::nullopt;
}

}  // namespace


std::string_view to_string(
  const MutationCode code) noexcept
{
  switch (code) {
    case MutationCode::kInserted:
      return "inserted";

    case MutationCode::kUpdated:
      return "updated";

    case MutationCode::kNotFound:
      return "not_found";

    case MutationCode::kInvalidRecord:
      return "invalid_record";

    case MutationCode::kStaleRevision:
      return "stale_revision";

    case MutationCode::kRevisionSequenceError:
      return "revision_sequence_error";

    case MutationCode::kLocationIdConflict:
      return "location_id_conflict";

    case MutationCode::kIdentityConflict:
      return "identity_conflict";

    case MutationCode::kTagConflict:
      return "tag_conflict";

    case MutationCode::kRetired:
      return "retired";

    default:
      return "unknown";
  }
}


std::string_view to_string(
  const ResolveCode code) noexcept
{
  switch (code) {
    case ResolveCode::kResolved:
      return "resolved";

    case ResolveCode::kInvalidQuery:
      return "invalid_query";

    case ResolveCode::kNotFound:
      return "not_found";

    case ResolveCode::kAmbiguous:
      return "ambiguous";

    case ResolveCode::kDisabled:
      return "disabled";

    case ResolveCode::kRetired:
      return "retired";

    case ResolveCode::kMapMismatch:
      return "map_mismatch";

    default:
      return "unknown";
  }
}


std::string_view to_string(
  const MatchType type) noexcept
{
  switch (type) {
    case MatchType::kLocationId:
      return "location_id";

    case MatchType::kDisplayName:
      return "display_name";

    case MatchType::kAlias:
      return "alias";

    case MatchType::kNone:
    default:
      return "none";
  }
}


RegistryMutationResult InMemoryRegistry::insert(
  LocationRecordData record)
{
  const auto validation =
    validate_location_draft(record.location);

  if (!validation.valid()) {
    return validation_failure(
      validation,
      "location record validation failed");
  }

  std::string envelope_reason;

  if (
    !valid_record_envelope(
      record,
      &envelope_reason))
  {
    return mutation_failure(
      MutationCode::kInvalidRecord,
      envelope_reason);
  }

  std::unique_lock<std::shared_mutex> lock{
    mutex_};

  if (
    records_.find(
      record.location.location_id) !=
    records_.end())
  {
    return mutation_failure(
      MutationCode::kLocationIdConflict,
      "location ID already exists");
  }

  std::string conflict;

  if (
    has_identity_collision_locked(
      record,
      "",
      &conflict))
  {
    return mutation_failure(
      MutationCode::kIdentityConflict,
      "identity key conflicts with " +
      conflict);
  }

  if (
    has_tag_collision_locked(
      record,
      "",
      &conflict))
  {
    return mutation_failure(
      MutationCode::kTagConflict,
      "AprilTag conflicts with " +
      conflict);
  }

  const auto location_id =
    record.location.location_id;

  records_.emplace(
    location_id,
    record);

  return mutation_success(
    MutationCode::kInserted,
    "location inserted",
    record);
}


RegistryMutationResult InMemoryRegistry::replace(
  LocationRecordData record,
  const std::uint64_t expected_current_revision)
{
  const auto validation =
    validate_location_draft(record.location);

  if (!validation.valid()) {
    return validation_failure(
      validation,
      "location record validation failed");
  }

  std::string envelope_reason;

  if (
    !valid_record_envelope(
      record,
      &envelope_reason))
  {
    return mutation_failure(
      MutationCode::kInvalidRecord,
      envelope_reason);
  }

  std::unique_lock<std::shared_mutex> lock{
    mutex_};

  const auto existing =
    records_.find(
      record.location.location_id);

  if (existing == records_.end()) {
    return mutation_failure(
      MutationCode::kNotFound,
      "location does not exist");
  }

  if (
    existing->second.record_revision !=
    expected_current_revision)
  {
    return mutation_failure(
      MutationCode::kStaleRevision,
      "expected revision does not match current revision");
  }

  if (
    expected_current_revision ==
    std::numeric_limits<
      std::uint64_t>::max())
  {
    return mutation_failure(
      MutationCode::kRevisionSequenceError,
      "record revision cannot be incremented");
  }

  const auto required_revision =
    expected_current_revision +
    std::uint64_t{1U};

  if (
    record.record_revision !=
    required_revision)
  {
    return mutation_failure(
      MutationCode::kRevisionSequenceError,
      "replacement revision must equal current revision plus one");
  }

  std::string conflict;

  if (
    has_identity_collision_locked(
      record,
      record.location.location_id,
      &conflict))
  {
    return mutation_failure(
      MutationCode::kIdentityConflict,
      "identity key conflicts with " +
      conflict);
  }

  if (
    has_tag_collision_locked(
      record,
      record.location.location_id,
      &conflict))
  {
    return mutation_failure(
      MutationCode::kTagConflict,
      "AprilTag conflicts with " +
      conflict);
  }

  existing->second = record;

  return mutation_success(
    MutationCode::kUpdated,
    "location replaced",
    record);
}


RegistryMutationResult
InMemoryRegistry::set_enabled(
  const std::string_view location_id,
  const std::uint64_t expected_current_revision,
  const bool enabled)
{
  const auto canonical_id =
    canonicalize_location_id(location_id);

  if (
    !is_canonical_location_id(
      canonical_id))
  {
    return mutation_failure(
      MutationCode::kInvalidRecord,
      "location ID is invalid");
  }

  std::unique_lock<std::shared_mutex> lock{
    mutex_};

  const auto existing =
    records_.find(canonical_id);

  if (existing == records_.end()) {
    return mutation_failure(
      MutationCode::kNotFound,
      "location does not exist");
  }

  if (
    existing->second.record_revision !=
    expected_current_revision)
  {
    return mutation_failure(
      MutationCode::kStaleRevision,
      "expected revision does not match current revision");
  }

  if (
    existing->second.state ==
    LocationState::kRetired)
  {
    return mutation_failure(
      MutationCode::kRetired,
      "retired location cannot be enabled or disabled");
  }

  if (
    existing->second.record_revision ==
    std::numeric_limits<
      std::uint64_t>::max())
  {
    return mutation_failure(
      MutationCode::kRevisionSequenceError,
      "record revision cannot be incremented");
  }

  auto updated = existing->second;
  updated.enabled = enabled;

  updated.record_revision +=
    std::uint64_t{1U};

  existing->second = updated;

  return mutation_success(
    MutationCode::kUpdated,
    enabled ?
      "location enabled" :
      "location disabled",
    updated);
}


ResolveResult InMemoryRegistry::resolve(
  const std::string_view query,
  const ResolveOptions & options) const
{
  ResolveResult invalid;

  invalid.normalized_query =
    normalize_lookup_key(query);

  if (invalid.normalized_query.empty()) {
    invalid.resolved = false;
    invalid.code = ResolveCode::kInvalidQuery;
    invalid.reason = "query is empty";
    return invalid;
  }

  if (options.enforce_map_context) {
    const auto map_validation =
      validate_map_context(options.map);

    if (!map_validation.valid()) {
      invalid.resolved = false;
      invalid.code =
        ResolveCode::kInvalidQuery;

      invalid.reason =
        "map context is invalid";

      return invalid;
    }
  }

  std::shared_lock<std::shared_mutex> lock{
    mutex_};

  const auto canonical_query =
    canonicalize_location_id(query);

  if (
    is_canonical_location_id(
      canonical_query))
  {
    const auto exact =
      records_.find(canonical_query);

    if (exact != records_.end()) {
      if (
        options.enforce_map_context &&
        !same_map_context(
          exact->second.location.map,
          options.map))
      {
        ResolveResult mismatch;
        mismatch.resolved = false;
        mismatch.code =
          ResolveCode::kMapMismatch;

        mismatch.reason =
          "location belongs to a different map context";

        mismatch.normalized_query =
          invalid.normalized_query;

        mismatch.match_type =
          MatchType::kLocationId;

        mismatch.record = exact->second;
        return mismatch;
      }

      return resolve_status(
        exact->second,
        MatchType::kLocationId,
        invalid.normalized_query);
    }
  }

  std::vector<Match> all_matches;

  for (const auto & entry : records_) {
    const auto match_type =
      generic_match_type(
        entry.second,
        invalid.normalized_query);

    if (match_type.has_value()) {
      all_matches.push_back(
        Match{
          entry.second,
          match_type.value()});
    }
  }

  if (all_matches.empty()) {
    ResolveResult not_found;
    not_found.resolved = false;
    not_found.code = ResolveCode::kNotFound;
    not_found.reason = "location was not found";

    not_found.normalized_query =
      invalid.normalized_query;

    return not_found;
  }

  std::vector<Match> contextual_matches;

  for (const auto & match : all_matches) {
    if (
      !options.enforce_map_context ||
      same_map_context(
        match.record.location.map,
        options.map))
    {
      contextual_matches.push_back(match);
    }
  }

  if (contextual_matches.empty()) {
    ResolveResult mismatch;
    mismatch.resolved = false;
    mismatch.code = ResolveCode::kMapMismatch;

    mismatch.reason =
      "matching identity exists only in another map context";

    mismatch.normalized_query =
      invalid.normalized_query;

    return mismatch;
  }

  std::vector<Match> non_retired_matches;

  for (
    const auto & match :
    contextual_matches)
  {
    if (
      match.record.state !=
      LocationState::kRetired)
    {
      non_retired_matches.push_back(match);
    }
  }

  const auto & preferred_matches =
    non_retired_matches.empty() ?
    contextual_matches :
    non_retired_matches;

  if (preferred_matches.size() > 1U) {
    ResolveResult ambiguous;
    ambiguous.resolved = false;
    ambiguous.code =
      ResolveCode::kAmbiguous;

    ambiguous.reason =
      "query matches multiple locations";

    ambiguous.normalized_query =
      invalid.normalized_query;

    for (
      const auto & match :
      preferred_matches)
    {
      ambiguous.ambiguous_location_ids.push_back(
        match.record.location.location_id);
    }

    return ambiguous;
  }

  const auto & selected =
    preferred_matches.front();

  return resolve_status(
    selected.record,
    selected.type,
    invalid.normalized_query);
}


std::optional<LocationRecordData>
InMemoryRegistry::get(
  const std::string_view location_id) const
{
  const auto canonical_id =
    canonicalize_location_id(location_id);

  if (
    !is_canonical_location_id(
      canonical_id))
  {
    return std::nullopt;
  }

  std::shared_lock<std::shared_mutex> lock{
    mutex_};

  const auto record =
    records_.find(canonical_id);

  if (record == records_.end()) {
    return std::nullopt;
  }

  return record->second;
}


std::vector<LocationRecordData>
InMemoryRegistry::list() const
{
  std::shared_lock<std::shared_mutex> lock{
    mutex_};

  std::vector<LocationRecordData> output;
  output.reserve(records_.size());

  for (const auto & entry : records_) {
    output.push_back(entry.second);
  }

  return output;
}


std::size_t
InMemoryRegistry::size() const noexcept
{
  std::shared_lock<std::shared_mutex> lock{
    mutex_};

  return records_.size();
}


void InMemoryRegistry::clear()
{
  std::unique_lock<std::shared_mutex> lock{
    mutex_};

  records_.clear();
}


bool
InMemoryRegistry::has_identity_collision_locked(
  const LocationRecordData & candidate,
  const std::string_view excluded_location_id,
  std::string * conflicting_location_id) const
{
  if (!reserves_identity(candidate)) {
    return false;
  }

  for (const auto & entry : records_) {
    if (
      entry.first ==
      excluded_location_id)
    {
      continue;
    }

    if (!reserves_identity(entry.second)) {
      continue;
    }

    if (
      !same_map_context(
        candidate.location.map,
        entry.second.location.map))
    {
      continue;
    }

    if (
      records_have_identity_overlap(
        candidate,
        entry.second))
    {
      if (
        conflicting_location_id !=
        nullptr)
      {
        *conflicting_location_id =
          entry.first;
      }

      return true;
    }
  }

  return false;
}


bool
InMemoryRegistry::has_tag_collision_locked(
  const LocationRecordData & candidate,
  const std::string_view excluded_location_id,
  std::string * conflicting_location_id) const
{
  if (!reserves_identity(candidate)) {
    return false;
  }

  for (const auto & entry : records_) {
    if (
      entry.first ==
      excluded_location_id)
    {
      continue;
    }

    if (!reserves_identity(entry.second)) {
      continue;
    }

    if (
      !same_map_context(
        candidate.location.map,
        entry.second.location.map))
    {
      continue;
    }

    const bool same_tag =
      trim_ascii(
        candidate.location.tag.family) ==
      trim_ascii(
        entry.second.location.tag.family) &&
      candidate.location.tag.id ==
      entry.second.location.tag.id;

    if (same_tag) {
      if (
        conflicting_location_id !=
        nullptr)
      {
        *conflicting_location_id =
          entry.first;
      }

      return true;
    }
  }

  return false;
}

}  // namespace savo_locations
