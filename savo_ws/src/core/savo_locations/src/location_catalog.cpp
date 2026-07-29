#include "savo_locations/location_catalog.hpp"

#include <limits>
#include <mutex>
#include <utility>

#include "savo_locations/normalization.hpp"

namespace savo_locations
{
namespace
{

bool same_map_context(
  const MapContext & left,
  const MapContext & right) noexcept
{
  return
    left.map_id == right.map_id &&
    left.map_revision == right.map_revision;
}


bool same_tag(
  const TagBinding & left,
  const TagBinding & right)
{
  return
    normalize_lookup_key(left.family) ==
      normalize_lookup_key(right.family) &&
    left.id == right.id;
}


std::string candidate_key(
  const std::string_view candidate_id)
{
  return trim_ascii(candidate_id);
}


CandidateMutationResult candidate_failure(
  const CandidateMutationCode code,
  std::string reason)
{
  CandidateMutationResult result;
  result.success = false;
  result.code = code;
  result.reason = std::move(reason);
  return result;
}


CandidateMutationResult candidate_validation_failure(
  const ValidationResult & validation)
{
  CandidateMutationResult result;
  result.success = false;

  result.code =
    CandidateMutationCode::kInvalidCandidate;

  result.reason =
    "candidate validation failed";

  result.validation_issues =
    validation.issues();

  return result;
}


CandidateMutationResult candidate_success(
  const CandidateMutationCode code,
  std::string reason,
  const CandidateRecordData & candidate)
{
  CandidateMutationResult result;
  result.success = true;
  result.code = code;
  result.reason = std::move(reason);
  result.candidate = candidate;
  return result;
}


ApprovalResult approval_failure(
  const ApprovalCode code,
  std::string reason)
{
  ApprovalResult result;
  result.success = false;
  result.code = code;
  result.reason = std::move(reason);
  return result;
}


ApprovalResult approval_validation_failure(
  const ValidationResult & validation)
{
  ApprovalResult result;
  result.success = false;
  result.code = ApprovalCode::kInvalidLocation;

  result.reason =
    "approved location validation failed";

  result.validation_issues =
    validation.issues();

  return result;
}


std::string choose_text(
  const std::string & requested,
  const std::string & suggested)
{
  const auto cleaned =
    collapse_ascii_whitespace(requested);

  if (!cleaned.empty()) {
    return cleaned;
  }

  return collapse_ascii_whitespace(
    suggested);
}


        std::vector<std::string> choose_aliases(
          const std::vector<std::string> & requested,
          const std::vector<std::string> & suggested,
          const std::string_view location_id,
          const std::string_view display_name)
        {
          const bool using_suggested_aliases =
            requested.empty();

          const auto & source =
            using_suggested_aliases ?
            suggested :
            requested;

          const auto location_id_key =
            normalize_lookup_key(location_id);

          const auto display_name_key =
            normalize_lookup_key(display_name);

          std::vector<std::string> aliases;
          aliases.reserve(source.size());

          for (const auto & alias : source) {
            const auto cleaned =
              collapse_ascii_whitespace(alias);

            if (using_suggested_aliases) {
              const auto alias_key =
                normalize_lookup_key(cleaned);

              const bool redundant =
                !alias_key.empty() &&
                (
                  alias_key == location_id_key ||
                  alias_key == display_name_key
                );

              if (redundant) {
                continue;
              }
            }

            aliases.push_back(cleaned);
          }

          return aliases;
        }

}  // namespace


std::string_view to_string(
  const CandidateMutationCode code) noexcept
{
  switch (code) {
    case CandidateMutationCode::kRegistered:
      return "registered";

    case CandidateMutationCode::kUpdated:
      return "updated";

    case CandidateMutationCode::kRejected:
      return "rejected";

    case CandidateMutationCode::kNotFound:
      return "not_found";

    case CandidateMutationCode::kInvalidCandidate:
      return "invalid_candidate";

    case CandidateMutationCode::kStaleRevision:
      return "stale_revision";

    case CandidateMutationCode::
      kRevisionSequenceError:
      return "revision_sequence_error";

    case CandidateMutationCode::
      kCandidateIdConflict:
      return "candidate_id_conflict";

    case CandidateMutationCode::kTagConflict:
      return "tag_conflict";

    case CandidateMutationCode::kNotPending:
      return "not_pending";

    case CandidateMutationCode::
      kEmptyReviewReason:
      return "empty_review_reason";

    default:
      return "unknown";
  }
}


std::string_view to_string(
  const ApprovalCode code) noexcept
{
  switch (code) {
    case ApprovalCode::kApproved:
      return "approved";

    case ApprovalCode::kInvalidRequest:
      return "invalid_request";

    case ApprovalCode::kCandidateNotFound:
      return "candidate_not_found";

    case ApprovalCode::kCandidateNotPending:
      return "candidate_not_pending";

    case ApprovalCode::kStaleRevision:
      return "stale_revision";

    case ApprovalCode::kRevisionSequenceError:
      return "revision_sequence_error";

    case ApprovalCode::kMissingApproachPose:
      return "missing_approach_pose";

    case ApprovalCode::kInvalidLocation:
      return "invalid_location";

    case ApprovalCode::kLocationConflict:
      return "location_conflict";

    case ApprovalCode::kLocationMutationFailed:
      return "location_mutation_failed";

    default:
      return "unknown";
  }
}


RegistryMutationResult
InMemoryLocationCatalog::insert_location(
  LocationRecordData record)
{
  std::unique_lock<std::shared_mutex> lock{
    mutex_};

  return locations_.insert(std::move(record));
}


RegistryMutationResult
InMemoryLocationCatalog::replace_location(
  LocationRecordData record,
  const std::uint64_t expected_current_revision)
{
  std::unique_lock<std::shared_mutex> lock{
    mutex_};

  return locations_.replace(
    std::move(record),
    expected_current_revision);
}


RegistryMutationResult
InMemoryLocationCatalog::set_location_enabled(
  const std::string_view location_id,
  const std::uint64_t expected_current_revision,
  const bool enabled)
{
  std::unique_lock<std::shared_mutex> lock{
    mutex_};

  return locations_.set_enabled(
    location_id,
    expected_current_revision,
    enabled);
}


ResolveResult
InMemoryLocationCatalog::resolve_location(
  const std::string_view query,
  const ResolveOptions & options) const
{
  std::shared_lock<std::shared_mutex> lock{
    mutex_};

  return locations_.resolve(query, options);
}


std::optional<LocationRecordData>
InMemoryLocationCatalog::get_location(
  const std::string_view location_id) const
{
  std::shared_lock<std::shared_mutex> lock{
    mutex_};

  return locations_.get(location_id);
}


std::vector<LocationRecordData>
InMemoryLocationCatalog::list_locations() const
{
  std::shared_lock<std::shared_mutex> lock{
    mutex_};

  return locations_.list();
}


CandidateMutationResult
InMemoryLocationCatalog::register_candidate(
  CandidateDraft candidate)
{
  candidate.candidate_id =
    candidate_key(candidate.candidate_id);

  const auto validation =
    validate_candidate_draft(candidate);

  if (!validation.valid()) {
    return candidate_validation_failure(
      validation);
  }

  std::unique_lock<std::shared_mutex> lock{
    mutex_};

  if (
    candidates_.find(candidate.candidate_id) !=
    candidates_.end())
  {
    return candidate_failure(
      CandidateMutationCode::
        kCandidateIdConflict,
      "candidate ID already exists");
  }

  std::string conflict;

  if (
    candidate_tag_conflict_locked(
      candidate,
      "",
      &conflict))
  {
    return candidate_failure(
      CandidateMutationCode::kTagConflict,
      "candidate AprilTag conflicts with " +
      conflict);
  }

  if (
    candidate_tag_conflicts_with_location_locked(
      candidate,
      &conflict))
  {
    return candidate_failure(
      CandidateMutationCode::kTagConflict,
      "candidate AprilTag conflicts with location " +
      conflict);
  }

  CandidateRecordData record;
  record.state =
    CandidateState::kPendingReview;

  record.candidate_revision = 1U;
  record.candidate = std::move(candidate);

  const auto key =
    record.candidate.candidate_id;

  candidates_.emplace(key, record);

  return candidate_success(
    CandidateMutationCode::kRegistered,
    "candidate registered",
    record);
}


CandidateMutationResult
InMemoryLocationCatalog::restore_candidate_record(
  CandidateRecordData record)
{
  record.candidate.candidate_id =
    candidate_key(
      record.candidate.candidate_id);

  const auto validation =
    validate_candidate_draft(
      record.candidate);

  if (!validation.valid()) {
    return candidate_validation_failure(
      validation);
  }

  if (
    record.candidate_revision == 0U ||
    record.state == CandidateState::kUnknown)
  {
    return candidate_failure(
      CandidateMutationCode::kInvalidCandidate,
      "persisted candidate envelope is invalid");
  }

  switch (record.state) {
    case CandidateState::kPendingReview:
      if (
        !trim_ascii(record.review_reason).empty() ||
        !trim_ascii(
          record.approved_location_id).empty())
      {
        return candidate_failure(
          CandidateMutationCode::kInvalidCandidate,
          "pending candidate envelope is invalid");
      }
      break;

    case CandidateState::kApproved:
      if (
        trim_ascii(
          record.approved_location_id).empty())
      {
        return candidate_failure(
          CandidateMutationCode::kInvalidCandidate,
          "approved candidate location is required");
      }
      break;

    case CandidateState::kRejected:
      if (
        trim_ascii(record.review_reason).empty() ||
        !trim_ascii(
          record.approved_location_id).empty())
      {
        return candidate_failure(
          CandidateMutationCode::kInvalidCandidate,
          "rejected candidate envelope is invalid");
      }
      break;

    case CandidateState::kUnknown:
    default:
      return candidate_failure(
        CandidateMutationCode::kInvalidCandidate,
        "candidate state is invalid");
  }

  std::unique_lock<std::shared_mutex> lock{
    mutex_};

  const auto key =
    record.candidate.candidate_id;

  if (candidates_.find(key) != candidates_.end()) {
    return candidate_failure(
      CandidateMutationCode::kCandidateIdConflict,
      "candidate ID already exists");
  }

  if (
    record.state == CandidateState::kPendingReview)
  {
    std::string conflict;

    if (
      candidate_tag_conflict_locked(
        record.candidate,
        "",
        &conflict))
    {
      return candidate_failure(
        CandidateMutationCode::kTagConflict,
        "candidate AprilTag conflicts with " +
        conflict);
    }

    if (
      candidate_tag_conflicts_with_location_locked(
        record.candidate,
        &conflict))
    {
      return candidate_failure(
        CandidateMutationCode::kTagConflict,
        "candidate AprilTag conflicts with location " +
        conflict);
    }
  }

  if (
    record.state == CandidateState::kApproved &&
    !locations_.get(
      record.approved_location_id).has_value())
  {
    return candidate_failure(
      CandidateMutationCode::kInvalidCandidate,
      "approved candidate references a missing location");
  }

  candidates_.emplace(key, record);

  return candidate_success(
    CandidateMutationCode::kUpdated,
    "candidate record restored",
    record);
}


CandidateMutationResult
InMemoryLocationCatalog::replace_candidate(
  const std::string_view candidate_id,
  CandidateDraft replacement,
  const std::uint64_t expected_current_revision)
{
  const auto key = candidate_key(candidate_id);

  replacement.candidate_id = key;

  const auto validation =
    validate_candidate_draft(replacement);

  if (!validation.valid()) {
    return candidate_validation_failure(
      validation);
  }

  std::unique_lock<std::shared_mutex> lock{
    mutex_};

  const auto existing =
    candidates_.find(key);

  if (existing == candidates_.end()) {
    return candidate_failure(
      CandidateMutationCode::kNotFound,
      "candidate does not exist");
  }

  if (
    existing->second.state !=
    CandidateState::kPendingReview)
  {
    return candidate_failure(
      CandidateMutationCode::kNotPending,
      "candidate is no longer pending");
  }

  if (
    existing->second.candidate_revision !=
    expected_current_revision)
  {
    return candidate_failure(
      CandidateMutationCode::kStaleRevision,
      "expected candidate revision does not match");
  }

  if (
    existing->second.candidate_revision ==
    std::numeric_limits<
      std::uint64_t>::max())
  {
    return candidate_failure(
      CandidateMutationCode::
        kRevisionSequenceError,
      "candidate revision cannot be incremented");
  }

  std::string conflict;

  if (
    candidate_tag_conflict_locked(
      replacement,
      key,
      &conflict))
  {
    return candidate_failure(
      CandidateMutationCode::kTagConflict,
      "candidate AprilTag conflicts with " +
      conflict);
  }

  if (
    candidate_tag_conflicts_with_location_locked(
      replacement,
      &conflict))
  {
    return candidate_failure(
      CandidateMutationCode::kTagConflict,
      "candidate AprilTag conflicts with location " +
      conflict);
  }

  auto updated = existing->second;
  updated.candidate = std::move(replacement);

  updated.candidate_revision +=
    std::uint64_t{1U};

  existing->second = updated;

  return candidate_success(
    CandidateMutationCode::kUpdated,
    "candidate updated",
    updated);
}


CandidateMutationResult
InMemoryLocationCatalog::reject_candidate(
  const std::string_view candidate_id,
  const std::uint64_t expected_current_revision,
  std::string review_reason)
{
  const auto key = candidate_key(candidate_id);

  review_reason =
    collapse_ascii_whitespace(review_reason);

  if (review_reason.empty()) {
    return candidate_failure(
      CandidateMutationCode::
        kEmptyReviewReason,
      "rejection reason is required");
  }

  std::unique_lock<std::shared_mutex> lock{
    mutex_};

  const auto existing =
    candidates_.find(key);

  if (existing == candidates_.end()) {
    return candidate_failure(
      CandidateMutationCode::kNotFound,
      "candidate does not exist");
  }

  if (
    existing->second.state !=
    CandidateState::kPendingReview)
  {
    return candidate_failure(
      CandidateMutationCode::kNotPending,
      "candidate is no longer pending");
  }

  if (
    existing->second.candidate_revision !=
    expected_current_revision)
  {
    return candidate_failure(
      CandidateMutationCode::kStaleRevision,
      "expected candidate revision does not match");
  }

  if (
    existing->second.candidate_revision ==
    std::numeric_limits<
      std::uint64_t>::max())
  {
    return candidate_failure(
      CandidateMutationCode::
        kRevisionSequenceError,
      "candidate revision cannot be incremented");
  }

  auto rejected = existing->second;

  rejected.state =
    CandidateState::kRejected;

  rejected.candidate_revision +=
    std::uint64_t{1U};

  rejected.review_reason =
    std::move(review_reason);

  existing->second = rejected;

  return candidate_success(
    CandidateMutationCode::kRejected,
    "candidate rejected",
    rejected);
}


ApprovalResult
InMemoryLocationCatalog::approve_candidate(
  const ApprovalRequest & request)
{
  const auto key =
    candidate_key(request.candidate_id);

  if (key.empty()) {
    return approval_failure(
      ApprovalCode::kInvalidRequest,
      "candidate ID is required");
  }

  std::unique_lock<std::shared_mutex> lock{
    mutex_};

  const auto existing =
    candidates_.find(key);

  if (existing == candidates_.end()) {
    return approval_failure(
      ApprovalCode::kCandidateNotFound,
      "candidate does not exist");
  }

  if (
    existing->second.state !=
    CandidateState::kPendingReview)
  {
    return approval_failure(
      ApprovalCode::kCandidateNotPending,
      "candidate is no longer pending");
  }

  if (
    existing->second.candidate_revision !=
    request.expected_candidate_revision)
  {
    return approval_failure(
      ApprovalCode::kStaleRevision,
      "expected candidate revision does not match");
  }

  if (
    existing->second.candidate_revision ==
    std::numeric_limits<
      std::uint64_t>::max())
  {
    return approval_failure(
      ApprovalCode::kRevisionSequenceError,
      "candidate revision cannot be incremented");
  }

  const auto & source =
    existing->second.candidate;

  const auto approach_pose =
    request.approach_pose.has_value() ?
    request.approach_pose :
    source.approach_pose;

  if (!approach_pose.has_value()) {
    return approval_failure(
      ApprovalCode::kMissingApproachPose,
      "approval requires a safe approach pose");
  }

  LocationDraft location;

  location.location_id =
    canonicalize_location_id(
      choose_text(
        request.location_id,
        source.suggested_location_id));

  location.display_name =
    choose_text(
      request.display_name,
      source.suggested_display_name);

  location.aliases =
            choose_aliases(
              request.aliases,
              source.suggested_aliases,
              location.location_id,
              location.display_name);

  location.semantic_type =
    choose_text(
      request.semantic_type,
      source.suggested_semantic_type);

  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
  location.approach_pose =
    approach_pose.value();
  #pragma GCC diagnostic pop

  location.confirmation_pose =
    request.confirmation_pose.has_value() ?
    request.confirmation_pose :
    source.confirmation_pose;

  location.tag_pose_map =
    source.tag_pose_map;

  location.map = source.map;
  location.tag = source.tag;

  location.arrival_confirmation_required =
    request.arrival_confirmation_required;

  location.building =
    choose_text(
      request.building,
      source.building);

  location.floor =
    choose_text(
      request.floor,
      source.floor);

  location.area =
    choose_text(
      request.area,
      source.area);

  location.notes =
    choose_text(
      request.notes,
      source.notes);

  const auto validation =
    validate_location_draft(location);

  if (!validation.valid()) {
    return approval_validation_failure(
      validation);
  }

  LocationRecordData location_record;

  location_record.state =
    LocationState::kApproved;

  location_record.enabled = true;
  location_record.record_revision = 1U;

  location_record.location =
    std::move(location);

  location_record.source_candidate_id =
    key;

  auto approved_candidate =
    existing->second;

  approved_candidate.state =
    CandidateState::kApproved;

  approved_candidate.candidate_revision +=
    std::uint64_t{1U};

  approved_candidate.review_reason =
    "approved";

  approved_candidate.approved_location_id =
    location_record.location.location_id;

  const auto location_result =
    locations_.insert(location_record);

  if (!location_result.success) {
    ApprovalResult result;
    result.success = false;
    result.location_mutation_code =
      location_result.code;

    result.reason =
      location_result.reason;

    result.validation_issues =
      location_result.validation_issues;

    switch (location_result.code) {
      case MutationCode::kInvalidRecord:
        result.code =
          ApprovalCode::kInvalidLocation;
        break;

      case MutationCode::
          kLocationIdConflict:
      case MutationCode::kIdentityConflict:
      case MutationCode::kTagConflict:
        result.code =
          ApprovalCode::kLocationConflict;
        break;

      default:
        result.code =
          ApprovalCode::
            kLocationMutationFailed;
        break;
    }

    return result;
  }

  existing->second =
    approved_candidate;

  ApprovalResult result;
  result.success = true;
  result.code = ApprovalCode::kApproved;
  result.reason = "candidate approved";

  result.location_mutation_code =
    MutationCode::kInserted;

  result.candidate =
    approved_candidate;

  result.location =
    location_result.record;

  return result;
}


std::optional<CandidateRecordData>
InMemoryLocationCatalog::get_candidate(
  const std::string_view candidate_id) const
{
  const auto key = candidate_key(candidate_id);

  std::shared_lock<std::shared_mutex> lock{
    mutex_};

  const auto candidate =
    candidates_.find(key);

  if (candidate == candidates_.end()) {
    return std::nullopt;
  }

  return candidate->second;
}


std::vector<CandidateRecordData>
InMemoryLocationCatalog::list_candidates() const
{
  std::shared_lock<std::shared_mutex> lock{
    mutex_};

  std::vector<CandidateRecordData> output;
  output.reserve(candidates_.size());

  for (const auto & entry : candidates_) {
    output.push_back(entry.second);
  }

  return output;
}


std::size_t
InMemoryLocationCatalog::location_size() const
{
  std::shared_lock<std::shared_mutex> lock{
    mutex_};

  return locations_.size();
}


std::size_t
InMemoryLocationCatalog::candidate_size() const
{
  std::shared_lock<std::shared_mutex> lock{
    mutex_};

  return candidates_.size();
}


void InMemoryLocationCatalog::clear()
{
  std::unique_lock<std::shared_mutex> lock{
    mutex_};

  locations_.clear();
  candidates_.clear();
}


bool
InMemoryLocationCatalog::
candidate_tag_conflict_locked(
  const CandidateDraft & candidate,
  const std::string_view excluded_candidate_id,
  std::string * conflicting_entity) const
{
  for (const auto & entry : candidates_) {
    if (entry.first == excluded_candidate_id) {
      continue;
    }

    if (
      entry.second.state !=
      CandidateState::kPendingReview)
    {
      continue;
    }

    if (
      !same_map_context(
        candidate.map,
        entry.second.candidate.map))
    {
      continue;
    }

    if (
      same_tag(
        candidate.tag,
        entry.second.candidate.tag))
    {
      if (conflicting_entity != nullptr) {
        *conflicting_entity = entry.first;
      }

      return true;
    }
  }

  return false;
}


bool
InMemoryLocationCatalog::
candidate_tag_conflicts_with_location_locked(
  const CandidateDraft & candidate,
  std::string * conflicting_location_id) const
{
  const auto locations = locations_.list();

  for (const auto & location : locations) {
    if (
      location.state ==
      LocationState::kRetired)
    {
      continue;
    }

    if (
      !same_map_context(
        candidate.map,
        location.location.map))
    {
      continue;
    }

    if (
      same_tag(
        candidate.tag,
        location.location.tag))
    {
      if (
        conflicting_location_id !=
        nullptr)
      {
        *conflicting_location_id =
          location.location.location_id;
      }

      return true;
    }
  }

  return false;
}

}  // namespace savo_locations
