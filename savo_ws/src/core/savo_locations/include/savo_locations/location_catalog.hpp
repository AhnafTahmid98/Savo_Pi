#ifndef SAVO_LOCATIONS__LOCATION_CATALOG_HPP_
#define SAVO_LOCATIONS__LOCATION_CATALOG_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <vector>

#include "savo_locations/model.hpp"
#include "savo_locations/registry.hpp"
#include "savo_locations/validation.hpp"

namespace savo_locations
{

enum class CandidateMutationCode : std::uint8_t
{
  kRegistered = 0U,
  kUpdated,
  kRejected,
  kNotFound,
  kInvalidCandidate,
  kStaleRevision,
  kRevisionSequenceError,
  kCandidateIdConflict,
  kTagConflict,
  kNotPending,
  kEmptyReviewReason,
};


enum class ApprovalCode : std::uint8_t
{
  kApproved = 0U,
  kInvalidRequest,
  kCandidateNotFound,
  kCandidateNotPending,
  kStaleRevision,
  kRevisionSequenceError,
  kMissingApproachPose,
  kInvalidLocation,
  kLocationConflict,
  kLocationMutationFailed,
};


struct CandidateMutationResult
{
  bool success{false};

  CandidateMutationCode code{
    CandidateMutationCode::kInvalidCandidate};

  std::string reason;

  std::vector<ValidationIssue> validation_issues;

  std::optional<CandidateRecordData> candidate;
};


struct ApprovalResult
{
  bool success{false};

  ApprovalCode code{
    ApprovalCode::kInvalidRequest};

  std::string reason;

  std::vector<ValidationIssue> validation_issues;

  MutationCode location_mutation_code{
    MutationCode::kInvalidRecord};

  std::optional<CandidateRecordData> candidate;
  std::optional<LocationRecordData> location;
};


[[nodiscard]]
std::string_view to_string(
  CandidateMutationCode code) noexcept;

[[nodiscard]]
std::string_view to_string(
  ApprovalCode code) noexcept;


class InMemoryLocationCatalog
{
public:
  [[nodiscard]]
  RegistryMutationResult insert_location(
    LocationRecordData record);

  [[nodiscard]]
  RegistryMutationResult replace_location(
    LocationRecordData record,
    std::uint64_t expected_current_revision);

  [[nodiscard]]
  RegistryMutationResult set_location_enabled(
    std::string_view location_id,
    std::uint64_t expected_current_revision,
    bool enabled);

  [[nodiscard]]
  ResolveResult resolve_location(
    std::string_view query,
    const ResolveOptions & options =
      ResolveOptions{}) const;

  [[nodiscard]]
  std::optional<LocationRecordData> get_location(
    std::string_view location_id) const;

  [[nodiscard]]
  std::vector<LocationRecordData>
  list_locations() const;

  [[nodiscard]]
  CandidateMutationResult register_candidate(
    CandidateDraft candidate);

  // Restores a trusted persisted candidate record into a temporary
  // catalog used for deterministic mutation planning. The record is
  // still fully validated and conflict checked before insertion.
  [[nodiscard]]
  CandidateMutationResult restore_candidate_record(
    CandidateRecordData record);

  [[nodiscard]]
  CandidateMutationResult replace_candidate(
    std::string_view candidate_id,
    CandidateDraft replacement,
    std::uint64_t expected_current_revision);

  [[nodiscard]]
  CandidateMutationResult reject_candidate(
    std::string_view candidate_id,
    std::uint64_t expected_current_revision,
    std::string review_reason);

  [[nodiscard]]
  ApprovalResult approve_candidate(
    const ApprovalRequest & request);

  [[nodiscard]]
  std::optional<CandidateRecordData>
  get_candidate(
    std::string_view candidate_id) const;

  [[nodiscard]]
  std::vector<CandidateRecordData>
  list_candidates() const;

  [[nodiscard]]
  std::size_t location_size() const;

  [[nodiscard]]
  std::size_t candidate_size() const;

  void clear();

private:
  [[nodiscard]]
  bool candidate_tag_conflict_locked(
    const CandidateDraft & candidate,
    std::string_view excluded_candidate_id,
    std::string * conflicting_entity) const;

  [[nodiscard]]
  bool candidate_tag_conflicts_with_location_locked(
    const CandidateDraft & candidate,
    std::string * conflicting_location_id) const;

  mutable std::shared_mutex mutex_;

  InMemoryRegistry locations_;

  std::map<
    std::string,
    CandidateRecordData,
    std::less<>>
  candidates_;
};

}  // namespace savo_locations

#endif  // SAVO_LOCATIONS__LOCATION_CATALOG_HPP_
