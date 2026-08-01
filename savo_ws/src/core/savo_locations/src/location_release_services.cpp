#include "savo_locations/location_registry_node.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <type_traits>
#include <utility>

namespace savo_locations
{
namespace
{

std::uint32_t count32(const std::size_t value)
{
  return static_cast<std::uint32_t>(std::min(
      value,
      static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
}


std::uint8_t prepare_result_code(const SnapshotCode code)
{
  using Service = savo_msgs::srv::PrepareLocationRelease;
  switch (code) {
    case SnapshotCode::kInvalidArgument:
    case SnapshotCode::kValidationFailed:
      return Service::Response::RESULT_INVALID_REQUEST;
    case SnapshotCode::kPendingCandidates:
      return Service::Response::RESULT_PENDING_CANDIDATES;
    case SnapshotCode::kNoApprovedLocations:
      return Service::Response::RESULT_NO_APPROVED_LOCATIONS;
    case SnapshotCode::kReleaseConflict:
      return Service::Response::RESULT_CONFLICT;
    case SnapshotCode::kFilesystemError:
    case SnapshotCode::kDigestMismatch:
      return Service::Response::RESULT_SNAPSHOT_FAILED;
    default:
      return Service::Response::RESULT_STORAGE_UNAVAILABLE;
  }
}


template<typename ServiceT>
std::uint8_t correlated_result_code(const SnapshotCode code)
{
  switch (code) {
    case SnapshotCode::kInvalidArgument:
      return ServiceT::Response::RESULT_INVALID_REQUEST;
    case SnapshotCode::kReleaseNotFound:
      return ServiceT::Response::RESULT_NOT_FOUND;
    case SnapshotCode::kStaleTransaction:
    case SnapshotCode::kReleaseConflict:
      return ServiceT::Response::RESULT_STALE_TRANSACTION;
    case SnapshotCode::kDigestMismatch:
      return ServiceT::Response::RESULT_DIGEST_MISMATCH;
    default:
      if constexpr (
        std::is_same_v<ServiceT, savo_msgs::srv::VerifyLocationRelease>)
      {
        return ServiceT::Response::RESULT_CONTEXT_MISMATCH;
      } else if constexpr (
        std::is_same_v<ServiceT, savo_msgs::srv::CommitLocationRelease>)
      {
        return ServiceT::Response::RESULT_COMMIT_FAILED;
      } else {
        return ServiceT::Response::RESULT_ROLLBACK_FAILED;
      }
  }
}

}  // namespace


void LocationRegistryNode::handle_prepare_release(
  const std::shared_ptr<PrepareReleaseService::Request> request,
  std::shared_ptr<PrepareReleaseService::Response> response)
{
  std::lock_guard<std::mutex> serialized{mutation_mutex_};
  CatalogSnapshot current;
  std::string rejection;

  if (
    request->contract_version != PrepareReleaseService::Request::CONTRACT_VERSION ||
    request->request_id.empty() ||
    !begin_mutation("prepare_location_release", &current, &rejection))
  {
    response->result_code = PrepareReleaseService::Response::RESULT_INVALID_REQUEST;
    response->reason = rejection.empty() ?
      "contract version, request ID and write-ready storage are required" : rejection;
    publish_status();
    return;
  }

  IntegrityReport integrity;
  const auto integrity_result = store_->integrity_check(&integrity);
  if (!integrity_result.success || !integrity.healthy) {
    finish_mutation_degraded("location release preparation integrity check failed");
    response->result_code = PrepareReleaseService::Response::RESULT_STORAGE_UNAVAILABLE;
    response->reason = "SQLite integrity/readiness validation failed";
    publish_status();
    return;
  }

  PrepareLocationReleaseRequest domain_request;
  domain_request.release_id = request->release_id;
  domain_request.mission_id = request->mission_id;
  domain_request.map_id = request->map_id;
  domain_request.map_revision = request->map_revision;
  domain_request.actor_id = request->actor_id;
  domain_request.approval_reason = request->approval_reason;
  domain_request.releases_root = releases_root_;
  domain_request.require_approved_location = request->require_approved_location;

  LocationReleaseRecord release;
  std::uint64_t event_sequence = 0U;
  const auto result = repository_->prepare_location_release(
    domain_request, current, &release, &event_sequence);
  if (!result.success) {
    if (
      result.code == SnapshotCode::kSqlError ||
      result.code == SnapshotCode::kStoreNotOpen ||
      result.code == SnapshotCode::kEventJournalError)
    {
      finish_mutation_degraded(result.reason);
    } else {
      finish_mutation_rejected(result.reason);
    }
    response->result_code = prepare_result_code(result.code);
    response->reason = result.reason;
    publish_status();
    return;
  }

  finish_mutation_committed(
    std::move(current), event_sequence, "location_release_prepared");
  publish_committed_event(
    event_sequence,
    savo_msgs::msg::LocationEvent::EVENT_LOCATION_RELEASE_PREPARED,
    "", "", 0U, request->actor_id, "location release prepared");
  publish_status();

  response->success = true;
  response->result_code = PrepareReleaseService::Response::RESULT_SUCCEEDED;
  response->reason = result.reason;
  response->release_id = release.release_id;
  response->transaction_token = release.transaction_token;
  response->snapshot_path = release.snapshot_path;
  response->snapshot_sha256 = release.snapshot_sha256;
  response->location_count = count32(release.location_count);
  response->candidate_count = count32(release.candidate_count);
  response->previous_active_location_release_id =
    release.previous_active_location_release;
}


void LocationRegistryNode::handle_verify_release(
  const std::shared_ptr<VerifyReleaseService::Request> request,
  std::shared_ptr<VerifyReleaseService::Response> response)
{
  std::lock_guard<std::mutex> serialized{mutation_mutex_};
  {
    std::shared_lock<std::shared_mutex> lock{state_mutex_};
    if (!ready_ || !storage_healthy_ || repository_ == nullptr) {
      response->result_code = VerifyReleaseService::Response::RESULT_NOT_FOUND;
      response->reason = "location registry is not read-ready";
      return;
    }
  }
  if (
    request->contract_version != VerifyReleaseService::Request::CONTRACT_VERSION ||
    request->request_id.empty())
  {
    response->result_code = VerifyReleaseService::Response::RESULT_INVALID_REQUEST;
    response->reason = "contract version and request ID are required";
    return;
  }

  LocationReleaseRecord release;
  const auto result = repository_->verify_location_release(
    request->release_id, request->mission_id, request->transaction_token,
    request->expected_snapshot_sha256, &release);
  if (!result.success) {
    response->result_code = correlated_result_code<VerifyReleaseService>(result.code);
    response->reason = result.reason;
    return;
  }
  response->success = true;
  response->result_code = VerifyReleaseService::Response::RESULT_SUCCEEDED;
  response->reason = result.reason;
  response->release_id = release.release_id;
  response->snapshot_path = release.snapshot_path;
  response->snapshot_sha256 = release.snapshot_sha256;
  response->location_count = count32(release.location_count);
  response->candidate_count = count32(release.candidate_count);
  response->state = release.state;
}


void LocationRegistryNode::handle_commit_release(
  const std::shared_ptr<CommitReleaseService::Request> request,
  std::shared_ptr<CommitReleaseService::Response> response)
{
  std::lock_guard<std::mutex> serialized{mutation_mutex_};
  CatalogSnapshot current;
  std::string rejection;
  if (
    request->contract_version != CommitReleaseService::Request::CONTRACT_VERSION ||
    request->request_id.empty() ||
    !begin_mutation("commit_location_release", &current, &rejection))
  {
    response->result_code = CommitReleaseService::Response::RESULT_INVALID_REQUEST;
    response->reason = rejection.empty() ?
      "contract version, request ID and write-ready storage are required" : rejection;
    publish_status();
    return;
  }

  LocationReleaseRecord release;
  std::uint64_t event_sequence = 0U;
  const auto result = repository_->commit_location_release(
    request->release_id, request->mission_id, request->transaction_token,
    request->expected_snapshot_sha256, request->actor_id,
    &release, &event_sequence);
  if (!result.success) {
    if (result.code == SnapshotCode::kSqlError) {
      finish_mutation_degraded(result.reason);
    } else {
      finish_mutation_rejected(result.reason);
    }
    response->result_code = correlated_result_code<CommitReleaseService>(result.code);
    response->reason = result.reason;
    publish_status();
    return;
  }

  for (auto & location : current.locations) {
    if (
      location.location.map.map_id == release.map_id &&
      location.location.map.map_revision == release.map_revision &&
      location.state == LocationState::kApproved)
    {
      location.location.map.map_release_id = release.release_id;
    }
  }
  finish_mutation_committed(
    std::move(current), event_sequence, "location_release_committed");
  publish_committed_event(
    event_sequence,
    savo_msgs::msg::LocationEvent::EVENT_LOCATION_RELEASE_COMMITTED,
    "", "", 0U, request->actor_id, "location release committed");
  publish_snapshot();
  publish_status();

  std::string active_release;
  static_cast<void>(repository_->active_location_release_id(&active_release));
  response->success = true;
  response->result_code = CommitReleaseService::Response::RESULT_SUCCEEDED;
  response->reason = result.reason;
  response->release_id = release.release_id;
  response->active_location_release_id = active_release;
  response->state = release.state;
}


void LocationRegistryNode::handle_rollback_release(
  const std::shared_ptr<RollbackReleaseService::Request> request,
  std::shared_ptr<RollbackReleaseService::Response> response)
{
  std::lock_guard<std::mutex> serialized{mutation_mutex_};
  CatalogSnapshot current;
  std::string rejection;
  if (
    request->contract_version != RollbackReleaseService::Request::CONTRACT_VERSION ||
    request->request_id.empty() ||
    !begin_mutation("rollback_location_release", &current, &rejection))
  {
    response->result_code = RollbackReleaseService::Response::RESULT_INVALID_REQUEST;
    response->reason = rejection.empty() ?
      "contract version, request ID and write-ready storage are required" : rejection;
    publish_status();
    return;
  }

  LocationReleaseRecord release;
  std::uint64_t event_sequence = 0U;
  const auto result = repository_->rollback_location_release(
    request->release_id, request->mission_id, request->transaction_token,
    request->expected_snapshot_sha256, request->actor_id,
    request->rollback_reason, &release, &event_sequence);
  if (!result.success) {
    if (result.code == SnapshotCode::kSqlError) {
      finish_mutation_degraded(result.reason);
    } else {
      finish_mutation_rejected(result.reason);
    }
    response->result_code = correlated_result_code<RollbackReleaseService>(result.code);
    response->reason = result.reason;
    publish_status();
    return;
  }

  for (auto & location : current.locations) {
    if (location.location.map.map_release_id == release.release_id) {
      location.location.map.map_release_id =
        release.previous_active_location_release;
    }
  }
  finish_mutation_committed(
    std::move(current), event_sequence, "location_release_rolled_back");
  publish_committed_event(
    event_sequence,
    savo_msgs::msg::LocationEvent::EVENT_LOCATION_RELEASE_ROLLED_BACK,
    "", "", 0U, request->actor_id, request->rollback_reason);
  publish_snapshot();
  publish_status();

  std::string active_release;
  static_cast<void>(repository_->active_location_release_id(&active_release));
  response->success = true;
  response->result_code = RollbackReleaseService::Response::RESULT_SUCCEEDED;
  response->reason = result.reason;
  response->release_id = release.release_id;
  response->active_location_release_id = active_release;
  response->state = release.state;
}

}  // namespace savo_locations
