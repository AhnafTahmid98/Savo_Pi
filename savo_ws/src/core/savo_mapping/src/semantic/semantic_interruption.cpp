// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_mapping/semantic_interruption.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace savo_mapping
{
namespace
{

SemanticObservationResult reject(
  const SemanticObservationDisposition disposition,
  std::string reason)
{
  return {false, disposition, std::move(reason)};
}

}  // namespace

SemanticInterruptionCore::SemanticInterruptionCore(
  SemanticInterruptionConfig config)
: config_(std::move(config))
{
  if (config_.expected_family.empty() ||
    !std::isfinite(config_.minimum_detection_quality) ||
    config_.minimum_detection_quality < 0.0 ||
    config_.minimum_detection_quality > 1.0 ||
    config_.observation_timeout_ns <= 0 ||
    config_.mission_status_timeout_ns <= 0 ||
    config_.pause_timeout_ns <= 0 ||
    config_.semantic_input_timeout_ns <= 0 ||
    config_.registration_timeout_ns <= 0 ||
    config_.resume_timeout_ns <= 0 ||
    config_.duplicate_cooldown_ns < 0)
  {
    throw std::invalid_argument("invalid semantic interruption configuration");
  }
}

void SemanticInterruptionCore::UpdateMission(
  const SemanticMissionContext & mission,
  const std::int64_t now_ns)
{
  if (!snapshot_.active || mission.mission_id != snapshot_.mission_id) {
    return;
  }

  if ((snapshot_.state == SemanticInterruptionState::Pausing ||
    snapshot_.state == SemanticInterruptionState::WaitingForMissionPaused) &&
    mission.state == kMissionStatePaused)
  {
    snapshot_.state = SemanticInterruptionState::WaitingForSemantics;
    snapshot_.reason = "mission_paused_waiting_for_semantics";
    snapshot_.deadline_ns = now_ns + config_.semantic_input_timeout_ns;
    return;
  }

  if (snapshot_.state == SemanticInterruptionState::Resuming &&
    mission.state == snapshot_.interrupted_mission_state)
  {
    snapshot_.resume_complete = true;
    snapshot_.active = false;
    snapshot_.state = resuming_after_failure_ ?
      SemanticInterruptionState::Failed : SemanticInterruptionState::Completed;
    snapshot_.reason = resuming_after_failure_ ?
      failure_reason_ : "interrupted_activity_resumed";
    snapshot_.deadline_ns = 0;
    apply_cooldown(now_ns);
    resuming_after_failure_ = false;
    failure_reason_.clear();
  }
}

SemanticObservationResult SemanticInterruptionCore::Observe(
  const SemanticObservationEvidence & observation,
  const SemanticMissionContext & mission,
  const bool known_location,
  const std::int64_t now_ns)
{
  if (!config_.enabled) {
    return reject(SemanticObservationDisposition::Disabled, "semantic_interruption_disabled");
  }

  const bool same_active_tag = snapshot_.active &&
    snapshot_.mission_id == mission.mission_id &&
    snapshot_.tag_family == observation.family &&
    snapshot_.tag_id == observation.tag_id;
  if (snapshot_.active) {
    if (same_active_tag) {
      suppress_duplicate();
      return reject(
        SemanticObservationDisposition::ActiveDuplicate,
        "active_tag_observation_suppressed");
    }
    suppress_duplicate();
    return reject(
      SemanticObservationDisposition::MissionIneligible,
      "semantic_interruption_already_active");
  }

  if (!observation.detected || observation.family != config_.expected_family ||
    observation.tag_id < 0 || !observation.pose_valid || !observation.pose_finite ||
    !std::isfinite(observation.detection_quality) ||
    observation.detection_quality < config_.minimum_detection_quality ||
    observation.stamp_ns <= 0 || now_ns < observation.stamp_ns ||
    now_ns - observation.stamp_ns > config_.observation_timeout_ns)
  {
    return reject(
      SemanticObservationDisposition::InvalidObservation,
      "observation_validation_failed");
  }

  if (!mission.received || !mission.active || mission.mission_id.empty() ||
    mission.actor_id.empty() || mission.map_id.empty() || mission.map_revision == 0U)
  {
    return reject(
      SemanticObservationDisposition::MissionUnavailable,
      "active_mission_context_unavailable");
  }

  if (mission.received_ns <= 0 || now_ns < mission.received_ns ||
    now_ns - mission.received_ns > config_.mission_status_timeout_ns)
  {
    return reject(
      SemanticObservationDisposition::MissionStale,
      "mission_status_stale");
  }

  if (!IsEligibleMissionState(mission.state)) {
    return reject(
      SemanticObservationDisposition::MissionIneligible,
      "mission_state_not_semantic_observable");
  }

  if (mission.mission_start_ns > 0 && observation.stamp_ns < mission.mission_start_ns) {
    return reject(
      SemanticObservationDisposition::PredatesMission,
      "observation_predates_current_mission");
  }

  const auto key = observation_key(
    mission.mission_id, observation.family, observation.tag_id);
  const auto cooldown = cooldown_until_ns_.find(key);
  if (cooldown != cooldown_until_ns_.end() && now_ns < cooldown->second) {
    suppress_duplicate();
    return reject(
      SemanticObservationDisposition::CooldownDuplicate,
      "tag_observation_in_cooldown");
  }

  if (registered_tags_.count(tag_key(observation.family, observation.tag_id)) > 0U) {
    suppress_duplicate();
    return reject(
      SemanticObservationDisposition::RegisteredDuplicate,
      "tag_registered_during_process_lifetime");
  }

  if (known_location) {
    suppress_duplicate();
    return reject(
      SemanticObservationDisposition::KnownLocationDuplicate,
      "tag_already_present_in_location_registry");
  }

  snapshot_.state = SemanticInterruptionState::TagDetected;
  snapshot_.reason = "eligible_apriltag_detected";
  snapshot_.active = true;
  snapshot_.mission_id = mission.mission_id;
  snapshot_.mission_actor_id = mission.actor_id;
  snapshot_.map_id = mission.map_id;
  snapshot_.map_revision = mission.map_revision;
  snapshot_.interrupted_mission_state = mission.state;
  snapshot_.tag_family = observation.family;
  snapshot_.tag_id = observation.tag_id;
  snapshot_.observation_sequence = observation.observation_sequence;
  snapshot_.candidate_id.clear();
  snapshot_.semantic_submission_received = false;
  snapshot_.registration_started = false;
  snapshot_.registration_complete = false;
  snapshot_.resume_requested = false;
  snapshot_.resume_complete = false;
  snapshot_.deadline_ns = 0;
  return {true, SemanticObservationDisposition::Accepted, snapshot_.reason};
}

void SemanticInterruptionCore::BeginPause(const std::int64_t now_ns)
{
  if (!snapshot_.active || snapshot_.state != SemanticInterruptionState::TagDetected) {
    throw std::logic_error("pause requires a detected active interruption");
  }
  snapshot_.state = SemanticInterruptionState::Pausing;
  snapshot_.reason = "requesting_autonomous_mapping_pause";
  snapshot_.deadline_ns = now_ns + config_.pause_timeout_ns;
}

void SemanticInterruptionCore::PauseAccepted(const std::int64_t now_ns)
{
  if (!snapshot_.active || snapshot_.state != SemanticInterruptionState::Pausing) {
    return;
  }
  snapshot_.state = SemanticInterruptionState::WaitingForMissionPaused;
  snapshot_.reason = "waiting_for_typed_mission_paused_state";
  snapshot_.deadline_ns = now_ns + config_.pause_timeout_ns;
}

SemanticSubmissionResult SemanticInterruptionCore::ValidateSubmission(
  const SemanticSubmissionEvidence & submission) const
{
  if (!snapshot_.active) {
    return {false, kSubmitNoActive, "no_active_semantic_interruption"};
  }
  if (snapshot_.state == SemanticInterruptionState::Registering ||
    snapshot_.semantic_submission_received)
  {
    return {false, kSubmitBusy, "semantic_submission_already_in_progress"};
  }
  if (snapshot_.state != SemanticInterruptionState::WaitingForSemantics) {
    return {false, kSubmitRejected, "mission_not_confirmed_paused"};
  }
  if (submission.mission_id != snapshot_.mission_id) {
    return {false, kSubmitMissionMismatch, "semantic_submission_mission_mismatch"};
  }
  if (submission.tag_family != snapshot_.tag_family ||
    submission.tag_id != snapshot_.tag_id)
  {
    return {false, kSubmitTagMismatch, "semantic_submission_tag_mismatch"};
  }
  if (submission.actor_id.empty() || submission.suggested_location_id.empty() ||
    submission.suggested_display_name.empty() ||
    submission.suggested_semantic_type.empty() ||
    (submission.approach_pose_valid && !submission.approach_pose_acceptable) ||
    (submission.confirmation_pose_valid && !submission.confirmation_pose_acceptable))
  {
    return {false, kSubmitInvalid, "semantic_submission_invalid"};
  }
  return {true, kSubmitAccepted, "semantic_submission_accepted"};
}

void SemanticInterruptionCore::BeginRegistration(
  const std::string & candidate_id,
  const std::int64_t now_ns)
{
  if (candidate_id.empty() || !snapshot_.active ||
    snapshot_.state != SemanticInterruptionState::WaitingForSemantics)
  {
    throw std::logic_error("registration requires accepted paused semantics");
  }
  snapshot_.candidate_id = candidate_id;
  snapshot_.semantic_submission_received = true;
  snapshot_.registration_started = true;
  snapshot_.state = SemanticInterruptionState::Registering;
  snapshot_.reason = "mapped_location_registration_started";
  snapshot_.deadline_ns = now_ns + config_.registration_timeout_ns;
}

void SemanticInterruptionCore::RegistrationSucceeded(
  const std::string & candidate_id,
  const std::int64_t now_ns)
{
  if (!snapshot_.active || snapshot_.state != SemanticInterruptionState::Registering) {
    return;
  }
  snapshot_.candidate_id = candidate_id;
  snapshot_.registration_complete = true;
  registered_tags_.insert(tag_key(snapshot_.tag_family, snapshot_.tag_id));
  snapshot_.reason = "location_candidate_registered";
  snapshot_.deadline_ns = now_ns + config_.resume_timeout_ns;
}

void SemanticInterruptionCore::BeginResume(const std::int64_t now_ns)
{
  if (!snapshot_.active || !snapshot_.registration_complete) {
    throw std::logic_error("resume requires successful registration");
  }
  snapshot_.state = SemanticInterruptionState::Resuming;
  snapshot_.reason = "requesting_autonomous_mapping_resume";
  snapshot_.resume_requested = true;
  snapshot_.deadline_ns = now_ns + config_.resume_timeout_ns;
}

void SemanticInterruptionCore::BeginFailureResume(const std::int64_t now_ns)
{
  if (!snapshot_.active || snapshot_.state != SemanticInterruptionState::Failed) {
    throw std::logic_error("failure resume requires an active failed interruption");
  }
  resuming_after_failure_ = true;
  failure_reason_ = snapshot_.reason;
  snapshot_.state = SemanticInterruptionState::Resuming;
  snapshot_.resume_requested = true;
  snapshot_.deadline_ns = now_ns + config_.resume_timeout_ns;
}

void SemanticInterruptionCore::Fail(
  const std::string & reason,
  const std::int64_t now_ns)
{
  snapshot_.state = SemanticInterruptionState::Failed;
  snapshot_.reason = reason.empty() ? "semantic_interruption_failed" : reason;
  snapshot_.deadline_ns = 0;
  apply_cooldown(now_ns);
  if (config_.failure_policy == SemanticFailurePolicy::RemainPaused) {
    snapshot_.active = false;
  }
}

bool SemanticInterruptionCore::Tick(const std::int64_t now_ns)
{
  if (!snapshot_.active || snapshot_.deadline_ns <= 0 || now_ns < snapshot_.deadline_ns) {
    return false;
  }

  std::string reason;
  switch (snapshot_.state) {
    case SemanticInterruptionState::Pausing:
    case SemanticInterruptionState::WaitingForMissionPaused:
      reason = "mission_pause_timed_out";
      break;
    case SemanticInterruptionState::WaitingForSemantics:
      reason = "semantic_input_timed_out";
      break;
    case SemanticInterruptionState::Registering:
      reason = "mapped_location_registration_timed_out";
      break;
    case SemanticInterruptionState::Resuming:
      reason = "mission_resume_timed_out";
      break;
    default:
      return false;
  }
  Fail(reason, now_ns);
  return true;
}

const SemanticInterruptionSnapshot & SemanticInterruptionCore::snapshot() const
{
  return snapshot_;
}

bool SemanticInterruptionCore::should_resume_after_failure() const
{
  return config_.failure_policy == SemanticFailurePolicy::RequestResume;
}

bool SemanticInterruptionCore::IsEligibleMissionState(const std::uint8_t state)
{
  return state == kMissionStateExploring || state == kMissionStateCoverage;
}

std::string SemanticInterruptionCore::StateText(const SemanticInterruptionState state)
{
  switch (state) {
    case SemanticInterruptionState::Idle:
      return "idle";
    case SemanticInterruptionState::TagDetected:
      return "tag_detected";
    case SemanticInterruptionState::Pausing:
      return "pausing";
    case SemanticInterruptionState::WaitingForMissionPaused:
      return "waiting_for_mission_paused";
    case SemanticInterruptionState::WaitingForSemantics:
      return "waiting_for_semantics";
    case SemanticInterruptionState::Registering:
      return "registering";
    case SemanticInterruptionState::Resuming:
      return "resuming";
    case SemanticInterruptionState::Completed:
      return "completed";
    case SemanticInterruptionState::Failed:
      return "failed";
  }
  return "unknown";
}

std::string SemanticInterruptionCore::active_key() const
{
  return observation_key(snapshot_.mission_id, snapshot_.tag_family, snapshot_.tag_id);
}

std::string SemanticInterruptionCore::observation_key(
  const std::string & mission_id,
  const std::string & family,
  const std::int32_t tag_id)
{
  return mission_id + "|" + family + "|" + std::to_string(tag_id);
}

std::string SemanticInterruptionCore::tag_key(
  const std::string & family,
  const std::int32_t tag_id)
{
  return family + "|" + std::to_string(tag_id);
}

void SemanticInterruptionCore::suppress_duplicate()
{
  ++snapshot_.duplicate_observations_suppressed;
}

void SemanticInterruptionCore::apply_cooldown(const std::int64_t now_ns)
{
  if (!snapshot_.mission_id.empty() && !snapshot_.tag_family.empty() && snapshot_.tag_id >= 0) {
    cooldown_until_ns_[active_key()] = now_ns + config_.duplicate_cooldown_ns;
  }
}

}  // namespace savo_mapping
