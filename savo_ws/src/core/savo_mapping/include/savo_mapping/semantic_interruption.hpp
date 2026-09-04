// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <set>
#include <string>
#include <unordered_map>

namespace savo_mapping
{

enum class SemanticInterruptionState : std::uint8_t
{
  Idle = 0,
  TagDetected = 1,
  Pausing = 2,
  WaitingForMissionPaused = 3,
  WaitingForSemantics = 4,
  Registering = 5,
  Resuming = 6,
  Completed = 7,
  Failed = 8,
};

enum class SemanticObservationDisposition : std::uint8_t
{
  Accepted = 0,
  Disabled,
  InvalidObservation,
  MissionUnavailable,
  MissionStale,
  MissionIneligible,
  PredatesMission,
  ActiveDuplicate,
  CooldownDuplicate,
  RegisteredDuplicate,
  KnownLocationDuplicate,
};

enum class SemanticFailurePolicy : std::uint8_t
{
  RemainPaused = 0,
  RequestResume = 1,
};

struct SemanticInterruptionConfig
{
  bool enabled{true};
  std::string expected_family{"tag36h11"};
  double minimum_detection_quality{0.70};
  std::int64_t observation_timeout_ns{500000000};
  std::int64_t mission_status_timeout_ns{2000000000};
  std::int64_t pause_timeout_ns{10000000000};
  std::int64_t semantic_input_timeout_ns{300000000000};
  std::int64_t registration_timeout_ns{30000000000};
  std::int64_t resume_timeout_ns{10000000000};
  std::int64_t duplicate_cooldown_ns{30000000000};
  SemanticFailurePolicy failure_policy{SemanticFailurePolicy::RemainPaused};
};

struct SemanticMissionContext
{
  bool received{false};
  bool active{false};
  std::string mission_id{};
  std::string actor_id{};
  std::string map_id{};
  std::uint32_t map_revision{0U};
  std::uint8_t state{0U};
  std::int64_t mission_start_ns{0};
  std::int64_t received_ns{0};
};

struct SemanticObservationEvidence
{
  bool detected{true};
  bool pose_valid{false};
  bool pose_finite{false};
  std::string family{};
  std::int32_t tag_id{-1};
  float detection_quality{0.0F};
  std::uint64_t observation_sequence{0U};
  std::int64_t stamp_ns{0};
};

struct SemanticSubmissionEvidence
{
  std::string mission_id{};
  std::string actor_id{};
  std::string tag_family{};
  std::int32_t tag_id{-1};
  std::string suggested_location_id{};
  std::string suggested_display_name{};
  std::string suggested_semantic_type{};
  bool approach_pose_valid{false};
  bool approach_pose_acceptable{true};
  bool confirmation_pose_valid{false};
  bool confirmation_pose_acceptable{true};
};

struct SemanticObservationResult
{
  bool accepted{false};
  SemanticObservationDisposition disposition{
    SemanticObservationDisposition::InvalidObservation};
  std::string reason{};
};

struct SemanticSubmissionResult
{
  bool accepted{false};
  std::uint8_t result_code{0U};
  std::string reason{};
};

struct SemanticInterruptionSnapshot
{
  SemanticInterruptionState state{SemanticInterruptionState::Idle};
  std::string reason{"idle"};
  bool active{false};
  std::string mission_id{};
  std::string mission_actor_id{};
  std::string map_id{};
  std::uint32_t map_revision{0U};
  std::uint8_t interrupted_mission_state{0U};
  std::string tag_family{};
  std::int32_t tag_id{-1};
  std::uint64_t observation_sequence{0U};
  std::uint64_t duplicate_observations_suppressed{0U};
  std::string candidate_id{};
  bool semantic_submission_received{false};
  bool registration_started{false};
  bool registration_complete{false};
  bool resume_requested{false};
  bool resume_complete{false};
  std::int64_t deadline_ns{0};
};

class SemanticInterruptionCore
{
public:
  static constexpr std::uint8_t kMissionStateExploring{3U};
  static constexpr std::uint8_t kMissionStatePaused{5U};
  static constexpr std::uint8_t kMissionStateCoverage{19U};

  static constexpr std::uint8_t kSubmitAccepted{0U};
  static constexpr std::uint8_t kSubmitNoActive{1U};
  static constexpr std::uint8_t kSubmitTagMismatch{2U};
  static constexpr std::uint8_t kSubmitMissionMismatch{3U};
  static constexpr std::uint8_t kSubmitInvalid{4U};
  static constexpr std::uint8_t kSubmitBusy{5U};
  static constexpr std::uint8_t kSubmitRejected{6U};

  explicit SemanticInterruptionCore(SemanticInterruptionConfig config = {});

  void UpdateMission(const SemanticMissionContext & mission, std::int64_t now_ns);
  SemanticObservationResult Observe(
    const SemanticObservationEvidence & observation,
    const SemanticMissionContext & mission,
    bool known_location,
    std::int64_t now_ns);
  void BeginPause(std::int64_t now_ns);
  void PauseAccepted(std::int64_t now_ns);
  SemanticSubmissionResult ValidateSubmission(
    const SemanticSubmissionEvidence & submission) const;
  void BeginRegistration(const std::string & candidate_id, std::int64_t now_ns);
  void RegistrationSucceeded(const std::string & candidate_id, std::int64_t now_ns);
  void BeginResume(std::int64_t now_ns);
  void BeginFailureResume(std::int64_t now_ns);
  void Fail(const std::string & reason, std::int64_t now_ns);
  bool Tick(std::int64_t now_ns);

  [[nodiscard]] const SemanticInterruptionSnapshot & snapshot() const;
  [[nodiscard]] bool should_resume_after_failure() const;
  [[nodiscard]] static bool IsEligibleMissionState(std::uint8_t state);
  [[nodiscard]] static std::string StateText(SemanticInterruptionState state);

private:
  [[nodiscard]] std::string active_key() const;
  [[nodiscard]] static std::string observation_key(
    const std::string & mission_id,
    const std::string & family,
    std::int32_t tag_id);
  [[nodiscard]] static std::string tag_key(
    const std::string & family,
    std::int32_t tag_id);
  void suppress_duplicate();
  void apply_cooldown(std::int64_t now_ns);

  SemanticInterruptionConfig config_{};
  SemanticInterruptionSnapshot snapshot_{};
  std::unordered_map<std::string, std::int64_t> cooldown_until_ns_{};
  std::set<std::string> registered_tags_{};
  bool resuming_after_failure_{false};
  std::string failure_reason_{};
};

}  // namespace savo_mapping
