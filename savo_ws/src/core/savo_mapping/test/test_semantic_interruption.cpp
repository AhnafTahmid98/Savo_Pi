// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include "savo_mapping/semantic_interruption.hpp"

namespace
{

using savo_mapping::SemanticFailurePolicy;
using savo_mapping::SemanticInterruptionConfig;
using savo_mapping::SemanticInterruptionCore;
using savo_mapping::SemanticInterruptionState;
using savo_mapping::SemanticMissionContext;
using savo_mapping::SemanticObservationDisposition;
using savo_mapping::SemanticObservationEvidence;
using savo_mapping::SemanticSubmissionEvidence;

constexpr std::int64_t kSecond{1000000000LL};

SemanticMissionContext mission(
  const std::uint8_t state = SemanticInterruptionCore::kMissionStateExploring)
{
  SemanticMissionContext value;
  value.received = true;
  value.active = true;
  value.mission_id = "mission_alpha";
  value.map_id = "first_floor";
  value.map_revision = 1U;
  value.state = state;
  value.mission_start_ns = 10 * kSecond;
  value.received_ns = 20 * kSecond;
  return value;
}

SemanticObservationEvidence observation(
  const std::int32_t tag_id = 42,
  const std::int64_t stamp_ns = 20 * kSecond)
{
  SemanticObservationEvidence value;
  value.detected = true;
  value.pose_valid = true;
  value.pose_finite = true;
  value.family = "tag36h11";
  value.tag_id = tag_id;
  value.detection_quality = 0.92F;
  value.observation_sequence = 7U;
  value.stamp_ns = stamp_ns;
  return value;
}

SemanticSubmissionEvidence submission()
{
  SemanticSubmissionEvidence value;
  value.mission_id = "mission_alpha";
  value.actor_id = "operator_cli";
  value.tag_family = "tag36h11";
  value.tag_id = 42;
  value.suggested_location_id = "lab_a";
  value.suggested_display_name = "Lab A";
  value.suggested_semantic_type = "laboratory";
  return value;
}

void pause_for_semantics(SemanticInterruptionCore & core)
{
  ASSERT_TRUE(core.Observe(observation(), mission(), false, 20 * kSecond).accepted);
  core.BeginPause(20 * kSecond);
  core.PauseAccepted(20 * kSecond);
  core.UpdateMission(
    mission(SemanticInterruptionCore::kMissionStatePaused),
    21 * kSecond);
  ASSERT_EQ(
    core.snapshot().state,
    SemanticInterruptionState::WaitingForSemantics);
}

}  // namespace

TEST(SemanticInterruption, FiltersInvalidStaleIneligibleAndKnownObservations)
{
  SemanticInterruptionCore core;
  auto weak = observation();
  weak.detection_quality = 0.2F;
  EXPECT_EQ(
    core.Observe(weak, mission(), false, 20 * kSecond).disposition,
    SemanticObservationDisposition::InvalidObservation);

  auto stale_mission = mission();
  stale_mission.received_ns = 1;
  EXPECT_EQ(
    core.Observe(observation(), stale_mission, false, 20 * kSecond).disposition,
    SemanticObservationDisposition::MissionStale);

  EXPECT_EQ(
    core.Observe(
      observation(), mission(SemanticInterruptionCore::kMissionStatePaused),
      false, 20 * kSecond).disposition,
    SemanticObservationDisposition::MissionIneligible);

  EXPECT_EQ(
    core.Observe(observation(), mission(), true, 20 * kSecond).disposition,
    SemanticObservationDisposition::KnownLocationDuplicate);
}

TEST(SemanticInterruption, RejectsObservationsThatPredateMission)
{
  SemanticInterruptionConfig config;
  config.observation_timeout_ns = 5 * kSecond;
  SemanticInterruptionCore core(config);
  auto current_mission = mission();
  current_mission.received_ns = 12 * kSecond;
  EXPECT_EQ(
    core.Observe(
      observation(42, 9 * kSecond), current_mission, false, 12 * kSecond).disposition,
    SemanticObservationDisposition::PredatesMission);
}

TEST(SemanticInterruption, PausesAndWaitsForConfirmedPausedState)
{
  SemanticInterruptionCore core;
  ASSERT_TRUE(core.Observe(observation(), mission(), false, 20 * kSecond).accepted);
  EXPECT_EQ(core.snapshot().state, SemanticInterruptionState::TagDetected);

  core.BeginPause(20 * kSecond);
  EXPECT_EQ(core.snapshot().state, SemanticInterruptionState::Pausing);
  core.PauseAccepted(20 * kSecond);
  EXPECT_EQ(
    core.snapshot().state,
    SemanticInterruptionState::WaitingForMissionPaused);

  core.UpdateMission(mission(), 21 * kSecond);
  EXPECT_EQ(
    core.snapshot().state,
    SemanticInterruptionState::WaitingForMissionPaused);
  core.UpdateMission(
    mission(SemanticInterruptionCore::kMissionStatePaused),
    22 * kSecond);
  EXPECT_EQ(
    core.snapshot().state,
    SemanticInterruptionState::WaitingForSemantics);
}

TEST(SemanticInterruption, ValidatesSemanticSubmissionIdentityAndRequiredFields)
{
  SemanticInterruptionCore core;
  pause_for_semantics(core);

  auto wrong_mission = submission();
  wrong_mission.mission_id = "other";
  EXPECT_EQ(
    core.ValidateSubmission(wrong_mission).result_code,
    SemanticInterruptionCore::kSubmitMissionMismatch);

  auto wrong_tag = submission();
  wrong_tag.tag_id = 7;
  EXPECT_EQ(
    core.ValidateSubmission(wrong_tag).result_code,
    SemanticInterruptionCore::kSubmitTagMismatch);

  auto invalid = submission();
  invalid.suggested_display_name.clear();
  EXPECT_EQ(
    core.ValidateSubmission(invalid).result_code,
    SemanticInterruptionCore::kSubmitInvalid);

  EXPECT_TRUE(core.ValidateSubmission(submission()).accepted);
}

TEST(SemanticInterruption, CompletesRegistrationAndResumeFlow)
{
  SemanticInterruptionCore core;
  pause_for_semantics(core);
  ASSERT_TRUE(core.ValidateSubmission(submission()).accepted);
  core.BeginRegistration("am6_mission_alpha_tag36h11_42", 22 * kSecond);
  EXPECT_EQ(core.snapshot().state, SemanticInterruptionState::Registering);

  core.RegistrationSucceeded("am6_mission_alpha_tag36h11_42", 23 * kSecond);
  core.BeginResume(23 * kSecond);
  EXPECT_EQ(core.snapshot().state, SemanticInterruptionState::Resuming);

  core.UpdateMission(mission(), 24 * kSecond);
  EXPECT_EQ(core.snapshot().state, SemanticInterruptionState::Completed);
  EXPECT_FALSE(core.snapshot().active);
  EXPECT_TRUE(core.snapshot().registration_complete);
  EXPECT_TRUE(core.snapshot().resume_complete);
}

TEST(SemanticInterruption, SuppressesActiveRegisteredAndCooldownDuplicates)
{
  SemanticInterruptionCore core;
  ASSERT_TRUE(core.Observe(observation(), mission(), false, 20 * kSecond).accepted);
  EXPECT_EQ(
    core.Observe(observation(), mission(), false, 20 * kSecond).disposition,
    SemanticObservationDisposition::ActiveDuplicate);
  EXPECT_EQ(core.snapshot().duplicate_observations_suppressed, 1U);

  core.Fail("fixture_failure", 21 * kSecond);
  EXPECT_EQ(
    core.Observe(
      observation(42, 22 * kSecond), mission(), false, 22 * kSecond).disposition,
    SemanticObservationDisposition::CooldownDuplicate);

  EXPECT_TRUE(core.Observe(
      observation(43, 22 * kSecond), mission(), false, 22 * kSecond).accepted);
}

TEST(SemanticInterruption, SuccessfulTagIsSuppressedForProcessLifetime)
{
  SemanticInterruptionCore core;
  pause_for_semantics(core);
  core.BeginRegistration("candidate", 22 * kSecond);
  core.RegistrationSucceeded("candidate", 23 * kSecond);
  core.BeginResume(23 * kSecond);
  core.UpdateMission(mission(), 24 * kSecond);

  auto later_mission = mission();
  later_mission.mission_id = "mission_beta";
  later_mission.mission_start_ns = 60 * kSecond;
  later_mission.received_ns = 60 * kSecond;
  EXPECT_EQ(
    core.Observe(
      observation(42, 60 * kSecond), later_mission, false, 60 * kSecond).disposition,
    SemanticObservationDisposition::RegisteredDuplicate);
}

TEST(SemanticInterruption, TimesOutPauseSemanticsRegistrationAndResume)
{
  SemanticInterruptionConfig config;
  config.pause_timeout_ns = kSecond;
  config.semantic_input_timeout_ns = kSecond;
  config.registration_timeout_ns = kSecond;
  config.resume_timeout_ns = kSecond;

  SemanticInterruptionCore pause_core(config);
  ASSERT_TRUE(pause_core.Observe(observation(), mission(), false, 20 * kSecond).accepted);
  pause_core.BeginPause(20 * kSecond);
  EXPECT_TRUE(pause_core.Tick(22 * kSecond));
  EXPECT_EQ(pause_core.snapshot().reason, "mission_pause_timed_out");

  SemanticInterruptionCore semantic_core(config);
  pause_for_semantics(semantic_core);
  EXPECT_TRUE(semantic_core.Tick(23 * kSecond));
  EXPECT_EQ(semantic_core.snapshot().reason, "semantic_input_timed_out");

  SemanticInterruptionCore registration_core(config);
  pause_for_semantics(registration_core);
  registration_core.BeginRegistration("candidate", 22 * kSecond);
  EXPECT_TRUE(registration_core.Tick(24 * kSecond));
  EXPECT_EQ(
    registration_core.snapshot().reason,
    "mapped_location_registration_timed_out");

  SemanticInterruptionCore resume_core(config);
  pause_for_semantics(resume_core);
  resume_core.BeginRegistration("candidate", 22 * kSecond);
  resume_core.RegistrationSucceeded("candidate", 22 * kSecond);
  resume_core.BeginResume(22 * kSecond);
  EXPECT_TRUE(resume_core.Tick(24 * kSecond));
  EXPECT_EQ(resume_core.snapshot().reason, "mission_resume_timed_out");
}

TEST(SemanticInterruption, FailurePolicyCanRequestTypedResume)
{
  SemanticInterruptionConfig config;
  config.failure_policy = SemanticFailurePolicy::RequestResume;
  SemanticInterruptionCore core(config);
  pause_for_semantics(core);
  core.BeginRegistration("candidate", 22 * kSecond);
  core.Fail("registration_failed", 23 * kSecond);
  ASSERT_TRUE(core.should_resume_after_failure());
  ASSERT_TRUE(core.snapshot().active);

  core.BeginFailureResume(23 * kSecond);
  EXPECT_EQ(core.snapshot().state, SemanticInterruptionState::Resuming);
  core.UpdateMission(mission(), 24 * kSecond);
  EXPECT_EQ(core.snapshot().state, SemanticInterruptionState::Failed);
  EXPECT_EQ(core.snapshot().reason, "registration_failed");
  EXPECT_TRUE(core.snapshot().resume_complete);
}
