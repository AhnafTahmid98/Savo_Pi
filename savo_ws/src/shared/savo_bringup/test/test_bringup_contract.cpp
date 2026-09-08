// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include "savo_bringup/bringup_contract.hpp"

TEST(BringupContract, ParsesEveryPublishedValue)
{
  EXPECT_EQ(
    savo_bringup::ParseHostRole("core"),
    savo_bringup::HostRole::kCore);
  EXPECT_EQ(
    savo_bringup::ParseHostRole("EDGE"),
    savo_bringup::HostRole::kEdge);
  EXPECT_EQ(
    savo_bringup::ParseRobotMode("autonomous_mapping"),
    savo_bringup::RobotMode::kAutonomousMapping);
  EXPECT_EQ(
    savo_bringup::ParseBringupProfile("lidar_d435_voxel"),
    savo_bringup::BringupProfile::kLidarD435Voxel);
}

TEST(BringupContract, RejectsUnknownValues)
{
  EXPECT_FALSE(savo_bringup::ParseHostRole("remote"));
  EXPECT_FALSE(savo_bringup::ParseRobotMode("mappingish"));
  EXPECT_FALSE(savo_bringup::ParseBringupProfile("unsafe"));
}

TEST(BringupContract, AutonomousMappingRequiresMappingAndNavigation)
{
  const auto requirements = savo_bringup::RequirementsFor(
    savo_bringup::HostRole::kCore,
    savo_bringup::RobotMode::kAutonomousMapping,
    savo_bringup::BringupProfile::kLidarOnly,
    false,
    false,
    false,
    false);

  EXPECT_TRUE(requirements.core_required);
  EXPECT_TRUE(requirements.supervisor_required);
  EXPECT_TRUE(requirements.mapping_required);
  EXPECT_TRUE(requirements.navigation_required);
  EXPECT_TRUE(requirements.locked_geometry_required);
  EXPECT_FALSE(requirements.voxel_layer_enabled);
}

TEST(BringupContract, EdgeRequirementsFollowEnabledComponents)
{
  const auto requirements = savo_bringup::RequirementsFor(
    savo_bringup::HostRole::kEdge,
    savo_bringup::RobotMode::kSavedMapNavigation,
    savo_bringup::BringupProfile::kLidarD435Voxel,
    true,
    true,
    true,
    false);

  EXPECT_TRUE(requirements.edge_required);
  EXPECT_TRUE(requirements.bridge_required);
  EXPECT_TRUE(requirements.realsense_required);
  EXPECT_TRUE(requirements.vo_required);
  EXPECT_FALSE(requirements.speech_required);
  EXPECT_TRUE(requirements.voxel_layer_enabled);
}

TEST(BringupContract, VoxelProfileFailsClosedUntilValidated)
{
  EXPECT_EQ(
    savo_bringup::ValidateCombination(
      savo_bringup::HostRole::kCore,
      savo_bringup::RobotMode::kSavedMapNavigation,
      savo_bringup::BringupProfile::kLidarD435Voxel,
      false,
      true,
      false),
    "d435_voxel_profile_requires_explicit_hardware_validation");
}

TEST(BringupContract, MotionRequiresLockedGeometry)
{
  EXPECT_EQ(
    savo_bringup::ValidateCombination(
      savo_bringup::HostRole::kCore,
      savo_bringup::RobotMode::kManual,
      savo_bringup::BringupProfile::kLidarOnly,
      false,
      false,
      false),
    "motion_profile_requires_locked_geometry_validation");
}

TEST(BringupContract, BenchDiagnosticsAllowsProvisionalGeometry)
{
  EXPECT_TRUE(
    savo_bringup::ValidateCombination(
      savo_bringup::HostRole::kCore,
      savo_bringup::RobotMode::kDiagnostics,
      savo_bringup::BringupProfile::kBench,
      false,
      false,
      true).empty());
}

TEST(BringupContract, SerializedNamesRemainStable)
{
  EXPECT_EQ(savo_bringup::ToString(savo_bringup::HostRole::kAll), "all");
  EXPECT_EQ(
    savo_bringup::ToString(savo_bringup::RobotMode::kSavedMapNavigation),
    "saved_map_navigation");
  EXPECT_EQ(
    savo_bringup::ToString(savo_bringup::BringupProfile::kLidarD435Voxel),
    "lidar_d435_voxel");
  EXPECT_EQ(
    savo_bringup::ToString(savo_bringup::ReadinessState::kWaitingForMapContext),
    "waiting_for_map_context");
  EXPECT_EQ(
    savo_bringup::ParseReadinessState("waiting_for_navigation"),
    savo_bringup::ReadinessState::kWaitingForNavigation);
  EXPECT_FALSE(savo_bringup::ParseReadinessState("almost_ready"));
}

TEST(BringupContract, ProductionAndDistributedAuthorityFailClosed)
{
  EXPECT_EQ(
    savo_bringup::ValidateCombination(
      savo_bringup::HostRole::kCore,
      savo_bringup::RobotMode::kSafeIdle,
      savo_bringup::BringupProfile::kProduction,
      false, false, false),
    "production_profile_requires_locked_geometry");
  EXPECT_EQ(
    savo_bringup::ValidateCombination(
      savo_bringup::HostRole::kAll,
      savo_bringup::RobotMode::kSafeIdle,
      savo_bringup::BringupProfile::kLidarOnly,
      false, true, false),
    "all_host_role_is_bench_only");
}

TEST(BringupContract, ReadinessCoversReadyWaitingStaleBlockedAndDegraded)
{
  using savo_bringup::DependencyStatus;
  using savo_bringup::ReadinessState;
  const DependencyStatus ready{"safety", ReadinessState::kWaitingForSafety,
    true, true, true, true, false, "clear"};
  const auto ready_decision = savo_bringup::EvaluateReadiness({ready}, true, false);
  EXPECT_TRUE(ready_decision.ready);
  EXPECT_EQ(ready_decision.state, ReadinessState::kReady);

  auto waiting = ready;
  waiting.observed = false;
  const auto waiting_decision = savo_bringup::EvaluateReadiness({waiting}, true, false);
  EXPECT_FALSE(waiting_decision.ready);
  EXPECT_EQ(waiting_decision.state, ReadinessState::kWaitingForSafety);

  auto stale = ready;
  stale.fresh = false;
  const auto stale_decision = savo_bringup::EvaluateReadiness({stale}, true, false);
  EXPECT_EQ(stale_decision.state, ReadinessState::kWaitingForSafety);
  EXPECT_EQ(stale_decision.missing, std::vector<std::string>{"safety:stale"});

  auto failed = ready;
  failed.failed = true;
  failed.detail = "fault";
  const auto blocked = savo_bringup::EvaluateReadiness({failed}, true, false);
  EXPECT_EQ(blocked.state, ReadinessState::kBlocked);
  EXPECT_EQ(blocked.failed, std::vector<std::string>{"safety:fault"});

  auto optional = failed;
  optional.required = false;
  optional.name = "speech";
  const auto degraded = savo_bringup::EvaluateReadiness({ready, optional}, true, false);
  EXPECT_TRUE(degraded.ready);
  EXPECT_EQ(degraded.state, ReadinessState::kDegraded);

  const auto invalid = savo_bringup::EvaluateReadiness({}, false, false);
  EXPECT_FALSE(invalid.ready);
  EXPECT_EQ(invalid.state, ReadinessState::kBlocked);
}

TEST(BringupContract, StartupStageRequiresContinuousStableReadiness)
{
  using savo_bringup::QualityLevel;
  using savo_bringup::StartupStageInput;
  using savo_bringup::StartupStageState;
  using savo_bringup::StartupStageTiming;
  using savo_bringup::StartupStageTracker;

  StartupStageTracker tracker(StartupStageTiming{1.0, 2.0, 10.0});
  StartupStageInput input;
  input.processes_started = true;
  input.dependencies_ready = true;
  input.quality = QualityLevel::kGood;

  EXPECT_EQ(tracker.Update(0.5, input).state, StartupStageState::kWaitingForDependencies);
  EXPECT_EQ(tracker.Update(1.0, input).state, StartupStageState::kStabilizing);

  input.dependencies_ready = false;
  input.reason = "imu_stale";
  EXPECT_EQ(tracker.Update(2.5, input).state, StartupStageState::kWaitingForDependencies);

  input.dependencies_ready = true;
  EXPECT_EQ(tracker.Update(3.0, input).state, StartupStageState::kStabilizing);
  EXPECT_FALSE(tracker.Update(4.9, input).ready);
  const auto ready = tracker.Update(5.0, input);
  EXPECT_TRUE(ready.ready);
  EXPECT_EQ(ready.state, StartupStageState::kReady);
  EXPECT_EQ(ready.quality, QualityLevel::kGood);
}

TEST(BringupContract, StartupStageTimeoutIsTerminal)
{
  using savo_bringup::StartupStageInput;
  using savo_bringup::StartupStageState;
  using savo_bringup::StartupStageTiming;
  using savo_bringup::StartupStageTracker;

  StartupStageTracker tracker(StartupStageTiming{0.0, 1.0, 3.0});
  StartupStageInput input;
  input.processes_started = true;
  input.reason = "tof_left_not_observed";
  const auto failed = tracker.Update(3.0, input);
  EXPECT_TRUE(failed.failed);
  EXPECT_EQ(failed.state, StartupStageState::kFailed);
  EXPECT_EQ(failed.reason, "stage_startup_timeout:tof_left_not_observed");

  input.dependencies_ready = true;
  EXPECT_TRUE(tracker.Update(3.1, input).failed);
}

TEST(BringupContract, StartupStageRejectsBelowMinimumRequiredQuality)
{
  using savo_bringup::QualityLevel;
  using savo_bringup::StartupStageInput;
  using savo_bringup::StartupStageState;
  using savo_bringup::StartupStageTiming;
  using savo_bringup::StartupStageTracker;

  StartupStageTracker tracker(StartupStageTiming{0.0, 1.0, 5.0});
  StartupStageInput input;
  input.processes_started = true;
  input.dependencies_ready = true;
  input.quality = QualityLevel::kBelowMinimum;

  const auto decision = tracker.Update(2.0, input);
  EXPECT_FALSE(decision.ready);
  EXPECT_EQ(decision.state, StartupStageState::kWaitingForDependencies);
  EXPECT_EQ(decision.reason, "required_quality_below_minimum");
}

TEST(BringupContract, QualityAggregationIsWorstRequiredAndFailClosed)
{
  using savo_bringup::QualityLevel;
  EXPECT_EQ(
    savo_bringup::WorstRequiredQuality(
      {QualityLevel::kExcellent, QualityLevel::kMinimum, QualityLevel::kGood}, true),
    QualityLevel::kMinimum);
  EXPECT_EQ(
    savo_bringup::WorstRequiredQuality({}, true),
    QualityLevel::kMinimum);
  EXPECT_EQ(
    savo_bringup::WorstRequiredQuality({QualityLevel::kExcellent}, false),
    QualityLevel::kBelowMinimum);
}

TEST(BringupContract, HealthyPerceptionAggregateIgnoresOptionalStaleChildren)
{
  using savo_bringup::QualityLevel;
  const auto evaluation =
    savo_bringup::EvaluateStructuredHealthPayload(
    R"({
    "overall_ok": true,
    "overall_status": "OK",
    "quality": "MINIMUM",
    "required_sensors": ["tof_left", "tof_right"],
    "optional_sensors": ["depth_front", "ultrasonic_front"],
    "disabled_sensors": ["ultrasonic_front"],
    "sensors": [
      {
        "sensor_name": "depth_front",
        "status": "STALE",
        "ok": false,
        "required": false,
        "optional": true
      },
      {"sensor_name": "tof_left", "status": "OK", "ok": true, "required": true},
      {"sensor_name": "tof_right", "status": "OK", "ok": true, "required": true}
    ]
  })");

  ASSERT_TRUE(evaluation);
  EXPECT_TRUE(evaluation->valid);
  EXPECT_TRUE(evaluation->ready);
  EXPECT_FALSE(evaluation->failed);
  EXPECT_EQ(evaluation->quality, QualityLevel::kMinimum);
}

TEST(BringupContract, UnhealthyPerceptionAggregateFailsClosed)
{
  const auto evaluation =
    savo_bringup::EvaluateStructuredHealthPayload(
    R"({
    "overall_ok": false,
    "overall_status": "STALE",
    "quality": "BELOW_MINIMUM",
    "stale_required_sensors": ["tof_left"]
  })");

  ASSERT_TRUE(evaluation);
  EXPECT_TRUE(evaluation->valid);
  EXPECT_FALSE(evaluation->ready);
  EXPECT_TRUE(evaluation->failed);
}

TEST(BringupContract, BelowMinimumAggregateQualityCannotBecomeReady)
{
  using savo_bringup::QualityLevel;
  const auto evaluation =
    savo_bringup::EvaluateStructuredHealthPayload(
    R"({
    "overall_ok": true,
    "overall_status": "OK",
    "quality": "BELOW_MINIMUM"
  })");

  ASSERT_TRUE(evaluation);
  EXPECT_TRUE(evaluation->valid);
  EXPECT_FALSE(evaluation->ready);
  EXPECT_FALSE(evaluation->failed);
  EXPECT_EQ(evaluation->quality, QualityLevel::kBelowMinimum);
}

TEST(BringupContract, StructuredHealthNeverReportsReadyAndFailed)
{
  for (const auto * payload : {
    R"({"overall_ok":true,"overall_status":"OK","quality":"MINIMUM"})",
    R"({"overall_ok":false,"overall_status":"ERROR","quality":"BELOW_MINIMUM"})",
    R"({"ready":true,"state":"STALE","quality":"EXCELLENT"})"})
  {
    const auto evaluation = savo_bringup::EvaluateStructuredHealthPayload(payload);
    ASSERT_TRUE(evaluation);
    EXPECT_FALSE(evaluation->ready && evaluation->failed);
  }
}

TEST(BringupContract, LocalizationAggregateIgnoresOptionalVoChildFailure)
{
  using savo_bringup::QualityLevel;
  const auto evaluation =
    savo_bringup::EvaluateStructuredHealthPayload(
    R"({
    "state": "DEGRADED",
    "ready": true,
    "quality": "GOOD",
    "components": {
      "imu": {"required": true, "ready": true},
      "wheel_odom": {"required": true, "ready": true},
      "vo_odom": {"required": false, "ready": false, "state": "STALE"}
    }
  })");

  ASSERT_TRUE(evaluation);
  EXPECT_TRUE(evaluation->valid);
  EXPECT_TRUE(evaluation->ready);
  EXPECT_FALSE(evaluation->failed);
  EXPECT_EQ(evaluation->quality, QualityLevel::kGood);
}
