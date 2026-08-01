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
