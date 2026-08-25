// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "savo_supervisor/supervisor_policy.hpp"

namespace
{

savo_supervisor::ComponentSummary ready_component(
  const std::string & name,
  bool degraded = false)
{
  savo_supervisor::ComponentSummary result;
  result.name = name;
  result.enabled = true;
  result.required = true;
  result.ready = true;
  result.degraded = degraded;
  result.state = degraded ?
    savo_supervisor::ComponentState::DEGRADED :
    savo_supervisor::ComponentState::OK;
  return result;
}

std::vector<savo_supervisor::ComponentSummary> healthy_core()
{
  return {
    ready_component("base"),
    ready_component("control"),
    ready_component("perception"),
    ready_component("lidar"),
    ready_component("localization"),
    ready_component("power")};
}

savo_supervisor::SafetySummary clear_safety()
{
  savo_supervisor::SafetySummary result;
  result.observation = savo_supervisor::SafetyObservation::CLEAR;
  result.ready = true;
  result.reason_code = "safety_clear";
  return result;
}

rclcpp::Time test_time()
{
  return rclcpp::Time(10, 0, RCL_ROS_TIME);
}

}  // namespace

TEST(CoreReadinessPolicy, HealthyCoreEnablesMotionAndMapping)
{
  savo_supervisor::SupervisorPolicy policy;
  const auto state = policy.EvaluateSupervisor(
    healthy_core(), clear_safety(), test_time(), 10.0);
  EXPECT_TRUE(state.ready);
  EXPECT_TRUE(state.capabilities.core_motion_ready);
  EXPECT_TRUE(state.capabilities.can_manual_drive);
  EXPECT_TRUE(state.capabilities.can_rotate);
  EXPECT_TRUE(state.capabilities.can_start_geometric_mapping);
}

TEST(CoreReadinessPolicy, SafetyStopBlocksMotionWithoutFaultingSupervisor)
{
  savo_supervisor::SupervisorPolicy policy;
  auto safety = clear_safety();
  safety.observation = savo_supervisor::SafetyObservation::STOPPED;
  safety.stop_active = true;
  safety.reason_code = "safety_stop_active";

  const auto state = policy.EvaluateSupervisor(
    healthy_core(), safety, test_time(), 10.0);
  EXPECT_EQ(state.lifecycle, savo_supervisor::Lifecycle::RUNNING);
  EXPECT_EQ(state.health, savo_supervisor::AggregateHealth::DEGRADED);
  EXPECT_TRUE(state.ready);
  EXPECT_FALSE(state.capabilities.core_motion_ready);
  EXPECT_FALSE(state.capabilities.can_manual_drive);
}

TEST(CoreReadinessPolicy, UnknownSafetyFaultsTruthfulReadiness)
{
  savo_supervisor::SupervisorPolicy policy;
  savo_supervisor::SafetySummary safety;
  safety.ready = false;
  safety.reason_code = "safety_stop_stale";
  const auto state = policy.EvaluateSupervisor(
    healthy_core(), safety, test_time(), 10.0);
  EXPECT_EQ(state.lifecycle, savo_supervisor::Lifecycle::FAULTED);
  EXPECT_FALSE(state.ready);
}

TEST(CoreReadinessPolicy, LowPowerAllowsManualMotionButBlocksNewMapping)
{
  savo_supervisor::SupervisorPolicy policy;
  auto core = healthy_core();
  core.back() = ready_component("power", true);
  const auto state = policy.EvaluateSupervisor(
    core, clear_safety(), test_time(), 10.0);
  EXPECT_TRUE(state.capabilities.can_manual_drive);
  EXPECT_FALSE(state.capabilities.can_start_geometric_mapping);
}

TEST(CoreReadinessPolicy, MissingRequiredLidarFaultsSupervisor)
{
  savo_supervisor::SupervisorPolicy policy;
  auto core = healthy_core();
  core[3].ready = false;
  core[3].state = savo_supervisor::ComponentState::STALE;
  core[3].reason_code = "lidar_heartbeat_stale";
  const auto state = policy.EvaluateSupervisor(
    core, clear_safety(), test_time(), 10.0);
  EXPECT_EQ(state.lifecycle, savo_supervisor::Lifecycle::FAULTED);
  EXPECT_FALSE(state.ready);
}

TEST(CoreReadinessPolicy, MissingOptionalLidarAllowsSafeLocalMotionOnly)
{
  savo_supervisor::SupervisorPolicy policy;
  auto core = healthy_core();
  core[3].required = false;
  core[3].ready = false;
  core[3].state = savo_supervisor::ComponentState::STALE;
  core[3].reason_code = "lidar_heartbeat_stale";
  const auto state = policy.EvaluateSupervisor(
    core, clear_safety(), test_time(), 10.0);
  EXPECT_EQ(state.lifecycle, savo_supervisor::Lifecycle::RUNNING);
  EXPECT_TRUE(state.ready);
  EXPECT_TRUE(state.capabilities.can_manual_drive);
  EXPECT_FALSE(state.capabilities.can_start_geometric_mapping);
}

TEST(CoreReadinessPolicy, MissingRequiredLocalizationBlocksMappingAndStartup)
{
  savo_supervisor::SupervisorPolicy policy;
  auto core = healthy_core();
  core[4].ready = false;
  core[4].state = savo_supervisor::ComponentState::STALE;
  core[4].reason_code = "localization_heartbeat_stale";
  const auto state = policy.EvaluateSupervisor(
    core, clear_safety(), test_time(), 10.0);
  EXPECT_EQ(state.lifecycle, savo_supervisor::Lifecycle::FAULTED);
  EXPECT_FALSE(state.ready);
  EXPECT_FALSE(state.capabilities.can_start_geometric_mapping);
}

TEST(CoreReadinessPolicy, ZeroSlowdownBlocksMotionLikeAStop)
{
  savo_supervisor::SupervisorPolicy policy;
  auto safety = clear_safety();
  safety.observation = savo_supervisor::SafetyObservation::SLOWDOWN;
  safety.slowdown_factor = 0.0;
  safety.reason_code = "safety_slowdown_active";

  const auto state = policy.EvaluateSupervisor(
    healthy_core(), safety, test_time(), 10.0);
  EXPECT_EQ(state.lifecycle, savo_supervisor::Lifecycle::RUNNING);
  EXPECT_TRUE(state.ready);
  EXPECT_FALSE(state.capabilities.core_motion_ready);
  EXPECT_FALSE(state.capabilities.can_manual_drive);
  EXPECT_FALSE(state.capabilities.can_rotate);
}

TEST(CoreReadinessPolicy, BaseSafetyBlockRevokesMotionCapability)
{
  savo_supervisor::SupervisorPolicy policy;
  auto core = healthy_core();
  core[0] = ready_component("base", true);
  core[0].reason_code = "base_safely_blocked";

  const auto state = policy.EvaluateSupervisor(
    core, clear_safety(), test_time(), 10.0);
  EXPECT_EQ(state.lifecycle, savo_supervisor::Lifecycle::RUNNING);
  EXPECT_EQ(state.health, savo_supervisor::AggregateHealth::DEGRADED);
  EXPECT_TRUE(state.ready);
  EXPECT_FALSE(state.capabilities.core_motion_ready);
  EXPECT_FALSE(state.capabilities.can_manual_drive);
  EXPECT_FALSE(state.capabilities.can_rotate);
}

TEST(CoreReadinessPolicy, ControlInhibitionRevokesMotionCapability)
{
  savo_supervisor::SupervisorPolicy policy;
  auto core = healthy_core();
  core[1] = ready_component("control", true);
  core[1].reason_code = "control_safely_inhibited";

  const auto state = policy.EvaluateSupervisor(
    core, clear_safety(), test_time(), 10.0);
  EXPECT_EQ(state.lifecycle, savo_supervisor::Lifecycle::RUNNING);
  EXPECT_TRUE(state.ready);
  EXPECT_FALSE(state.capabilities.core_motion_ready);
  EXPECT_FALSE(state.capabilities.can_manual_drive);
}
