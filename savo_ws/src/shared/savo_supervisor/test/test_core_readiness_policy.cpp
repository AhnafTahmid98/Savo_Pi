// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "savo_supervisor/supervisor_policy.hpp"
#include "savo_supervisor/system_authority.hpp"

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
  auto edge_ups = ready_component("edge_ups");
  edge_ups.required = false;
  return {
    ready_component("base"),
    ready_component("control"),
    ready_component("perception"),
    ready_component("lidar"),
    ready_component("localization"),
    ready_component("base_battery"),
    ready_component("core_ups"),
    edge_ups};
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

TEST(CoreReadinessPolicy, ExpiredRequiredMessageFaultsReadinessWithoutImmediatePersistentLatch)
{
  savo_supervisor::SupervisorPolicy policy;
  savo_supervisor::ComponentStatus lidar;
  lidar.config = policy.lidar;
  lidar.summary_valid = true;
  lidar.summary_ready = true;
  lidar.summary_state = "OK";
  lidar.heartbeat_valid = true;
  lidar.heartbeat_alive = true;
  lidar.heartbeat_ready = true;
  lidar.heartbeat_state = "OK";
  lidar.ever_operational = true;
  const rclcpp::Time last_received(10, 0, RCL_ROS_TIME);
  lidar.summary_tracker.observe_message(last_received, std::nullopt, false, "");
  lidar.heartbeat_tracker.observe_message(last_received, std::nullopt, false, "");
  auto components = healthy_core();
  components[3] = policy.EvaluateComponent(lidar, last_received, 10.0);
  auto core = policy.EvaluateSupervisor(components, clear_safety(), last_received, 10.0);
  auto dependencies = savo_supervisor::EvaluateCoreSystemDependencies(core);
  dependencies.startup_dependencies_ready = true;
  savo_supervisor::SystemAuthority authority;
  savo_supervisor::SystemAuthorityRequest arm;
  arm.command = savo_supervisor::SystemCommand::kArm;
  arm.request_id = "fresh-core";
  arm.actor_id = "operator";
  ASSERT_TRUE(authority.Handle(arm, dependencies).accepted);

  const rclcpp::Time expired(12, 1, RCL_ROS_TIME);
  components[3] = policy.EvaluateComponent(lidar, expired, 12.0);
  ASSERT_EQ(components[3].state, savo_supervisor::ComponentState::STALE);
  core = policy.EvaluateSupervisor(components, clear_safety(), expired, 12.0);
  EXPECT_EQ(core.lifecycle, savo_supervisor::Lifecycle::FAULTED);
  EXPECT_FALSE(core.capabilities.core_motion_ready);
  dependencies = savo_supervisor::EvaluateCoreSystemDependencies(core);
  EXPECT_EQ(dependencies.core_fault.reason, "lidar:STALE:lidar_summary_stale");
  ASSERT_TRUE(authority.Update(dependencies));
  EXPECT_FALSE(authority.snapshot(dependencies).armed);
  EXPECT_FALSE(authority.snapshot(dependencies).fault_latched);

  // A clock integrity fault must not receive the freshness-only persistence grace.
  lidar.summary_tracker.observe_message(rclcpp::Time(9, 0, RCL_ROS_TIME),
    std::nullopt, false, "");
  components[3] = policy.EvaluateComponent(lidar, expired, 12.0);
  ASSERT_EQ(components[3].state, savo_supervisor::ComponentState::INVALID);
  core = policy.EvaluateSupervisor(components, clear_safety(), expired, 12.0);
  dependencies = savo_supervisor::EvaluateCoreSystemDependencies(core);
  EXPECT_EQ(dependencies.core_fault.kind, savo_supervisor::CoreFaultKind::kCritical);
  EXPECT_TRUE(authority.Update(dependencies));
  EXPECT_TRUE(authority.snapshot(dependencies).fault_latched);
  EXPECT_EQ(authority.snapshot(dependencies).reason,
    "core_fault_latched:lidar:INVALID:ros_time_regression_detected");
}

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
  core[5] = ready_component("base_battery", true);
  const auto state = policy.EvaluateSupervisor(
    core, clear_safety(), test_time(), 10.0);
  EXPECT_TRUE(state.capabilities.can_manual_drive);
  EXPECT_FALSE(state.capabilities.can_start_geometric_mapping);
}

TEST(CoreReadinessPolicy, OptionalEdgeUpsFailureDoesNotBlockCoreMapping)
{
  savo_supervisor::SupervisorPolicy policy;
  auto core = healthy_core();
  core.back().ready = false;
  core.back().state = savo_supervisor::ComponentState::ERROR;
  core.back().reason_code = "edge_ups_critical";

  const auto state = policy.EvaluateSupervisor(
    core, clear_safety(), test_time(), 10.0);
  EXPECT_EQ(state.lifecycle, savo_supervisor::Lifecycle::RUNNING);
  EXPECT_TRUE(state.capabilities.core_motion_ready);
  EXPECT_TRUE(state.capabilities.can_start_geometric_mapping);
}

TEST(CoreReadinessPolicy, RequiredEdgeUpsFailureBlocksCoreMapping)
{
  savo_supervisor::SupervisorPolicy policy;
  auto core = healthy_core();
  core.back().required = true;
  core.back().ready = false;
  core.back().state = savo_supervisor::ComponentState::ERROR;
  core.back().reason_code = "edge_ups_critical";

  const auto state = policy.EvaluateSupervisor(
    core, clear_safety(), test_time(), 10.0);
  EXPECT_EQ(state.lifecycle, savo_supervisor::Lifecycle::FAULTED);
  EXPECT_FALSE(state.capabilities.core_motion_ready);
  EXPECT_FALSE(state.capabilities.can_start_geometric_mapping);
}

TEST(CoreReadinessPolicy, LowCoreUpsIsOperationalButBlocksNewMapping)
{
  savo_supervisor::SupervisorPolicy policy;
  auto core = healthy_core();
  core[6] = ready_component("core_ups", true);

  const auto state = policy.EvaluateSupervisor(
    core, clear_safety(), test_time(), 10.0);
  EXPECT_EQ(state.lifecycle, savo_supervisor::Lifecycle::RUNNING);
  EXPECT_TRUE(state.capabilities.core_motion_ready);
  EXPECT_FALSE(state.capabilities.can_start_geometric_mapping);
}

TEST(CoreReadinessPolicy, CriticalBaseBatteryFailsClosed)
{
  savo_supervisor::SupervisorPolicy policy;
  auto core = healthy_core();
  core[5].ready = false;
  core[5].state = savo_supervisor::ComponentState::ERROR;
  core[5].reason_code = "base_battery_critical";

  const auto state = policy.EvaluateSupervisor(
    core, clear_safety(), test_time(), 10.0);
  EXPECT_EQ(state.lifecycle, savo_supervisor::Lifecycle::FAULTED);
  EXPECT_EQ(state.reason_code, "base_battery_critical");
  EXPECT_FALSE(state.capabilities.core_motion_ready);
}

TEST(CoreReadinessPolicy, StaleCoreUpsFailsClosedWithSourceReason)
{
  savo_supervisor::SupervisorPolicy policy;
  auto core = healthy_core();
  core[6].ready = false;
  core[6].state = savo_supervisor::ComponentState::STALE;
  core[6].reason_code = "core_ups_stale";

  const auto state = policy.EvaluateSupervisor(
    core, clear_safety(), test_time(), 10.0);
  EXPECT_EQ(state.lifecycle, savo_supervisor::Lifecycle::FAULTED);
  EXPECT_EQ(state.reason_code, "core_ups_stale");
  EXPECT_FALSE(state.capabilities.core_motion_ready);
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
