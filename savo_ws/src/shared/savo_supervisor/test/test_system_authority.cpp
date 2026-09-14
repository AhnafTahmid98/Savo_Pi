// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include <chrono>
#include <string>

#include "savo_supervisor/system_authority.hpp"

namespace
{

savo_supervisor::SystemDependencySnapshot healthy_dependencies()
{
  savo_supervisor::SystemDependencySnapshot value;
  value.core_ready = true;
  value.safety_known = true;
  value.startup_dependencies_ready = true;
  value.remote_commands_ready = true;
  value.mission_idle = true;
  return value;
}

savo_supervisor::SystemAuthorityRequest request(savo_supervisor::SystemCommand command)
{
  savo_supervisor::SystemAuthorityRequest value;
  value.command = command;
  value.request_id = "system-request-1";
  value.actor_id = "operator";
  return value;
}

}  // namespace

TEST(SystemAuthority, ArmFailsClosedUntilStartupDependenciesAreReady)
{
  savo_supervisor::SystemAuthority authority;
  auto dependencies = healthy_dependencies();
  dependencies.startup_dependencies_ready = false;
  const auto decision = authority.Handle(
    request(savo_supervisor::SystemCommand::kArm), dependencies);
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.code, savo_supervisor::SystemAuthorityCode::kNotReady);
}

TEST(SystemAuthority, ExplicitLocalArmDoesNotRequireRemoteCommandPath)
{
  savo_supervisor::SystemAuthority authority;
  auto dependencies = healthy_dependencies();
  dependencies.remote_commands_ready = false;
  dependencies.degraded = true;

  const auto decision = authority.Handle(
    request(savo_supervisor::SystemCommand::kArm), dependencies);
  ASSERT_TRUE(decision.accepted);
  const auto snapshot = authority.snapshot(dependencies);
  EXPECT_TRUE(snapshot.startup_ready);
  EXPECT_TRUE(snapshot.armed);
  EXPECT_FALSE(snapshot.remote_commands_ready);
  EXPECT_EQ(
    snapshot.state,
    savo_supervisor::SystemAuthorityState::kArmedDegraded);
}

TEST(SystemAuthority, ExplicitArmAndDisarmAreGenerationProtected)
{
  savo_supervisor::SystemAuthority authority;
  const auto dependencies = healthy_dependencies();
  const auto armed = authority.Handle(
    request(savo_supervisor::SystemCommand::kArm), dependencies);
  ASSERT_TRUE(armed.accepted);
  auto snapshot = authority.snapshot(dependencies);
  EXPECT_TRUE(snapshot.armed);
  EXPECT_EQ(snapshot.state, savo_supervisor::SystemAuthorityState::kArmed);

  auto stale = request(savo_supervisor::SystemCommand::kDisarm);
  stale.expected_generation = snapshot.generation + 1U;
  EXPECT_FALSE(authority.Handle(stale, dependencies).accepted);

  auto disarm = request(savo_supervisor::SystemCommand::kDisarm);
  disarm.expected_generation = snapshot.generation;
  EXPECT_TRUE(authority.Handle(disarm, dependencies).accepted);
  EXPECT_FALSE(authority.snapshot(dependencies).armed);
}

TEST(SystemAuthority, CoreFaultLatchesAfterArm)
{
  savo_supervisor::SystemAuthority authority;
  auto dependencies = healthy_dependencies();
  ASSERT_TRUE(authority.Handle(
      request(savo_supervisor::SystemCommand::kArm), dependencies).accepted);
  dependencies.core_fault = {savo_supervisor::CoreFaultKind::kCritical,
    "control:ERROR:control_error"};
  EXPECT_TRUE(authority.Update(dependencies));
  const auto snapshot = authority.snapshot(dependencies);
  EXPECT_TRUE(snapshot.fault_latched);
  EXPECT_FALSE(snapshot.armed);
}

TEST(SystemAuthority, SingleFreshnessInterruptionDisarmsWithoutPersistentLatch)
{
  savo_supervisor::SystemAuthority authority;
  auto dependencies = healthy_dependencies();
  ASSERT_TRUE(authority.Handle(
      request(savo_supervisor::SystemCommand::kArm), dependencies).accepted);
  dependencies.core_ready = false;
  dependencies.core_fault = {savo_supervisor::CoreFaultKind::kUnavailable,
    "localization:STALE:localization_heartbeat_stale"};
  EXPECT_TRUE(authority.Update(dependencies));
  EXPECT_FALSE(authority.snapshot(dependencies).armed);
  EXPECT_FALSE(authority.snapshot(dependencies).fault_latched);
  dependencies = healthy_dependencies();
  (void)authority.Update(dependencies);
  EXPECT_FALSE(authority.snapshot(dependencies).armed);
}

TEST(SystemAuthority, FaultLatchRequiresRecoveredIdleSystemToClear)
{
  savo_supervisor::SystemAuthority authority;
  authority.RestoreFaultLatch(true, 4U, "restored_fault");
  auto dependencies = healthy_dependencies();
  dependencies.mission_idle = false;
  EXPECT_FALSE(authority.Handle(
      request(savo_supervisor::SystemCommand::kClearFaultLatch), dependencies).accepted);
  dependencies.mission_idle = true;
  const auto cleared = authority.Handle(
    request(savo_supervisor::SystemCommand::kClearFaultLatch), dependencies);
  EXPECT_TRUE(cleared.accepted);
  EXPECT_FALSE(authority.snapshot(dependencies).fault_latched);
}

TEST(SystemAuthority, ControlledShutdownDisarmsAndCannotRearm)
{
  savo_supervisor::SystemAuthority authority;
  const auto dependencies = healthy_dependencies();
  ASSERT_TRUE(authority.Handle(
      request(savo_supervisor::SystemCommand::kArm), dependencies).accepted);
  ASSERT_TRUE(authority.Handle(
      request(savo_supervisor::SystemCommand::kBeginShutdown), dependencies).accepted);
  const auto snapshot = authority.snapshot(dependencies);
  EXPECT_TRUE(snapshot.shutdown_requested);
  EXPECT_FALSE(snapshot.armed);
  EXPECT_FALSE(authority.Handle(
      request(savo_supervisor::SystemCommand::kArm), dependencies).accepted);
}

TEST(SystemAuthority, SafetyStopDoesNotCreatePersistentCoreFaultLatch)
{
  savo_supervisor::SystemAuthority authority;
  auto dependencies = healthy_dependencies();
  ASSERT_TRUE(authority.Handle(
      request(savo_supervisor::SystemCommand::kArm), dependencies).accepted);
  dependencies.safety_known = true;
  dependencies.core_ready = true;
  EXPECT_FALSE(authority.Update(dependencies));
  const auto snapshot = authority.snapshot(dependencies);
  EXPECT_TRUE(snapshot.armed);
  EXPECT_FALSE(snapshot.fault_latched);
}

TEST(SystemAuthority, RestoredFaultLatchNeverRestoresArmedState)
{
  savo_supervisor::SystemAuthority authority;
  const auto dependencies = healthy_dependencies();
  authority.RestoreFaultLatch(true, 9U, "persisted_core_fault");
  const auto snapshot = authority.snapshot(dependencies);
  EXPECT_FALSE(snapshot.armed);
  EXPECT_TRUE(snapshot.fault_latched);
  EXPECT_EQ(snapshot.state, savo_supervisor::SystemAuthorityState::kFaultLatched);
  EXPECT_EQ(snapshot.generation, 9U);
}

TEST(SystemAuthority, OptionalCapabilityLossReportsArmedDegraded)
{
  savo_supervisor::SystemAuthority authority;
  auto dependencies = healthy_dependencies();
  ASSERT_TRUE(authority.Handle(
      request(savo_supervisor::SystemCommand::kArm), dependencies).accepted);
  dependencies.degraded = true;
  const auto snapshot = authority.snapshot(dependencies);
  EXPECT_TRUE(snapshot.armed);
  EXPECT_EQ(
    snapshot.state,
    savo_supervisor::SystemAuthorityState::kArmedDegraded);
  EXPECT_TRUE(snapshot.remote_commands_ready);
}

TEST(SystemAuthority, IdempotentCommandUsesAlreadyInStateResult)
{
  savo_supervisor::SystemAuthority authority;
  const auto dependencies = healthy_dependencies();
  ASSERT_TRUE(authority.Handle(
      request(savo_supervisor::SystemCommand::kArm), dependencies).accepted);
  const auto repeated = authority.Handle(
    request(savo_supervisor::SystemCommand::kArm), dependencies);
  EXPECT_TRUE(repeated.accepted);
  EXPECT_EQ(
    repeated.code,
    savo_supervisor::SystemAuthorityCode::kAlreadyInState);
}

TEST(SystemAuthority, RestartNeverRestoresStaleArmedReason)
{
  savo_supervisor::SystemAuthority authority;
  const auto dependencies = healthy_dependencies();
  authority.RestoreFaultLatch(false, 5U, "system_armed");
  const auto snapshot = authority.snapshot(dependencies);
  EXPECT_FALSE(snapshot.armed);
  EXPECT_EQ(snapshot.reason, "system_disarmed_after_restart");
}

TEST(SystemAuthority, ClearArmAndRepeatedHealthyEvaluationsDoNotRelatch)
{
  savo_supervisor::SystemAuthority authority;
  const auto dependencies = healthy_dependencies();
  authority.RestoreFaultLatch(true, 42U, "old_fault");
  ASSERT_TRUE(authority.Handle(
      request(savo_supervisor::SystemCommand::kClearFaultLatch), dependencies).accepted);
  EXPECT_EQ(authority.snapshot(dependencies).generation, 43U);
  EXPECT_EQ(authority.snapshot(dependencies).state,
    savo_supervisor::SystemAuthorityState::kReadyToArm);
  ASSERT_TRUE(authority.Handle(
      request(savo_supervisor::SystemCommand::kArm), dependencies).accepted);
  for (int index = 0; index < 100; ++index) {
    EXPECT_FALSE(authority.Update(dependencies));
    EXPECT_TRUE(authority.snapshot(dependencies).armed);
    EXPECT_FALSE(authority.snapshot(dependencies).fault_latched);
  }
}

TEST(SystemAuthority, ContinuousUnavailabilityLatchesAfterDisarmingAtFirstObservation)
{
  using namespace std::chrono_literals;
  savo_supervisor::SystemAuthority authority;
  auto dependencies = healthy_dependencies();
  ASSERT_TRUE(authority.Handle(
      request(savo_supervisor::SystemCommand::kArm), dependencies).accepted);
  const auto start = std::chrono::steady_clock::time_point{};
  dependencies.core_ready = false;
  dependencies.core_fault = {savo_supervisor::CoreFaultKind::kUnavailable,
    "lidar:STALE:lidar_heartbeat_stale"};
  ASSERT_TRUE(authority.Update(dependencies, start));
  EXPECT_FALSE(authority.snapshot(dependencies).armed);
  EXPECT_FALSE(authority.snapshot(dependencies).remote_commands_ready);
  EXPECT_FALSE(authority.Update(dependencies, start + 999ms));
  EXPECT_FALSE(authority.snapshot(dependencies).fault_latched);
  ASSERT_TRUE(authority.Update(dependencies, start + 1s));
  const auto latched = authority.snapshot(dependencies);
  EXPECT_TRUE(latched.fault_latched);
  EXPECT_EQ(latched.reason, "core_fault_latched:lidar:STALE:lidar_heartbeat_stale");
  EXPECT_FALSE(authority.Update(healthy_dependencies(), start + 2s));
  EXPECT_EQ(authority.snapshot(healthy_dependencies()).reason, latched.reason);
}

TEST(SystemAuthority, RecoveryResetsQualificationButNeverAutomaticallyRearms)
{
  using namespace std::chrono_literals;
  savo_supervisor::SystemAuthority authority({true, true});
  const auto healthy = healthy_dependencies();
  ASSERT_TRUE(authority.Update(healthy));
  auto stale = healthy;
  stale.core_ready = false;
  stale.core_fault = {savo_supervisor::CoreFaultKind::kUnavailable, "control:STALE:status_stale"};
  const auto start = std::chrono::steady_clock::time_point{};
  EXPECT_TRUE(authority.Update(stale, start));
  EXPECT_FALSE(authority.Update(healthy, start + 500ms));
  EXPECT_FALSE(authority.Update(stale, start + 2s));
  EXPECT_FALSE(authority.snapshot(stale).fault_latched);
  EXPECT_FALSE(authority.Update(healthy, start + 3s));
  EXPECT_FALSE(authority.snapshot(healthy).armed);
  EXPECT_TRUE(authority.Handle(
      request(savo_supervisor::SystemCommand::kArm), healthy).accepted);
}

TEST(SystemAuthority, CriticalFaultDuringQualificationLatchesImmediately)
{
  using namespace std::chrono_literals;
  savo_supervisor::SystemAuthority authority;
  auto dependencies = healthy_dependencies();
  ASSERT_TRUE(authority.Handle(
      request(savo_supervisor::SystemCommand::kArm), dependencies).accepted);
  const auto start = std::chrono::steady_clock::time_point{};
  dependencies.core_ready = false;
  dependencies.core_fault = {savo_supervisor::CoreFaultKind::kUnavailable, "control:STALE:stale"};
  EXPECT_TRUE(authority.Update(dependencies, start));
  dependencies.core_fault = {savo_supervisor::CoreFaultKind::kCritical,
    "base_battery:ERROR:base_battery_critical"};
  EXPECT_TRUE(authority.Update(dependencies, start + 1ms));
  EXPECT_TRUE(authority.snapshot(dependencies).fault_latched);
  EXPECT_EQ(authority.snapshot(dependencies).reason,
    "core_fault_latched:base_battery:ERROR:base_battery_critical");
}

TEST(SystemAuthority, CoreClassificationKeepsSourceStateAndPrioritizesCriticalFaults)
{
  savo_supervisor::SupervisorState core;
  core.lifecycle = savo_supervisor::Lifecycle::FAULTED;
  core.health = savo_supervisor::AggregateHealth::ERROR;
  core.safety = savo_supervisor::SafetyObservation::CLEAR;
  core.safety_summary.ready = true;
  savo_supervisor::ComponentSummary component;
  component.name = "localization";
  component.required = true;
  component.enabled = true;
  component.state = savo_supervisor::ComponentState::STALE;
  component.reason_code = "localization_heartbeat_stale";
  core.component_summaries = {component};
  auto dependencies = savo_supervisor::EvaluateCoreSystemDependencies(core);
  EXPECT_FALSE(dependencies.core_ready);
  EXPECT_EQ(dependencies.core_fault.kind, savo_supervisor::CoreFaultKind::kUnavailable);
  EXPECT_EQ(dependencies.core_fault.reason, "localization:STALE:localization_heartbeat_stale");
  component.name = "core_ups";
  component.state = savo_supervisor::ComponentState::ERROR;
  component.reason_code = "core_ups_critical";
  core.component_summaries.push_back(component);
  dependencies = savo_supervisor::EvaluateCoreSystemDependencies(core);
  EXPECT_EQ(dependencies.core_fault.kind, savo_supervisor::CoreFaultKind::kCritical);
  EXPECT_EQ(dependencies.core_fault.reason, "core_ups:ERROR:core_ups_critical");
  core.component_summaries.back().state = savo_supervisor::ComponentState::INVALID;
  core.component_summaries.back().reason_code = "ros_time_regression_detected";
  dependencies = savo_supervisor::EvaluateCoreSystemDependencies(core);
  EXPECT_EQ(dependencies.core_fault.kind, savo_supervisor::CoreFaultKind::kCritical);
  EXPECT_EQ(dependencies.core_fault.reason, "core_ups:INVALID:ros_time_regression_detected");
}

TEST(SystemAuthority, SafetyFreshnessBlocksImmediatelyButIntegrityFaultIsImmediatelyLatchWorthy)
{
  for (const std::string reason : {"safety_stop_missing", "safety_stop_stale",
      "safety_slowdown_missing", "safety_slowdown_stale"})
  {
    savo_supervisor::SupervisorState core;
    core.lifecycle = savo_supervisor::Lifecycle::FAULTED;
    core.safety_summary.reason_code = reason;
    auto dependencies = savo_supervisor::EvaluateCoreSystemDependencies(core);
    EXPECT_FALSE(dependencies.core_ready);
    EXPECT_FALSE(dependencies.safety_known);
    EXPECT_EQ(dependencies.core_fault.kind, savo_supervisor::CoreFaultKind::kUnavailable);
    savo_supervisor::SystemAuthority authority;
    ASSERT_TRUE(authority.Handle(
        request(savo_supervisor::SystemCommand::kArm), healthy_dependencies()).accepted);
    EXPECT_TRUE(authority.Update(dependencies));
    EXPECT_FALSE(authority.snapshot(dependencies).armed);
    EXPECT_FALSE(authority.snapshot(dependencies).fault_latched);
    core.safety_summary.reason_code = "safety_message_invalid";
    dependencies = savo_supervisor::EvaluateCoreSystemDependencies(core);
    EXPECT_EQ(dependencies.core_fault.kind, savo_supervisor::CoreFaultKind::kCritical);
    EXPECT_TRUE(authority.Update(dependencies));
    EXPECT_TRUE(authority.snapshot(dependencies).fault_latched);
    EXPECT_EQ(authority.snapshot(dependencies).reason,
      "core_fault_latched:safety:UNKNOWN:safety_message_invalid");
  }
}

TEST(SystemAuthority, EnvironmentalStopAndOptionalEdgeFaultAreNotCoreFaults)
{
  savo_supervisor::SupervisorState core;
  core.lifecycle = savo_supervisor::Lifecycle::RUNNING;
  core.health = savo_supervisor::AggregateHealth::DEGRADED;
  core.ready = true;
  core.capabilities.core_health_ready = true;
  core.capabilities.core_safety_ready = true;
  core.capabilities.core_motion_ready = false;
  core.safety = savo_supervisor::SafetyObservation::STOPPED;
  core.safety_summary.ready = true;
  core.safety_summary.stop_active = true;
  savo_supervisor::ComponentSummary edge;
  edge.name = "edge_ups";
  edge.enabled = true;
  edge.required = false;
  edge.state = savo_supervisor::ComponentState::ERROR;
  core.component_summaries = {edge};
  auto dependencies = savo_supervisor::EvaluateCoreSystemDependencies(core);
  dependencies.startup_dependencies_ready = true;
  EXPECT_TRUE(dependencies.core_ready);
  EXPECT_EQ(dependencies.core_fault.kind, savo_supervisor::CoreFaultKind::kNone);
  savo_supervisor::SystemAuthority authority;
  ASSERT_TRUE(authority.Handle(
      request(savo_supervisor::SystemCommand::kArm), healthy_dependencies()).accepted);
  EXPECT_FALSE(authority.Update(dependencies));
  EXPECT_TRUE(authority.snapshot(dependencies).armed);
  EXPECT_FALSE(authority.snapshot(dependencies).fault_latched);
}
