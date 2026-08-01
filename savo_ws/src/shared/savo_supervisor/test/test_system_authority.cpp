// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

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
  dependencies.core_faulted = true;
  EXPECT_TRUE(authority.Update(dependencies));
  const auto snapshot = authority.snapshot(dependencies);
  EXPECT_TRUE(snapshot.fault_latched);
  EXPECT_FALSE(snapshot.armed);
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
  dependencies.core_faulted = false;
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
