// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include "savo_supervisor/mission_authority.hpp"

namespace
{

savo_supervisor::MissionDependencySnapshot healthy_dependencies()
{
  savo_supervisor::MissionDependencySnapshot dependencies;
  dependencies.core.lifecycle = savo_supervisor::Lifecycle::RUNNING;
  dependencies.core.health = savo_supervisor::AggregateHealth::OK;
  dependencies.core.safety = savo_supervisor::SafetyObservation::CLEAR;
  dependencies.core.ready = true;
  dependencies.core.capabilities.core_health_ready = true;
  dependencies.core.capabilities.core_safety_ready = true;
  dependencies.core.capabilities.core_motion_ready = true;
  dependencies.core.capabilities.can_manual_drive = true;
  dependencies.core.capabilities.can_rotate = true;
  dependencies.core.capabilities.can_start_geometric_mapping = true;

  dependencies.mapping.received = true;
  dependencies.mapping.fresh = true;
  dependencies.mapping.valid = true;
  dependencies.mapping.healthy = true;
  dependencies.mapping.ready = true;
  dependencies.mapping.slam_active = true;

  dependencies.navigation.received = true;
  dependencies.navigation.fresh = true;
  dependencies.navigation.valid = true;
  dependencies.navigation.ready = true;
  dependencies.navigation.goal_acceptance_allowed = true;

  dependencies.head.received = true;
  dependencies.head.fresh = true;
  dependencies.head.valid = true;
  dependencies.head.operational = true;
  dependencies.head.pan_tilt_ready = true;
  dependencies.head.camera_ready = true;
  dependencies.head.camera_pose_ready = true;

  dependencies.locations.received = true;
  dependencies.locations.fresh = true;
  dependencies.locations.valid = true;
  dependencies.locations.read_ready = true;
  dependencies.locations.write_ready = true;
  dependencies.locations.storage_healthy = true;

  dependencies.endpoints.autonomous_mapping_action = true;
  dependencies.endpoints.rotate_to_heading_action = true;
  dependencies.endpoints.coverage_action = true;
  dependencies.endpoints.apriltag_confirmation_action = true;

  dependencies.map_context.type = savo_supervisor::MapContextType::kLiveMapping;
  dependencies.map_context.map_id = "floor_2";
  dependencies.map_context.map_revision = 1U;
  dependencies.system.armed = true;
  dependencies.system.remote_commands_ready = true;
  return dependencies;
}

savo_supervisor::MissionAuthorizationRequest request(
  savo_supervisor::AuthorityCommand command,
  savo_supervisor::MissionOperation operation)
{
  savo_supervisor::MissionAuthorizationRequest value;
  value.command = command;
  value.operation = operation;
  value.request_id = "request-1";
  value.actor_id = "operator";
  value.map_id = "floor_2";
  value.map_revision = 1U;
  value.motion_required = savo_supervisor::IsExclusiveOperation(operation);
  return value;
}

}  // namespace

TEST(MissionAuthority, HealthySemanticMappingCapabilitiesAreReady)
{
  savo_supervisor::MissionAuthority authority;
  const auto capabilities = authority.EvaluateCapabilities(healthy_dependencies());
  EXPECT_TRUE(capabilities.can_start_manual_mapping);
  EXPECT_TRUE(capabilities.can_start_autonomous_mapping);
  EXPECT_TRUE(capabilities.can_run_scan360);
  EXPECT_TRUE(capabilities.can_run_coverage);
  EXPECT_TRUE(capabilities.semantic_mapping_ready);
  EXPECT_TRUE(capabilities.can_register_location);
}

TEST(MissionAuthority, NavigationRequiresApprovedSavedRelease)
{
  auto dependencies = healthy_dependencies();
  savo_supervisor::MissionAuthority authority;
  EXPECT_FALSE(authority.EvaluateCapabilities(dependencies).can_navigate);

  dependencies.map_context.type = savo_supervisor::MapContextType::kSavedRelease;
  dependencies.map_context.map_release_id = "release-1";
  dependencies.map_context.approved = true;
  EXPECT_TRUE(authority.EvaluateCapabilities(dependencies).can_navigate);
}

TEST(MissionAuthority, ExclusiveOperationOwnershipRejectsConflict)
{
  auto dependencies = healthy_dependencies();
  savo_supervisor::MissionAuthority authority;
  auto first = request(
    savo_supervisor::AuthorityCommand::kAcquire,
    savo_supervisor::MissionOperation::kAutonomousMapping);
  first.require_semantic = true;
  EXPECT_TRUE(authority.Handle(first, dependencies).authorized);

  auto conflicting = request(
    savo_supervisor::AuthorityCommand::kAcquire,
    savo_supervisor::MissionOperation::kManualControl);
  conflicting.request_id = "request-2";
  EXPECT_FALSE(authority.Handle(conflicting, dependencies).authorized);
  EXPECT_EQ(
    authority.state().operation,
    savo_supervisor::MissionOperation::kAutonomousMapping);
}

TEST(MissionAuthority, RuntimeFaultRevokesAndRequiresExplicitResume)
{
  auto dependencies = healthy_dependencies();
  savo_supervisor::MissionAuthority authority;
  auto acquire = request(
    savo_supervisor::AuthorityCommand::kAcquire,
    savo_supervisor::MissionOperation::kAutonomousMapping);
  acquire.require_semantic = true;
  ASSERT_TRUE(authority.Handle(acquire, dependencies).authorized);

  dependencies.navigation.fresh = false;
  EXPECT_TRUE(authority.Revalidate(dependencies));
  EXPECT_EQ(authority.state().state, savo_supervisor::OperationState::kRevoked);

  dependencies.navigation.fresh = true;
  EXPECT_FALSE(authority.Revalidate(dependencies));
  EXPECT_EQ(authority.state().state, savo_supervisor::OperationState::kRevoked);

  auto resume = request(
    savo_supervisor::AuthorityCommand::kResume,
    savo_supervisor::MissionOperation::kAutonomousMapping);
  resume.expected_generation = authority.state().generation;
  EXPECT_TRUE(authority.Handle(resume, dependencies).authorized);
  EXPECT_EQ(authority.state().state, savo_supervisor::OperationState::kActive);
}

TEST(MissionAuthority, SafetyStopBlocksMotionWithoutBlockingReview)
{
  auto dependencies = healthy_dependencies();
  dependencies.core.safety = savo_supervisor::SafetyObservation::STOPPED;
  dependencies.core.capabilities.core_motion_ready = false;

  savo_supervisor::MissionAuthority authority;
  auto navigation = request(
    savo_supervisor::AuthorityCommand::kCheck,
    savo_supervisor::MissionOperation::kNavigateToLocation);
  EXPECT_FALSE(authority.Handle(navigation, dependencies).authorized);

  auto review = request(
    savo_supervisor::AuthorityCommand::kCheck,
    savo_supervisor::MissionOperation::kReviewLocation);
  review.motion_required = false;
  EXPECT_TRUE(authority.Handle(review, dependencies).authorized);
}

TEST(MissionAuthority, ActiveLeaseRejectsWrongActorAndStaleGeneration)
{
  auto dependencies = healthy_dependencies();
  savo_supervisor::MissionAuthority authority;
  auto acquire = request(
    savo_supervisor::AuthorityCommand::kAcquire,
    savo_supervisor::MissionOperation::kAutonomousMapping);
  acquire.require_semantic = true;
  ASSERT_TRUE(authority.Handle(acquire, dependencies).authorized);

  auto check = request(
    savo_supervisor::AuthorityCommand::kCheck,
    savo_supervisor::MissionOperation::kAutonomousMapping);
  check.require_semantic = true;
  check.actor_id = "different-operator";
  check.expected_generation = authority.state().generation;
  const auto wrong_actor = authority.Handle(check, dependencies);
  EXPECT_FALSE(wrong_actor.authorized);
  EXPECT_EQ(
    wrong_actor.code,
    savo_supervisor::MissionAuthorizationCode::kOwnershipMismatch);

  check.actor_id = "operator";
  check.expected_generation = authority.state().generation + 1U;
  const auto stale_generation = authority.Handle(check, dependencies);
  EXPECT_FALSE(stale_generation.authorized);
  EXPECT_EQ(
    stale_generation.code,
    savo_supervisor::MissionAuthorizationCode::kOwnershipMismatch);
}

TEST(MissionAuthority, NavigationLeaseRevokesWhenSavedMapContextChanges)
{
  auto dependencies = healthy_dependencies();
  dependencies.map_context.type = savo_supervisor::MapContextType::kSavedRelease;
  dependencies.map_context.map_release_id = "release-1";
  dependencies.map_context.approved = true;

  savo_supervisor::MissionAuthority authority;
  auto acquire = request(
    savo_supervisor::AuthorityCommand::kAcquire,
    savo_supervisor::MissionOperation::kNavigateToLocation);
  acquire.map_release_id = "release-1";
  ASSERT_TRUE(authority.Handle(acquire, dependencies).authorized);

  dependencies.map_context.map_release_id = "release-2";
  EXPECT_TRUE(authority.Revalidate(dependencies));
  EXPECT_EQ(authority.state().state, savo_supervisor::OperationState::kRevoked);
  EXPECT_EQ(
    authority.state().reason,
    "runtime_authorization_revoked:navigation_not_ready_or_map_mismatch");
}

TEST(MissionAuthority, DisarmedSystemBlocksExclusiveMotion)
{
  auto dependencies = healthy_dependencies();
  dependencies.system.armed = false;
  savo_supervisor::MissionAuthority authority;
  auto acquire = request(
    savo_supervisor::AuthorityCommand::kAcquire,
    savo_supervisor::MissionOperation::kManualControl);
  const auto decision = authority.Handle(acquire, dependencies);
  EXPECT_FALSE(decision.authorized);
  EXPECT_EQ(decision.reason, "system_not_armed");
}

TEST(MissionAuthority, BridgeLossRevokesOnlyRemoteOriginLease)
{
  auto dependencies = healthy_dependencies();
  savo_supervisor::MissionAuthority remote_authority;
  auto remote = request(
    savo_supervisor::AuthorityCommand::kAcquire,
    savo_supervisor::MissionOperation::kManualControl);
  remote.remote_origin = true;
  ASSERT_TRUE(remote_authority.Handle(remote, dependencies).authorized);
  dependencies.system.remote_commands_ready = false;
  EXPECT_TRUE(remote_authority.Revalidate(dependencies));
  EXPECT_EQ(
    remote_authority.state().reason,
    "runtime_authorization_revoked:remote_command_path_not_ready");

  dependencies.system.remote_commands_ready = true;
  savo_supervisor::MissionAuthority local_authority;
  auto local = request(
    savo_supervisor::AuthorityCommand::kAcquire,
    savo_supervisor::MissionOperation::kManualControl);
  local.remote_origin = false;
  ASSERT_TRUE(local_authority.Handle(local, dependencies).authorized);
  dependencies.system.remote_commands_ready = false;
  EXPECT_FALSE(local_authority.Revalidate(dependencies));
  EXPECT_EQ(local_authority.state().state, savo_supervisor::OperationState::kActive);
}

TEST(MissionAuthority, MissingBridgeRejectsRemoteButAllowsSafeLocalOrigin)
{
  auto dependencies = healthy_dependencies();
  dependencies.system.remote_commands_ready = false;
  savo_supervisor::MissionAuthority authority;

  auto remote = request(
    savo_supervisor::AuthorityCommand::kAcquire,
    savo_supervisor::MissionOperation::kManualControl);
  remote.remote_origin = true;
  const auto denied = authority.Handle(remote, dependencies);
  EXPECT_FALSE(denied.authorized);
  EXPECT_EQ(denied.reason, "remote_command_path_not_ready");

  auto local = remote;
  local.request_id = "local-request-1";
  local.actor_id = "local-operator";
  local.remote_origin = false;
  const auto allowed = authority.Handle(local, dependencies);
  EXPECT_TRUE(allowed.authorized);
  EXPECT_EQ(authority.state().state, savo_supervisor::OperationState::kActive);
}

TEST(MissionAuthority, MissingOrStaleHeadBlocksHeadRequiredOperations)
{
  auto dependencies = healthy_dependencies();
  savo_supervisor::MissionAuthority authority;

  dependencies.head.received = false;
  dependencies.head.valid = false;
  auto capabilities = authority.EvaluateCapabilities(dependencies);
  EXPECT_FALSE(capabilities.head_ready);
  EXPECT_FALSE(capabilities.semantic_mapping_ready);
  EXPECT_FALSE(capabilities.can_start_autonomous_mapping);

  dependencies.head.received = true;
  dependencies.head.valid = true;
  dependencies.head.fresh = false;
  capabilities = authority.EvaluateCapabilities(dependencies);
  EXPECT_FALSE(capabilities.head_ready);
  EXPECT_FALSE(capabilities.semantic_mapping_ready);
  EXPECT_FALSE(capabilities.can_start_autonomous_mapping);
}
