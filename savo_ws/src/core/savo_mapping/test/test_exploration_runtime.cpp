#include "savo_mapping/exploration_runtime.hpp"

#include <gtest/gtest.h>

#include <string>

namespace
{

using savo_mapping::ExplorationMode;
using savo_mapping::MappingMode;
using savo_mapping::SessionState;
using savo_mapping::WorkflowPhase;

using savo_mapping::exploration_runtime::
RuntimeDisposition;

using savo_mapping::exploration_runtime::
RuntimeInputs;

RuntimeInputs ready_frontier_inputs()
{
  RuntimeInputs inputs;

  inputs.mode =
    MappingMode::Frontier;

  inputs.exploration_mode =
    ExplorationMode::Frontier;

  inputs.workflow_phase =
    WorkflowPhase::Exploring;

  inputs.session_state =
    SessionState::Active;

  inputs.readiness_received = true;
  inputs.mapping_ready = true;

  inputs.safety_stop_received = true;
  inputs.safety_stop_active = false;

  inputs.handoff_state_received = true;
  inputs.handoff_active = false;

  return inputs;
}

}  // namespace

TEST(
  ExplorationRuntimeContract,
  DispositionStringsAreStable)
{
  using savo_mapping::
  exploration_runtime::to_string;

  EXPECT_EQ(
    std::string{
    to_string(
        RuntimeDisposition::
      WaitingForAuthority)},
    "waiting_for_authority");

  EXPECT_EQ(
    std::string{
    to_string(
        RuntimeDisposition::Disabled)},
    "disabled");

  EXPECT_EQ(
    std::string{
    to_string(
        RuntimeDisposition::Enabled)},
    "enabled");

  EXPECT_EQ(
    std::string{
    to_string(
        RuntimeDisposition::
      CancelRequired)},
    "cancel_required");
}

TEST(
  ExplorationRuntimeContract,
  IncompleteAuthorityWaitsConservatively)
{
  const RuntimeInputs inputs;

  EXPECT_FALSE(
    savo_mapping::
    exploration_runtime::
    authority_state_complete(inputs));

  const auto decision =
    savo_mapping::
    exploration_runtime::
    evaluate(inputs);

  EXPECT_EQ(
    decision.disposition,
    RuntimeDisposition::
    WaitingForAuthority);

  EXPECT_FALSE(
    decision.frontier_enabled);

  EXPECT_FALSE(
    decision.cancel_active_goal);

  EXPECT_EQ(
    decision.reason,
    "waiting_for_authority");
}

TEST(
  ExplorationRuntimeContract,
  IncompleteAuthorityCancelsActiveGoal)
{
  RuntimeInputs inputs;

  inputs.handoff_state_received = true;
  inputs.handoff_active = true;

  const auto decision =
    savo_mapping::
    exploration_runtime::
    evaluate(inputs);

  EXPECT_EQ(
    decision.disposition,
    RuntimeDisposition::
    CancelRequired);

  EXPECT_FALSE(
    decision.frontier_enabled);

  EXPECT_TRUE(
    decision.cancel_active_goal);

  EXPECT_EQ(
    decision.reason,
    "cancel_required: "
    "authority_state_incomplete");
}

TEST(
  ExplorationRuntimeContract,
  DirectFrontierModeEnablesPlanner)
{
  const auto inputs =
    ready_frontier_inputs();

  EXPECT_TRUE(
    savo_mapping::
    exploration_runtime::
    authority_state_complete(inputs));

  EXPECT_TRUE(
    savo_mapping::
    exploration_runtime::
    frontier_authorized_by_workflow(
      inputs));

  const auto decision =
    savo_mapping::
    exploration_runtime::
    evaluate(inputs);

  EXPECT_EQ(
    decision.disposition,
    RuntimeDisposition::Enabled);

  EXPECT_TRUE(
    decision.frontier_enabled);

  EXPECT_FALSE(
    decision.cancel_active_goal);

  EXPECT_EQ(
    decision.reason,
    "enabled: frontier_authorized");
}

TEST(
  ExplorationRuntimeContract,
  AutonomousFrontierStrategyEnablesPlanner)
{
  auto inputs =
    ready_frontier_inputs();

  inputs.mode =
    MappingMode::Autonomous;

  const auto decision =
    savo_mapping::
    exploration_runtime::
    evaluate(inputs);

  EXPECT_EQ(
    decision.disposition,
    RuntimeDisposition::Enabled);

  EXPECT_TRUE(
    decision.frontier_enabled);
}

TEST(
  ExplorationRuntimeContract,
  WorkflowMismatchDisablesPlanner)
{
  auto inputs =
    ready_frontier_inputs();

  inputs.workflow_phase =
    WorkflowPhase::Mapping;

  const auto decision =
    savo_mapping::
    exploration_runtime::
    evaluate(inputs);

  EXPECT_EQ(
    decision.disposition,
    RuntimeDisposition::Disabled);

  EXPECT_FALSE(
    decision.frontier_enabled);

  EXPECT_FALSE(
    decision.cancel_active_goal);

  EXPECT_EQ(
    decision.reason,
    "disabled: "
    "frontier_not_authorized");
}

TEST(
  ExplorationRuntimeContract,
  DifferentExplorationStrategyCancelsGoal)
{
  auto inputs =
    ready_frontier_inputs();

  inputs.mode =
    MappingMode::Coverage;

  inputs.exploration_mode =
    ExplorationMode::Coverage;

  inputs.workflow_phase =
    WorkflowPhase::Coverage;

  inputs.handoff_active = true;

  const auto decision =
    savo_mapping::
    exploration_runtime::
    evaluate(inputs);

  EXPECT_EQ(
    decision.disposition,
    RuntimeDisposition::
    CancelRequired);

  EXPECT_TRUE(
    decision.cancel_active_goal);

  EXPECT_EQ(
    decision.reason,
    "cancel_required: "
    "frontier_not_authorized");
}

TEST(
  ExplorationRuntimeContract,
  SafetyStopDisablesOrCancels)
{
  auto inputs =
    ready_frontier_inputs();

  inputs.safety_stop_active = true;

  auto decision =
    savo_mapping::
    exploration_runtime::
    evaluate(inputs);

  EXPECT_EQ(
    decision.disposition,
    RuntimeDisposition::Disabled);

  EXPECT_EQ(
    decision.reason,
    "disabled: safety_stop_active");

  inputs.handoff_active = true;

  decision =
    savo_mapping::
    exploration_runtime::
    evaluate(inputs);

  EXPECT_EQ(
    decision.disposition,
    RuntimeDisposition::
    CancelRequired);

  EXPECT_TRUE(
    decision.cancel_active_goal);

  EXPECT_EQ(
    decision.reason,
    "cancel_required: "
    "safety_stop_active");
}

TEST(
  ExplorationRuntimeContract,
  MappingReadinessLossCancelsGoal)
{
  auto inputs =
    ready_frontier_inputs();

  inputs.mapping_ready = false;
  inputs.handoff_active = true;

  const auto decision =
    savo_mapping::
    exploration_runtime::
    evaluate(inputs);

  EXPECT_EQ(
    decision.disposition,
    RuntimeDisposition::
    CancelRequired);

  EXPECT_TRUE(
    decision.cancel_active_goal);

  EXPECT_EQ(
    decision.reason,
    "cancel_required: "
    "mapping_not_ready");
}

TEST(
  ExplorationRuntimeContract,
  InactiveSessionCancelsGoal)
{
  auto inputs =
    ready_frontier_inputs();

  inputs.session_state =
    SessionState::Paused;

  inputs.handoff_active = true;

  const auto decision =
    savo_mapping::
    exploration_runtime::
    evaluate(inputs);

  EXPECT_EQ(
    decision.disposition,
    RuntimeDisposition::
    CancelRequired);

  EXPECT_TRUE(
    decision.cancel_active_goal);

  EXPECT_EQ(
    decision.reason,
    "cancel_required: "
    "session_not_active");
}

TEST(
  ExplorationRuntimeContract,
  ActiveHandoffRemainsValidWhenAuthorized)
{
  auto inputs =
    ready_frontier_inputs();

  inputs.handoff_active = true;

  const auto decision =
    savo_mapping::
    exploration_runtime::
    evaluate(inputs);

  EXPECT_EQ(
    decision.disposition,
    RuntimeDisposition::Enabled);

  EXPECT_TRUE(
    decision.frontier_enabled);

  EXPECT_FALSE(
    decision.cancel_active_goal);
}
