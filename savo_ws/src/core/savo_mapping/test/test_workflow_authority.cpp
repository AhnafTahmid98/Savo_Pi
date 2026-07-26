#include "savo_mapping/workflow_authority.hpp"

#include <gtest/gtest.h>

#include <string>

namespace
{

using savo_mapping::ExplorationMode;
using savo_mapping::MappingMode;
using savo_mapping::SessionState;
using savo_mapping::WorkflowPhase;
using savo_mapping::workflow::AuthorityDisposition;
using savo_mapping::workflow::ModeChangeRequest;
using savo_mapping::workflow::WorkflowAuthorityInputs;
using savo_mapping::workflow::WorkflowAuthorityState;

WorkflowAuthorityState active_monitor_state()
{
  WorkflowAuthorityState state;
  state.session_state = SessionState::Active;
  return state;
}

WorkflowAuthorityState active_frontier_state()
{
  WorkflowAuthorityState state;
  state.mode = MappingMode::Frontier;
  state.exploration_mode = ExplorationMode::Frontier;
  state.workflow_phase = WorkflowPhase::Exploring;
  state.session_state = SessionState::Active;
  return state;
}

WorkflowAuthorityInputs ready_inputs()
{
  WorkflowAuthorityInputs inputs;
  inputs.mapping_ready = true;
  inputs.navigation_handoff_ready = true;
  return inputs;
}

}  // namespace

TEST(
  WorkflowAuthorityContract,
  DefaultStateAndDispositionStringsAreStable)
{
  const auto state =
    savo_mapping::workflow::
    make_default_workflow_authority_state();

  EXPECT_EQ(
    state.mode,
    MappingMode::MonitorOnly);

  EXPECT_EQ(
    state.exploration_mode,
    ExplorationMode::Idle);

  EXPECT_EQ(
    state.workflow_phase,
    WorkflowPhase::Idle);

  EXPECT_EQ(
    state.session_state,
    SessionState::Idle);

  EXPECT_TRUE(
    savo_mapping::workflow::
    is_workflow_authority_state_consistent(
      state));

  EXPECT_EQ(
    std::string{
    savo_mapping::workflow::to_string(
        AuthorityDisposition::Accepted)},
    "accepted");

  EXPECT_EQ(
    std::string{
    savo_mapping::workflow::to_string(
        AuthorityDisposition::Rejected)},
    "rejected");

  EXPECT_EQ(
    std::string{
    savo_mapping::workflow::to_string(
        AuthorityDisposition::CancelRequired)},
    "cancel_required");

  EXPECT_EQ(
    std::string{
    savo_mapping::workflow::to_string(
        AuthorityDisposition::SafetyHold)},
    "safety_hold");
}

TEST(
  WorkflowAuthorityContract,
  ModesResolveStableStrategiesAndPhases)
{
  const ModeChangeRequest manual{
    MappingMode::Manual,
    ExplorationMode::Coverage
  };

  const ModeChangeRequest frontier{
    MappingMode::Frontier,
    ExplorationMode::Idle
  };

  const ModeChangeRequest scan360{
    MappingMode::Scan360,
    ExplorationMode::Idle
  };

  const ModeChangeRequest coverage{
    MappingMode::Coverage,
    ExplorationMode::Idle
  };

  const ModeChangeRequest semantic{
    MappingMode::SemanticReview,
    ExplorationMode::Idle
  };

  EXPECT_EQ(
    savo_mapping::workflow::
    resolve_exploration_mode(manual),
    ExplorationMode::Idle);

  EXPECT_EQ(
    savo_mapping::workflow::
    resolve_exploration_mode(frontier),
    ExplorationMode::Frontier);

  EXPECT_EQ(
    savo_mapping::workflow::
    resolve_exploration_mode(scan360),
    ExplorationMode::Scan360);

  EXPECT_EQ(
    savo_mapping::workflow::
    resolve_exploration_mode(coverage),
    ExplorationMode::Coverage);

  EXPECT_EQ(
    savo_mapping::workflow::
    resolve_exploration_mode(semantic),
    ExplorationMode::SemanticReview);

  EXPECT_EQ(
    savo_mapping::workflow::
    workflow_phase_for(
      MappingMode::Manual,
      ExplorationMode::Idle),
    WorkflowPhase::Mapping);

  EXPECT_EQ(
    savo_mapping::workflow::
    workflow_phase_for(
      MappingMode::Frontier,
      ExplorationMode::Frontier),
    WorkflowPhase::Exploring);

  EXPECT_EQ(
    savo_mapping::workflow::
    workflow_phase_for(
      MappingMode::Scan360,
      ExplorationMode::Scan360),
    WorkflowPhase::Scan360);

  EXPECT_EQ(
    savo_mapping::workflow::
    workflow_phase_for(
      MappingMode::Coverage,
      ExplorationMode::Coverage),
    WorkflowPhase::Coverage);

  EXPECT_EQ(
    savo_mapping::workflow::
    workflow_phase_for(
      MappingMode::SemanticReview,
      ExplorationMode::SemanticReview),
    WorkflowPhase::SemanticReview);
}

TEST(
  WorkflowAuthorityContract,
  AutonomousRequiresSupportedStrategy)
{
  EXPECT_FALSE(
    savo_mapping::workflow::
    resolve_exploration_mode(
      ModeChangeRequest{
    MappingMode::Autonomous,
    ExplorationMode::Idle})
    .has_value());

  EXPECT_FALSE(
    savo_mapping::workflow::
    resolve_exploration_mode(
      ModeChangeRequest{
    MappingMode::Autonomous,
    ExplorationMode::SemanticReview})
    .has_value());

  EXPECT_EQ(
    savo_mapping::workflow::
    resolve_exploration_mode(
      ModeChangeRequest{
    MappingMode::Autonomous,
    ExplorationMode::Frontier}),
    ExplorationMode::Frontier);

  EXPECT_EQ(
    savo_mapping::workflow::
    resolve_exploration_mode(
      ModeChangeRequest{
    MappingMode::Autonomous,
    ExplorationMode::Scan360}),
    ExplorationMode::Scan360);

  EXPECT_EQ(
    savo_mapping::workflow::
    resolve_exploration_mode(
      ModeChangeRequest{
    MappingMode::Autonomous,
    ExplorationMode::Coverage}),
    ExplorationMode::Coverage);
}

TEST(
  WorkflowAuthorityContract,
  RejectsInconsistentCurrentState)
{
  auto current = active_monitor_state();
  current.exploration_mode =
    ExplorationMode::Frontier;

  const auto decision =
    savo_mapping::workflow::
    evaluate_mode_change(
      current,
      ready_inputs(),
      ModeChangeRequest{
    MappingMode::MonitorOnly,
    ExplorationMode::Idle});

  EXPECT_EQ(
    decision.disposition,
    AuthorityDisposition::Rejected);

  EXPECT_EQ(
    decision.reason,
    "rejected: inconsistent_current_state");

  EXPECT_FALSE(
    decision.cancel_active_exploration);

  EXPECT_FALSE(
    decision.movement_authorized);

  EXPECT_FALSE(
    decision.frontier_enabled);
}

TEST(
  WorkflowAuthorityContract,
  MonitorAndManualRespectSessionPolicy)
{
  auto current =
    savo_mapping::workflow::
    make_default_workflow_authority_state();

  auto decision =
    savo_mapping::workflow::
    evaluate_mode_change(
      current,
      WorkflowAuthorityInputs{},
      ModeChangeRequest{
    MappingMode::MonitorOnly,
    ExplorationMode::Coverage});

  EXPECT_EQ(
    decision.disposition,
    AuthorityDisposition::Accepted);

  EXPECT_EQ(
    decision.reason,
    "accepted: no_change");

  decision =
    savo_mapping::workflow::
    evaluate_mode_change(
      current,
      WorkflowAuthorityInputs{},
      ModeChangeRequest{
    MappingMode::Manual,
    ExplorationMode::Idle});

  EXPECT_EQ(
    decision.disposition,
    AuthorityDisposition::Rejected);

  EXPECT_EQ(
    decision.reason,
    "rejected: session_not_active");

  current.session_state =
    SessionState::Active;

  decision =
    savo_mapping::workflow::
    evaluate_mode_change(
      current,
      WorkflowAuthorityInputs{},
      ModeChangeRequest{
    MappingMode::Manual,
    ExplorationMode::Coverage});

  EXPECT_EQ(
    decision.disposition,
    AuthorityDisposition::Accepted);

  EXPECT_EQ(
    decision.next_state.mode,
    MappingMode::Manual);

  EXPECT_EQ(
    decision.next_state.exploration_mode,
    ExplorationMode::Idle);

  EXPECT_EQ(
    decision.next_state.workflow_phase,
    WorkflowPhase::Mapping);

  EXPECT_FALSE(
    decision.movement_authorized);
}

TEST(
  WorkflowAuthorityContract,
  FrontierRequiresMapAndNavigationReadiness)
{
  const auto current =
    active_monitor_state();

  WorkflowAuthorityInputs inputs;

  auto decision =
    savo_mapping::workflow::
    evaluate_mode_change(
      current,
      inputs,
      ModeChangeRequest{
    MappingMode::Frontier,
    ExplorationMode::Idle});

  EXPECT_EQ(
    decision.reason,
    "rejected: mapping_not_ready");

  inputs.mapping_ready = true;

  decision =
    savo_mapping::workflow::
    evaluate_mode_change(
      current,
      inputs,
      ModeChangeRequest{
    MappingMode::Frontier,
    ExplorationMode::Idle});

  EXPECT_EQ(
    decision.reason,
    "rejected: navigation_handoff_not_ready");

  inputs.navigation_handoff_ready = true;

  decision =
    savo_mapping::workflow::
    evaluate_mode_change(
      current,
      inputs,
      ModeChangeRequest{
    MappingMode::Frontier,
    ExplorationMode::Idle});

  EXPECT_EQ(
    decision.disposition,
    AuthorityDisposition::Accepted);

  EXPECT_EQ(
    decision.next_state.workflow_phase,
    WorkflowPhase::Exploring);

  EXPECT_TRUE(
    decision.movement_authorized);

  EXPECT_TRUE(
    decision.frontier_enabled);

  EXPECT_TRUE(
    savo_mapping::workflow::
    is_frontier_runtime_enabled(
      decision));
}

TEST(
  WorkflowAuthorityContract,
  Scan360AndSemanticDoNotNeedNavigationHandoff)
{
  const auto current =
    active_monitor_state();

  WorkflowAuthorityInputs inputs;
  inputs.mapping_ready = true;

  const auto scan360 =
    savo_mapping::workflow::
    evaluate_mode_change(
      current,
      inputs,
      ModeChangeRequest{
    MappingMode::Scan360,
    ExplorationMode::Idle});

  EXPECT_EQ(
    scan360.disposition,
    AuthorityDisposition::Accepted);

  EXPECT_EQ(
    scan360.next_state.workflow_phase,
    WorkflowPhase::Scan360);

  EXPECT_TRUE(
    scan360.movement_authorized);

  EXPECT_FALSE(
    scan360.frontier_enabled);

  const auto semantic =
    savo_mapping::workflow::
    evaluate_mode_change(
      current,
      inputs,
      ModeChangeRequest{
    MappingMode::SemanticReview,
    ExplorationMode::Idle});

  EXPECT_EQ(
    semantic.disposition,
    AuthorityDisposition::Accepted);

  EXPECT_EQ(
    semantic.next_state.workflow_phase,
    WorkflowPhase::SemanticReview);

  EXPECT_FALSE(
    semantic.movement_authorized);
}

TEST(
  WorkflowAuthorityContract,
  AutonomousUsesRequestedFrontierStrategy)
{
  const auto decision =
    savo_mapping::workflow::
    evaluate_mode_change(
      active_monitor_state(),
      ready_inputs(),
      ModeChangeRequest{
    MappingMode::Autonomous,
    ExplorationMode::Frontier});

  EXPECT_EQ(
    decision.disposition,
    AuthorityDisposition::Accepted);

  EXPECT_EQ(
    decision.next_state.mode,
    MappingMode::Autonomous);

  EXPECT_EQ(
    decision.next_state.exploration_mode,
    ExplorationMode::Frontier);

  EXPECT_EQ(
    decision.next_state.workflow_phase,
    WorkflowPhase::Exploring);

  EXPECT_TRUE(
    decision.frontier_enabled);
}

TEST(
  WorkflowAuthorityContract,
  SafetyStopBlocksAndCancelsMovement)
{
  auto inputs = ready_inputs();
  inputs.safety_stop_active = true;

  auto decision =
    savo_mapping::workflow::
    evaluate_mode_change(
      active_monitor_state(),
      inputs,
      ModeChangeRequest{
    MappingMode::Frontier,
    ExplorationMode::Idle});

  EXPECT_EQ(
    decision.disposition,
    AuthorityDisposition::SafetyHold);

  EXPECT_EQ(
    decision.reason,
    "safety_hold: safety_stop_active");

  EXPECT_FALSE(
    decision.cancel_active_exploration);

  EXPECT_FALSE(
    decision.movement_authorized);

  EXPECT_FALSE(
    decision.frontier_enabled);

  inputs.exploration_goal_active = true;

  decision =
    savo_mapping::workflow::
    evaluate_mode_change(
      active_frontier_state(),
      inputs,
      ModeChangeRequest{
    MappingMode::Frontier,
    ExplorationMode::Idle});

  EXPECT_EQ(
    decision.disposition,
    AuthorityDisposition::SafetyHold);

  EXPECT_TRUE(
    decision.cancel_active_exploration);

  EXPECT_FALSE(
    decision.frontier_enabled);
}

TEST(
  WorkflowAuthorityContract,
  ConflictingModeRequiresGoalCancellationFirst)
{
  auto inputs = ready_inputs();
  inputs.exploration_goal_active = true;

  const auto decision =
    savo_mapping::workflow::
    evaluate_mode_change(
      active_frontier_state(),
      inputs,
      ModeChangeRequest{
    MappingMode::MonitorOnly,
    ExplorationMode::Idle});

  EXPECT_EQ(
    decision.disposition,
    AuthorityDisposition::CancelRequired);

  EXPECT_EQ(
    decision.reason,
    "cancel_required: active_navigation_goal");

  EXPECT_TRUE(
    decision.cancel_active_exploration);

  EXPECT_EQ(
    decision.next_state.mode,
    MappingMode::Frontier);

  EXPECT_FALSE(
    decision.frontier_enabled);
}

TEST(
  WorkflowAuthorityContract,
  SameActiveModeIsIdempotent)
{
  auto inputs = ready_inputs();
  inputs.exploration_goal_active = true;

  const auto decision =
    savo_mapping::workflow::
    evaluate_mode_change(
      active_frontier_state(),
      inputs,
      ModeChangeRequest{
    MappingMode::Frontier,
    ExplorationMode::Idle});

  EXPECT_EQ(
    decision.disposition,
    AuthorityDisposition::Accepted);

  EXPECT_EQ(
    decision.reason,
    "accepted: no_change");

  EXPECT_FALSE(
    decision.cancel_active_exploration);

  EXPECT_TRUE(
    decision.movement_authorized);

  EXPECT_TRUE(
    decision.frontier_enabled);
}

TEST(
  WorkflowAuthorityContract,
  SessionOrReadinessLossCancelsActiveGoal)
{
  auto current =
    active_frontier_state();

  current.session_state =
    SessionState::Paused;

  auto inputs = ready_inputs();
  inputs.exploration_goal_active = true;

  auto decision =
    savo_mapping::workflow::
    evaluate_mode_change(
      current,
      inputs,
      ModeChangeRequest{
    MappingMode::Frontier,
    ExplorationMode::Idle});

  EXPECT_EQ(
    decision.disposition,
    AuthorityDisposition::CancelRequired);

  EXPECT_EQ(
    decision.reason,
    "cancel_required: session_not_active");

  EXPECT_TRUE(
    decision.cancel_active_exploration);

  current.session_state =
    SessionState::Active;

  inputs.mapping_ready = false;

  decision =
    savo_mapping::workflow::
    evaluate_mode_change(
      current,
      inputs,
      ModeChangeRequest{
    MappingMode::Frontier,
    ExplorationMode::Idle});

  EXPECT_EQ(
    decision.disposition,
    AuthorityDisposition::CancelRequired);

  EXPECT_EQ(
    decision.reason,
    "cancel_required: mapping_not_ready");

  EXPECT_TRUE(
    decision.cancel_active_exploration);
}
