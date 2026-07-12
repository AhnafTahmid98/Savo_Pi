#include "savo_mapping/manual_workflow_state.hpp"

#include <gtest/gtest.h>

#include <string_view>

namespace
{

using savo_mapping::workflow::
  ManualWorkflowState;

using savo_mapping::workflow::
  SlamLifecycleObservation;

SlamLifecycleObservation active_observation()
{
  SlamLifecycleObservation observation;

  observation.service_available = true;
  observation.response_received = true;
  observation.response_fresh = true;
  observation.startup_grace_expired = true;
  observation.ever_active = true;

  observation.state_id =
    savo_mapping::workflow::
    lifecycle_state_id::ACTIVE;

  return observation;
}

}  // namespace

TEST(ManualWorkflowState, ConvertsStatesToStrings)
{
  using savo_mapping::workflow::to_string;

  EXPECT_EQ(
    to_string(ManualWorkflowState::STARTING),
    "starting");

  EXPECT_EQ(
    to_string(
      ManualWorkflowState::WAITING_FOR_INPUTS),
    "waiting_for_inputs");

  EXPECT_EQ(
    to_string(ManualWorkflowState::MAPPING),
    "mapping");

  EXPECT_EQ(
    to_string(ManualWorkflowState::PAUSED),
    "paused");

  EXPECT_EQ(
    to_string(ManualWorkflowState::ERROR),
    "error");

  EXPECT_EQ(
    to_string(ManualWorkflowState::COMPLETED),
    "completed");
}

TEST(ManualWorkflowState, StartsBeforeLifecycleResponse)
{
  SlamLifecycleObservation observation;

  EXPECT_EQ(
    savo_mapping::workflow::
    evaluate_manual_workflow_state(
      observation,
      false,
      "idle"),
    ManualWorkflowState::STARTING);
}

TEST(ManualWorkflowState, WaitsForMappingInputs)
{
  const auto observation = active_observation();

  EXPECT_EQ(
    savo_mapping::workflow::
    evaluate_manual_workflow_state(
      observation,
      false,
      "idle"),
    ManualWorkflowState::WAITING_FOR_INPUTS);
}

TEST(ManualWorkflowState, MapsWhenActiveAndReady)
{
  const auto observation = active_observation();

  EXPECT_EQ(
    savo_mapping::workflow::
    evaluate_manual_workflow_state(
      observation,
      true,
      "idle"),
    ManualWorkflowState::MAPPING);
}

TEST(ManualWorkflowState, PausesAfterDeactivation)
{
  auto observation = active_observation();

  observation.state_id =
    savo_mapping::workflow::
    lifecycle_state_id::INACTIVE;

  EXPECT_EQ(
    savo_mapping::workflow::
    evaluate_manual_workflow_state(
      observation,
      true,
      "idle"),
    ManualWorkflowState::PAUSED);
}

TEST(ManualWorkflowState, SessionPauseOverridesLifecycle)
{
  const auto observation = active_observation();

  EXPECT_EQ(
    savo_mapping::workflow::
    evaluate_manual_workflow_state(
      observation,
      true,
      "paused"),
    ManualWorkflowState::PAUSED);
}

TEST(ManualWorkflowState, SessionCompletionIsFinal)
{
  auto observation = active_observation();

  observation.response_fresh = false;

  EXPECT_EQ(
    savo_mapping::workflow::
    evaluate_manual_workflow_state(
      observation,
      false,
      "completed"),
    ManualWorkflowState::COMPLETED);
}

TEST(ManualWorkflowState, RejectsStaleLifecycleResponse)
{
  auto observation = active_observation();

  observation.response_fresh = false;

  EXPECT_EQ(
    savo_mapping::workflow::
    evaluate_manual_workflow_state(
      observation,
      true,
      "idle"),
    ManualWorkflowState::ERROR);
}

TEST(ManualWorkflowState, RejectsMissingResponseAfterGrace)
{
  SlamLifecycleObservation observation;

  observation.startup_grace_expired = true;

  EXPECT_EQ(
    savo_mapping::workflow::
    evaluate_manual_workflow_state(
      observation,
      false,
      "idle"),
    ManualWorkflowState::ERROR);
}

TEST(ManualWorkflowState, RejectsLifecycleErrorProcessing)
{
  auto observation = active_observation();

  observation.state_id =
    savo_mapping::workflow::
    lifecycle_state_id::ERROR_PROCESSING;

  EXPECT_EQ(
    savo_mapping::workflow::
    evaluate_manual_workflow_state(
      observation,
      false,
      "idle"),
    ManualWorkflowState::ERROR);
}

TEST(ManualWorkflowState, RejectsFinalizedLifecycle)
{
  auto observation = active_observation();

  observation.state_id =
    savo_mapping::workflow::
    lifecycle_state_id::FINALIZED;

  EXPECT_EQ(
    savo_mapping::workflow::
    evaluate_manual_workflow_state(
      observation,
      false,
      "idle"),
    ManualWorkflowState::ERROR);
}
