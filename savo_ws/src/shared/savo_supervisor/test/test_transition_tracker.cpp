// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "savo_supervisor/reason_codes.hpp"
#include "savo_supervisor/transition_tracker.hpp"

namespace
{

using savo_supervisor::AggregateHealth;
using savo_supervisor::ComponentState;
using savo_supervisor::Lifecycle;
using savo_supervisor::SupervisorEventType;
using savo_supervisor::SupervisorState;
using savo_supervisor::TransitionTracker;

rclcpp::Time test_time(double seconds)
{
  return rclcpp::Time(
    static_cast<int64_t>(seconds * 1e9),
    RCL_STEADY_TIME);
}

SupervisorState make_state(
  ComponentState component_state,
  Lifecycle lifecycle,
  AggregateHealth health,
  bool ready,
  const std::string & reason_code)
{
  SupervisorState state;
  state.lifecycle = lifecycle;
  state.health = health;
  state.ready = ready;
  state.reason_code = reason_code;

  savo_supervisor::ComponentSummary component;
  component.name = "localization";
  component.enabled = true;
  component.required = true;
  component.state = component_state;
  component.ready = ready;
  component.reason_code = reason_code;

  state.component_summaries.push_back(component);

  return state;
}

}  // namespace

TEST(TransitionTracker, InitialStatePublishesStarted)
{
  TransitionTracker tracker;

  const auto event = tracker.Observe(
    make_state(
      ComponentState::INITIALIZING,
      Lifecycle::STARTING,
      AggregateHealth::UNKNOWN,
      false,
      savo_supervisor::reason::kSupervisorStarting));

  ASSERT_TRUE(event.has_value());

  EXPECT_EQ(
    event->type,
    SupervisorEventType::SUPERVISOR_STARTED);

  EXPECT_FALSE(event->has_previous);
}

TEST(TransitionTracker, IdenticalStateIsSuppressed)
{
  TransitionTracker tracker;

  const auto state = make_state(
    ComponentState::INITIALIZING,
    Lifecycle::STARTING,
    AggregateHealth::UNKNOWN,
    false,
    savo_supervisor::reason::kSupervisorStarting);

  ASSERT_TRUE(tracker.Observe(state).has_value());
  EXPECT_FALSE(tracker.Observe(state).has_value());
}

TEST(TransitionTracker, DetectsStartupCompletion)
{
  TransitionTracker tracker;

  auto starting = make_state(
    ComponentState::OK,
    Lifecycle::STARTING,
    AggregateHealth::UNKNOWN,
    false,
    savo_supervisor::reason::kSupervisorStarting);

  auto running = starting;
  running.lifecycle = Lifecycle::RUNNING;
  running.health = AggregateHealth::OK;
  running.ready = true;
  running.reason_code =
    savo_supervisor::reason::kSupervisorOperational;

  running.component_summaries[0].ready = true;

  ASSERT_TRUE(tracker.Observe(starting).has_value());

  const auto event = tracker.Observe(running);

  ASSERT_TRUE(event.has_value());

  EXPECT_EQ(
    event->type,
    SupervisorEventType::STARTUP_COMPLETED);
}

TEST(TransitionTracker, DetectsComponentStale)
{
  TransitionTracker tracker;

  const auto healthy = make_state(
    ComponentState::OK,
    Lifecycle::RUNNING,
    AggregateHealth::OK,
    true,
    savo_supervisor::reason::kSupervisorOperational);

  const auto stale = make_state(
    ComponentState::STALE,
    Lifecycle::FAULTED,
    AggregateHealth::ERROR,
    false,
    savo_supervisor::reason::kLocalizationHealthStale);

  ASSERT_TRUE(tracker.Observe(healthy).has_value());

  const auto event = tracker.Observe(stale);

  ASSERT_TRUE(event.has_value());

  EXPECT_EQ(
    event->type,
    SupervisorEventType::COMPONENT_STALE);

  EXPECT_TRUE(event->has_component);
  EXPECT_EQ(event->component_name, "localization");
}

TEST(TransitionTracker, DetectsComponentInvalid)
{
  TransitionTracker tracker;

  const auto healthy = make_state(
    ComponentState::OK,
    Lifecycle::RUNNING,
    AggregateHealth::OK,
    true,
    savo_supervisor::reason::kSupervisorOperational);

  auto invalid = make_state(
    ComponentState::INVALID,
    Lifecycle::FAULTED,
    AggregateHealth::ERROR,
    false,
    savo_supervisor::reason::kLocalizationMessageInvalid);

  invalid.component_summaries[0].
  malformed_message_count = 1;

  ASSERT_TRUE(tracker.Observe(healthy).has_value());

  const auto event = tracker.Observe(invalid);

  ASSERT_TRUE(event.has_value());

  EXPECT_EQ(
    event->type,
    SupervisorEventType::COMPONENT_INVALID);
}

TEST(TransitionTracker, DetectsComponentError)
{
  TransitionTracker tracker;

  const auto healthy = make_state(
    ComponentState::OK,
    Lifecycle::RUNNING,
    AggregateHealth::OK,
    true,
    savo_supervisor::reason::kSupervisorOperational);

  const auto failed = make_state(
    ComponentState::ERROR,
    Lifecycle::FAULTED,
    AggregateHealth::ERROR,
    false,
    savo_supervisor::reason::kLocalizationError);

  ASSERT_TRUE(tracker.Observe(healthy).has_value());

  const auto event = tracker.Observe(failed);

  ASSERT_TRUE(event.has_value());

  EXPECT_EQ(
    event->type,
    SupervisorEventType::COMPONENT_ERROR);
}

TEST(TransitionTracker, DetectsRecovery)
{
  TransitionTracker tracker;

  const auto stale = make_state(
    ComponentState::STALE,
    Lifecycle::FAULTED,
    AggregateHealth::ERROR,
    false,
    savo_supervisor::reason::kLocalizationHealthStale);

  auto recovered = make_state(
    ComponentState::OK,
    Lifecycle::RUNNING,
    AggregateHealth::OK,
    true,
    savo_supervisor::reason::kSupervisorOperational);

  recovered.component_summaries[0].
  recovery_count = 1;

  ASSERT_TRUE(tracker.Observe(stale).has_value());

  const auto event = tracker.Observe(recovered);

  ASSERT_TRUE(event.has_value());

  EXPECT_EQ(
    event->type,
    SupervisorEventType::COMPONENT_RECOVERED);
}

TEST(TransitionTracker, DetectsMalformedCounterIncrease)
{
  TransitionTracker tracker;

  auto previous = make_state(
    ComponentState::OK,
    Lifecycle::RUNNING,
    AggregateHealth::OK,
    true,
    savo_supervisor::reason::kSupervisorOperational);

  auto current = previous;

  current.component_summaries[0].
  malformed_message_count = 1;

  ASSERT_TRUE(tracker.Observe(previous).has_value());

  const auto event = tracker.Observe(current);

  ASSERT_TRUE(event.has_value());

  EXPECT_EQ(
    event->type,
    SupervisorEventType::COMPONENT_INVALID);
}

TEST(
  TransitionTracker,
  ReasonChangeWithinSameFaultIsNotRepeatedStale)
{
  TransitionTracker tracker;

  const auto health_missing = make_state(
    ComponentState::STALE,
    Lifecycle::FAULTED,
    AggregateHealth::ERROR,
    false,
    savo_supervisor::reason::kLocalizationHealthMissing);

  const auto heartbeat_missing = make_state(
    ComponentState::STALE,
    Lifecycle::FAULTED,
    AggregateHealth::ERROR,
    false,
    savo_supervisor::reason::kLocalizationHeartbeatMissing);

  ASSERT_TRUE(
    tracker.Observe(health_missing).has_value());

  const auto event =
    tracker.Observe(heartbeat_missing);

  ASSERT_TRUE(event.has_value());

  EXPECT_EQ(
    event->type,
    SupervisorEventType::REASON_CHANGED);

  EXPECT_FALSE(event->has_component);
}

TEST(TransitionTracker, EventJsonContainsOldAndNewState)
{
  TransitionTracker tracker;

  const auto healthy = make_state(
    ComponentState::OK,
    Lifecycle::RUNNING,
    AggregateHealth::OK,
    true,
    savo_supervisor::reason::kSupervisorOperational);

  const auto stale = make_state(
    ComponentState::STALE,
    Lifecycle::FAULTED,
    AggregateHealth::ERROR,
    false,
    savo_supervisor::reason::kLocalizationHealthStale);

  ASSERT_TRUE(tracker.Observe(healthy).has_value());

  const auto event = tracker.Observe(stale);
  ASSERT_TRUE(event.has_value());

  const auto json =
    savo_supervisor::CompactTransitionJson(
      event.value(),
      test_time(5.0));

  EXPECT_NE(
    json.find("\"event_type\":\"component_stale\""),
    std::string::npos);

  EXPECT_NE(
    json.find("\"previous\""),
    std::string::npos);

  EXPECT_NE(
    json.find("\"current\""),
    std::string::npos);

  EXPECT_NE(
    json.find("\"component\""),
    std::string::npos);
}

TEST(TransitionTracker, ResetAllowsNewStartedEvent)
{
  TransitionTracker tracker;

  const auto state = make_state(
    ComponentState::OK,
    Lifecycle::RUNNING,
    AggregateHealth::OK,
    true,
    savo_supervisor::reason::kSupervisorOperational);

  ASSERT_TRUE(tracker.Observe(state).has_value());
  ASSERT_FALSE(tracker.Observe(state).has_value());

  tracker.Reset();

  const auto event = tracker.Observe(state);

  ASSERT_TRUE(event.has_value());

  EXPECT_EQ(
    event->type,
    SupervisorEventType::SUPERVISOR_STARTED);
}
