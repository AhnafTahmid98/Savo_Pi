// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "savo_supervisor/reason_codes.hpp"
#include "savo_supervisor/supervisor_policy.hpp"

namespace
{

using savo_supervisor::AggregateHealth;
using savo_supervisor::ComponentState;
using savo_supervisor::ComponentStatus;
using savo_supervisor::Lifecycle;
using savo_supervisor::SupervisorPolicy;

rclcpp::Time test_time(double seconds)
{
  return rclcpp::Time(
    static_cast<int64_t>(seconds * 1e9),
    RCL_STEADY_TIME);
}

builtin_interfaces::msg::Time message_stamp(
  int32_t seconds)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = seconds;
  stamp.nanosec = 0;
  return stamp;
}

ComponentStatus make_status(
  bool required = true)
{
  ComponentStatus status;
  status.config =
    SupervisorPolicy::DefaultLocalizationConfig();

  status.config.required = required;

  return status;
}

void observe_all(
  ComponentStatus & status,
  double receive_seconds)
{
  const auto receive_time =
    test_time(receive_seconds);

  const auto stamp =
    std::optional<builtin_interfaces::msg::Time>{
    message_stamp(
      static_cast<int32_t>(receive_seconds))};

  status.health_tracker.observe_message(
    receive_time,
    stamp,
    false,
    "");

  status.summary_tracker.observe_message(
    receive_time,
    stamp,
    false,
    "");

  status.heartbeat_tracker.observe_message(
    receive_time,
    stamp,
    false,
    "");
}

void set_consistent_state(
  ComponentStatus & status,
  const std::string & state,
  bool ready,
  bool degraded)
{
  status.health_valid = true;
  status.summary_valid = true;
  status.heartbeat_valid = true;

  status.health_state = state;
  status.health_ready = ready;
  status.health_degraded = degraded;
  status.health_reason_code =
    state == "OK" ?
    savo_supervisor::reason::kLocalizationOperational :
    state == "DEGRADED" ?
    savo_supervisor::reason::kLocalizationDegraded :
    savo_supervisor::reason::kLocalizationNotReady;

  status.summary_state = state;
  status.summary_ready = ready;
  status.summary_degraded = degraded;
  status.summary_reason_code =
    status.health_reason_code;

  status.heartbeat_state = state;
  status.heartbeat_alive = true;
  status.heartbeat_ready = ready;
}

}  // namespace

TEST(SupervisorPolicy, DefaultLocalizationConfig)
{
  const auto config =
    SupervisorPolicy::DefaultLocalizationConfig();

  EXPECT_EQ(config.name, "localization");
  EXPECT_TRUE(config.enabled);
  EXPECT_TRUE(config.required);

  EXPECT_EQ(
    config.health_topic,
    "/savo_localization/health");

  EXPECT_EQ(
    config.summary_topic,
    "/savo_localization/state_summary");

  EXPECT_EQ(
    config.heartbeat_topic,
    "/savo_localization/heartbeat");

  EXPECT_DOUBLE_EQ(config.health_timeout_s, 1.5);
  EXPECT_DOUBLE_EQ(config.summary_timeout_s, 1.5);
  EXPECT_DOUBLE_EQ(config.heartbeat_timeout_s, 2.5);
  EXPECT_DOUBLE_EQ(config.consistency_transition_grace_s, 1.5);
  EXPECT_EQ(config.expected_schema_version, 1);
}

TEST(SupervisorPolicy, MissingDuringStartupIsInitializing)
{
  SupervisorPolicy policy;
  auto status = make_status();

  const auto result =
    policy.EvaluateComponent(
      status,
      test_time(1.0),
      1.0);

  EXPECT_EQ(result.state, ComponentState::INITIALIZING);
  EXPECT_FALSE(result.ready);

  EXPECT_EQ(
    result.reason_code,
    savo_supervisor::reason::kLocalizationInitializing);
}

TEST(SupervisorPolicy, MissingHealthAfterGraceIsStale)
{
  SupervisorPolicy policy;
  auto status = make_status();

  const auto result =
    policy.EvaluateComponent(
      status,
      test_time(5.0),
      5.0);

  EXPECT_EQ(result.state, ComponentState::STALE);
  EXPECT_FALSE(result.ready);

  EXPECT_EQ(
    result.reason_code,
    savo_supervisor::reason::kLocalizationHealthMissing);
}

TEST(SupervisorPolicy, MissingSummaryHasDistinctReason)
{
  SupervisorPolicy policy;
  auto status = make_status();

  const auto now = test_time(5.0);
  const auto stamp =
    std::optional<builtin_interfaces::msg::Time>{
    message_stamp(5)};

  status.health_tracker.observe_message(
    now,
    stamp,
    false,
    "");
  status.health_valid = true;

  const auto result =
    policy.EvaluateComponent(status, now, 5.0);

  EXPECT_EQ(result.state, ComponentState::STALE);

  EXPECT_EQ(
    result.reason_code,
    savo_supervisor::reason::kLocalizationSummaryMissing);
}

TEST(SupervisorPolicy, MissingHeartbeatHasDistinctReason)
{
  SupervisorPolicy policy;
  auto status = make_status();

  const auto now = test_time(5.0);
  const auto stamp =
    std::optional<builtin_interfaces::msg::Time>{
    message_stamp(5)};

  status.health_tracker.observe_message(
    now,
    stamp,
    false,
    "");

  status.summary_tracker.observe_message(
    now,
    stamp,
    false,
    "");
  status.health_valid = true;
  status.summary_valid = true;

  const auto result =
    policy.EvaluateComponent(status, now, 5.0);

  EXPECT_EQ(result.state, ComponentState::STALE);

  EXPECT_EQ(
    result.reason_code,
    savo_supervisor::reason::kLocalizationHeartbeatMissing);
}

TEST(SupervisorPolicy, HealthyLocalizationIsReady)
{
  SupervisorPolicy policy;
  auto status = make_status();

  observe_all(status, 10.0);
  set_consistent_state(status, "OK", true, false);

  const auto result =
    policy.EvaluateComponent(
      status,
      test_time(10.1),
      10.1);

  EXPECT_EQ(result.state, ComponentState::OK);
  EXPECT_TRUE(result.ready);
  EXPECT_FALSE(result.degraded);
  EXPECT_TRUE(result.received);

  EXPECT_EQ(
    result.reason_code,
    savo_supervisor::reason::kLocalizationOperational);
}

TEST(SupervisorPolicy, DegradedLocalizationCanRemainReady)
{
  SupervisorPolicy policy;
  auto status = make_status();

  observe_all(status, 10.0);
  set_consistent_state(
    status,
    "DEGRADED",
    true,
    true);

  const auto result =
    policy.EvaluateComponent(
      status,
      test_time(10.1),
      10.1);

  EXPECT_EQ(result.state, ComponentState::DEGRADED);
  EXPECT_TRUE(result.ready);
  EXPECT_TRUE(result.degraded);
}

TEST(SupervisorPolicy, DegradedToOkArrivalSkewRemainsReady)
{
  SupervisorPolicy policy;
  auto status = make_status();

  observe_all(status, 10.0);
  set_consistent_state(status, "DEGRADED", true, true);

  status.health_tracker.observe_message(
    test_time(10.5), message_stamp(11), false, "");
  status.health_state = "OK";
  status.health_degraded = false;
  status.health_reason_code = savo_supervisor::reason::kLocalizationOperational;

  auto result = policy.EvaluateComponent(status, test_time(10.6), 10.6);
  EXPECT_EQ(result.state, ComponentState::DEGRADED);
  EXPECT_TRUE(result.ready);

  status.summary_tracker.observe_message(
    test_time(10.7), message_stamp(11), false, "");
  status.summary_state = "OK";
  status.summary_degraded = false;
  status.summary_reason_code = savo_supervisor::reason::kLocalizationOperational;

  result = policy.EvaluateComponent(status, test_time(10.8), 10.8);
  EXPECT_EQ(result.state, ComponentState::DEGRADED);
  EXPECT_TRUE(result.ready);

  status.heartbeat_tracker.observe_message(
    test_time(10.9), message_stamp(11), false, "");
  status.heartbeat_state = "OK";

  result = policy.EvaluateComponent(status, test_time(11.0), 11.0);
  EXPECT_EQ(result.state, ComponentState::OK);
  EXPECT_TRUE(result.ready);
  EXPECT_FALSE(result.degraded);
}

TEST(SupervisorPolicy, OkToDegradedArrivalSkewRemainsReady)
{
  SupervisorPolicy policy;
  auto status = make_status();

  observe_all(status, 20.0);
  set_consistent_state(status, "OK", true, false);

  status.health_tracker.observe_message(
    test_time(20.2), message_stamp(21), false, "");
  status.health_state = "DEGRADED";
  status.health_degraded = true;
  status.health_reason_code = savo_supervisor::reason::kLocalizationDegraded;

  auto result = policy.EvaluateComponent(status, test_time(20.3), 20.3);
  EXPECT_EQ(result.state, ComponentState::DEGRADED);
  EXPECT_TRUE(result.ready);

  status.summary_tracker.observe_message(
    test_time(20.4), message_stamp(21), false, "");
  status.summary_state = "DEGRADED";
  status.summary_degraded = true;
  status.summary_reason_code = savo_supervisor::reason::kLocalizationDegraded;

  status.heartbeat_tracker.observe_message(
    test_time(20.6), message_stamp(21), false, "");
  status.heartbeat_state = "DEGRADED";

  result = policy.EvaluateComponent(status, test_time(20.7), 20.7);
  EXPECT_EQ(result.state, ComponentState::DEGRADED);
  EXPECT_TRUE(result.ready);
  EXPECT_TRUE(result.degraded);
}

TEST(SupervisorPolicy, PersistentCompatibleStateMismatchFailsClosed)
{
  SupervisorPolicy policy;
  auto status = make_status();

  observe_all(status, 30.0);
  set_consistent_state(status, "OK", true, false);

  status.health_tracker.observe_message(
    test_time(30.1), message_stamp(31), false, "");
  status.health_state = "DEGRADED";
  status.health_degraded = true;
  status.health_reason_code = savo_supervisor::reason::kLocalizationDegraded;

  auto result = policy.EvaluateComponent(status, test_time(30.2), 30.2);
  ASSERT_EQ(result.state, ComponentState::DEGRADED);
  ASSERT_TRUE(result.ready);

  status.health_tracker.observe_message(
    test_time(31.6), message_stamp(32), false, "");
  status.summary_tracker.observe_message(
    test_time(31.6), message_stamp(32), false, "");
  status.heartbeat_tracker.observe_message(
    test_time(31.6), message_stamp(32), false, "");

  result = policy.EvaluateComponent(status, test_time(31.8), 31.8);
  EXPECT_EQ(result.state, ComponentState::INVALID);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(
    result.reason_code,
    savo_supervisor::reason::kLocalizationStateInconsistent);
}

TEST(SupervisorPolicy, ReadinessDisagreementIsNotTransitionCompatible)
{
  SupervisorPolicy policy;
  auto status = make_status();

  observe_all(status, 40.0);
  set_consistent_state(status, "OK", true, false);
  status.health_ready = false;

  const auto result = policy.EvaluateComponent(status, test_time(40.1), 40.1);
  EXPECT_EQ(result.state, ComponentState::INVALID);
  EXPECT_FALSE(result.ready);
  EXPECT_EQ(
    result.reason_code,
    savo_supervisor::reason::kLocalizationStateInconsistent);
}

TEST(SupervisorPolicy, StaleHealthHasDistinctReason)
{
  SupervisorPolicy policy;
  auto status = make_status();

  observe_all(status, 10.0);
  set_consistent_state(status, "OK", true, false);

  status.summary_tracker.observe_message(
    test_time(11.0),
    message_stamp(11),
    false,
    "");

  status.heartbeat_tracker.observe_message(
    test_time(11.0),
    message_stamp(11),
    false,
    "");

  const auto result =
    policy.EvaluateComponent(
      status,
      test_time(12.0),
      12.0);

  EXPECT_EQ(result.state, ComponentState::STALE);

  EXPECT_EQ(
    result.reason_code,
    savo_supervisor::reason::kLocalizationHealthStale);
}

TEST(SupervisorPolicy, StaleSummaryHasDistinctReason)
{
  SupervisorPolicy policy;
  auto status = make_status();

  observe_all(status, 10.0);
  set_consistent_state(status, "OK", true, false);

  status.health_tracker.observe_message(
    test_time(11.0),
    message_stamp(11),
    false,
    "");

  status.heartbeat_tracker.observe_message(
    test_time(11.0),
    message_stamp(11),
    false,
    "");

  const auto result =
    policy.EvaluateComponent(
      status,
      test_time(12.0),
      12.0);

  EXPECT_EQ(result.state, ComponentState::STALE);

  EXPECT_EQ(
    result.reason_code,
    savo_supervisor::reason::kLocalizationSummaryStale);
}

TEST(SupervisorPolicy, StaleHeartbeatHasDistinctReason)
{
  SupervisorPolicy policy;
  auto status = make_status();

  observe_all(status, 10.0);
  set_consistent_state(status, "OK", true, false);

  status.health_tracker.observe_message(
    test_time(12.0),
    message_stamp(12),
    false,
    "");

  status.summary_tracker.observe_message(
    test_time(12.0),
    message_stamp(12),
    false,
    "");

  const auto result =
    policy.EvaluateComponent(
      status,
      test_time(13.0),
      13.0);

  EXPECT_EQ(result.state, ComponentState::STALE);

  EXPECT_EQ(
    result.reason_code,
    savo_supervisor::reason::kLocalizationHeartbeatStale);
}

TEST(SupervisorPolicy, DeadHeartbeatIsError)
{
  SupervisorPolicy policy;
  auto status = make_status();

  observe_all(status, 10.0);
  set_consistent_state(status, "OK", true, false);

  status.heartbeat_alive = false;

  const auto result =
    policy.EvaluateComponent(
      status,
      test_time(10.1),
      10.1);

  EXPECT_EQ(result.state, ComponentState::ERROR);

  EXPECT_EQ(
    result.reason_code,
    savo_supervisor::reason::kLocalizationHeartbeatNotAlive);
}

TEST(SupervisorPolicy, InconsistentStateIsInvalid)
{
  SupervisorPolicy policy;
  auto status = make_status();

  observe_all(status, 10.0);
  set_consistent_state(status, "OK", true, false);

  status.health_state = "ERROR";
  status.health_ready = false;

  const auto result =
    policy.EvaluateComponent(
      status,
      test_time(10.1),
      10.1);

  EXPECT_EQ(result.state, ComponentState::INVALID);

  EXPECT_EQ(
    result.reason_code,
    savo_supervisor::reason::kLocalizationStateInconsistent);
}

TEST(SupervisorPolicy, UnsupportedSchemaIsDistinct)
{
  SupervisorPolicy policy;
  auto status = make_status();

  observe_all(status, 10.0);
  set_consistent_state(status, "OK", true, false);

  status.health_valid = false;
  status.health_reason_code =
    savo_supervisor::reason::kLocalizationSchemaUnsupported;

  status.health_tracker.observe_message(
    test_time(10.1),
    message_stamp(10),
    true,
    "unsupported schema");

  const auto result =
    policy.EvaluateComponent(
      status,
      test_time(10.2),
      10.2);

  EXPECT_EQ(result.state, ComponentState::INVALID);

  EXPECT_EQ(
    result.reason_code,
    savo_supervisor::reason::kLocalizationSchemaUnsupported);
}

TEST(SupervisorPolicy, MalformedCountersArePropagated)
{
  SupervisorPolicy policy;
  auto status = make_status();

  observe_all(status, 10.0);
  set_consistent_state(status, "OK", true, false);

  status.health_valid = false;

  status.health_tracker.observe_message(
    test_time(10.1),
    message_stamp(10),
    true,
    "bad JSON");

  auto result =
    policy.EvaluateComponent(
      status,
      test_time(10.2),
      10.2);

  ASSERT_EQ(result.state, ComponentState::INVALID);
  EXPECT_EQ(result.malformed_message_count, 1u);

  status.health_valid = true;

  status.health_tracker.observe_message(
    test_time(10.3),
    message_stamp(11),
    false,
    "");

  result =
    policy.EvaluateComponent(
      status,
      test_time(10.4),
      10.4);

  EXPECT_EQ(result.state, ComponentState::OK);
  EXPECT_EQ(result.malformed_message_count, 1u);
  EXPECT_EQ(result.recovery_count, 1u);
}

TEST(SupervisorPolicy, TimeRegressionIsInvalid)
{
  SupervisorPolicy policy;
  auto status = make_status();

  observe_all(status, 10.0);
  set_consistent_state(status, "OK", true, false);

  status.health_tracker.observe_message(
    test_time(9.0),
    message_stamp(11),
    false,
    "");

  const auto result =
    policy.EvaluateComponent(
      status,
      test_time(9.1),
      9.1);

  EXPECT_EQ(result.state, ComponentState::INVALID);

  EXPECT_EQ(
    result.reason_code,
    savo_supervisor::reason::kRosTimeRegressionDetected);
}

TEST(SupervisorPolicy, RequiredHealthyComponentRunsSupervisor)
{
  SupervisorPolicy policy;

  savo_supervisor::ComponentSummary component;
  component.name = "localization";
  component.enabled = true;
  component.required = true;
  component.state = ComponentState::OK;
  component.ready = true;

  const auto result =
    policy.EvaluateSupervisor(
      component,
      test_time(10.0),
      10.0);

  EXPECT_EQ(result.lifecycle, Lifecycle::RUNNING);
  EXPECT_EQ(result.health, AggregateHealth::OK);
  EXPECT_TRUE(result.ready);
  EXPECT_FALSE(result.degraded);
}

TEST(SupervisorPolicy, RequiredDegradedComponentCanRun)
{
  SupervisorPolicy policy;

  savo_supervisor::ComponentSummary component;
  component.name = "localization";
  component.enabled = true;
  component.required = true;
  component.state = ComponentState::DEGRADED;
  component.ready = true;
  component.degraded = true;
  component.reason_code =
    savo_supervisor::reason::kLocalizationDegraded;

  const auto result =
    policy.EvaluateSupervisor(
      component,
      test_time(10.0),
      10.0);

  EXPECT_EQ(result.lifecycle, Lifecycle::RUNNING);
  EXPECT_EQ(result.health, AggregateHealth::DEGRADED);
  EXPECT_TRUE(result.ready);
  EXPECT_TRUE(result.degraded);
}

TEST(SupervisorPolicy, RequiredStaleComponentFaultsAfterGrace)
{
  SupervisorPolicy policy;

  savo_supervisor::ComponentSummary component;
  component.name = "localization";
  component.enabled = true;
  component.required = true;
  component.state = ComponentState::STALE;
  component.ready = false;
  component.reason_code =
    savo_supervisor::reason::kLocalizationHealthStale;

  const auto result =
    policy.EvaluateSupervisor(
      component,
      test_time(10.0),
      10.0);

  EXPECT_EQ(result.lifecycle, Lifecycle::FAULTED);
  EXPECT_EQ(result.health, AggregateHealth::ERROR);
  EXPECT_FALSE(result.ready);
}

TEST(SupervisorPolicy, OptionalUnavailableComponentDoesNotBlock)
{
  SupervisorPolicy policy;

  savo_supervisor::ComponentSummary component;
  component.name = "localization";
  component.enabled = true;
  component.required = false;
  component.state = ComponentState::STALE;
  component.ready = false;
  component.reason_code =
    savo_supervisor::reason::kLocalizationHealthMissing;

  const auto result =
    policy.EvaluateSupervisor(
      component,
      test_time(10.0),
      10.0);

  EXPECT_EQ(result.lifecycle, Lifecycle::RUNNING);
  EXPECT_EQ(result.health, AggregateHealth::DEGRADED);
  EXPECT_TRUE(result.ready);
  EXPECT_TRUE(result.degraded);
}

TEST(SupervisorPolicy, DisabledOptionalComponentIsHealthy)
{
  SupervisorPolicy policy;

  savo_supervisor::ComponentSummary component;
  component.name = "localization";
  component.enabled = false;
  component.required = false;
  component.state = ComponentState::DISABLED;
  component.ready = true;

  const auto result =
    policy.EvaluateSupervisor(
      component,
      test_time(10.0),
      10.0);

  EXPECT_EQ(result.lifecycle, Lifecycle::RUNNING);
  EXPECT_EQ(result.health, AggregateHealth::OK);
  EXPECT_TRUE(result.ready);
}

TEST(SupervisorPolicy, StateJsonNeverWritesInfinity)
{
  SupervisorPolicy policy;

  savo_supervisor::SupervisorState state;
  state.reason_code =
    savo_supervisor::reason::kSupervisorStarting;

  savo_supervisor::ComponentSummary component;
  component.name = "localization";
  component.last_message_age_s = -1.0;
  component.timeout_s = 2.5;

  state.component_summaries.push_back(component);

  const auto json =
    policy.CompactStateJson(
      state,
      test_time(1.0));

  EXPECT_EQ(json.find("inf"), std::string::npos);
  EXPECT_EQ(json.find("nan"), std::string::npos);
  EXPECT_NE(
    json.find("\"last_message_age_s\":-1"),
    std::string::npos);
}

TEST(SupervisorPolicy, HonorsConfiguredStartupGrace)
{
  SupervisorPolicy policy;
  policy.startup_grace_s = 10.0;

  savo_supervisor::ComponentSummary localization;
  localization.name = "localization";
  localization.enabled = true;
  localization.required = true;
  localization.state = ComponentState::ERROR;
  localization.ready = false;
  localization.reason_code = "localization_health_missing";

  const rclcpp::Time now(4, 0, RCL_ROS_TIME);

  const auto before_grace =
    policy.EvaluateSupervisor(localization, now, 4.0);

  EXPECT_EQ(before_grace.lifecycle, Lifecycle::STARTING);
  EXPECT_EQ(before_grace.health, AggregateHealth::UNKNOWN);
  EXPECT_FALSE(before_grace.ready);
  EXPECT_EQ(before_grace.reason_code, "supervisor_starting");

  const auto after_grace =
    policy.EvaluateSupervisor(localization, now, 10.0);

  EXPECT_EQ(after_grace.lifecycle, Lifecycle::FAULTED);
  EXPECT_EQ(after_grace.health, AggregateHealth::ERROR);
  EXPECT_FALSE(after_grace.ready);
  EXPECT_EQ(after_grace.reason_code, "localization_health_missing");
}
