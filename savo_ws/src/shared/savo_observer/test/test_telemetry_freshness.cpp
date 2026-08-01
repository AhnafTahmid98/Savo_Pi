// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "savo_observer/telemetry_freshness.hpp"

TEST(TelemetryFreshness, DistinguishesDisabledUnavailableFreshAndStale)
{
  savo_observer::TelemetryFreshness freshness({
    {"core", true, true, 1000},
    {"edge", false, false, 1000},
    {"speech", true, false, 1000}});
  ASSERT_TRUE(freshness.Observe("core", 100, "connected"));
  auto states = freshness.Evaluate(500);
  EXPECT_EQ(states[0].state, savo_observer::DependencyState::kFresh);
  EXPECT_EQ(states[1].state, savo_observer::DependencyState::kDisabled);
  EXPECT_EQ(states[2].state, savo_observer::DependencyState::kUnavailable);
  states = freshness.Evaluate(1200);
  EXPECT_EQ(states[0].state, savo_observer::DependencyState::kStale);
}

TEST(TelemetryFreshness, PreservesDegradedAndFailedStates)
{
  savo_observer::TelemetryFreshness freshness({{"core", true, true, 1000}});
  ASSERT_TRUE(freshness.Observe("core", 100, "warning", true, false));
  EXPECT_EQ(
    freshness.Evaluate(200).front().state,
    savo_observer::DependencyState::kDegraded);
  ASSERT_TRUE(freshness.Observe("core", 300, "fault", false, true));
  EXPECT_EQ(
    freshness.Evaluate(400).front().state,
    savo_observer::DependencyState::kFailed);
}

TEST(TelemetryFreshness, RejectsClockReversal)
{
  savo_observer::TelemetryFreshness freshness({{"core", true, true, 1000}});
  ASSERT_TRUE(freshness.Observe("core", 100, "connected"));
  EXPECT_FALSE(freshness.Observe("core", 99, "old"));
  EXPECT_TRUE(freshness.ClockReversalDetected());
}

TEST(TelemetryFreshness, RejectsInvalidPolicies)
{
  EXPECT_THROW(
    savo_observer::TelemetryFreshness({{"", true, true, 1000}}),
    std::invalid_argument);
  EXPECT_THROW(
    savo_observer::TelemetryFreshness({{"core", true, true, 0}}),
    std::invalid_argument);
}
