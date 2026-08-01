// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "savo_observer/telemetry_snapshot.hpp"

TEST(TelemetrySnapshot, HistoryIsBoundedAndMonotonic)
{
  savo_observer::BoundedSeries series(2);
  series.Push({1, 1.0});
  series.Push({2, 2.0});
  series.Push({3, 3.0});
  ASSERT_EQ(series.Samples().size(), 2U);
  EXPECT_EQ(series.Samples().front().monotonic_ms, 2);
  EXPECT_THROW(series.Push({1, 4.0}), std::invalid_argument);
}

TEST(TelemetrySnapshot, RequiredStaleDisconnectsAndAlerts)
{
  std::vector<savo_observer::DependencyEvaluation> dependencies{
    {"core", savo_observer::DependencyState::kStale, true, 5001, "old"},
    {"edge", savo_observer::DependencyState::kDisabled, false, -1, "disabled"}};
  const auto snapshot = savo_observer::BuildSnapshot(4, 100, dependencies, false);
  EXPECT_EQ(snapshot.state, savo_observer::ObserverState::kDisconnected);
  EXPECT_FALSE(snapshot.connected);
  ASSERT_EQ(snapshot.alerts.size(), 1U);
  EXPECT_NE(savo_observer::SerializeSnapshot(snapshot).find("core:stale"), std::string::npos);
}

TEST(TelemetrySnapshot, OptionalDegradationRemainsConnected)
{
  std::vector<savo_observer::DependencyEvaluation> dependencies{
    {"core", savo_observer::DependencyState::kFresh, true, 10, "connected"},
    {"edge", savo_observer::DependencyState::kStale, false, 5001, "old"}};
  const auto snapshot = savo_observer::BuildSnapshot(5, 200, dependencies, false);
  EXPECT_EQ(snapshot.state, savo_observer::ObserverState::kDegraded);
  EXPECT_TRUE(snapshot.connected);
}

TEST(TelemetrySnapshot, ClockReversalFailsClosed)
{
  const auto snapshot = savo_observer::BuildSnapshot(6, 300, {}, true);
  EXPECT_EQ(snapshot.state, savo_observer::ObserverState::kFailed);
  EXPECT_FALSE(snapshot.connected);
}
