// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <limits>
#include <stdexcept>
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

TEST(TelemetrySnapshot, NoMetricsPreservesExistingFields)
{
  const std::vector<savo_observer::DependencyEvaluation> dependencies{
    {"core", savo_observer::DependencyState::kStale, true, 5001, "old"}};
  const auto snapshot = savo_observer::BuildSnapshot(4, 100, dependencies, false);
  EXPECT_TRUE(snapshot.metrics.empty());
  EXPECT_EQ(
    savo_observer::SerializeSnapshot(snapshot),
    "{\"schema\":\"savo_observer.telemetry.v1\",\"sequence\":4,\"monotonic_ms\":100,"
    "\"state\":\"disconnected\",\"connected\":false,\"dependencies\":[{\"name\":\"core\","
    "\"state\":\"stale\",\"required\":true,\"age_ms\":5001,\"detail\":\"old\"}],"
    "\"alerts\":[\"core:stale\"],\"metrics\":{}}");
}

TEST(TelemetrySnapshot, NumericMetricsSerializeInNameOrder)
{
  const std::vector<savo_observer::NumericMetric> metrics{
    {"voltage", 12.5}, {"distance", 1.25}};
  const auto snapshot = savo_observer::BuildSnapshot(7, 400, {}, false, metrics);
  const auto reordered = savo_observer::BuildSnapshot(
    7, 400, {}, false, {{"distance", 1.25}, {"voltage", 12.5}});
  EXPECT_EQ(snapshot.state, savo_observer::ObserverState::kConnected);
  EXPECT_TRUE(snapshot.connected);
  EXPECT_TRUE(snapshot.alerts.empty());
  EXPECT_EQ(snapshot.metrics.at("voltage"), 12.5);
  EXPECT_EQ(
    savo_observer::SerializeSnapshot(snapshot),
    "{\"schema\":\"savo_observer.telemetry.v1\",\"sequence\":7,\"monotonic_ms\":400,"
    "\"state\":\"connected\",\"connected\":true,\"dependencies\":[],\"alerts\":[],"
    "\"metrics\":{\"distance\":1.25,\"voltage\":12.5}}");
  EXPECT_EQ(
    savo_observer::SerializeSnapshot(snapshot), savo_observer::SerializeSnapshot(reordered));
}

TEST(TelemetrySnapshot, ZeroAndNegativeFiniteMetricsAreAccepted)
{
  const auto snapshot = savo_observer::BuildSnapshot(
    8, 500, {}, false, {{"zero", 0.0}, {"negative", -2.5}});
  EXPECT_NE(
    savo_observer::SerializeSnapshot(snapshot).find("\"metrics\":{\"negative\":-2.5,\"zero\":0}"),
    std::string::npos);
}

TEST(TelemetrySnapshot, MetricNamesUseJsonEscaping)
{
  const auto snapshot = savo_observer::BuildSnapshot(
    9, 600, {}, false, {{"sensor\"\\\n\r\t\x01", 1.0}});
  EXPECT_NE(
    savo_observer::SerializeSnapshot(snapshot).find(
      "\"metrics\":{\"sensor\\\"\\\\\\n\\r\\t\\u0001\":1}"),
    std::string::npos);
}

TEST(TelemetrySnapshot, EmptyMetricNamesAreRejected)
{
  EXPECT_THROW(
    savo_observer::BuildSnapshot(1, 0, {}, false, {{"", 1.0}}), std::invalid_argument);
}

TEST(TelemetrySnapshot, NanMetricsAreRejected)
{
  EXPECT_THROW(
    savo_observer::BuildSnapshot(
      1, 0, {}, false, {{"sensor", std::numeric_limits<double>::quiet_NaN()}}),
    std::invalid_argument);
}

TEST(TelemetrySnapshot, PositiveInfinityMetricsAreRejected)
{
  EXPECT_THROW(
    savo_observer::BuildSnapshot(
      1, 0, {}, false, {{"sensor", std::numeric_limits<double>::infinity()}}),
    std::invalid_argument);
}

TEST(TelemetrySnapshot, NegativeInfinityMetricsAreRejected)
{
  EXPECT_THROW(
    savo_observer::BuildSnapshot(
      1, 0, {}, false, {{"sensor", -std::numeric_limits<double>::infinity()}}),
    std::invalid_argument);
}

TEST(TelemetrySnapshot, DuplicateMetricNamesAreRejected)
{
  EXPECT_THROW(
    savo_observer::BuildSnapshot(1, 0, {}, false, {{"sensor", 1.0}, {"sensor", 2.0}}),
    std::invalid_argument);
}

TEST(TelemetrySnapshot, SerializationRejectsInvalidMetricsInDirectSnapshots)
{
  savo_observer::TelemetrySnapshot snapshot;
  snapshot.metrics = {{"", 1.0}};
  EXPECT_THROW(savo_observer::SerializeSnapshot(snapshot), std::invalid_argument);
  for (const double value : {
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity()})
  {
    snapshot.metrics = {{"sensor", value}};
    EXPECT_THROW(savo_observer::SerializeSnapshot(snapshot), std::invalid_argument);
  }
}
