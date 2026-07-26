// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <string_view>

#include "savo_bridge/freshness_tracker.hpp"

namespace
{

using Tracker = savo_bridge::FreshnessTracker;
using Duration = Tracker::Duration;
using TimePoint = Tracker::TimePoint;

[[nodiscard]] Duration seconds_duration(const int value)
{
  return std::chrono::duration_cast<Duration>(
    std::chrono::seconds(value));
}

[[nodiscard]] Duration milliseconds_duration(const int value)
{
  return std::chrono::duration_cast<Duration>(
    std::chrono::milliseconds(value));
}

[[nodiscard]] bool freshness_contract_holds()
{
  bool invalid_timeout_rejected = false;

  try {
    const Tracker invalid_tracker(Duration::zero());
    (void)invalid_tracker;
  } catch (const std::invalid_argument &) {
    invalid_timeout_rejected = true;
  } catch (...) {
    return false;
  }

  if (!invalid_timeout_rejected) {
    return false;
  }

  Tracker tracker(seconds_duration(1));
  const TimePoint origin{seconds_duration(10)};

  auto state = tracker.snapshot(origin);

  if (state.observed ||
    state.state != savo_bridge::FreshnessState::kNeverObserved ||
    state.accepted_observations != 0U ||
    state.rejected_regressions != 0U ||
    state.age != Duration::zero())
  {
    return false;
  }

  if (!tracker.observe(origin)) {
    return false;
  }

  state = tracker.snapshot(
    origin + milliseconds_duration(999));

  if (!state.observed ||
    state.state != savo_bridge::FreshnessState::kFresh ||
    state.accepted_observations != 1U ||
    state.rejected_regressions != 0U ||
    state.age != milliseconds_duration(999))
  {
    return false;
  }

  state = tracker.snapshot(origin + seconds_duration(1));

  if (state.state != savo_bridge::FreshnessState::kStale ||
    state.age != seconds_duration(1))
  {
    return false;
  }

  if (!tracker.observe(origin + seconds_duration(2))) {
    return false;
  }

  if (tracker.observe(origin + milliseconds_duration(1500))) {
    return false;
  }

  state = tracker.snapshot(
    origin + milliseconds_duration(2100));

  if (state.state != savo_bridge::FreshnessState::kFresh ||
    state.accepted_observations != 2U ||
    state.rejected_regressions != 1U ||
    state.age != milliseconds_duration(100))
  {
    return false;
  }

  state = tracker.snapshot(
    origin + milliseconds_duration(1900));

  if (state.state !=
    savo_bridge::FreshnessState::kClockRegression)
  {
    return false;
  }

  if (state.age != Duration::zero()) {
    return false;
  }

  if (tracker.stale_after() != seconds_duration(1)) {
    return false;
  }

  if (savo_bridge::to_string(
      savo_bridge::FreshnessState::kNeverObserved) !=
    std::string_view("never_observed"))
  {
    return false;
  }

  if (savo_bridge::to_string(
      savo_bridge::FreshnessState::kFresh) !=
    std::string_view("fresh"))
  {
    return false;
  }

  if (savo_bridge::to_string(
      savo_bridge::FreshnessState::kStale) !=
    std::string_view("stale"))
  {
    return false;
  }

  if (savo_bridge::to_string(
      savo_bridge::FreshnessState::kClockRegression) !=
    std::string_view("clock_regression"))
  {
    return false;
  }

  tracker.reset();
  state = tracker.snapshot(origin + seconds_duration(3));

  return
    !state.observed &&
    state.state == savo_bridge::FreshnessState::kNeverObserved &&
    state.accepted_observations == 0U &&
    state.rejected_regressions == 0U &&
    state.age == Duration::zero();
}

TEST(SavoBridgeFreshness, MonotonicFreshnessContract)
{
  EXPECT_TRUE(freshness_contract_holds());
}

}  // namespace
