// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include "savo_bridge/freshness_tracker.hpp"

#include <stdexcept>

namespace savo_bridge
{

FreshnessTracker::FreshnessTracker(Duration stale_after)
: stale_after_(stale_after)
{
  if (stale_after_ <= Duration::zero()) {
    throw std::invalid_argument(
            "FreshnessTracker stale_after must be positive");
  }
}

bool FreshnessTracker::observe(TimePoint observed_at) noexcept
{
  const std::lock_guard<std::mutex> lock(mutex_);

  if (last_observed_.has_value() &&
    observed_at < last_observed_.value())
  {
    ++rejected_regressions_;
    return false;
  }

  last_observed_ = observed_at;
  ++accepted_observations_;
  return true;
}

FreshnessTracker::Snapshot FreshnessTracker::snapshot(
  TimePoint evaluated_at) const noexcept
{
  const std::lock_guard<std::mutex> lock(mutex_);

  Snapshot result;
  result.accepted_observations = accepted_observations_;
  result.rejected_regressions = rejected_regressions_;

  if (!last_observed_.has_value()) {
    return result;
  }

  result.observed = true;

  if (evaluated_at < last_observed_.value()) {
    result.state = FreshnessState::kClockRegression;
    return result;
  }

  result.age = evaluated_at - last_observed_.value();

  if (result.age >= stale_after_) {
    result.state = FreshnessState::kStale;
  } else {
    result.state = FreshnessState::kFresh;
  }

  return result;
}

void FreshnessTracker::reset() noexcept
{
  const std::lock_guard<std::mutex> lock(mutex_);

  last_observed_.reset();
  accepted_observations_ = 0U;
  rejected_regressions_ = 0U;
}

FreshnessTracker::Duration FreshnessTracker::stale_after() const noexcept
{
  return stale_after_;
}

}  // namespace savo_bridge
