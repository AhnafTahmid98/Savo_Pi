// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_observer/telemetry_freshness.hpp"

#include <set>
#include <stdexcept>
#include <utility>

namespace savo_observer
{

TelemetryFreshness::TelemetryFreshness(std::vector<FreshnessPolicy> policies)
: policies_(std::move(policies))
{
  std::set<std::string> names;
  for (const auto & policy : policies_) {
    if (policy.name.empty() || policy.stale_after_ms <= 0) {
      throw std::invalid_argument("freshness policies require a name and positive timeout");
    }
    if (!names.insert(policy.name).second) {
      throw std::invalid_argument("freshness policy names must be unique");
    }
  }
}

bool TelemetryFreshness::Observe(
  const std::string & name, const std::int64_t monotonic_ms, std::string detail,
  const bool degraded, const bool failed)
{
  if (monotonic_ms < 0 ||
    (latest_monotonic_ms_ >= 0 && monotonic_ms < latest_monotonic_ms_))
  {
    clock_reversal_detected_ = true;
    return false;
  }
  auto found_policy = false;
  for (const auto & policy : policies_) {
    found_policy = found_policy || policy.name == name;
  }
  if (!found_policy) {
    return false;
  }
  auto & observation = observations_[name];
  if (observation.seen && monotonic_ms < observation.monotonic_ms) {
    clock_reversal_detected_ = true;
    return false;
  }
  latest_monotonic_ms_ = monotonic_ms;
  observation = {true, degraded, failed, monotonic_ms, std::move(detail)};
  return true;
}

std::vector<DependencyEvaluation> TelemetryFreshness::Evaluate(
  const std::int64_t monotonic_ms) const
{
  std::vector<DependencyEvaluation> output;
  output.reserve(policies_.size());
  for (const auto & policy : policies_) {
    DependencyEvaluation evaluation;
    evaluation.name = policy.name;
    evaluation.required = policy.required;
    if (!policy.enabled) {
      evaluation.state = DependencyState::kDisabled;
      evaluation.detail = "explicitly_disabled";
      output.push_back(std::move(evaluation));
      continue;
    }
    const auto found = observations_.find(policy.name);
    if (found == observations_.end() || !found->second.seen) {
      output.push_back(std::move(evaluation));
      continue;
    }
    const auto & observation = found->second;
    evaluation.age_ms = monotonic_ms - observation.monotonic_ms;
    evaluation.detail = observation.detail;
    if (evaluation.age_ms < 0) {
      evaluation.state = DependencyState::kFailed;
      evaluation.detail = "clock_reversal";
    } else if (observation.failed) {
      evaluation.state = DependencyState::kFailed;
    } else if (evaluation.age_ms > policy.stale_after_ms) {
      evaluation.state = DependencyState::kStale;
    } else if (observation.degraded) {
      evaluation.state = DependencyState::kDegraded;
    } else {
      evaluation.state = DependencyState::kFresh;
    }
    output.push_back(std::move(evaluation));
  }
  return output;
}

bool TelemetryFreshness::ClockReversalDetected() const noexcept
{
  return clock_reversal_detected_;
}

}  // namespace savo_observer
