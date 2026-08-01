// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef SAVO_OBSERVER__TELEMETRY_FRESHNESS_HPP_
#define SAVO_OBSERVER__TELEMETRY_FRESHNESS_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "savo_observer/observer_contract.hpp"

namespace savo_observer
{

struct FreshnessPolicy
{
  std::string name;
  bool enabled{true};
  bool required{false};
  std::int64_t stale_after_ms{3000};
};

struct DependencyEvaluation
{
  std::string name;
  DependencyState state{DependencyState::kUnavailable};
  bool required{false};
  std::int64_t age_ms{-1};
  std::string detail{"not_observed"};
};

class TelemetryFreshness
{
public:
  explicit TelemetryFreshness(std::vector<FreshnessPolicy> policies);

  [[nodiscard]] bool Observe(
    const std::string & name, std::int64_t monotonic_ms, std::string detail,
    bool degraded = false, bool failed = false);
  [[nodiscard]] std::vector<DependencyEvaluation> Evaluate(
    std::int64_t monotonic_ms) const;
  [[nodiscard]] bool ClockReversalDetected() const noexcept;

private:
  struct Observation
  {
    bool seen{false};
    bool degraded{false};
    bool failed{false};
    std::int64_t monotonic_ms{0};
    std::string detail{"not_observed"};
  };

  std::vector<FreshnessPolicy> policies_;
  std::map<std::string, Observation> observations_;
  std::int64_t latest_monotonic_ms_{-1};
  bool clock_reversal_detected_{false};
};

}  // namespace savo_observer

#endif  // SAVO_OBSERVER__TELEMETRY_FRESHNESS_HPP_
