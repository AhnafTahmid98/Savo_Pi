// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef SAVO_OBSERVER__TELEMETRY_SNAPSHOT_HPP_
#define SAVO_OBSERVER__TELEMETRY_SNAPSHOT_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "savo_observer/telemetry_freshness.hpp"

namespace savo_observer
{

struct NumericSample
{
  std::int64_t monotonic_ms{0};
  double value{0.0};
};

class BoundedSeries
{
public:
  explicit BoundedSeries(std::size_t capacity);
  void Push(NumericSample sample);
  [[nodiscard]] const std::deque<NumericSample> & Samples() const noexcept;
  [[nodiscard]] std::size_t Capacity() const noexcept;

private:
  std::size_t capacity_;
  std::deque<NumericSample> samples_;
};

struct TelemetrySnapshot
{
  std::uint64_t sequence{0};
  std::int64_t monotonic_ms{0};
  ObserverState state{ObserverState::kStarting};
  bool connected{false};
  std::vector<DependencyEvaluation> dependencies;
  std::vector<std::string> alerts;
};

[[nodiscard]] TelemetrySnapshot BuildSnapshot(
  std::uint64_t sequence, std::int64_t monotonic_ms,
  std::vector<DependencyEvaluation> dependencies, bool clock_reversal_detected);
[[nodiscard]] std::string SerializeSnapshot(const TelemetrySnapshot & snapshot);

}  // namespace savo_observer

#endif  // SAVO_OBSERVER__TELEMETRY_SNAPSHOT_HPP_
