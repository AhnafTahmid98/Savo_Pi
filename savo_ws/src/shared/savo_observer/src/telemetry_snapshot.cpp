// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_observer/telemetry_snapshot.hpp"

#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace savo_observer
{
namespace
{

std::string Escape(const std::string & input)
{
  std::ostringstream output;
  for (const unsigned char character : input) {
    switch (character) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0') <<
            static_cast<int>(character);
        } else {
          output << static_cast<char>(character);
        }
    }
  }
  return output.str();
}

}  // namespace

BoundedSeries::BoundedSeries(const std::size_t capacity)
: capacity_(capacity)
{
  if (capacity_ == 0U) {
    throw std::invalid_argument("bounded series capacity must be positive");
  }
}

void BoundedSeries::Push(const NumericSample sample)
{
  if (!samples_.empty() && sample.monotonic_ms < samples_.back().monotonic_ms) {
    throw std::invalid_argument("numeric sample clock reversal");
  }
  samples_.push_back(sample);
  while (samples_.size() > capacity_) {
    samples_.pop_front();
  }
}

const std::deque<NumericSample> & BoundedSeries::Samples() const noexcept
{
  return samples_;
}

std::size_t BoundedSeries::Capacity() const noexcept
{
  return capacity_;
}

TelemetrySnapshot BuildSnapshot(
  const std::uint64_t sequence, const std::int64_t monotonic_ms,
  std::vector<DependencyEvaluation> dependencies, const bool clock_reversal_detected)
{
  TelemetrySnapshot snapshot;
  snapshot.sequence = sequence;
  snapshot.monotonic_ms = monotonic_ms;
  snapshot.dependencies = std::move(dependencies);

  bool required_problem = false;
  bool required_failed = false;
  bool optional_problem = false;
  for (const auto & dependency : snapshot.dependencies) {
    const bool unavailable = dependency.state == DependencyState::kUnavailable;
    const bool stale = dependency.state == DependencyState::kStale;
    const bool degraded = dependency.state == DependencyState::kDegraded;
    const bool failed = dependency.state == DependencyState::kFailed;
    if ((unavailable || stale || degraded || failed) && dependency.required) {
      required_problem = true;
      required_failed = required_failed || failed;
      snapshot.alerts.push_back(
        dependency.name + ":" + std::string(ToString(dependency.state)));
    } else if ((stale || degraded || failed) && !dependency.required) {
      optional_problem = true;
      snapshot.alerts.push_back(
        dependency.name + ":" + std::string(ToString(dependency.state)));
    }
  }
  if (clock_reversal_detected) {
    snapshot.alerts.emplace_back("monotonic_clock_reversal_rejected");
    snapshot.state = ObserverState::kFailed;
  } else if (required_failed) {
    snapshot.state = ObserverState::kFailed;
  } else if (required_problem) {
    snapshot.state = ObserverState::kDisconnected;
  } else if (optional_problem) {
    snapshot.state = ObserverState::kDegraded;
  } else {
    snapshot.state = ObserverState::kConnected;
  }
  snapshot.connected = snapshot.state == ObserverState::kConnected ||
    snapshot.state == ObserverState::kDegraded;
  return snapshot;
}

std::string SerializeSnapshot(const TelemetrySnapshot & snapshot)
{
  std::ostringstream output;
  output << "{\"schema\":\"savo_observer.telemetry.v1\",\"sequence\":" <<
    snapshot.sequence << ",\"monotonic_ms\":" << snapshot.monotonic_ms <<
    ",\"state\":\"" << ToString(snapshot.state) << "\",\"connected\":" <<
    (snapshot.connected ? "true" : "false") << ",\"dependencies\":[";
  for (std::size_t index = 0; index < snapshot.dependencies.size(); ++index) {
    const auto & dependency = snapshot.dependencies[index];
    if (index > 0U) {
      output << ',';
    }
    output << "{\"name\":\"" << Escape(dependency.name) << "\",\"state\":\"" <<
      ToString(dependency.state) << "\",\"required\":" <<
      (dependency.required ? "true" : "false") << ",\"age_ms\":" <<
      dependency.age_ms << ",\"detail\":\"" << Escape(dependency.detail) << "\"}";
  }
  output << "],\"alerts\":[";
  for (std::size_t index = 0; index < snapshot.alerts.size(); ++index) {
    if (index > 0U) {
      output << ',';
    }
    output << "\"" << Escape(snapshot.alerts[index]) << "\"";
  }
  output << "]}";
  return output.str();
}

}  // namespace savo_observer
