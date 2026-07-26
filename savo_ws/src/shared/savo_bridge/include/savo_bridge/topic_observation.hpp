// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#ifndef SAVO_BRIDGE__TOPIC_OBSERVATION_HPP_
#define SAVO_BRIDGE__TOPIC_OBSERVATION_HPP_

#include <cstdint>
#include <string>
#include <string_view>

#include "savo_bridge/freshness_tracker.hpp"

namespace savo_bridge
{

enum class TopicRequirement : std::uint8_t
{
  kRequired,
  kOptional,
};

[[nodiscard]] constexpr std::string_view to_string(
  TopicRequirement requirement) noexcept
{
  switch (requirement) {
    case TopicRequirement::kRequired:
      return "required";
    case TopicRequirement::kOptional:
      return "optional";
  }

  return "unknown";
}

enum class TopicClassification : std::uint8_t
{
  kNeverObserved,
  kFresh,
  kStale,
  kClockRegression,
};

[[nodiscard]] constexpr std::string_view to_string(
  TopicClassification classification) noexcept
{
  switch (classification) {
    case TopicClassification::kNeverObserved:
      return "never_observed";
    case TopicClassification::kFresh:
      return "fresh";
    case TopicClassification::kStale:
      return "stale";
    case TopicClassification::kClockRegression:
      return "clock_regression";
  }

  return "unknown";
}

[[nodiscard]] constexpr TopicClassification classify_topic(
  FreshnessState freshness_state) noexcept
{
  switch (freshness_state) {
    case FreshnessState::kNeverObserved:
      return TopicClassification::kNeverObserved;
    case FreshnessState::kFresh:
      return TopicClassification::kFresh;
    case FreshnessState::kStale:
      return TopicClassification::kStale;
    case FreshnessState::kClockRegression:
      return TopicClassification::kClockRegression;
  }

  return TopicClassification::kClockRegression;
}

struct TopicObservationConfig
{
  std::string topic_name;
  TopicRequirement requirement{TopicRequirement::kRequired};
  FreshnessTracker::Duration stale_after{
    FreshnessTracker::Duration::zero()};
};

class TopicObservation final
{
public:
  using Clock = FreshnessTracker::Clock;
  using TimePoint = FreshnessTracker::TimePoint;
  using Duration = FreshnessTracker::Duration;

  struct Snapshot
  {
    std::string topic_name;
    TopicRequirement requirement{TopicRequirement::kRequired};
    TopicClassification classification{
      TopicClassification::kNeverObserved};
    bool observed{false};
    std::uint64_t accepted_observations{0U};
    std::uint64_t rejected_regressions{0U};
    Duration age{Duration::zero()};
    Duration stale_after{Duration::zero()};
  };

  explicit TopicObservation(TopicObservationConfig config);

  [[nodiscard]] bool observe(TimePoint observed_at) noexcept;
  [[nodiscard]] Snapshot snapshot(TimePoint evaluated_at) const;

  void reset() noexcept;

  [[nodiscard]] const TopicObservationConfig & config() const noexcept;

private:
  [[nodiscard]] static TopicObservationConfig validate_config(
    TopicObservationConfig config);

  const TopicObservationConfig config_;
  FreshnessTracker freshness_tracker_;
};

}  // namespace savo_bridge

#endif  // SAVO_BRIDGE__TOPIC_OBSERVATION_HPP_
