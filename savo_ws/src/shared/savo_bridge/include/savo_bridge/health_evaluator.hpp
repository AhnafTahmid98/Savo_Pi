// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#ifndef SAVO_BRIDGE__HEALTH_EVALUATOR_HPP_
#define SAVO_BRIDGE__HEALTH_EVALUATOR_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "savo_bridge/topic_observation.hpp"

namespace savo_bridge
{

enum class HealthStatus : std::uint8_t
{
  kUnknown,
  kHealthy,
  kDegraded,
  kUnhealthy,
};

[[nodiscard]] constexpr std::string_view to_string(
  HealthStatus status) noexcept
{
  switch (status) {
    case HealthStatus::kUnknown:
      return "unknown";
    case HealthStatus::kHealthy:
      return "healthy";
    case HealthStatus::kDegraded:
      return "degraded";
    case HealthStatus::kUnhealthy:
      return "unhealthy";
  }

  return "unknown";
}

enum class HealthReason : std::uint8_t
{
  kNoTopics,
  kAllTopicsFresh,
  kOptionalTopicUnavailable,
  kRequiredTopicUnavailable,
  kClockRegression,
};

[[nodiscard]] constexpr std::string_view to_string(
  HealthReason reason) noexcept
{
  switch (reason) {
    case HealthReason::kNoTopics:
      return "no_topics";
    case HealthReason::kAllTopicsFresh:
      return "all_topics_fresh";
    case HealthReason::kOptionalTopicUnavailable:
      return "optional_topic_unavailable";
    case HealthReason::kRequiredTopicUnavailable:
      return "required_topic_unavailable";
    case HealthReason::kClockRegression:
      return "clock_regression";
  }

  return "unknown";
}

struct HealthEvaluation
{
  HealthStatus status{HealthStatus::kUnknown};
  HealthReason reason{HealthReason::kNoTopics};

  bool required_topics_ready{false};
  bool all_topics_fresh{false};

  std::size_t total_topics{0U};
  std::size_t required_topics{0U};
  std::size_t optional_topics{0U};

  std::size_t fresh_topics{0U};
  std::size_t stale_topics{0U};
  std::size_t never_observed_topics{0U};
  std::size_t clock_regression_topics{0U};

  std::vector<std::string> required_unavailable_topics;
  std::vector<std::string> optional_unavailable_topics;
  std::vector<std::string> clock_regression_topic_names;
};

class HealthEvaluator final
{
public:
  using TopicSnapshot = TopicObservation::Snapshot;

  [[nodiscard]] HealthEvaluation evaluate(
    const std::vector<TopicSnapshot> & topics) const;

private:
  static void validate_snapshot(const TopicSnapshot & snapshot);
};

}  // namespace savo_bridge

#endif  // SAVO_BRIDGE__HEALTH_EVALUATOR_HPP_
