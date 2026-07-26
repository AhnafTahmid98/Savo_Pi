// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include "savo_bridge/topic_observation.hpp"

#include <cctype>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace savo_bridge
{
namespace
{

[[nodiscard]] bool is_valid_absolute_topic_name(
  const std::string_view topic_name) noexcept
{
  if (topic_name.size() < 2U) {
    return false;
  }

  if (topic_name.front() != '/' || topic_name.back() == '/') {
    return false;
  }

  if (topic_name.find("//") != std::string_view::npos) {
    return false;
  }

  for (const char character : topic_name) {
    const auto value = static_cast<unsigned char>(character);

    if (character != '/' &&
      character != '_' &&
      std::isalnum(value) == 0)
    {
      return false;
    }
  }

  return true;
}

}  // namespace

TopicObservation::TopicObservation(TopicObservationConfig config)
: config_(validate_config(std::move(config))),
  freshness_tracker_(config_.stale_after)
{
}

bool TopicObservation::observe(TimePoint observed_at) noexcept
{
  return freshness_tracker_.observe(observed_at);
}

TopicObservation::Snapshot TopicObservation::snapshot(
  TimePoint evaluated_at) const
{
  const auto freshness = freshness_tracker_.snapshot(evaluated_at);

  Snapshot result;
  result.topic_name = config_.topic_name;
  result.requirement = config_.requirement;
  result.classification = classify_topic(freshness.state);
  result.observed = freshness.observed;
  result.accepted_observations = freshness.accepted_observations;
  result.rejected_regressions = freshness.rejected_regressions;
  result.age = freshness.age;
  result.stale_after = config_.stale_after;

  return result;
}

void TopicObservation::reset() noexcept
{
  freshness_tracker_.reset();
}

const TopicObservationConfig & TopicObservation::config() const noexcept
{
  return config_;
}

TopicObservationConfig TopicObservation::validate_config(
  TopicObservationConfig config)
{
  if (!is_valid_absolute_topic_name(config.topic_name)) {
    throw std::invalid_argument(
            "TopicObservation topic_name must be a valid absolute topic");
  }

  if (config.stale_after <= Duration::zero()) {
    throw std::invalid_argument(
            "TopicObservation stale_after must be positive");
  }

  return config;
}

}  // namespace savo_bridge
