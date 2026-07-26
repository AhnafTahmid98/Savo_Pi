// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include "savo_bridge/observation_config.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace savo_bridge
{
namespace
{

[[nodiscard]] TopicRequirement parse_requirement(
  const std::string & value)
{
  if (value == "required") {
    return TopicRequirement::kRequired;
  }

  if (value == "optional") {
    return TopicRequirement::kOptional;
  }

  throw std::invalid_argument(
          "Observation requirement must be "
          "'required' or 'optional'");
}

void validate_parameter_lengths(
  const ObservationParameterSet & parameters)
{
  const std::size_t topic_count =
    parameters.topic_names.size();

  if (
    parameters.requirements.size() != topic_count ||
    parameters.stale_after_ms.size() != topic_count)
  {
    throw std::invalid_argument(
            "Observation parameter arrays must have "
            "equal lengths");
  }
}

[[nodiscard]] TopicObservation::Duration
make_stale_duration(
  const std::int64_t stale_after_ms)
{
  if (stale_after_ms <= 0) {
    throw std::invalid_argument(
            "Observation stale threshold must be "
            "greater than zero milliseconds");
  }

  using Milliseconds = std::chrono::milliseconds;

  const std::int64_t maximum_milliseconds =
    std::chrono::duration_cast<Milliseconds>(
    TopicObservation::Duration::max()).count();

  if (stale_after_ms > maximum_milliseconds) {
    throw std::invalid_argument(
            "Observation stale threshold exceeds "
            "the monotonic-clock duration range");
  }

  const TopicObservation::Duration duration =
    std::chrono::duration_cast<
    TopicObservation::Duration>(
    Milliseconds(stale_after_ms));

  if (duration <= TopicObservation::Duration::zero()) {
    throw std::invalid_argument(
            "Observation stale threshold is too small "
            "for the monotonic clock");
  }

  return duration;
}

[[nodiscard]] TopicObservationConfig
validate_topic_config(
  TopicObservationConfig config)
{
  const TopicObservation observation(config);
  return observation.config();
}

[[nodiscard]] std::set<std::string>
make_forbidden_topic_set(
  const std::vector<std::string> &
  forbidden_topic_names)
{
  std::set<std::string> result;

  for (const std::string & topic_name :
    forbidden_topic_names)
  {
    TopicObservationConfig validation_config;
    validation_config.topic_name = topic_name;
    validation_config.requirement =
      TopicRequirement::kOptional;

    validation_config.stale_after =
      std::chrono::milliseconds(1);

    (void)validate_topic_config(
      std::move(validation_config));

    if (!result.insert(topic_name).second) {
      throw std::invalid_argument(
              "Forbidden observation topic names "
              "must be unique");
    }
  }

  return result;
}

}  // namespace

std::vector<TopicObservationConfig>
make_observation_configs(
  const ObservationParameterSet & parameters,
  const std::vector<std::string> &
  forbidden_topic_names)
{
  validate_parameter_lengths(parameters);

  const std::set<std::string> forbidden_topics =
    make_forbidden_topic_set(
    forbidden_topic_names);

  std::set<std::string> configured_topics;
  std::vector<TopicObservationConfig> result;

  result.reserve(parameters.topic_names.size());

  for (std::size_t index = 0U;
    index < parameters.topic_names.size();
    ++index)
  {
    const std::string & topic_name =
      parameters.topic_names.at(index);

    if (
      forbidden_topics.find(topic_name) !=
      forbidden_topics.end())
    {
      throw std::invalid_argument(
              "Bridge-owned topics cannot be configured "
              "as external observations");
    }

    if (!configured_topics.insert(topic_name).second) {
      throw std::invalid_argument(
              "Observation topic names must be unique");
    }

    TopicObservationConfig config;
    config.topic_name = topic_name;

    config.requirement = parse_requirement(
      parameters.requirements.at(index));

    config.stale_after = make_stale_duration(
      parameters.stale_after_ms.at(index));

    result.push_back(
      validate_topic_config(std::move(config)));
  }

  return result;
}

}  // namespace savo_bridge
