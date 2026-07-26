// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#ifndef SAVO_BRIDGE__OBSERVATION_CONFIG_HPP_
#define SAVO_BRIDGE__OBSERVATION_CONFIG_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "savo_bridge/topic_observation.hpp"

namespace savo_bridge
{

struct ObservationParameterSet
{
  std::vector<std::string> topic_names;
  std::vector<std::string> requirements;
  std::vector<std::int64_t> stale_after_ms;
};

[[nodiscard]] std::vector<TopicObservationConfig>
make_observation_configs(
  const ObservationParameterSet & parameters,
  const std::vector<std::string> &
  forbidden_topic_names);

}  // namespace savo_bridge

#endif  // SAVO_BRIDGE__OBSERVATION_CONFIG_HPP_
