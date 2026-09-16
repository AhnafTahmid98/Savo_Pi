// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <nlohmann/json.hpp>

namespace savo_mapping::phase_authority
{
using Clock = std::chrono::steady_clock;

class Context
{
public:
  void observe(const std::string & payload, Clock::time_point now)
  {
    valid_ = false;
    if (received_ && now < *received_) {return;}
    received_ = now;
    try {
      const auto value = nlohmann::json::parse(payload);
      if (value.at("schema_version") != 1 || value.at("node") != "savo_mapping") {return;}
      for (const auto * key : {"mission_id", "actor_id", "request_id", "map_id"}) {
        if (!value.at(key).is_string() || value.at(key).get<std::string>().empty()) {return;}
      }
      for (const auto * key : {"active", "require_semantic", "coverage_allowed",
          "semantic_allowed"})
      {
        if (!value.at(key).is_boolean()) {return;}
      }
      for (const auto * key : {"generation", "map_revision"}) {
        if (!value.at(key).is_number_integer() || value.at(key) <= 0) {return;}
      }
      if (value.at("map_revision") > std::numeric_limits<std::uint32_t>::max()) {return;}
      mission_id = value.at("mission_id").get<std::string>();
      actor_id = value.at("actor_id").get<std::string>();
      request_id = value.at("request_id").get<std::string>();
      map_id = value.at("map_id").get<std::string>();
      map_revision = value.at("map_revision").get<std::uint32_t>();
      generation = value.at("generation").get<std::uint64_t>();
      require_semantic = value.at("require_semantic").get<bool>();
      active_ = value.at("active").get<bool>();
      coverage_ = value.at("coverage_allowed").get<bool>();
      semantic_ = value.at("semantic_allowed").get<bool>();
      valid_ = (!coverage_ || active_) && (!semantic_ || require_semantic);
    } catch (const nlohmann::json::exception &) {
      return;
    }
  }

  bool fresh(Clock::time_point now) const
  {
    return valid_ && received_ && now >= *received_ &&
           now - *received_ <= std::chrono::milliseconds(1500);
  }
  bool coverage_ready(Clock::time_point now) const {return fresh(now) && active_ && coverage_;}
  bool semantic_ready(
    const std::string & actor, const std::string & map,
    std::uint32_t revision, Clock::time_point now) const
  {
    return fresh(now) && semantic_ && actor == actor_id && map == map_id &&
           revision == map_revision;
  }
  bool same_lease(const Context & other) const
  {
    return valid_ && other.valid_ && mission_id == other.mission_id && actor_id == other.actor_id &&
           request_id == other.request_id && map_id == other.map_id &&
           map_revision == other.map_revision && generation == other.generation &&
           require_semantic == other.require_semantic;
  }
  std::string scoped_request_id(const std::string & original) const
  {
    return request_id + ":" + std::to_string(generation) + ":" + original;
  }

  std::string mission_id;
  std::string actor_id;
  std::string request_id;
  std::string map_id;
  std::uint32_t map_revision{0U};
  std::uint64_t generation{0U};
  bool require_semantic{false};

private:
  bool valid_{false};
  bool active_{false};
  bool coverage_{false};
  bool semantic_{false};
  std::optional<Clock::time_point> received_;
};
}  // namespace savo_mapping::phase_authority
