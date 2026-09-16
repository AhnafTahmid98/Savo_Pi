// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>

namespace savo_mapping::local_authority
{
using Clock = std::chrono::steady_clock;

struct Identity
{
  std::string mission_id;
  std::string actor_id;
  std::string request_id;
  std::string map_id;
  std::uint32_t map_revision{0U};
  bool require_semantic{false};
};

class Health
{
public:
  void observe(const std::string & payload, Clock::time_point now)
  {
    valid_ = false;
    received_at_ = now;
    reason_ = "mapping_local_health_invalid";
    try {
      const auto json = nlohmann::json::parse(payload);
      if (json.at("schema_version") != 1 || json.at("node") != "savo_mapping" ||
        !json.at("admission_ready").is_boolean() ||
        !json.at("continuation_ready").is_boolean() ||
        !json.at("semantic_ready").is_boolean() || !json.at("reason").is_string())
      {
        return;
      }
      admission_ = json.at("admission_ready").get<bool>();
      continuation_ = json.at("continuation_ready").get<bool>();
      semantic_ = json.at("semantic_ready").get<bool>();
      reason_ = json.at("reason").get<std::string>();
      valid_ = !reason_.empty() && (!admission_ || continuation_);
    } catch (const nlohmann::json::exception &) {
      return;
    }
  }

  bool ready(bool admission, bool semantic, Clock::time_point now) const
  {
    return fresh(now) && (admission ? admission_ : continuation_) && (!semantic || semantic_);
  }

  bool fresh(Clock::time_point now) const
  {
    return valid_ && received_at_.has_value() && now >= *received_at_ &&
           now - *received_at_ <= std::chrono::milliseconds(1500);
  }

  std::string reason(Clock::time_point now) const
  {
    return fresh(now) ? reason_ : "mapping_local_health_missing_stale_or_invalid";
  }

private:
  bool valid_{false};
  bool admission_{false};
  bool continuation_{false};
  bool semantic_{false};
  std::optional<Clock::time_point> received_at_;
  std::string reason_{"mapping_local_health_missing"};
};

class Authority
{
public:
  bool acquire(
    const Identity & identity, std::uint64_t supplied_generation,
    const Health & health, Clock::time_point now)
  {
    if (owned_) {return deny("mapping_local_authority_busy");}
    if (supplied_generation != 0U) {
      return deny("mapping_local_preacquired_generation_not_supported");
    }
    if (identity.mission_id.empty() || identity.actor_id.empty() || identity.request_id.empty() ||
      identity.map_id.empty() || identity.map_revision == 0U)
    {
      return deny("mapping_local_identity_invalid");
    }
    if (!health.ready(true, identity.require_semantic, now)) {
      return deny(health.ready(true, false, now) ?
        "mapping_local_semantic_unavailable" : health.reason(now));
    }
    identity_ = identity;
    ++generation_;
    owned_ = true;
    active_ = true;
    paused_ = false;
    reason_ = "mapping_local_authority_acquired";
    return true;
  }

  bool check(
    const Identity & identity, std::uint64_t generation,
    const Health & health, Clock::time_point now) const
  {
    return active_ && generation == generation_ && identity.mission_id == identity_.mission_id &&
           identity.actor_id == identity_.actor_id && identity.request_id == identity_.request_id &&
           identity.map_id == identity_.map_id && identity.map_revision == identity_.map_revision &&
           identity.require_semantic == identity_.require_semantic &&
           health.ready(false, identity_.require_semantic, now);
  }

  bool revalidate(const Health & health, Clock::time_point now)
  {
    if (!active_) {return false;}
    if (!health.ready(false, identity_.require_semantic, now)) {
      active_ = false;
      paused_ = false;
      reason_ = health.reason(now);
      return false;
    }
    return true;
  }

  void pause() {active_ = false; paused_ = owned_;}

  bool resume(const Health & health, Clock::time_point now)
  {
    if (!owned_ || !paused_ || !health.ready(true, identity_.require_semantic, now)) {
      return deny("mapping_local_resume_not_ready");
    }
    active_ = true;
    paused_ = false;
    reason_ = "mapping_local_authority_resumed";
    return true;
  }

  void release() {active_ = false; owned_ = false; paused_ = false;}
  bool active() const {return active_;}
  bool owned() const {return owned_;}
  std::uint64_t generation() const {return generation_;}
  const std::string & reason() const {return reason_;}
  const Identity & identity() const {return identity_;}

private:
  bool deny(const std::string & reason) {reason_ = reason; return false;}
  Identity identity_;
  bool owned_{false};
  bool active_{false};
  bool paused_{false};
  std::uint64_t generation_{0U};
  std::string reason_{"mapping_local_authority_idle"};
};
}  // namespace savo_mapping::local_authority
