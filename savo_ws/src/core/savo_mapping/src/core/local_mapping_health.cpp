// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary
#include "savo_mapping/local_mapping_health.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <nlohmann/json.hpp>

namespace savo_mapping::local_health
{
namespace
{
using Json = nlohmann::json;

enum class PowerPolicy
{
  NOMINAL,
  RESTRICTED,
  FAILED
};

struct PowerState
{
  std::string canonical;
  PowerPolicy policy;
};

std::optional<PowerState> power_state(std::string state)
{
  std::transform(
    state.begin(), state.end(), state.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});
  if (state == "ok" || state == "charging" || state == "full") {
    return PowerState{state, PowerPolicy::NOMINAL};
  }
  if (state == "low" || state == "warn" || state == "warning") {
    return PowerState{state, PowerPolicy::RESTRICTED};
  }
  if (state == "critical" || state == "error" || state == "stale" ||
    state == "unknown" || state == "invalid")
  {
    return PowerState{state, PowerPolicy::FAILED};
  }
  return std::nullopt;
}

std::string trim(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {return {};}
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1U);
}

Json strict_json(const std::string & payload)
{
  bool duplicate_top_level_key = false;
  std::set<std::string> keys;
  const Json::parser_callback_t callback =
    [&duplicate_top_level_key, &keys](int depth, Json::parse_event_t event, Json & parsed) {
      if (depth == 1 && event == Json::parse_event_t::key && parsed.is_string() &&
        !keys.insert(parsed.get<std::string>()).second)
      {
        duplicate_top_level_key = true;
        return false;
      }
      return true;
    };
  auto result = Json::parse(payload, callback);
  if (duplicate_top_level_key) {throw std::invalid_argument("duplicate top-level key");}
  if (!result.is_object()) {throw std::invalid_argument("payload is not an object");}
  return result;
}

std::map<std::string, std::string> fields(const std::string & payload)
{
  std::map<std::string, std::string> result;
  std::istringstream stream(payload);
  std::string item;
  while (std::getline(stream, item, ';')) {
    const auto separator = item.find('=');
    if (separator == std::string::npos ||
      !result.emplace(
        trim(item.substr(0, separator)), trim(item.substr(separator + 1))).second)
    {
      throw std::invalid_argument("invalid key/value contract");
    }
  }
  return result;
}

bool boolean(const std::string & value)
{
  if (value == "true") {return true;}
  if (value == "false") {return false;}
  throw std::invalid_argument("invalid boolean");
}
}  // namespace

Monitor::Monitor(bool edge_required)
: sources_{
    {"base", "/savo_base/base_state", 1.5},
    {"control", "/savo_control/control_status", 1.0},
    {"mux", "/savo_control/twist_mux/status", 1.0},
    {"shaper", "/savo_control/cmd_vel_shaper/status", 1.0},
    {"lidar", "/savo_lidar/state", 2.0},
    {"lidar_heartbeat", "/savo_lidar/heartbeat", 3.0},
    {"localization", "/savo_localization/health", 1.5},
    {"localization_summary", "/savo_localization/state_summary", 1.5},
    {"localization_heartbeat", "/savo_localization/heartbeat", 2.5},
    {"perception", "/savo_perception/range_health", 1.5},
    {"perception_safety", "/savo_perception/safety_state", 1.0},
    {"perception_heartbeat", "/savo_perception/heartbeat", 2.5},
    {"base_battery", "/savo_power/base/battery", 3.0},
    {"core_ups", "/savo_power/core/ups", 3.0},
    {"edge_ups", "/savo_power/edge/ups", 3.0, edge_required},
    {"safety_stop", "/safety/stop", 1.0},
    {"safety_slowdown", "/safety/slowdown_factor", 1.0},
    {"nav", "/savo_nav/status", 1.5},
    {"slam", "/savo_mapping/slam_health", 1.5},
    {"head", "/savo_head/status", 1.5, false},
    {"locations", "/savo_locations/status", 2.0, false}}
{
}

void Monitor::observe(
  const std::string & source, const std::string & payload, Clock::time_point now)
{
  auto & value = observations_[source];
  const auto previous_receipt = value.receipt;
  value.receipt = now;
  value.valid = false;
  value.admission = false;
  value.continuation = false;
  value.degraded = false;
  value.state.clear();
  value.reason = source + "_invalid";
  if (previous_receipt && now < *previous_receipt) {return;}
  try {
    bool operational = false;
    bool nominal = true;
    bool reason_explicit = false;
    if (source == "safety_stop") {
      operational = true;
      nominal = !boolean(payload);
    } else if (source == "safety_slowdown") {
      std::size_t consumed = 0;
      const double factor = std::stod(payload, &consumed);
      operational = consumed == payload.size() && std::isfinite(factor) &&
        factor >= 0.0 && factor <= 1.0;
      nominal = factor > 0.0;
    } else if (source == "control" || source == "mux" || source == "shaper") {
      const auto status = fields(payload);
      const auto mode = status.at("mode");
      operational = mode == "STOP" || mode == "NAV" || mode == "AUTO";
      for (const auto * flag : {"safety_stop", "external_stop", "recovery_active"}) {
        const auto found = status.find(flag);
        if (found == status.end()) {continue;}
        const bool active = boolean(found->second);
        if (std::string(flag) == "external_stop" && active) {operational = false;}
        if (active) {nominal = false;}
      }
    } else if (source == "nav") {
      const auto status = fields(payload);
      const auto state = status.at("state");
      const bool allowed = boolean(status.at("goal_acceptance_allowed"));
      if (status.at("reason").empty()) {return;}
      const auto failures = status.at("failed_dependencies");
      static const std::set<std::string> states{
        "offline", "starting", "waiting_for_map", "waiting_for_tf",
        "waiting_for_localization", "waiting_for_lidar", "waiting_for_pointcloud",
        "waiting_for_nav2", "waiting_for_costmaps", "ready", "degraded", "blocked", "fault"};
      if (states.count(state) == 0U) {return;}
      operational = state == "ready" && allowed && failures.empty();
      nominal = operational;
      std::istringstream stream(failures);
      std::string failure;
      bool saw_failure = false;
      bool environmental_failure = false;
      while (std::getline(stream, failure, ',')) {
        failure = trim(failure);
        if (failure.empty()) {return;}
        saw_failure = true;
        // Infrastructure may be ready in STOP or while the physical gate is closed.
        // Every other failed dependency remains blocking; do not match only the first reason.
        if (failure != "control_mode_permission" && failure != "safety_stop" &&
          failure != "safety_motion_permission") {operational = false;}
        if (failure == "safety_stop" || failure == "safety_motion_permission") {
          environmental_failure = true;
        }
      }
      if (state == "blocked" && !allowed && saw_failure) {
        operational = true;
        std::istringstream validation(failures);
        while (std::getline(validation, failure, ',')) {
          failure = trim(failure);
          if (failure != "control_mode_permission" && failure != "safety_stop" &&
            failure != "safety_motion_permission") {operational = false;}
        }
        nominal = operational && !environmental_failure;
      }
    } else if (source == "base_battery" || source == "core_ups" || source == "edge_ups") {
      std::string state;
      std::optional<double> voltage;
      bool ok = true;
      if (!payload.empty() && payload.front() == '{') {
        const auto json = Json::parse(payload);
        if (json.at("source") != source) {return;}
        state = json.at("state").get<std::string>();
        const auto & voltage_field = json.at("voltage_v");
        if (voltage_field.is_number()) {
          voltage = voltage_field.get<double>();
        } else if (!voltage_field.is_null()) {
          return;
        }
        ok = json.value("ok", true);
      } else {
        static const std::regex pattern(
          R"(^\s*(Base battery|Core UPS|Edge UPS)\s+([A-Za-z]+):\s+)"
          R"((n/a|[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+))\s+V(?:,.*)?\s*$)");
        std::smatch match;
        if (!std::regex_match(payload, match, pattern)) {return;}
        const std::string label = source == "base_battery" ? "Base battery" :
          source == "core_ups" ? "Core UPS" : "Edge UPS";
        if (match[1] != label) {return;}
        state = match[2];
        if (match[3] != "n/a") {voltage = std::stod(match[3]);}
      }
      const auto classified = power_state(state);
      if (!classified) {return;}
      state = classified->canonical;
      const bool voltage_ok = voltage.has_value() && std::isfinite(*voltage) && *voltage > 0.0;
      if (classified->policy != PowerPolicy::FAILED && !voltage_ok) {return;}
      nominal = classified->policy == PowerPolicy::NOMINAL;
      operational = ok && voltage_ok && classified->policy != PowerPolicy::FAILED;
      value.reason = source + "_" + state;
      reason_explicit = true;
    } else {
      const auto json = source.rfind("localization", 0) == 0 ?
        strict_json(payload) : Json::parse(payload);
      if (source == "base") {
        const auto level = json.at("status_level").get<std::string>();
        operational = json.at("backend").at("connected").get<bool>() &&
          json.at("diagnostics").value("last_board_error", std::string{}).empty();
        nominal = level == "OK" || level == "STALE";
        if (level == "SAFETY_STOP") {
          const auto & safety = json.at("safety");
          operational = operational && safety.at("safety_stop").get<bool>() &&
            (safety.at("estop_latched").is_null() || !safety.at("estop_latched").get<bool>());
        } else {
          operational = operational && (nominal || level == "BLOCKED");
        }
      } else if (source == "lidar" || source == "lidar_heartbeat") {
        const auto state = json.at("status").get<std::string>();
        operational = json.at("driver_running").get<bool>() &&
          json.value("hardware_ok", true) && (state == "OK" || state == "WARN");
        nominal = state == "OK" && json.value("scan_ok", true);
      } else if (source.rfind("localization", 0) == 0) {
        if (!json.at("schema_version").is_number_integer() || json.at("schema_version") != 1 ||
          !json.at("state").is_string() || !json.at("ready").is_boolean() ||
          !json.at("stamp_s").is_number())
        {
          return;
        }
        const double stamp = json.at("stamp_s").get<double>();
        const auto prior_stamp = value.stamp;
        if (!std::isfinite(stamp) || stamp < 0.0 || (prior_stamp && stamp < *prior_stamp)) {
          value.reason = source + "_timestamp_regression_or_invalid";
          return;
        }
        const auto state = json.at("state").get<std::string>();
        if (state != "INITIALIZING" && state != "OK" && state != "DEGRADED" &&
          state != "STALE" && state != "ERROR") {return;}
        value.state = state;
        if (source == "localization_heartbeat") {
          if (!json.at("alive").is_boolean()) {return;}
          operational = json.at("alive").get<bool>();
        } else {
          if (!json.at("degraded").is_boolean() || !json.at("reason_code").is_string() ||
            json.at("reason_code").get<std::string>().empty())
          {
            return;
          }
          value.degraded = json.at("degraded").get<bool>();
          operational = json.at("ready").get<bool>() &&
            (state == "OK" || state == "DEGRADED");
        }
        value.stamp = stamp;
      } else if (source == "perception") {
        operational = json.at(json.contains("overall_ok") ? "overall_ok" : "ok").get<bool>() &&
          json.at(json.contains("overall_status") ? "overall_status" : "status") == "OK";
      } else if (source == "perception_heartbeat") {
        operational = json.at("ok").get<bool>();
      } else if (source == "perception_safety") {
        const auto & decision = json.at("active_decision");
        const bool stop = decision.at("stop_required").get<bool>();
        const double factor = decision.at("slowdown_factor").get<double>();
        operational = std::isfinite(factor) && factor >= 0.0 && factor <= 1.0;
        nominal = !stop && factor > 0.0;
      } else if (source == "slam") {
        operational = json.at("service_available").get<bool>() &&
          json.at("response_received").get<bool>() && json.at("response_fresh").get<bool>() &&
          json.at("healthy").get<bool>() && json.at("state_id") == 3;
      } else if (source == "head") {
        operational = json.at("operational").get<bool>() && json.at("pan_tilt_ready").get<bool>() &&
          json.at("camera_ready").get<bool>() && json.at("camera_pose_ready").get<bool>();
      } else if (source == "locations") {
        operational = json.at("component") == "savo_locations" &&
          json.at("read_ready").get<bool>() && json.at("write_ready").get<bool>() &&
          json.at("storage_healthy").get<bool>() && !json.at("mutation_in_progress").get<bool>();
      } else {return;}
    }
    value.valid = true;
    value.continuation = operational;
    value.admission = operational && nominal;
    if (!reason_explicit) {
      value.reason = source + (operational ? (nominal ? "_ready" : "_restricted") : "_unavailable");
    }
  } catch (const std::exception &) {
    value.valid = false;
  }
}

Decision Monitor::evaluate(bool mapping_ready, Clock::time_point now) const
{
  Decision result;
  result.admission_ready = mapping_ready;
  result.continuation_ready = mapping_ready;
  result.semantic_ready = mapping_ready;
  result.reason = mapping_ready ? "ready" : "mapping_scan_map_odom_tf_not_ready";

  const auto localization = observations_.find("localization");
  const auto localization_summary = observations_.find("localization_summary");
  if (localization != observations_.end() && localization_summary != observations_.end() &&
    localization->second.valid && localization_summary->second.valid &&
    localization->second.stamp == localization_summary->second.stamp &&
    (localization->second.state != localization_summary->second.state ||
    localization->second.continuation != localization_summary->second.continuation ||
    localization->second.degraded != localization_summary->second.degraded))
  {
    result.admission_ready = false;
    result.continuation_ready = false;
    result.reason = "localization_streams_inconsistent";
  }
  for (const auto & source : sources_) {
    const auto found = observations_.find(source.name);
    const Observation empty;
    const auto & observed = found == observations_.end() ? empty : found->second;
    const bool receipt_fresh = observed.receipt && now >= *observed.receipt &&
      std::chrono::duration<double>(now - *observed.receipt).count() <= source.timeout_s;
    const bool fresh = observed.valid && receipt_fresh;
    if (source.name == "head" || source.name == "locations") {
      const bool semantic_source_ready = fresh && observed.continuation;
      result.semantic_ready = result.semantic_ready && semantic_source_ready;
    }
    if (!source.required) {continue;}
    if (!fresh || !observed.continuation) {
      if (result.continuation_ready) {
        result.reason = fresh ? observed.reason : source.name + "_missing_stale_or_invalid";
      }
      result.continuation_ready = false;
      result.admission_ready = false;
    } else if (!observed.admission) {
      if (result.admission_ready) {result.reason = observed.reason;}
      result.admission_ready = false;
    }
  }
  return result;
}

std::string Decision::json() const
{
  return Json{{"schema_version", 1}, {"node", "savo_mapping"},
    {"admission_ready", admission_ready}, {"continuation_ready", continuation_ready},
    {"semantic_ready", semantic_ready}, {"reason", reason}}.dump();
}
}  // namespace savo_mapping::local_health
