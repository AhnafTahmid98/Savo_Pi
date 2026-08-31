// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_supervisor/edge_payload_parser.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <string>

#include <nlohmann/json.hpp>

namespace savo_supervisor
{
namespace
{
using Json = nlohmann::json;

std::string lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
  });
  return value;
}

std::string trim(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1U);
}

bool required_boolean(const Json & json, const char * key, bool & value)
{
  if (!json.contains(key) || !json.at(key).is_boolean()) {
    return false;
  }
  value = json.at(key).get<bool>();
  return true;
}

}  // namespace

BridgeObservation EdgePayloadParser::ParseBridgeState(
  const std::string & payload) const
{
  BridgeObservation result;
  result.received = true;
  try {
    const auto json = Json::parse(payload);
    if (!json.is_object() ||
      json.value("schema_name", std::string{}) != "savo_bridge_state" ||
      json.value("schema_version", 0) != 2)
    {
      result.reason = "bridge_schema_invalid";
      return result;
    }
    const bool fields_valid =
      required_boolean(json, "process_alive", result.process_alive) &&
      required_boolean(json, "bridge_ready", result.bridge_ready) &&
      required_boolean(json, "commands_enabled", result.commands_enabled) &&
      required_boolean(json, "dds_active", result.dds_active) &&
      required_boolean(json, "core_visible", result.core_visible) &&
      required_boolean(json, "edge_visible", result.edge_visible) &&
      required_boolean(json, "stop_ready", result.stop_ready) &&
      required_boolean(json, "teleop_ready", result.teleop_ready) &&
      required_boolean(json, "navigation_ready", result.navigation_ready);
    if (!fields_valid) {
      result.reason = "bridge_state_missing_fields";
      return result;
    }
    result.valid = true;
    result.reason = json.value("readiness_reason", std::string{"bridge_reason_missing"});
  } catch (const std::exception & exception) {
    result.reason = std::string{"bridge_state_invalid:"} + exception.what();
  }
  return result;
}

RealSenseObservation EdgePayloadParser::ParseRealSenseStatus(
  const std::string & payload) const
{
  RealSenseObservation result;
  result.received = true;
  try {
    const auto json = Json::parse(payload);
    if (!json.is_object()) {
      result.reason = "realsense_status_not_object";
      return result;
    }
    bool pointcloud_ok = false;
    bool require_pointcloud = false;
    const bool fields_valid =
      required_boolean(json, "ok", result.healthy) &&
      required_boolean(json, "color_ok", result.color_ready) &&
      required_boolean(json, "depth_ok", result.depth_ready) &&
      required_boolean(json, "pointcloud_ok", pointcloud_ok) &&
      required_boolean(json, "require_pointcloud", require_pointcloud);
    if (!fields_valid) {
      result.reason = "realsense_status_missing_fields";
      return result;
    }
    result.pointcloud_ready = !require_pointcloud || pointcloud_ok;
    result.valid = true;
    result.reason = json.value(
      "message", result.healthy ? std::string{"realsense_operational"} :
      std::string{"realsense_unhealthy"});
  } catch (const std::exception & exception) {
    result.reason = std::string{"realsense_status_invalid:"} + exception.what();
  }
  return result;
}

VoiceObservation EdgePayloadParser::ParseSpeechReadiness(
  const std::string & payload) const
{
  VoiceObservation result;
  result.received = true;
  result.state = lower(payload);
  if (result.state == "ready") {
    result.valid = true;
    result.ready = true;
    result.reason = "speech_ready";
  } else if (result.state == "waiting_for_audio") {
    result.valid = true;
    result.reason = "speech_waiting_for_audio";
  } else if (result.state == "disabled") {
    result.valid = true;
    result.reason = "speech_disabled";
  } else if (result.state == "error") {
    result.valid = true;
    result.reason = "speech_error";
  } else {
    result.reason = "speech_readiness_invalid";
  }
  return result;
}

VisualOdometryObservation EdgePayloadParser::ParseVoHealth(
  const std::string & payload) const
{
  VisualOdometryObservation result;
  result.received = true;
  const auto normalized = lower(trim(payload));
  const auto separator = normalized.find(':');
  result.state = trim(
    separator == std::string::npos ? normalized : normalized.substr(0U, separator));
  result.reason = trim(
    separator == std::string::npos ? normalized : normalized.substr(separator + 1U));
  if (result.state == "ok") {
    result.valid = true;
    result.ready = true;
  } else if (result.state == "degraded") {
    result.valid = true;
    result.degraded = true;
  } else if (result.state == "stale") {
    result.valid = true;
    result.degraded = true;
  } else if (result.state == "waiting" || result.state == "error") {
    result.valid = true;
  } else {
    result.reason = "vo_health_invalid";
  }
  return result;
}

}  // namespace savo_supervisor
