// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_supervisor/mission_payload_parser.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <exception>
#include <sstream>
#include <string>
#include <unordered_map>

#include <nlohmann/json.hpp>

namespace savo_supervisor
{
namespace
{
using Json = nlohmann::json;

std::string upper(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
      return static_cast<char>(std::toupper(ch));
  });
  return value;
}

std::unordered_map<std::string, std::string> parse_fields(const std::string & payload)
{
  std::unordered_map<std::string, std::string> fields;
  std::stringstream stream(payload);
  std::string field;
  while (std::getline(stream, field, ';')) {
    const auto separator = field.find('=');
    if (separator == std::string::npos || separator == 0U) {
      continue;
    }
    fields[field.substr(0U, separator)] = field.substr(separator + 1U);
  }
  return fields;
}

bool parse_bool(const std::string & value, bool & output)
{
  if (value == "true") {
    output = true;
    return true;
  }
  if (value == "false") {
    output = false;
    return true;
  }
  return false;
}

std::string diagnostic_value(
  const diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key)
{
  const auto iterator = std::find_if(
    status.values.begin(), status.values.end(), [&key](const auto & value) {
      return value.key == key;
    });
  return iterator == status.values.end() ? std::string{} : iterator->value;
}

bool text_ready(const std::string & value)
{
  const auto normalized = upper(value);
  return normalized == "OK" || normalized == "READY" || normalized == "STREAMING";
}

}  // namespace

MappingObservation MissionPayloadParser::ParseMappingStatus(
  const std::string & payload) const
{
  MappingObservation result;
  result.received = true;
  try {
    const auto json = Json::parse(payload);
    if (!json.is_object()) {
      result.reason = "mapping_status_not_object";
      return result;
    }
    const std::array<const char *, 10> required{
      "mode", "workflow_phase", "session_state", "healthy", "ready",
      "slam_active", "map_received", "scan_received", "tf_ok", "odom_ok"};
    for (const auto * key : required) {
      if (!json.contains(key)) {
        result.reason = std::string{"mapping_status_missing_"} + key;
        return result;
      }
    }
    result.mode = json.at("mode").get<std::string>();
    result.workflow_phase = json.at("workflow_phase").get<std::string>();
    result.session_state = json.at("session_state").get<std::string>();
    result.healthy = json.at("healthy").get<bool>();
    result.ready = json.at("ready").get<bool>();
    result.slam_active = json.at("slam_active").get<bool>();
    result.active_map_name = json.value("active_map_name", std::string{});
    result.valid = true;
    result.reason = result.healthy ? "mapping_status_operational" : "mapping_unhealthy";
  } catch (const std::exception & exception) {
    result.reason = std::string{"mapping_status_invalid:"} + exception.what();
  }
  return result;
}

NavigationObservation MissionPayloadParser::ParseNavigationStatus(
  const std::string & payload) const
{
  NavigationObservation result;
  result.received = true;
  const auto fields = parse_fields(payload);
  const auto state = fields.find("state");
  const auto acceptance = fields.find("goal_acceptance_allowed");
  const auto reason = fields.find("reason");
  if (state == fields.end() || acceptance == fields.end() || reason == fields.end()) {
    result.reason = "navigation_status_missing_fields";
    return result;
  }
  bool allowed = false;
  if (!parse_bool(acceptance->second, allowed)) {
    result.reason = "navigation_goal_acceptance_invalid";
    return result;
  }
  result.state = upper(state->second);
  result.goal_acceptance_allowed = allowed;
  result.ready = result.state == "READY" && allowed;
  result.valid = true;
  result.reason = reason->second.empty() ? "navigation_reason_empty" : reason->second;
  return result;
}

LocationsObservation MissionPayloadParser::ParseLocationsStatus(
  const std::string & payload) const
{
  LocationsObservation result;
  result.received = true;
  try {
    const auto json = Json::parse(payload);
    if (!json.is_object() || json.value("component", std::string{}) != "savo_locations") {
      result.reason = "locations_status_component_invalid";
      return result;
    }
    result.read_ready = json.at("read_ready").get<bool>();
    result.write_ready = json.at("write_ready").get<bool>();
    result.storage_healthy = json.at("storage_healthy").get<bool>();
    result.mutation_in_progress = json.at("mutation_in_progress").get<bool>();
    result.valid = true;
    result.reason = json.value("reason", std::string{"locations_status_operational"});
  } catch (const std::exception & exception) {
    result.reason = std::string{"locations_status_invalid:"} + exception.what();
  }
  return result;
}

HeadObservation MissionPayloadParser::ParseHeadStatus(
  const diagnostic_msgs::msg::DiagnosticArray & message) const
{
  HeadObservation result;
  result.received = true;
  const auto iterator = std::find_if(
    message.status.begin(), message.status.end(), [](const auto & status) {
      return status.name == "savo_head.head_status";
    });
  if (iterator == message.status.end()) {
    result.reason = "head_aggregate_status_missing";
    return result;
  }
  const auto & status = *iterator;
  result.operational = status.level <= diagnostic_msgs::msg::DiagnosticStatus::WARN;
  result.pan_tilt_ready = text_ready(diagnostic_value(status, "pan_tilt_state"));
  result.camera_ready = diagnostic_value(status, "camera_stream_healthy") == "true";
  result.camera_pose_ready = diagnostic_value(status, "camera_pose_ready") == "true";
  result.valid = true;
  result.reason = status.message.empty() ? "head_status_operational" : status.message;
  return result;
}

}  // namespace savo_supervisor
