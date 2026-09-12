// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_supervisor/core_payload_parser.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <iomanip>
#include <initializer_list>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_map>

#include <nlohmann/json.hpp>

namespace savo_supervisor
{
namespace
{
using Json = nlohmann::json;
using KeyValues = std::unordered_map<std::string, std::string>;

std::string uppercase(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
      return static_cast<char>(std::toupper(ch));
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

KeyValues parse_key_values(const std::string & payload)
{
  KeyValues values;
  std::string normalized = payload;
  std::replace(normalized.begin(), normalized.end(), ';', ' ');
  std::istringstream stream(normalized);
  std::string token;
  while (stream >> token) {
    const auto separator = token.find('=');
    if (separator == std::string::npos || separator == 0U) {
      continue;
    }
    values[trim(token.substr(0U, separator))] = trim(token.substr(separator + 1U));
  }
  return values;
}

bool bool_text(const std::string & text, bool & value)
{
  const auto normalized = uppercase(trim(text));
  if (normalized == "TRUE" || normalized == "1" || normalized == "YES") {
    value = true;
    return true;
  }
  if (normalized == "FALSE" || normalized == "0" || normalized == "NO") {
    value = false;
    return true;
  }
  return false;
}

ParsedCorePayload invalid(const std::string & reason, const std::string & detail)
{
  ParsedCorePayload result;
  result.reason_code = reason;
  result.detail = detail;
  return result;
}

ParsedCorePayload operational(
  const std::string & reason,
  bool degraded = false,
  const std::string & detail = {})
{
  ParsedCorePayload result;
  result.valid = true;
  result.state = degraded ? "DEGRADED" : "OK";
  result.ready = true;
  result.degraded = degraded;
  result.alive = true;
  result.reason_code = reason;
  result.detail = detail;
  return result;
}

ParsedCorePayload unavailable(
  const std::string & reason,
  const std::string & detail,
  const std::string & state = "ERROR")
{
  ParsedCorePayload result;
  result.valid = true;
  result.state = state;
  result.ready = false;
  result.degraded = false;
  result.alive = state != "ERROR";
  result.reason_code = reason;
  result.detail = detail;
  return result;
}

bool parse_json_object(const std::string & payload, Json & object, std::string & detail)
{
  if (payload.empty()) {
    detail = "payload is empty";
    return false;
  }
  object = Json::parse(payload, nullptr, false, true);
  if (object.is_discarded() || !object.is_object()) {
    detail = "payload must be a JSON object";
    return false;
  }
  return true;
}

bool valid_control_mode(const std::string & mode)
{
  const auto normalized = uppercase(mode);
  return normalized == "STOP" || normalized == "MANUAL" ||
         normalized == "AUTO" || normalized == "NAV" ||
         normalized == "RECOVERY";
}

std::string first_present(
  const KeyValues & values,
  std::initializer_list<const char *> keys)
{
  for (const auto * key : keys) {
    const auto iterator = values.find(key);
    if (iterator != values.end()) {
      return iterator->second;
    }
  }
  return {};
}

ParsedCorePayload parse_power_state(
  const std::string & raw_state,
  const std::string & source)
{
  const auto state = uppercase(raw_state);
  if (state == "OK" || state == "FULL" || state == "CHARGING") {
    return operational("power_operational", false, source + "=" + state);
  }
  if (state == "LOW" || state == "WARN" || state == "WARNING") {
    return operational("power_low", true, source + "=" + state);
  }
  if (state == "STALE") {
    return unavailable("power_stale", source + "=" + state, "STALE");
  }
  if (state == "CRITICAL") {
    return unavailable("power_critical", source + "=" + state);
  }
  if (state == "ERROR") {
    return unavailable("power_error", source + "=" + state);
  }
  return unavailable("power_unknown", source + "=" + state);
}

std::string power_source_label(const std::string & source)
{
  if (source == "base_battery") {
    return "Base battery";
  }
  if (source == "core_ups") {
    return "Core UPS";
  }
  if (source == "edge_ups") {
    return "Edge UPS";
  }
  return {};
}

ParsedCorePayload parse_direct_power_state(
  const std::string & raw_state,
  const std::string & source,
  const std::optional<double> & voltage_v,
  const bool source_ok)
{
  const auto state = uppercase(raw_state);
  const auto detail = [&source, &state, &voltage_v]() {
      std::ostringstream stream;
      stream << "source=" << source << ";state=" << state;
      if (voltage_v.has_value()) {
        stream << ";voltage_v=" << std::fixed << std::setprecision(2) << *voltage_v;
      }
      return stream.str();
    }();

  ParsedCorePayload result;
  if (state == "ERROR" || !source_ok) {
    result = unavailable(source + "_error", detail);
  } else if (state == "STALE") {
    result = unavailable(source + "_stale", detail, "STALE");
  } else if (state == "CRITICAL") {
    result = unavailable(source + "_critical", detail);
  } else if (state == "LOW" || state == "WARN" || state == "WARNING") {
    result = operational(source + "_low", true, detail);
  } else if (state == "OK" || state == "FULL" || state == "CHARGING") {
    result = operational(source + "_operational", false, detail);
  } else {
    result = unavailable(source + "_unknown", detail);
  }

  if (result.ready &&
    (!voltage_v.has_value() || !std::isfinite(*voltage_v) || *voltage_v <= 0.0))
  {
    return invalid(source + "_measurement_invalid", "missing or invalid direct voltage");
  }
  result.voltage_v = voltage_v;
  return result;
}

}  // namespace

ParsedCorePayload CorePayloadParser::ParseBaseState(const std::string & payload) const
{
  Json object;
  std::string detail;
  if (!parse_json_object(payload, object, detail)) {
    return invalid("base_message_invalid", detail);
  }

  try {
    const auto level = uppercase(object.at("status_level").get<std::string>());
    const bool connected = object.at("backend").at("connected").get<bool>();
    const auto board_error = object.at("diagnostics").value("last_board_error", std::string{});
    if (!connected || !board_error.empty() || level == "ERROR") {
      return unavailable(
        "base_backend_unavailable",
        !board_error.empty() ? board_error : "base backend disconnected or reports error");
    }
    if (level == "SAFETY_STOP") {
      if (!object.contains("safety") || !object.at("safety").is_object()) {
        return invalid(
          "base_message_invalid",
          "SAFETY_STOP requires explicit safety state");
      }
      const auto & safety = object.at("safety");
      const bool safety_stop = safety.at("safety_stop").get<bool>();
      const bool estop_latched = !safety.at("estop_latched").is_null() &&
        safety.at("estop_latched").get<bool>();
      if (estop_latched) {
        return unavailable(
          "base_emergency_stop_latched",
          "base emergency stop is latched");
      }
      if (safety_stop) {
        return operational(
          "base_motion_interlocked",
          true,
          "base output is safely blocked by the perception motion interlock");
      }
      return invalid(
        "base_message_invalid",
        "SAFETY_STOP has no active safety or emergency stop source");
    }
    if (level == "BLOCKED") {
      return operational("base_safely_blocked", true, "base output is safely blocked");
    }
    if (level == "STALE") {
      return operational(
        "base_command_stale_safe_zero", false,
        "base watchdog is forcing a normal safe-idle zero command");
    }
    if (level != "OK") {
      return invalid("base_message_invalid", "unsupported status_level: " + level);
    }
    return operational("base_operational");
  } catch (const Json::exception & exception) {
    return invalid("base_message_invalid", exception.what());
  }
}

ParsedCorePayload CorePayloadParser::ParseControlStatus(const std::string & payload) const
{
  const auto values = parse_key_values(payload);
  const auto mode = first_present(values, {"mode"});
  if (mode.empty() || !valid_control_mode(mode)) {
    return invalid("control_message_invalid", "missing or unsupported control mode");
  }

  bool safety_stop = false;
  bool external_stop = false;
  bool recovery_active = false;
  for (const auto * key : {"safety_stop", "external_stop", "recovery_active"}) {
    const auto iterator = values.find(key);
    if (iterator == values.end()) {
      continue;
    }
    bool value = false;
    if (!bool_text(iterator->second, value)) {
      return invalid("control_message_invalid", std::string("invalid boolean: ") + key);
    }
    if (std::string{key} == "safety_stop") {
      safety_stop = value;
    } else if (std::string{key} == "external_stop") {
      external_stop = value;
    } else {
      recovery_active = value;
    }
  }

  // A stale or timed-out command is a normal safe-zero condition, not node failure.
  if (external_stop) {
    return operational("control_external_stop", true, "mode=" + uppercase(mode));
  }
  if (recovery_active) {
    return operational("control_recovery_active", true, "mode=" + uppercase(mode));
  }
  if (safety_stop) {
    return operational("control_motion_interlocked", true, "mode=" + uppercase(mode));
  }
  return operational("control_operational", false, "mode=" + uppercase(mode));
}

ParsedCorePayload CorePayloadParser::ParsePerceptionHealth(const std::string & payload) const
{
  Json object;
  std::string detail;
  if (!parse_json_object(payload, object, detail)) {
    return invalid("perception_health_invalid", detail);
  }
  try {
    const bool ok = object.contains("overall_ok") ?
      object.at("overall_ok").get<bool>() : object.at("ok").get<bool>();
    const auto status = uppercase(
      object.contains("overall_status") ?
      object.at("overall_status").get<std::string>() :
      object.value("status", std::string{"UNKNOWN"}));
    if (ok && status == "OK") {
      return operational("perception_operational");
    }
    if (status == "STALE") {
      return unavailable(
        "perception_required_sensor_stale",
        "required range sensor stale",
        "STALE");
    }
    return unavailable("perception_required_sensor_error", "required range sensor unhealthy");
  } catch (const Json::exception & exception) {
    return invalid("perception_health_invalid", exception.what());
  }
}

ParsedCorePayload CorePayloadParser::ParsePerceptionSafetyState(
  const std::string & payload) const
{
  Json object;
  std::string detail;
  if (!parse_json_object(payload, object, detail)) {
    return invalid("perception_safety_invalid", detail);
  }
  try {
    const auto & decision = object.at("active_decision");
    const bool stop = decision.at("stop_required").get<bool>();
    const double slowdown = decision.at("slowdown_factor").get<double>();
    if (!std::isfinite(slowdown) || slowdown < 0.0 || slowdown > 1.0) {
      return invalid("perception_safety_invalid", "slowdown_factor outside [0,1]");
    }
    const bool degraded = stop || slowdown < 0.999;
    return operational(
      stop ? "perception_safety_stop" :
      degraded ? "perception_slowdown_active" : "perception_safety_clear",
      degraded);
  } catch (const Json::exception & exception) {
    return invalid("perception_safety_invalid", exception.what());
  }
}

ParsedCorePayload CorePayloadParser::ParsePerceptionHeartbeat(
  const std::string & payload) const
{
  Json object;
  std::string detail;
  if (!parse_json_object(payload, object, detail)) {
    return invalid("perception_heartbeat_invalid", detail);
  }
  try {
    const bool ok = object.at("ok").get<bool>();
    auto result = ok ? operational("perception_heartbeat_alive") :
      unavailable("perception_heartbeat_not_alive", "heartbeat reports ok=false");
    result.alive = ok;
    return result;
  } catch (const Json::exception & exception) {
    return invalid("perception_heartbeat_invalid", exception.what());
  }
}

ParsedCorePayload CorePayloadParser::ParseLidarState(const std::string & payload) const
{
  Json object;
  std::string detail;
  if (!parse_json_object(payload, object, detail)) {
    return invalid("lidar_state_invalid", detail);
  }
  try {
    const auto status = uppercase(object.at("status").get<std::string>());
    const bool running = object.at("driver_running").get<bool>();
    const bool hardware_ok = object.value("hardware_ok", running);
    const bool scan_ok = object.value("scan_ok", running);
    if (!running || !hardware_ok || status == "ERROR") {
      return unavailable(
        "lidar_unavailable",
        object.value("last_error", std::string{"lidar unavailable"}));
    }
    if (status == "WARN" || status == "WARNING" || !scan_ok) {
      return operational("lidar_degraded", true, "driver running with reduced scan health");
    }
    if (status != "OK") {
      return invalid("lidar_state_invalid", "unsupported lidar status: " + status);
    }
    return operational("lidar_operational");
  } catch (const Json::exception & exception) {
    return invalid("lidar_state_invalid", exception.what());
  }
}

ParsedCorePayload CorePayloadParser::ParseLidarHeartbeat(const std::string & payload) const
{
  Json object;
  std::string detail;
  if (!parse_json_object(payload, object, detail)) {
    return invalid("lidar_heartbeat_invalid", detail);
  }
  try {
    const bool running = object.at("driver_running").get<bool>();
    const auto status = uppercase(object.value("status", std::string{"UNKNOWN"}));
    auto result = running && status != "ERROR" ?
      operational(status == "WARN" ? "lidar_heartbeat_degraded" : "lidar_heartbeat_alive",
        status == "WARN") :
      unavailable(
        "lidar_heartbeat_not_alive",
        object.value("last_error", std::string{"driver not running"}));
    result.alive = running;
    return result;
  } catch (const Json::exception & exception) {
    return invalid("lidar_heartbeat_invalid", exception.what());
  }
}

ParsedCorePayload CorePayloadParser::ParsePowerStatus(const std::string & payload) const
{
  const auto values = parse_key_values(payload);
  const auto state = first_present(values, {"overall", "state"});
  if (state.empty()) {
    return invalid("power_status_invalid", "missing overall power state");
  }
  return parse_power_state(state, "overall");
}

ParsedCorePayload CorePayloadParser::ParsePowerHealth(const std::string & payload) const
{
  const auto values = parse_key_values(payload);
  const auto level = first_present(values, {"level"});
  const auto state = first_present(values, {"state", "overall"});
  if (level.empty() && state.empty()) {
    return invalid("power_health_invalid", "missing power health level/state");
  }
  const auto normalized_level = uppercase(level);
  if (normalized_level == "ERROR" || normalized_level == "UNKNOWN") {
    return unavailable("power_health_error", payload);
  }
  if (normalized_level == "WARN") {
    auto result = parse_power_state(state.empty() ? "LOW" : state, "health");
    result.degraded = true;
    result.state = "DEGRADED";
    result.ready = true;
    result.reason_code = "power_health_warning";
    return result;
  }
  return parse_power_state(state.empty() ? "OK" : state, "health");
}

ParsedCorePayload CorePayloadParser::ParsePowerSource(
  const std::string & payload,
  const std::string & expected_source) const
{
  const auto expected_label = power_source_label(expected_source);
  if (expected_label.empty()) {
    return invalid("power_source_invalid", "unsupported expected power source");
  }

  Json object;
  std::string json_detail;
  if (!payload.empty() && payload.front() == '{' &&
    parse_json_object(payload, object, json_detail))
  {
    try {
      const auto source = object.at("source").get<std::string>();
      if (source != expected_source) {
        return invalid(
          expected_source + "_source_mismatch",
          "expected " + expected_source + ", received " + source);
      }
      std::optional<double> voltage_v;
      if (object.contains("voltage_v") && !object.at("voltage_v").is_null()) {
        voltage_v = object.at("voltage_v").get<double>();
      }
      return parse_direct_power_state(
        object.at("state").get<std::string>(), expected_source, voltage_v,
        object.value("ok", true));
    } catch (const Json::exception & exception) {
      return invalid(expected_source + "_measurement_invalid", exception.what());
    }
  }

  static const std::regex line_pattern(
    R"(^\s*(Base battery|Core UPS|Edge UPS)\s+([A-Za-z]+):\s+)"
    R"((n/a|[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+))\s+V(?:,.*)?\s*$)",
    std::regex::icase);
  std::smatch match;
  if (!std::regex_match(payload, match, line_pattern)) {
    return invalid(expected_source + "_measurement_invalid", "invalid direct power payload");
  }
  if (uppercase(match[1].str()) != uppercase(expected_label)) {
    return invalid(
      expected_source + "_source_mismatch",
      "expected " + expected_label + ", received " + match[1].str());
  }
  try {
    std::optional<double> voltage_v;
    if (uppercase(match[3].str()) != "N/A") {
      voltage_v = std::stod(match[3].str());
    }
    return parse_direct_power_state(
      match[2].str(), expected_source, voltage_v, true);
  } catch (const std::exception & exception) {
    return invalid(expected_source + "_measurement_invalid", exception.what());
  }
}

}  // namespace savo_supervisor
