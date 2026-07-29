// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_supervisor/localization_payload_parser.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <string>

#include <nlohmann/json.hpp>

#include "savo_supervisor/reason_codes.hpp"

namespace savo_supervisor
{
namespace
{

using Json = nlohmann::json;

bool valid_localization_state(
  const std::string & state)
{
  static const std::set<std::string> valid_states{
    "INITIALIZING",
    "OK",
    "DEGRADED",
    "STALE",
    "ERROR"};

  return valid_states.count(state) != 0U;
}

bool read_required_integer(
  const Json & object,
  const char * key,
  int & value,
  std::string & detail)
{
  const auto iterator = object.find(key);

  if (iterator == object.end()) {
    detail = std::string("missing required field: ") + key;
    return false;
  }

  if (!iterator->is_number_integer()) {
    detail = std::string("field must be an integer: ") + key;
    return false;
  }

  try {
    value = iterator->get<int>();
  } catch (const Json::exception &) {
    detail = std::string("integer field is out of range: ") + key;
    return false;
  }

  return true;
}

bool read_required_boolean(
  const Json & object,
  const char * key,
  bool & value,
  std::string & detail)
{
  const auto iterator = object.find(key);

  if (iterator == object.end()) {
    detail = std::string("missing required field: ") + key;
    return false;
  }

  if (!iterator->is_boolean()) {
    detail = std::string("field must be a boolean: ") + key;
    return false;
  }

  value = iterator->get<bool>();
  return true;
}

bool read_required_string(
  const Json & object,
  const char * key,
  std::string & value,
  std::string & detail,
  bool allow_empty = false)
{
  const auto iterator = object.find(key);

  if (iterator == object.end()) {
    detail = std::string("missing required field: ") + key;
    return false;
  }

  if (!iterator->is_string()) {
    detail = std::string("field must be a string: ") + key;
    return false;
  }

  value = iterator->get<std::string>();

  if (!allow_empty && value.empty()) {
    detail = std::string("field must not be empty: ") + key;
    return false;
  }

  return true;
}

bool read_optional_string(
  const Json & object,
  const char * key,
  std::string & value,
  std::string & detail)
{
  const auto iterator = object.find(key);

  if (iterator == object.end()) {
    value.clear();
    return true;
  }

  if (!iterator->is_string()) {
    detail = std::string("field must be a string: ") + key;
    return false;
  }

  value = iterator->get<std::string>();
  return true;
}

bool convert_stamp(
  const Json & object,
  builtin_interfaces::msg::Time & stamp,
  std::string & detail)
{
  const auto iterator = object.find("stamp_s");

  if (iterator == object.end()) {
    detail = "missing required field: stamp_s";
    return false;
  }

  if (!iterator->is_number()) {
    detail = "field must be numeric: stamp_s";
    return false;
  }

  double seconds = 0.0;

  try {
    seconds = iterator->get<double>();
  } catch (const Json::exception &) {
    detail = "timestamp is out of range";
    return false;
  }

  if (!std::isfinite(seconds)) {
    detail = "timestamp must be finite";
    return false;
  }

  if (seconds < 0.0) {
    detail = "timestamp must be non-negative";
    return false;
  }

  double integral_part = 0.0;
  const double fractional_part =
    std::modf(seconds, &integral_part);

  constexpr double maximum_seconds =
    static_cast<double>(
    std::numeric_limits<int32_t>::max());

  if (integral_part > maximum_seconds) {
    detail = "timestamp seconds exceed builtin time range";
    return false;
  }

  int64_t seconds_integer =
    static_cast<int64_t>(integral_part);

  int64_t nanoseconds =
    static_cast<int64_t>(
    std::llround(fractional_part * 1e9));

  if (nanoseconds >= 1000000000LL) {
    ++seconds_integer;
    nanoseconds = 0;
  }

  if (
    seconds_integer >
    static_cast<int64_t>(
      std::numeric_limits<int32_t>::max()))
  {
    detail = "timestamp rounding exceeds builtin time range";
    return false;
  }

  stamp.sec =
    static_cast<int32_t>(seconds_integer);

  stamp.nanosec =
    static_cast<uint32_t>(nanoseconds);

  return true;
}

Json parse_strict_json(
  const std::string & payload,
  bool & duplicate_top_level_key,
  std::string & duplicate_key)
{
  std::set<std::string> top_level_keys;

  const Json::parser_callback_t callback =
    [&top_level_keys,
      &duplicate_top_level_key,
      &duplicate_key](
    int depth,
    Json::parse_event_t event,
    Json & parsed)
    {
      if (
        event == Json::parse_event_t::key &&
        depth == 1 &&
        parsed.is_string())
      {
        const auto key =
          parsed.get<std::string>();

        if (!top_level_keys.insert(key).second) {
          duplicate_top_level_key = true;
          duplicate_key = key;
          return false;
        }
      }

      return true;
    };

  return Json::parse(
    payload,
    callback,
    false,
    true);
}

}  // namespace

LocalizationPayloadParser::LocalizationPayloadParser(
  int expected_schema_version)
: expected_schema_version_(expected_schema_version)
{
}

ParsedLocalizationPayload
LocalizationPayloadParser::ParseHealth(
  const std::string & payload) const
{
  return Parse(
    payload,
    LocalizationPayloadKind::HEALTH);
}

ParsedLocalizationPayload
LocalizationPayloadParser::ParseSummary(
  const std::string & payload) const
{
  return Parse(
    payload,
    LocalizationPayloadKind::SUMMARY);
}

ParsedLocalizationPayload
LocalizationPayloadParser::ParseHeartbeat(
  const std::string & payload) const
{
  return Parse(
    payload,
    LocalizationPayloadKind::HEARTBEAT);
}

ParsedLocalizationPayload
LocalizationPayloadParser::Parse(
  const std::string & payload,
  LocalizationPayloadKind kind) const
{
  ParsedLocalizationPayload result;

  if (payload.empty()) {
    result.reason_code =
      reason::kLocalizationMessageInvalid;
    result.detail = "payload is empty";
    return result;
  }

  bool duplicate_top_level_key = false;
  std::string duplicate_key;

  Json object;

  try {
    object = parse_strict_json(
      payload,
      duplicate_top_level_key,
      duplicate_key);
  } catch (const Json::exception & exception) {
    result.reason_code =
      reason::kLocalizationMessageInvalid;

    result.detail =
      std::string("JSON parser error: ") +
      exception.what();

    return result;
  }

  if (object.is_discarded()) {
    result.reason_code =
      reason::kLocalizationMessageInvalid;
    result.detail = "payload is not valid JSON";
    return result;
  }

  if (duplicate_top_level_key) {
    result.reason_code =
      reason::kLocalizationMessageInvalid;

    result.detail =
      "duplicate top-level field: " +
      duplicate_key;

    return result;
  }

  if (!object.is_object()) {
    result.reason_code =
      reason::kLocalizationMessageInvalid;
    result.detail = "payload must be a JSON object";
    return result;
  }

  int schema_version = 0;

  if (!read_required_integer(
      object,
      "schema_version",
      schema_version,
      result.detail))
  {
    result.reason_code =
      reason::kLocalizationMessageInvalid;
    return result;
  }

  if (schema_version != expected_schema_version_) {
    result.schema_supported = false;

    result.reason_code =
      reason::kLocalizationSchemaUnsupported;

    result.detail =
      "unsupported schema version: " +
      std::to_string(schema_version);

    return result;
  }

  if (!read_required_string(
      object,
      "state",
      result.state,
      result.detail))
  {
    result.reason_code =
      reason::kLocalizationMessageInvalid;
    return result;
  }

  if (!valid_localization_state(result.state)) {
    result.reason_code =
      reason::kLocalizationMessageInvalid;

    result.detail =
      "unsupported localization state: " +
      result.state;

    return result;
  }

  if (!read_required_boolean(
      object,
      "ready",
      result.ready,
      result.detail))
  {
    result.reason_code =
      reason::kLocalizationMessageInvalid;
    return result;
  }

  if (kind == LocalizationPayloadKind::HEALTH ||
    kind == LocalizationPayloadKind::SUMMARY)
  {
    if (!read_required_boolean(
        object,
        "degraded",
        result.degraded,
        result.detail))
    {
      result.reason_code =
        reason::kLocalizationMessageInvalid;
      return result;
    }

    if (!read_required_string(
        object,
        "reason_code",
        result.reason_code,
        result.detail))
    {
      result.reason_code =
        reason::kLocalizationMessageInvalid;
      return result;
    }
  }

  if (kind == LocalizationPayloadKind::HEARTBEAT) {
    if (!read_required_boolean(
        object,
        "alive",
        result.alive,
        result.detail))
    {
      result.reason_code =
        reason::kLocalizationMessageInvalid;
      return result;
    }

    if (!read_optional_string(
        object,
        "reason_code",
        result.reason_code,
        result.detail))
    {
      result.reason_code =
        reason::kLocalizationMessageInvalid;
      return result;
    }
  }

  builtin_interfaces::msg::Time stamp;

  if (!convert_stamp(
      object,
      stamp,
      result.detail))
  {
    result.reason_code =
      reason::kLocalizationMessageInvalid;
    return result;
  }

  result.stamp = stamp;
  result.valid = true;
  result.schema_supported = true;
  result.detail.clear();

  return result;
}

}  // namespace savo_supervisor
