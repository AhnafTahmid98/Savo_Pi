#include "savo_head/core/camera_status_report.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <string>
#include <utility>

namespace savo_head
{

namespace
{

std::string upper_copy(std::string value)
{
  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](unsigned char character) {
      return static_cast<char>(
        std::toupper(character));
    });

  return value;
}

std::optional<bool> parse_bool(
  const std::string & text)
{
  const auto normalized = upper_copy(text);

  if (
    normalized == "TRUE" ||
    normalized == "1" ||
    normalized == "YES")
  {
    return true;
  }

  if (
    normalized == "FALSE" ||
    normalized == "0" ||
    normalized == "NO")
  {
    return false;
  }

  return std::nullopt;
}

ComponentHealth unavailable_health(
  const CameraStatusContext & context,
  const std::string & reason)
{
  ComponentHealth health;

  if (context.required) {
    health = error_component(
      "savo_head.camera",
      reason);
  } else {
    health = warn_component(
      "savo_head.camera",
      HeadStatus::kDryrun,
      reason);
  }

  health.add_value("seen", context.seen);
  health.add_value("required", context.required);
  health.add_value("age_s", context.age_s);
  health.add_value(
    "stale_timeout_s",
    context.stale_timeout_s);

  health.add_value("reported_status", "UNKNOWN");
  health.add_value("reported_reason", reason);
  health.add_value("raw_text", context.raw_text);

  return health;
}

void add_report_values(
  ComponentHealth & health,
  const CameraStatusContext & context,
  const CameraStatusReport & report)
{
  health.add_value("seen", context.seen);
  health.add_value("required", context.required);
  health.add_value("age_s", context.age_s);
  health.add_value(
    "stale_timeout_s",
    context.stale_timeout_s);

  health.add_value(
    "status_valid",
    report.valid);

  health.add_value(
    "reported_status",
    report.status);

  health.add_value(
    "reported_reason",
    report.reason);

  health.add_value(
    "raw_text",
    context.raw_text);

  for (const auto & field : report.fields) {
    health.add_value(
      "camera_" + field.key,
      field.value);
  }
}

}  // namespace

const char * to_string(
  CameraReportedSeverity severity)
{
  switch (severity) {
    case CameraReportedSeverity::kUnknown:
      return "UNKNOWN";

    case CameraReportedSeverity::kOk:
      return "OK";

    case CameraReportedSeverity::kWarn:
      return "WARN";

    case CameraReportedSeverity::kError:
      return "ERROR";
  }

  return "UNKNOWN";
}

std::optional<std::string> CameraStatusReport::value(
  const std::string & key) const
{
  const auto iterator = std::find_if(
    fields.begin(),
    fields.end(),
    [&key](const CameraStatusField & field) {
      return field.key == key;
    });

  if (iterator == fields.end()) {
    return std::nullopt;
  }

  return iterator->value;
}

std::optional<bool> CameraStatusReport::bool_value(
  const std::string & key) const
{
  const auto field_value = value(key);

  if (!field_value.has_value()) {
    return std::nullopt;
  }

  return parse_bool(field_value.value());
}

CameraStatusReport parse_camera_status_text(
  const std::string & text)
{
  CameraStatusReport report;

  std::istringstream stream(text);
  std::string token;

  while (stream >> token) {
    const auto separator = token.find('=');

    if (
      separator == std::string::npos ||
      separator == 0U ||
      separator + 1U >= token.size())
    {
      continue;
    }

    const auto key = token.substr(0U, separator);
    const auto value = token.substr(separator + 1U);

    report.fields.push_back(
      CameraStatusField{key, value});
  }

  const auto status_value = report.value("status");
  const auto reason_value = report.value("reason");

  if (status_value.has_value()) {
    report.status =
      upper_copy(status_value.value());
  }

  if (reason_value.has_value()) {
    report.reason = reason_value.value();
  }

  if (report.status == "OK") {
    report.severity = CameraReportedSeverity::kOk;
  } else if (report.status == "WARN") {
    report.severity = CameraReportedSeverity::kWarn;
  } else if (report.status == "ERROR") {
    report.severity = CameraReportedSeverity::kError;
  } else {
    report.severity = CameraReportedSeverity::kUnknown;
  }

  report.valid =
    status_value.has_value() &&
    reason_value.has_value() &&
    report.severity != CameraReportedSeverity::kUnknown;

  if (!report.valid) {
    report.status = "UNKNOWN";
    report.reason = "camera_status_invalid";
  }

  return report;
}

ComponentHealth camera_status_health(
  const CameraStatusContext & context)
{
  if (!context.seen) {
    return unavailable_health(
      context,
      "camera_status_missing");
  }

  if (
    !std::isfinite(context.age_s) ||
    !std::isfinite(context.stale_timeout_s) ||
    context.stale_timeout_s <= 0.0)
  {
    auto health = error_component(
      "savo_head.camera",
      "camera_status_time_invalid");

    const auto report =
      parse_camera_status_text(context.raw_text);

    add_report_values(health, context, report);

    return health;
  }

  if (context.age_s > context.stale_timeout_s) {
    return unavailable_health(
      context,
      "camera_status_stale");
  }

  const auto report =
    parse_camera_status_text(context.raw_text);

  if (!report.valid) {
    auto health = error_component(
      "savo_head.camera",
      "camera_status_invalid");

    add_report_values(health, context, report);

    return health;
  }

  ComponentHealth health;

  switch (report.severity) {
    case CameraReportedSeverity::kOk:
      health = ok_component(
        "savo_head.camera",
        report.reason);
      break;

    case CameraReportedSeverity::kWarn:
      health = ComponentHealth{
        "savo_head.camera",
        HeadStatus::kOk,
        DiagnosticLevel::kWarn,
        report.reason,
        {}
      };
      break;

    case CameraReportedSeverity::kError:
      health = error_component(
        "savo_head.camera",
        report.reason);
      break;

    case CameraReportedSeverity::kUnknown:
      health = error_component(
        "savo_head.camera",
        "camera_status_invalid");
      break;
  }

  add_report_values(health, context, report);

  return health;
}

}  // namespace savo_head
