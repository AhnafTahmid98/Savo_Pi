#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "savo_head/core/diagnostics.hpp"

namespace savo_head
{

enum class CameraReportedSeverity
{
  kUnknown,
  kOk,
  kWarn,
  kError
};

struct CameraStatusField
{
  std::string key;
  std::string value;
};

struct CameraStatusReport
{
  bool valid{false};

  CameraReportedSeverity severity{
    CameraReportedSeverity::kUnknown};

  std::string status{"UNKNOWN"};
  std::string reason{"camera_status_invalid"};

  std::vector<CameraStatusField> fields{};

  [[nodiscard]] std::optional<std::string> value(
    const std::string & key) const;

  [[nodiscard]] std::optional<bool> bool_value(
    const std::string & key) const;
};

struct CameraStatusContext
{
  bool seen{false};
  bool required{false};

  double age_s{0.0};
  double stale_timeout_s{2.0};

  std::string raw_text{};
};

[[nodiscard]] const char * to_string(
  CameraReportedSeverity severity);

[[nodiscard]] CameraStatusReport parse_camera_status_text(
  const std::string & text);

[[nodiscard]] ComponentHealth camera_status_health(
  const CameraStatusContext & context);

}  // namespace savo_head
