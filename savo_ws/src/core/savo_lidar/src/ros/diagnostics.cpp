#include "savo_lidar/diagnostics.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace savo_lidar
{
namespace
{

std::string json_escape(const std::string & value)
{
  std::ostringstream out;

  for (const auto ch : value) {
    switch (ch) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }

  return out.str();
}

std::string json_string(const std::string & value)
{
  return "\"" + json_escape(value) + "\"";
}

std::string json_bool(bool value)
{
  return value ? "true" : "false";
}

std::string json_number(double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }

  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << value;
  return out.str();
}

std::string json_number(float value)
{
  return json_number(static_cast<double>(value));
}

}  // namespace

ScanStats compute_scan_stats(const LidarScan & scan)
{
  ScanStats stats;

  stats.total_points = scan.ranges_m.size();

  if (scan.ranges_m.empty()) {
    return stats;
  }

  float min_seen = std::numeric_limits<float>::infinity();
  float max_seen = -std::numeric_limits<float>::infinity();
  double sum = 0.0;

  for (const auto range_m : scan.ranges_m) {
    if (!is_valid_range(range_m, scan.range_min_m, scan.range_max_m)) {
      continue;
    }

    ++stats.valid_points;
    min_seen = std::min(min_seen, range_m);
    max_seen = std::max(max_seen, range_m);
    sum += static_cast<double>(range_m);
  }

  stats.valid_ratio =
    static_cast<double>(stats.valid_points) / static_cast<double>(stats.total_points);

  if (stats.valid_points > 0U) {
    stats.min_range_m = min_seen;
    stats.max_range_m = max_seen;
    stats.mean_range_m = static_cast<float>(sum / static_cast<double>(stats.valid_points));
  }

  if (scan.scan_time_s > 0.0) {
    stats.scan_rate_hz = 1.0 / scan.scan_time_s;
  }

  return stats;
}

std::string health_status_to_string(RplidarHealthStatus status)
{
  switch (status) {
    case RplidarHealthStatus::ok:
      return "OK";
    case RplidarHealthStatus::warning:
      return "WARN";
    case RplidarHealthStatus::error:
      return "ERROR";
    case RplidarHealthStatus::unknown:
    default:
      return "UNKNOWN";
  }
}

std::string driver_diagnostics_to_json(const DriverDiagnostics & diagnostics)
{
  std::ostringstream out;

  out << "{"
      << "\"component\":" << json_string(diagnostics.component) << ","
      << "\"status\":" << json_string(diagnostics.status) << ","
      << "\"message\":" << json_string(diagnostics.message) << ","
      << "\"hardware_ok\":" << json_bool(diagnostics.hardware_ok) << ","
      << "\"scan_ok\":" << json_bool(diagnostics.scan_ok) << ","
      << "\"driver_running\":" << json_bool(diagnostics.driver_running) << ","
      << "\"scan_count\":" << diagnostics.scan_count << ","
      << "\"scan_rate_hz\":" << json_number(diagnostics.scan_rate_hz) << ","
      << "\"valid_ratio\":" << json_number(diagnostics.valid_ratio) << ","
      << "\"total_points\":" << diagnostics.total_points << ","
      << "\"valid_points\":" << diagnostics.valid_points << ","
      << "\"min_range_m\":" << json_number(diagnostics.min_range_m) << ","
      << "\"max_range_m\":" << json_number(diagnostics.max_range_m) << ","
      << "\"mean_range_m\":" << json_number(diagnostics.mean_range_m) << ","
      << "\"backend\":" << json_string(diagnostics.backend) << ","
      << "\"model\":" << json_string(diagnostics.model) << ","
      << "\"frame_id\":" << json_string(diagnostics.frame_id) << ","
      << "\"scan_topic\":" << json_string(diagnostics.scan_topic) << ","
      << "\"serial_port\":" << json_string(diagnostics.serial_port) << ","
      << "\"last_error\":" << json_string(diagnostics.last_error)
      << "}";

  return out.str();
}

std::string make_driver_heartbeat_json(
  const std::string & component,
  const std::string & status,
  const std::string & message,
  bool driver_running,
  std::uint64_t scan_count,
  const std::string & last_error)
{
  std::ostringstream out;

  out << "{"
      << "\"component\":" << json_string(component) << ","
      << "\"status\":" << json_string(status) << ","
      << "\"message\":" << json_string(message) << ","
      << "\"driver_running\":" << json_bool(driver_running) << ","
      << "\"scan_count\":" << scan_count << ","
      << "\"last_error\":" << json_string(last_error)
      << "}";

  return out.str();
}

DriverDiagnostics make_ok_driver_diagnostics(
  const LidarScan & scan,
  std::uint64_t scan_count,
  double scan_rate_hz,
  const std::string & frame_id,
  const std::string & scan_topic,
  const std::string & serial_port)
{
  const auto stats = compute_scan_stats(scan);

  DriverDiagnostics diagnostics;
  diagnostics.status = STATUS_OK;
  diagnostics.message = "LiDAR driver running";
  diagnostics.hardware_ok = true;
  diagnostics.scan_ok = stats.valid_points > 0U;
  diagnostics.driver_running = true;
  diagnostics.scan_count = scan_count;
  diagnostics.scan_rate_hz = scan_rate_hz;
  diagnostics.valid_ratio = stats.valid_ratio;
  diagnostics.total_points = stats.total_points;
  diagnostics.valid_points = stats.valid_points;
  diagnostics.min_range_m = stats.min_range_m;
  diagnostics.max_range_m = stats.max_range_m;
  diagnostics.mean_range_m = stats.mean_range_m;
  diagnostics.frame_id = frame_id;
  diagnostics.scan_topic = scan_topic;
  diagnostics.serial_port = serial_port;

  return diagnostics;
}

DriverDiagnostics make_error_driver_diagnostics(
  const std::string & message,
  bool hardware_ok,
  bool driver_running,
  std::uint64_t scan_count,
  const std::string & frame_id,
  const std::string & scan_topic,
  const std::string & serial_port)
{
  DriverDiagnostics diagnostics;
  diagnostics.status = STATUS_ERROR;
  diagnostics.message = message;
  diagnostics.hardware_ok = hardware_ok;
  diagnostics.scan_ok = false;
  diagnostics.driver_running = driver_running;
  diagnostics.scan_count = scan_count;
  diagnostics.frame_id = frame_id;
  diagnostics.scan_topic = scan_topic;
  diagnostics.serial_port = serial_port;
  diagnostics.last_error = message;

  return diagnostics;
}

}  // namespace savo_lidar