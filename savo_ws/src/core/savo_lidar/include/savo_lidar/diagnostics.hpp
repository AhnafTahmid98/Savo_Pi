#pragma once

#include <cstdint>
#include <string>

#include "savo_lidar/rplidar_protocol.hpp"
#include "savo_lidar/scan_types.hpp"
#include "savo_lidar/visibility_control.hpp"

namespace savo_lidar
{

constexpr const char * STATUS_OK = "OK";
constexpr const char * STATUS_WARN = "WARN";
constexpr const char * STATUS_ERROR = "ERROR";
constexpr const char * STATUS_OFFLINE = "OFFLINE";

struct SAVO_LIDAR_PUBLIC DriverDiagnostics
{
  std::string component{"lidar_driver_node"};
  std::string status{STATUS_OFFLINE};
  std::string message{"driver not started"};

  bool hardware_ok{false};
  bool scan_ok{false};
  bool driver_running{false};

  std::uint64_t scan_count{0};

  double scan_rate_hz{0.0};
  double valid_ratio{0.0};

  std::size_t total_points{0};
  std::size_t valid_points{0};

  float min_range_m{0.0F};
  float max_range_m{0.0F};
  float mean_range_m{0.0F};

  std::string backend{"real"};
  std::string model{"rplidar_a1"};
  std::string frame_id{"laser"};
  std::string scan_topic{"/scan"};
  std::string serial_port{"/dev/ttyUSB0"};

  std::string last_error;
};

SAVO_LIDAR_PUBLIC ScanStats compute_scan_stats(const LidarScan & scan);

SAVO_LIDAR_PUBLIC std::string health_status_to_string(RplidarHealthStatus status);

SAVO_LIDAR_PUBLIC std::string driver_diagnostics_to_json(
  const DriverDiagnostics & diagnostics);

SAVO_LIDAR_PUBLIC std::string make_driver_heartbeat_json(
  const std::string & component,
  const std::string & status,
  const std::string & message,
  bool driver_running,
  std::uint64_t scan_count,
  const std::string & last_error);

SAVO_LIDAR_PUBLIC DriverDiagnostics make_ok_driver_diagnostics(
  const LidarScan & scan,
  std::uint64_t scan_count,
  double scan_rate_hz,
  const std::string & frame_id,
  const std::string & scan_topic,
  const std::string & serial_port);

SAVO_LIDAR_PUBLIC DriverDiagnostics make_error_driver_diagnostics(
  const std::string & message,
  bool hardware_ok,
  bool driver_running,
  std::uint64_t scan_count,
  const std::string & frame_id,
  const std::string & scan_topic,
  const std::string & serial_port);

}  // namespace savo_lidar