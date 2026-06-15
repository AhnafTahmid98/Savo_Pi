#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

#include "savo_lidar/visibility_control.hpp"

namespace savo_lidar
{

struct SAVO_LIDAR_PUBLIC SerialConfig
{
  std::string port{"/dev/ttyUSB0"};
  int baudrate{115200};
  double timeout_s{1.0};

  bool reconnect_on_error{true};
  double reconnect_delay_s{0.5};
  double max_reconnect_delay_s{5.0};

  void validate() const
  {
    if (port.empty()) {
      throw std::invalid_argument("serial port cannot be empty");
    }

    if (baudrate <= 0) {
      throw std::invalid_argument("baudrate must be > 0");
    }

    if (timeout_s <= 0.0) {
      throw std::invalid_argument("timeout_s must be > 0");
    }

    if (reconnect_delay_s < 0.0) {
      throw std::invalid_argument("reconnect_delay_s cannot be negative");
    }

    if (max_reconnect_delay_s < reconnect_delay_s) {
      throw std::invalid_argument("max_reconnect_delay_s cannot be less than reconnect_delay_s");
    }
  }
};

struct SAVO_LIDAR_PUBLIC RplidarConfig
{
  SerialConfig serial;

  std::string frame_id{"laser"};
  std::string scan_topic{"/scan"};
  std::string scan_mode{"standard"};

  double expected_scan_rate_hz{5.5};
  double motor_start_settle_s{1.0};
  double motor_stop_timeout_s{1.0};

  float min_range_m{0.15F};
  float max_range_m{12.0F};

  bool inverted{false};
  double angle_offset_rad{0.0};

  void validate() const
  {
    serial.validate();

    if (frame_id.empty()) {
      throw std::invalid_argument("frame_id cannot be empty");
    }

    if (scan_topic.empty()) {
      throw std::invalid_argument("scan_topic cannot be empty");
    }

    if (scan_mode.empty()) {
      throw std::invalid_argument("scan_mode cannot be empty");
    }

    if (expected_scan_rate_hz <= 0.0) {
      throw std::invalid_argument("expected_scan_rate_hz must be > 0");
    }

    if (motor_start_settle_s < 0.0) {
      throw std::invalid_argument("motor_start_settle_s cannot be negative");
    }

    if (motor_stop_timeout_s < 0.0) {
      throw std::invalid_argument("motor_stop_timeout_s cannot be negative");
    }

    if (min_range_m <= 0.0F) {
      throw std::invalid_argument("min_range_m must be > 0");
    }

    if (max_range_m <= min_range_m) {
      throw std::invalid_argument("max_range_m must be greater than min_range_m");
    }
  }
};

}  // namespace savo_lidar