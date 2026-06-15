#include "savo_lidar/rplidar_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace savo_lidar
{
namespace
{

double elapsed_s(const std::chrono::steady_clock::time_point & start)
{
  const auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now - start).count();
}

double remaining_timeout_s(
  const std::chrono::steady_clock::time_point & start,
  double timeout_s)
{
  return std::max(0.001, timeout_s - elapsed_s(start));
}

bool timeout_reached(
  const std::chrono::steady_clock::time_point & start,
  double timeout_s)
{
  return elapsed_s(start) >= timeout_s;
}

}  // namespace

RplidarDriver::RplidarDriver() = default;

RplidarDriver::RplidarDriver(const RplidarConfig & config)
: config_(config)
{
  config_.validate();
}

RplidarDriver::~RplidarDriver()
{
  stop();
}

void RplidarDriver::configure(const RplidarConfig & config)
{
  if (running()) {
    throw std::runtime_error("cannot reconfigure RPLIDAR while driver is running");
  }

  config.validate();
  config_ = config;
}

void RplidarDriver::start()
{
  if (running()) {
    return;
  }

  try {
    config_.validate();

    state_ = DriverState::starting;
    last_error_.clear();

    serial_.open(config_.serial);

    if (config_.motor_start_settle_s > 0.0) {
      const auto settle = std::chrono::duration<double>(config_.motor_start_settle_s);
      std::this_thread::sleep_for(
        std::chrono::duration_cast<std::chrono::milliseconds>(settle));
    }

    begin_scan();

    scan_start_time_ = std::chrono::steady_clock::now();
    state_ = DriverState::running;
  } catch (const std::exception & exc) {
    mark_error(exc.what());
    serial_.close();
    throw;
  }
}

void RplidarDriver::stop() noexcept
{
  try {
    if (serial_.is_open()) {
      send_command(RPLIDAR_CMD_STOP);
    }
  } catch (...) {
  }

  serial_.close();
  state_ = DriverState::stopped;
}

void RplidarDriver::reset()
{
  try {
    if (!serial_.is_open()) {
      serial_.open(config_.serial);
    }

    send_command(RPLIDAR_CMD_RESET);
    state_ = DriverState::stopped;
  } catch (const std::exception & exc) {
    mark_error(exc.what());
    throw;
  }
}

bool RplidarDriver::running() const noexcept
{
  return state_ == DriverState::running;
}

DriverState RplidarDriver::state() const noexcept
{
  return state_;
}

const RplidarConfig & RplidarDriver::config() const noexcept
{
  return config_;
}

RplidarDeviceInfo RplidarDriver::get_info()
{
  try {
    if (!serial_.is_open()) {
      serial_.open(config_.serial);
    }

    serial_.flush();
    send_command(RPLIDAR_CMD_GET_INFO);

    const auto descriptor = read_descriptor(config_.serial.timeout_s);
    if (descriptor.data_type != RPLIDAR_RESPONSE_TYPE_INFO) {
      throw RplidarProtocolError("unexpected RPLIDAR device-info response type");
    }

    const auto payload = serial_.read_exact(RPLIDAR_INFO_SIZE, config_.serial.timeout_s);
    return parse_device_info(payload);
  } catch (const std::exception & exc) {
    mark_error(exc.what());
    throw;
  }
}

RplidarHealth RplidarDriver::get_health()
{
  try {
    if (!serial_.is_open()) {
      serial_.open(config_.serial);
    }

    serial_.flush();
    send_command(RPLIDAR_CMD_GET_HEALTH);

    const auto descriptor = read_descriptor(config_.serial.timeout_s);
    if (descriptor.data_type != RPLIDAR_RESPONSE_TYPE_HEALTH) {
      throw RplidarProtocolError("unexpected RPLIDAR health response type");
    }

    const auto payload = serial_.read_exact(RPLIDAR_HEALTH_SIZE, config_.serial.timeout_s);
    return parse_health(payload);
  } catch (const std::exception & exc) {
    mark_error(exc.what());
    throw;
  }
}

LidarScan RplidarDriver::read_scan(double timeout_s)
{
  if (!running()) {
    throw std::runtime_error("RPLIDAR driver is not running");
  }

  if (timeout_s <= 0.0) {
    throw std::invalid_argument("read_scan timeout_s must be > 0");
  }

  LidarScan scan;
  scan.frame_id = config_.frame_id;
  scan.range_min_m = config_.min_range_m;
  scan.range_max_m = config_.max_range_m;
  scan.angle_min_rad = 0.0;
  scan.angle_max_rad = 2.0 * 3.14159265358979323846;
  scan.angle_increment_rad = 0.0;
  scan.sequence = scan_count_ + 1U;

  const auto start = std::chrono::steady_clock::now();

  bool have_started_rotation = false;

  try {
    while (!timeout_reached(start, timeout_s)) {
      const auto measurement = read_measurement(remaining_timeout_s(start, timeout_s));

      if (!measurement.valid) {
        continue;
      }

      if (measurement.start_flag) {
        if (have_started_rotation && !scan.empty()) {
          break;
        }

        have_started_rotation = true;
        scan_start_time_ = std::chrono::steady_clock::now();
      }

      if (!have_started_rotation) {
        continue;
      }

      append_measurement_to_scan(scan, measurement);
    }

    if (scan.empty()) {
      throw std::runtime_error("RPLIDAR scan timeout; no valid samples collected");
    }

    scan.scan_time_s = elapsed_s(scan_start_time_);

    if (scan.size() > 1U) {
      scan.angle_increment_rad =
        (scan.angle_max_rad - scan.angle_min_rad) / static_cast<double>(scan.size());
      scan.time_increment_s = scan.scan_time_s / static_cast<double>(scan.size());
    }

    ++scan_count_;
    scan.sequence = scan_count_;

    return scan;
  } catch (const std::exception & exc) {
    mark_error(exc.what());
    throw;
  }
}

std::uint64_t RplidarDriver::scan_count() const noexcept
{
  return scan_count_;
}

std::string RplidarDriver::last_error() const
{
  return last_error_;
}

void RplidarDriver::send_command(std::uint8_t command)
{
  const auto bytes = make_command(command);
  serial_.write_bytes(bytes);
}

RplidarResponseDescriptor RplidarDriver::read_descriptor(double timeout_s)
{
  const auto start = std::chrono::steady_clock::now();

  std::vector<std::uint8_t> descriptor;
  descriptor.reserve(RPLIDAR_DESCRIPTOR_SIZE);

  while (!timeout_reached(start, timeout_s)) {
    auto byte = serial_.read_exact(1U, remaining_timeout_s(start, timeout_s));

    if (descriptor.empty()) {
      if (byte[0] == RPLIDAR_RESPONSE_SYNC_BYTE_1) {
        descriptor.push_back(byte[0]);
      }
      continue;
    }

    if (descriptor.size() == 1U) {
      if (byte[0] == RPLIDAR_RESPONSE_SYNC_BYTE_2) {
        descriptor.push_back(byte[0]);
      } else if (byte[0] == RPLIDAR_RESPONSE_SYNC_BYTE_1) {
        descriptor[0] = byte[0];
      } else {
        descriptor.clear();
      }
      continue;
    }

    descriptor.push_back(byte[0]);

    if (descriptor.size() == RPLIDAR_DESCRIPTOR_SIZE) {
      return parse_descriptor(descriptor);
    }
  }

  throw RplidarProtocolError("timeout while reading RPLIDAR response descriptor");
}

RplidarMeasurement RplidarDriver::read_measurement(double timeout_s)
{
  const auto bytes = serial_.read_exact(RPLIDAR_SCAN_NODE_SIZE, timeout_s);
  return parse_scan_node(bytes.data(), bytes.size());
}

void RplidarDriver::begin_scan()
{
  serial_.flush();
  send_command(RPLIDAR_CMD_SCAN);

  const auto descriptor = read_descriptor(config_.serial.timeout_s);
  if (descriptor.data_type != RPLIDAR_RESPONSE_TYPE_SCAN) {
    throw RplidarProtocolError("unexpected RPLIDAR scan response type");
  }

  if (descriptor.payload_size != RPLIDAR_SCAN_NODE_SIZE) {
    throw RplidarProtocolError("unexpected RPLIDAR scan node payload size");
  }
}

void RplidarDriver::mark_error(const std::string & message)
{
  last_error_ = message;
  state_ = DriverState::error;
}

void RplidarDriver::append_measurement_to_scan(
  LidarScan & scan,
  const RplidarMeasurement & measurement)
{
  if (measurement.valid) {
    scan.ranges_m.push_back(measurement.distance_m);
    scan.intensities.push_back(static_cast<float>(measurement.quality));
    return;
  }

  scan.ranges_m.push_back(std::numeric_limits<float>::infinity());
  scan.intensities.push_back(0.0F);
}

const char * to_string(DriverState state)
{
  switch (state) {
    case DriverState::stopped:
      return "stopped";
    case DriverState::starting:
      return "starting";
    case DriverState::running:
      return "running";
    case DriverState::error:
      return "error";
    default:
      return "unknown";
  }
}

}  // namespace savo_lidar
