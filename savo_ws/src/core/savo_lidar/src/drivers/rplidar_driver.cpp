#include "savo_lidar/rplidar_driver.hpp"

#include "savo_lidar/scan_angle_compensator.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace savo_lidar
{
namespace
{

constexpr auto START_CANCELLATION_POLL_INTERVAL = std::chrono::milliseconds(20);

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
      const auto settle_deadline = std::chrono::steady_clock::now() + settle;

      while (std::chrono::steady_clock::now() < settle_deadline) {
        if (serial_.io_cancellation_requested()) {
          throw std::runtime_error("RPLIDAR startup cancelled");
        }

        const auto remaining = std::chrono::duration<double>(
          settle_deadline - std::chrono::steady_clock::now());
        std::this_thread::sleep_for(std::min(
          remaining,
          std::chrono::duration<double>(START_CANCELLATION_POLL_INTERVAL)));
      }
    }

    scan_assembler_.reset();
    begin_scan();
    state_ = DriverState::running;
  } catch (const std::exception & exc) {
    mark_error(exc.what());
    serial_.close();
    throw;
  }
}

void RplidarDriver::stop() noexcept
{
  // The acquisition worker is joined before stop() is called, so it is safe to
  // re-enable I/O briefly for the best-effort motor stop command.
  serial_.reset_io_cancellation();

  try {
    if (serial_.is_open()) {
      serial_.write_bytes(
        make_command(RPLIDAR_CMD_STOP),
        config_.motor_stop_timeout_s);
    }
  } catch (...) {
  }

  serial_.close();
  scan_assembler_.reset();
  state_ = DriverState::stopped;
}

void RplidarDriver::cancel_pending_operation() noexcept
{
  serial_.cancel_pending_io();
}

void RplidarDriver::reset()
{
  try {
    if (!serial_.is_open()) {
      serial_.open(config_.serial);
    }

    send_command(RPLIDAR_CMD_RESET);
    scan_assembler_.reset();
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

LidarScan RplidarDriver::read_scan(
  double timeout_s,
  const RosTimestampCallback & ros_timestamp_now)
{
  if (!running()) {
    throw std::runtime_error("RPLIDAR driver is not running");
  }

  if (timeout_s <= 0.0) {
    throw std::invalid_argument("read_scan timeout_s must be > 0");
  }

  if (!ros_timestamp_now) {
    throw std::invalid_argument("read_scan ROS timestamp callback must be set");
  }

  LidarScan scan;
  scan.frame_id = config_.frame_id;
  scan.range_min_m = config_.min_range_m;
  scan.range_max_m = config_.max_range_m;
  scan.sequence = scan_count_ + 1U;

  const auto start = std::chrono::steady_clock::now();

  try {
    while (!timeout_reached(start, timeout_s)) {
      const auto measurement = read_measurement(remaining_timeout_s(start, timeout_s));
      const auto received_at = std::chrono::steady_clock::now();
      const auto ros_received_time_ns =
        measurement.start_flag ? ros_timestamp_now() : 0;
      auto completed = scan_assembler_.add_measurement(
        measurement,
        received_at,
        ros_received_time_ns);

      if (completed) {
        scan = bin_scan_samples_by_angle(
          completed->samples,
          std::move(scan),
          config_.inverted,
          config_.angle_offset_rad);

        apply_scan_timing(scan, completed->scan_time_s);
        scan.ros_start_time_ns = completed->ros_start_time_ns;

        ++scan_count_;
        scan.sequence = scan_count_;
        return scan;
      }
    }

    throw std::runtime_error("RPLIDAR scan timeout; no complete revolution collected");
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
