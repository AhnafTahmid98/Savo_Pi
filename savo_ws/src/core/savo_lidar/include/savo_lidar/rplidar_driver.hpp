#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "savo_lidar/rplidar_protocol.hpp"
#include "savo_lidar/scan_types.hpp"
#include "savo_lidar/serial_config.hpp"
#include "savo_lidar/serial_port.hpp"
#include "savo_lidar/visibility_control.hpp"

namespace savo_lidar
{

enum class DriverState : std::uint8_t
{
  stopped = 0,
  starting = 1,
  running = 2,
  error = 3,
};

class SAVO_LIDAR_PUBLIC RplidarDriver
{
public:
  RplidarDriver();
  explicit RplidarDriver(const RplidarConfig & config);
  ~RplidarDriver();

  RplidarDriver(const RplidarDriver &) = delete;
  RplidarDriver & operator=(const RplidarDriver &) = delete;

  RplidarDriver(RplidarDriver &&) noexcept = default;
  RplidarDriver & operator=(RplidarDriver &&) noexcept = default;

  void configure(const RplidarConfig & config);

  void start();
  void stop() noexcept;
  void reset();

  bool running() const noexcept;
  DriverState state() const noexcept;

  const RplidarConfig & config() const noexcept;

  RplidarDeviceInfo get_info();
  RplidarHealth get_health();

  LidarScan read_scan(double timeout_s);

  std::uint64_t scan_count() const noexcept;
  std::string last_error() const;

private:
  void send_command(std::uint8_t command);
  RplidarResponseDescriptor read_descriptor(double timeout_s);
  RplidarMeasurement read_measurement(double timeout_s);

  void begin_scan();
  void mark_error(const std::string & message);

  static void append_measurement_to_scan(
    LidarScan & scan,
    const RplidarMeasurement & measurement);

  RplidarConfig config_;
  SerialPort serial_;

  DriverState state_{DriverState::stopped};
  std::uint64_t scan_count_{0};
  std::string last_error_;

  std::chrono::steady_clock::time_point scan_start_time_;
};

SAVO_LIDAR_PUBLIC const char * to_string(DriverState state);

}  // namespace savo_lidar