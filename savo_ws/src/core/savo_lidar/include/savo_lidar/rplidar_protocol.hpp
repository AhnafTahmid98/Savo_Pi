#pragma once

#include <cstdint>
#include <stdexcept>
#include <vector>

#include "savo_lidar/visibility_control.hpp"

namespace savo_lidar
{

constexpr std::uint8_t RPLIDAR_SYNC_BYTE = 0xA5;
constexpr std::uint8_t RPLIDAR_RESPONSE_SYNC_BYTE_1 = 0xA5;
constexpr std::uint8_t RPLIDAR_RESPONSE_SYNC_BYTE_2 = 0x5A;

constexpr std::uint8_t RPLIDAR_CMD_STOP = 0x25;
constexpr std::uint8_t RPLIDAR_CMD_RESET = 0x40;
constexpr std::uint8_t RPLIDAR_CMD_SCAN = 0x20;
constexpr std::uint8_t RPLIDAR_CMD_FORCE_SCAN = 0x21;
constexpr std::uint8_t RPLIDAR_CMD_GET_INFO = 0x50;
constexpr std::uint8_t RPLIDAR_CMD_GET_HEALTH = 0x52;

constexpr std::uint8_t RPLIDAR_RESPONSE_TYPE_SCAN = 0x81;
constexpr std::uint8_t RPLIDAR_RESPONSE_TYPE_INFO = 0x04;
constexpr std::uint8_t RPLIDAR_RESPONSE_TYPE_HEALTH = 0x06;

constexpr std::size_t RPLIDAR_DESCRIPTOR_SIZE = 7;
constexpr std::size_t RPLIDAR_SCAN_NODE_SIZE = 5;
constexpr std::size_t RPLIDAR_INFO_SIZE = 20;
constexpr std::size_t RPLIDAR_HEALTH_SIZE = 3;

enum class RplidarHealthStatus : std::uint8_t
{
  ok = 0,
  warning = 1,
  error = 2,
  unknown = 255,
};

struct SAVO_LIDAR_PUBLIC RplidarResponseDescriptor
{
  std::uint32_t payload_size{0};
  std::uint8_t send_mode{0};
  std::uint8_t data_type{0};
};

struct SAVO_LIDAR_PUBLIC RplidarDeviceInfo
{
  std::uint8_t model{0};
  std::uint16_t firmware_version{0};
  std::uint8_t hardware_version{0};
  std::uint8_t serial_number[16]{};
};

struct SAVO_LIDAR_PUBLIC RplidarHealth
{
  RplidarHealthStatus status{RplidarHealthStatus::unknown};
  std::uint16_t error_code{0};

  bool ok() const
  {
    return status == RplidarHealthStatus::ok;
  }
};

struct SAVO_LIDAR_PUBLIC RplidarMeasurement
{
  bool start_flag{false};
  bool quality_valid{false};

  std::uint8_t quality{0};
  double angle_rad{0.0};
  float distance_m{0.0F};

  bool valid{false};
};

class SAVO_LIDAR_PUBLIC RplidarProtocolError : public std::runtime_error
{
public:
  explicit RplidarProtocolError(const char * message)
  : std::runtime_error(message)
  {
  }

  explicit RplidarProtocolError(const std::string & message)
  : std::runtime_error(message)
  {
  }
};

SAVO_LIDAR_PUBLIC std::vector<std::uint8_t> make_command(std::uint8_t command);

SAVO_LIDAR_PUBLIC bool is_valid_descriptor_header(
  const std::vector<std::uint8_t> & bytes);

SAVO_LIDAR_PUBLIC RplidarResponseDescriptor parse_descriptor(
  const std::vector<std::uint8_t> & bytes);

SAVO_LIDAR_PUBLIC RplidarDeviceInfo parse_device_info(
  const std::vector<std::uint8_t> & bytes);

SAVO_LIDAR_PUBLIC RplidarHealth parse_health(
  const std::vector<std::uint8_t> & bytes);

SAVO_LIDAR_PUBLIC RplidarMeasurement parse_scan_node(
  const std::uint8_t * bytes,
  std::size_t size);

SAVO_LIDAR_PUBLIC double q6_angle_to_rad(std::uint16_t value);
SAVO_LIDAR_PUBLIC float q2_distance_to_m(std::uint16_t value);

}  // namespace savo_lidar