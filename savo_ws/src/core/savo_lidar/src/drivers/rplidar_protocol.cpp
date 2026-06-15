#include "savo_lidar/rplidar_protocol.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace savo_lidar
{
namespace
{

constexpr double PI = 3.14159265358979323846;

void require_size(
  const std::vector<std::uint8_t> & bytes,
  std::size_t expected,
  const char * name)
{
  if (bytes.size() < expected) {
    throw RplidarProtocolError(
      std::string(name) + " requires at least " + std::to_string(expected) + " bytes");
  }
}

}  // namespace

std::vector<std::uint8_t> make_command(std::uint8_t command)
{
  return {RPLIDAR_SYNC_BYTE, command};
}

bool is_valid_descriptor_header(const std::vector<std::uint8_t> & bytes)
{
  return bytes.size() >= RPLIDAR_DESCRIPTOR_SIZE &&
         bytes[0] == RPLIDAR_RESPONSE_SYNC_BYTE_1 &&
         bytes[1] == RPLIDAR_RESPONSE_SYNC_BYTE_2;
}

RplidarResponseDescriptor parse_descriptor(const std::vector<std::uint8_t> & bytes)
{
  require_size(bytes, RPLIDAR_DESCRIPTOR_SIZE, "RPLIDAR response descriptor");

  if (!is_valid_descriptor_header(bytes)) {
    throw RplidarProtocolError("invalid RPLIDAR response descriptor header");
  }

  RplidarResponseDescriptor descriptor;

  descriptor.payload_size =
    static_cast<std::uint32_t>(bytes[2]) |
    (static_cast<std::uint32_t>(bytes[3]) << 8U) |
    (static_cast<std::uint32_t>(bytes[4]) << 16U) |
    ((static_cast<std::uint32_t>(bytes[5]) & 0x3FU) << 24U);

  descriptor.send_mode = static_cast<std::uint8_t>((bytes[5] >> 6U) & 0x03U);
  descriptor.data_type = bytes[6];

  return descriptor;
}

RplidarDeviceInfo parse_device_info(const std::vector<std::uint8_t> & bytes)
{
  require_size(bytes, RPLIDAR_INFO_SIZE, "RPLIDAR device info");

  RplidarDeviceInfo info;

  info.model = bytes[0];
  info.firmware_version =
    static_cast<std::uint16_t>(bytes[1]) |
    (static_cast<std::uint16_t>(bytes[2]) << 8U);
  info.hardware_version = bytes[3];

  for (std::size_t i = 0; i < 16; ++i) {
    info.serial_number[i] = bytes[4 + i];
  }

  return info;
}

RplidarHealth parse_health(const std::vector<std::uint8_t> & bytes)
{
  require_size(bytes, RPLIDAR_HEALTH_SIZE, "RPLIDAR health response");

  RplidarHealth health;

  switch (bytes[0]) {
    case 0:
      health.status = RplidarHealthStatus::ok;
      break;
    case 1:
      health.status = RplidarHealthStatus::warning;
      break;
    case 2:
      health.status = RplidarHealthStatus::error;
      break;
    default:
      health.status = RplidarHealthStatus::unknown;
      break;
  }

  health.error_code =
    static_cast<std::uint16_t>(bytes[1]) |
    (static_cast<std::uint16_t>(bytes[2]) << 8U);

  return health;
}

RplidarMeasurement parse_scan_node(const std::uint8_t * bytes, std::size_t size)
{
  if (bytes == nullptr) {
    throw RplidarProtocolError("RPLIDAR scan node buffer is null");
  }

  if (size < RPLIDAR_SCAN_NODE_SIZE) {
    throw RplidarProtocolError("RPLIDAR scan node requires 5 bytes");
  }

  const bool start_flag = (bytes[0] & 0x01U) != 0U;
  const bool inverted_start_flag = (bytes[0] & 0x02U) != 0U;
  const bool start_flags_valid = start_flag != inverted_start_flag;

  const bool check_bit = (bytes[1] & 0x01U) != 0U;

  const auto quality = static_cast<std::uint8_t>(bytes[0] >> 2U);

  const auto angle_q6 = static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(bytes[1]) >> 1U) |
    (static_cast<std::uint16_t>(bytes[2]) << 7U));

  const auto distance_q2 = static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(bytes[3]) |
    (static_cast<std::uint16_t>(bytes[4]) << 8U));

  RplidarMeasurement measurement;

  measurement.start_flag = start_flag;
  measurement.quality_valid = quality > 0U;
  measurement.quality = quality;
  measurement.angle_rad = q6_angle_to_rad(angle_q6);
  measurement.distance_m = q2_distance_to_m(distance_q2);
  measurement.valid = start_flags_valid &&
                      check_bit &&
                      distance_q2 > 0U &&
                      std::isfinite(measurement.angle_rad) &&
                      std::isfinite(measurement.distance_m);

  return measurement;
}

double q6_angle_to_rad(std::uint16_t value)
{
  const double angle_deg = static_cast<double>(value) / 64.0;
  return angle_deg * PI / 180.0;
}

float q2_distance_to_m(std::uint16_t value)
{
  const double distance_mm = static_cast<double>(value) / 4.0;
  return static_cast<float>(distance_mm / 1000.0);
}

}  // namespace savo_lidar