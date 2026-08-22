#include "savo_lidar/scan_angle_compensator.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace savo_lidar
{
namespace
{

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;

}  // namespace

double normalize_scan_angle_rad(double angle_rad)
{
  if (!std::isfinite(angle_rad)) {
    throw std::invalid_argument("scan angle must be finite");
  }

  double normalized = std::fmod(angle_rad + PI, TWO_PI);
  if (normalized < 0.0) {
    normalized += TWO_PI;
  }
  return normalized - PI;
}

double transform_scan_angle_rad(
  double sensor_angle_rad,
  bool inverted,
  double angle_offset_rad)
{
  if (!std::isfinite(angle_offset_rad)) {
    throw std::invalid_argument("angle_offset_rad must be finite");
  }

  const double directed_angle = inverted ? -sensor_angle_rad : sensor_angle_rad;
  return normalize_scan_angle_rad(directed_angle + angle_offset_rad);
}

LidarScan bin_scan_samples_by_angle(
  const std::vector<LidarSample> & samples,
  LidarScan scan,
  bool inverted,
  double angle_offset_rad,
  std::size_t bin_count)
{
  if (bin_count < 2U) {
    throw std::invalid_argument("scan angle bin_count must be at least 2");
  }
  if (!std::isfinite(angle_offset_rad)) {
    throw std::invalid_argument("angle_offset_rad must be finite");
  }

  scan.angle_min_rad = -PI;
  scan.angle_increment_rad = TWO_PI / static_cast<double>(bin_count);
  scan.angle_max_rad =
    scan.angle_min_rad +
    static_cast<double>(bin_count - 1U) * scan.angle_increment_rad;

  scan.ranges_m.assign(bin_count, std::numeric_limits<float>::infinity());
  scan.intensities.assign(bin_count, 0.0F);

  for (const auto & sample : samples) {
    if (!sample.valid || !std::isfinite(sample.angle_rad) ||
      !is_valid_range(sample.range_m, scan.range_min_m, scan.range_max_m))
    {
      continue;
    }

    const double angle = transform_scan_angle_rad(
      sample.angle_rad,
      inverted,
      angle_offset_rad);
    const double bin_position =
      (angle - scan.angle_min_rad) / scan.angle_increment_rad;
    const auto nearest_bin = static_cast<long long>(std::llround(bin_position));
    const auto signed_bin_count = static_cast<long long>(bin_count);
    auto wrapped_bin = nearest_bin % signed_bin_count;
    if (wrapped_bin < 0) {
      wrapped_bin += signed_bin_count;
    }
    const auto index = static_cast<std::size_t>(wrapped_bin);

    if (!is_finite_range(scan.ranges_m[index]) ||
      sample.range_m < scan.ranges_m[index])
    {
      scan.ranges_m[index] = sample.range_m;
      scan.intensities[index] =
        std::isfinite(sample.intensity) ? sample.intensity : 0.0F;
    }
  }

  return scan;
}

}  // namespace savo_lidar
