#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "savo_lidar/rplidar_protocol.hpp"
#include "savo_lidar/scan_angle_compensator.hpp"
#include "savo_lidar/scan_types.hpp"

namespace
{

constexpr double PI = 3.14159265358979323846;

savo_lidar::LidarSample sample_at_degrees(
  double angle_deg,
  float range_m,
  float intensity)
{
  savo_lidar::LidarSample sample;
  const auto angle_q6 = static_cast<std::uint16_t>(std::lround(angle_deg * 64.0));
  sample.angle_rad = savo_lidar::q6_angle_to_rad(angle_q6);
  sample.range_m = range_m;
  sample.intensity = intensity;
  sample.valid = true;
  return sample;
}

savo_lidar::LidarScan make_scan()
{
  savo_lidar::LidarScan scan;
  scan.range_min_m = 0.15F;
  scan.range_max_m = 12.0F;
  return scan;
}

TEST(ScanAngleCompensator, MapsHardwareCardinalAnglesToRosBins)
{
  const std::vector<savo_lidar::LidarSample> samples{
    sample_at_degrees(270.0, 2.70F, 27.0F),
    sample_at_degrees(0.0, 1.00F, 10.0F),
    sample_at_degrees(180.0, 2.00F, 20.0F),
    sample_at_degrees(90.0, 1.90F, 19.0F),
  };

  const auto scan = savo_lidar::bin_scan_samples_by_angle(
    samples, make_scan(), false, 0.0);

  EXPECT_FLOAT_EQ(scan.ranges_m[180], 1.00F);  // 0 degrees
  EXPECT_FLOAT_EQ(scan.intensities[180], 10.0F);
  EXPECT_FLOAT_EQ(scan.ranges_m[270], 1.90F);  // +90 degrees
  EXPECT_FLOAT_EQ(scan.intensities[270], 19.0F);
  EXPECT_FLOAT_EQ(scan.ranges_m[0], 2.00F);  // 180 degrees == -180 degrees
  EXPECT_FLOAT_EQ(scan.intensities[0], 20.0F);
  EXPECT_FLOAT_EQ(scan.ranges_m[90], 2.70F);  // 270 degrees == -90 degrees
  EXPECT_FLOAT_EQ(scan.intensities[90], 27.0F);
}

TEST(ScanAngleCompensator, UsesMeasuredAnglesNotInputOrder)
{
  const std::vector<savo_lidar::LidarSample> samples{
    sample_at_degrees(42.25, 4.20F, 42.0F),
    sample_at_degrees(345.75, 3.45F, 34.0F),
    sample_at_degrees(11.50, 1.15F, 11.0F),
  };

  const auto scan = savo_lidar::bin_scan_samples_by_angle(
    samples, make_scan(), false, 0.0);

  EXPECT_FLOAT_EQ(scan.ranges_m[222], 4.20F);
  EXPECT_FLOAT_EQ(scan.intensities[222], 42.0F);
  EXPECT_FLOAT_EQ(scan.ranges_m[166], 3.45F);
  EXPECT_FLOAT_EQ(scan.intensities[166], 34.0F);
  EXPECT_FLOAT_EQ(scan.ranges_m[192], 1.15F);
  EXPECT_FLOAT_EQ(scan.intensities[192], 11.0F);
  EXPECT_TRUE(std::isinf(scan.ranges_m[1]));
  EXPECT_FLOAT_EQ(scan.intensities[1], 0.0F);
}

TEST(ScanAngleCompensator, KeepsRangeAndIntensityTogetherOnBinCollision)
{
  const std::vector<savo_lidar::LidarSample> samples{
    sample_at_degrees(0.20, 2.00F, 20.0F),
    sample_at_degrees(359.80, 0.80F, 8.0F),
  };

  const auto scan = savo_lidar::bin_scan_samples_by_angle(
    samples, make_scan(), false, 0.0);

  EXPECT_FLOAT_EQ(scan.ranges_m[180], 0.80F);
  EXPECT_FLOAT_EQ(scan.intensities[180], 8.0F);
}

TEST(ScanAngleCompensator, WrapsSamplesNearZeroAndFullRotation)
{
  const std::vector<savo_lidar::LidarSample> samples{
    sample_at_degrees(359.0, 3.59F, 35.0F),
    sample_at_degrees(0.75, 0.75F, 7.0F),
  };

  const auto scan = savo_lidar::bin_scan_samples_by_angle(
    samples, make_scan(), false, 0.0);

  EXPECT_FLOAT_EQ(scan.ranges_m[179], 3.59F);
  EXPECT_FLOAT_EQ(scan.ranges_m[181], 0.75F);
}

TEST(ScanAngleCompensator, AppliesAngleOffsetAfterNativeSensorAngle)
{
  const std::vector<savo_lidar::LidarSample> samples{
    sample_at_degrees(0.0, 1.00F, 10.0F),
    sample_at_degrees(350.0, 3.50F, 35.0F),
  };

  const auto scan = savo_lidar::bin_scan_samples_by_angle(
    samples, make_scan(), false, PI / 2.0);

  EXPECT_FLOAT_EQ(scan.ranges_m[270], 1.00F);
  EXPECT_FLOAT_EQ(scan.ranges_m[260], 3.50F);
}

TEST(ScanAngleCompensator, InvertedNegatesAngleBeforeOffset)
{
  const std::vector<savo_lidar::LidarSample> samples{
    sample_at_degrees(90.0, 0.90F, 9.0F),
    sample_at_degrees(270.0, 2.70F, 27.0F),
  };

  const auto scan = savo_lidar::bin_scan_samples_by_angle(
    samples, make_scan(), true, 0.0);

  EXPECT_FLOAT_EQ(scan.ranges_m[90], 0.90F);
  EXPECT_FLOAT_EQ(scan.ranges_m[270], 2.70F);

  const auto offset_scan = savo_lidar::bin_scan_samples_by_angle(
    {sample_at_degrees(90.0, 0.60F, 6.0F)},
    make_scan(),
    true,
    PI / 6.0);
  EXPECT_FLOAT_EQ(offset_scan.ranges_m[120], 0.60F);
}

TEST(ScanAngleCompensator, MissingAndInvalidBinsStayInvalid)
{
  auto invalid_sample = sample_at_degrees(45.0, 0.10F, 45.0F);
  invalid_sample.valid = false;
  const auto out_of_range_sample = sample_at_degrees(46.0, 0.10F, 46.0F);

  const auto scan = savo_lidar::bin_scan_samples_by_angle(
    {invalid_sample, out_of_range_sample}, make_scan(), false, 0.0);

  EXPECT_TRUE(std::isinf(scan.ranges_m[225]));
  EXPECT_FLOAT_EQ(scan.intensities[225], 0.0F);
  EXPECT_TRUE(std::isinf(scan.ranges_m[226]));
  EXPECT_FLOAT_EQ(scan.intensities[226], 0.0F);
}

TEST(ScanAngleCompensator, ProducesInternallyConsistentLaserScanGeometry)
{
  const auto scan = savo_lidar::bin_scan_samples_by_angle(
    {}, make_scan(), false, 0.0);

  ASSERT_EQ(scan.ranges_m.size(), 360U);
  ASSERT_EQ(scan.intensities.size(), scan.ranges_m.size());
  EXPECT_DOUBLE_EQ(scan.angle_min_rad, -PI);
  EXPECT_GT(scan.angle_increment_rad, 0.0);
  EXPECT_NEAR(scan.angle_increment_rad, PI / 180.0, 1.0e-12);
  EXPECT_NEAR(
    scan.angle_max_rad,
    scan.angle_min_rad +
    static_cast<double>(scan.ranges_m.size() - 1U) * scan.angle_increment_rad,
    1.0e-12);
}

TEST(ScanAngleCompensator, RejectsNonFiniteAngleOffset)
{
  EXPECT_THROW(
    savo_lidar::bin_scan_samples_by_angle(
      {},
      make_scan(),
      false,
      std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
}

}  // namespace
