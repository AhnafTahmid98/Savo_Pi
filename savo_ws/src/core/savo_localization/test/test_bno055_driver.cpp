#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "fake_i2c_bus.hpp"
#include "savo_localization/bno055_driver.hpp"

namespace savo_localization
{
namespace
{

BNO055CalibrationProfile valid_profile()
{
  return BNO055CalibrationProfile{
    -500, 123, -1,
    -6400, 2345, -42,
    -2000, 777, -9,
    1000, 480};
}

struct DriverFixture
{
  DriverFixture()
  {
    auto owned_bus = std::make_unique<test::FakeI2CBus>();
    bus = owned_bus.get();
    driver = std::make_unique<BNO055Driver>(std::move(owned_bus));
    EXPECT_TRUE(driver->open());
    driver->set_mode(BNO055Mode::NDOF);
  }

  test::FakeI2CBus * bus{nullptr};
  std::unique_ptr<BNO055Driver> driver;
};

TEST(BNO055CalibrationRegistersTest, UsesDocumentedContiguousPageZeroAddresses)
{
  using namespace bno055_registers;
  static_assert(ACC_OFFSET_X_LSB == 0x55);
  static_assert(ACC_OFFSET_X_MSB == 0x56);
  static_assert(ACC_OFFSET_Y_LSB == 0x57);
  static_assert(ACC_OFFSET_Y_MSB == 0x58);
  static_assert(ACC_OFFSET_Z_LSB == 0x59);
  static_assert(ACC_OFFSET_Z_MSB == 0x5A);
  static_assert(MAG_OFFSET_X_LSB == 0x5B);
  static_assert(MAG_OFFSET_X_MSB == 0x5C);
  static_assert(MAG_OFFSET_Y_LSB == 0x5D);
  static_assert(MAG_OFFSET_Y_MSB == 0x5E);
  static_assert(MAG_OFFSET_Z_LSB == 0x5F);
  static_assert(MAG_OFFSET_Z_MSB == 0x60);
  static_assert(GYR_OFFSET_X_LSB == 0x61);
  static_assert(GYR_OFFSET_X_MSB == 0x62);
  static_assert(GYR_OFFSET_Y_LSB == 0x63);
  static_assert(GYR_OFFSET_Y_MSB == 0x64);
  static_assert(GYR_OFFSET_Z_LSB == 0x65);
  static_assert(GYR_OFFSET_Z_MSB == 0x66);
  static_assert(ACC_RADIUS_LSB == 0x67);
  static_assert(ACC_RADIUS_MSB == 0x68);
  static_assert(MAG_RADIUS_LSB == 0x69);
  static_assert(MAG_RADIUS_MSB == 0x6A);
  static_assert(CALIBRATION_BLOCK_LENGTH == 22U);
  SUCCEED();
}

TEST(BNO055CalibrationEncodingTest, EncodesSignedOffsetsAndUnsignedRadiiLittleEndian)
{
  const auto bytes = encode_bno055_calibration_profile(valid_profile());
  EXPECT_EQ(bytes[0], 0x0C);
  EXPECT_EQ(bytes[1], 0xFE);
  EXPECT_EQ(bytes[4], 0xFF);
  EXPECT_EQ(bytes[5], 0xFF);
  EXPECT_EQ(bytes[6], 0x00);
  EXPECT_EQ(bytes[7], 0xE7);
  EXPECT_EQ(bytes[12], 0x30);
  EXPECT_EQ(bytes[13], 0xF8);
  EXPECT_EQ(bytes[18], 0xE8);
  EXPECT_EQ(bytes[19], 0x03);
  EXPECT_EQ(bytes[20], 0xE0);
  EXPECT_EQ(bytes[21], 0x01);
}

TEST(BNO055CalibrationEncodingTest, DecodesNegativeAndPositiveValuesExactly)
{
  const auto profile = valid_profile();
  EXPECT_EQ(decode_bno055_calibration_profile(
      encode_bno055_calibration_profile(profile)), profile);
}

TEST(BNO055CalibrationValidationTest, RejectsEveryDocumentedRangeViolation)
{
  auto profile = valid_profile();
  std::string error;

  profile.accel_offset_x = 501;
  EXPECT_FALSE(validate_bno055_calibration_profile(profile, &error));
  profile = valid_profile();
  profile.mag_offset_y = -6401;
  EXPECT_FALSE(validate_bno055_calibration_profile(profile, &error));
  profile = valid_profile();
  profile.gyro_offset_z = 2001;
  EXPECT_FALSE(validate_bno055_calibration_profile(profile, &error));
  profile = valid_profile();
  profile.accel_radius = 2049;
  EXPECT_FALSE(validate_bno055_calibration_profile(profile, &error));
  profile = valid_profile();
  profile.mag_radius = 143;
  EXPECT_FALSE(validate_bno055_calibration_profile(profile, &error));
}

TEST(BNO055DriverCalibrationTest, ReadsCompleteBlockAndRestoresOperationalMode)
{
  DriverFixture fixture;
  const auto profile = valid_profile();
  const auto encoded = encode_bno055_calibration_profile(profile);
  fixture.bus->load_block(
    bno055_registers::ACC_OFFSET_X_LSB,
    std::vector<uint8_t>(encoded.begin(), encoded.end()));
  fixture.bus->byte_writes.clear();

  EXPECT_EQ(fixture.driver->read_calibration_profile(), profile);
  ASSERT_FALSE(fixture.bus->block_reads.empty());
  EXPECT_EQ(
    fixture.bus->block_reads.back().start_register,
    bno055_registers::ACC_OFFSET_X_LSB);
  EXPECT_EQ(
    fixture.bus->block_reads.back().values.size(),
    bno055_registers::CALIBRATION_BLOCK_LENGTH);
  ASSERT_GE(fixture.bus->byte_writes.size(), 3U);
  EXPECT_EQ(fixture.bus->byte_writes.front(),
    (std::make_pair<uint8_t, uint8_t>(0x3D, 0x00)));
  EXPECT_NE(
    std::find(
      fixture.bus->byte_writes.begin(), fixture.bus->byte_writes.end(),
      std::make_pair<uint8_t, uint8_t>(0x07, 0x00)),
    fixture.bus->byte_writes.end());
  EXPECT_EQ(fixture.bus->byte_writes.back(),
    (std::make_pair<uint8_t, uint8_t>(0x3D, 0x0C)));
  EXPECT_EQ(fixture.driver->mode(), BNO055Mode::NDOF);
  EXPECT_EQ(fixture.bus->registers[0x3D], static_cast<uint8_t>(BNO055Mode::NDOF));
}

TEST(BNO055DriverCalibrationTest, WritesCompleteBlockAndVerifiesReadback)
{
  DriverFixture fixture;
  const auto profile = valid_profile();
  fixture.bus->byte_writes.clear();

  EXPECT_NO_THROW(fixture.driver->write_calibration_profile(profile));
  ASSERT_EQ(fixture.bus->block_writes.size(), 1U);
  EXPECT_EQ(
    fixture.bus->block_writes.front().start_register,
    bno055_registers::ACC_OFFSET_X_LSB);
  EXPECT_EQ(
    fixture.bus->block_writes.front().values.size(),
    bno055_registers::CALIBRATION_BLOCK_LENGTH);
  ASSERT_GE(fixture.bus->byte_writes.size(), 3U);
  EXPECT_EQ(fixture.bus->byte_writes.front(),
    (std::make_pair<uint8_t, uint8_t>(0x3D, 0x00)));
  EXPECT_NE(
    std::find(
      fixture.bus->byte_writes.begin(), fixture.bus->byte_writes.end(),
      std::make_pair<uint8_t, uint8_t>(0x07, 0x00)),
    fixture.bus->byte_writes.end());
  EXPECT_EQ(fixture.bus->byte_writes.back(),
    (std::make_pair<uint8_t, uint8_t>(0x3D, 0x0C)));
  EXPECT_EQ(fixture.driver->mode(), BNO055Mode::NDOF);
}

TEST(BNO055DriverCalibrationTest, ReadbackMismatchFailsAndRestoresMode)
{
  DriverFixture fixture;
  fixture.bus->corrupt_next_calibration_readback = true;

  EXPECT_THROW(
    fixture.driver->write_calibration_profile(valid_profile()),
    std::runtime_error);
  EXPECT_EQ(fixture.driver->mode(), BNO055Mode::NDOF);
  EXPECT_EQ(fixture.bus->registers[0x3D], static_cast<uint8_t>(BNO055Mode::NDOF));
}

TEST(BNO055DriverCalibrationTest, RecoverableReadFailureRestoresMode)
{
  DriverFixture fixture;
  fixture.bus->fail_next_block_read = true;

  EXPECT_THROW(fixture.driver->read_calibration_profile(), std::runtime_error);
  EXPECT_EQ(fixture.driver->mode(), BNO055Mode::NDOF);
}

TEST(BNO055DriverCalibrationTest, ShortBlockReadNeverReportsSuccessAndRestoresMode)
{
  DriverFixture fixture;
  fixture.bus->short_next_block_read_by = 1U;

  EXPECT_THROW(fixture.driver->read_calibration_profile(), std::runtime_error);
  EXPECT_EQ(fixture.driver->mode(), BNO055Mode::NDOF);
}

TEST(BNO055DriverCalibrationTest, InvalidProfileIsRejectedBeforeHardwareWrite)
{
  DriverFixture fixture;
  auto invalid = valid_profile();
  invalid.accel_offset_x = 501;

  EXPECT_THROW(
    fixture.driver->write_calibration_profile(invalid),
    std::invalid_argument);
  EXPECT_TRUE(fixture.bus->block_writes.empty());
  EXPECT_EQ(fixture.driver->mode(), BNO055Mode::NDOF);
}

TEST(BNO055DriverCalibrationTest, PartialWriteNeverReportsSuccessAndRestoresMode)
{
  DriverFixture fixture;
  fixture.bus->fail_block_write_after_bytes = 5U;

  EXPECT_THROW(
    fixture.driver->write_calibration_profile(valid_profile()),
    std::runtime_error);
  EXPECT_EQ(fixture.bus->block_writes.size(), 1U);
  EXPECT_EQ(fixture.driver->mode(), BNO055Mode::NDOF);
}

TEST(BNO055DriverCalibrationTest, ExistingSampleDecodingRemainsUnchanged)
{
  DriverFixture fixture;
  fixture.bus->load_block(0x08, {100, 0, 156, 255, 0, 0});
  fixture.bus->load_block(0x0E, {16, 0, 240, 255, 32, 0});
  fixture.bus->load_block(0x14, {16, 0, 240, 255, 32, 0});
  fixture.bus->load_block(0x1A, {16, 0, 32, 0, 48, 0});
  fixture.bus->registers[0x34] = 25;
  fixture.bus->registers[0x35] = 0xFF;
  fixture.bus->registers[0x39] = 5;
  fixture.bus->registers[0x3A] = 0;

  const BNO055Sample sample = fixture.driver->read_sample(true, true, true);
  EXPECT_DOUBLE_EQ(sample.accel_mps2.x, 1.0);
  EXPECT_DOUBLE_EQ(sample.accel_mps2.y, -1.0);
  EXPECT_DOUBLE_EQ(sample.gyro_dps.x, 1.0);
  EXPECT_DOUBLE_EQ(sample.mag_ut.y, -1.0);
  EXPECT_DOUBLE_EQ(sample.euler_deg.yaw_deg, 1.0);
  EXPECT_EQ(sample.temperature_c, 25);
  EXPECT_TRUE(sample.status.calibration.fully_calibrated());
}

}  // namespace
}  // namespace savo_localization
