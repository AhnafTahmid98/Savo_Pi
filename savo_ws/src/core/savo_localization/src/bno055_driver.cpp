#include "savo_localization/bno055_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace savo_localization
{

namespace
{

constexpr uint8_t REG_CHIP_ID = 0x00;
constexpr uint8_t REG_PAGE_ID = 0x07;

constexpr uint8_t REG_ACCEL_DATA_X_LSB = 0x08;
constexpr uint8_t REG_MAG_DATA_X_LSB = 0x0E;
constexpr uint8_t REG_GYRO_DATA_X_LSB = 0x14;
constexpr uint8_t REG_EULER_H_LSB = 0x1A;
constexpr uint8_t REG_TEMP = 0x34;

constexpr uint8_t REG_CALIB_STAT = 0x35;
constexpr uint8_t REG_SYS_STATUS = 0x39;
constexpr uint8_t REG_SYS_ERR = 0x3A;

constexpr uint8_t REG_OPR_MODE = 0x3D;
constexpr uint8_t REG_PWR_MODE = 0x3E;
constexpr uint8_t REG_SYS_TRIGGER = 0x3F;

constexpr uint8_t SYS_TRIGGER_RESET = 0x20;

constexpr auto MODE_SWITCH_DELAY = std::chrono::milliseconds(25);
constexpr auto RESET_DELAY = std::chrono::milliseconds(700);
constexpr auto STARTUP_DELAY = std::chrono::milliseconds(650);

constexpr int16_t ACCEL_OFFSET_LIMIT_LSB = 500;
constexpr int16_t MAG_OFFSET_LIMIT_LSB = 6400;
constexpr int16_t GYRO_OFFSET_LIMIT_LSB = 2000;
constexpr uint16_t ACCEL_RADIUS_LIMIT_LSB = 2048;
constexpr uint16_t MAG_RADIUS_MIN_LSB = 144;
constexpr uint16_t MAG_RADIUS_MAX_LSB = 1280;

int16_t decode_s16_le(const uint8_t lsb, const uint8_t msb)
{
  const uint16_t raw = static_cast<uint16_t>(lsb) |
    (static_cast<uint16_t>(msb) << 8U);
  const int32_t value = raw <= static_cast<uint16_t>(std::numeric_limits<int16_t>::max()) ?
    static_cast<int32_t>(raw) : static_cast<int32_t>(raw) - 65536;
  return static_cast<int16_t>(value);
}

uint16_t decode_u16_le(const uint8_t lsb, const uint8_t msb)
{
  return static_cast<uint16_t>(lsb) |
         (static_cast<uint16_t>(msb) << 8U);
}

void encode_s16_le(
  const int16_t value,
  std::array<uint8_t, bno055_registers::CALIBRATION_BLOCK_LENGTH> & bytes,
  const std::size_t index)
{
  const int32_t widened = static_cast<int32_t>(value);
  const uint16_t raw = static_cast<uint16_t>(
    widened < 0 ? 65536 + widened : widened);
  bytes[index] = static_cast<uint8_t>(raw & 0xFFU);
  bytes[index + 1U] = static_cast<uint8_t>((raw >> 8U) & 0xFFU);
}

void encode_u16_le(
  const uint16_t value,
  std::array<uint8_t, bno055_registers::CALIBRATION_BLOCK_LENGTH> & bytes,
  const std::size_t index)
{
  bytes[index] = static_cast<uint8_t>(value & 0xFFU);
  bytes[index + 1U] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

bool value_within(const int16_t value, const int16_t limit)
{
  return value >= -limit && value <= limit;
}

}  // namespace

bool ImuCalibration::motion_ready() const
{
  return gyro >= 2 && accel >= 2;
}

bool ImuCalibration::fully_calibrated() const
{
  return system >= 3 && gyro >= 3 && accel >= 3 && mag >= 3;
}

bool operator==(
  const BNO055CalibrationProfile & lhs,
  const BNO055CalibrationProfile & rhs)
{
  return lhs.accel_offset_x == rhs.accel_offset_x &&
         lhs.accel_offset_y == rhs.accel_offset_y &&
         lhs.accel_offset_z == rhs.accel_offset_z &&
         lhs.mag_offset_x == rhs.mag_offset_x &&
         lhs.mag_offset_y == rhs.mag_offset_y &&
         lhs.mag_offset_z == rhs.mag_offset_z &&
         lhs.gyro_offset_x == rhs.gyro_offset_x &&
         lhs.gyro_offset_y == rhs.gyro_offset_y &&
         lhs.gyro_offset_z == rhs.gyro_offset_z &&
         lhs.accel_radius == rhs.accel_radius &&
         lhs.mag_radius == rhs.mag_radius;
}

bool operator!=(
  const BNO055CalibrationProfile & lhs,
  const BNO055CalibrationProfile & rhs)
{
  return !(lhs == rhs);
}

bool validate_bno055_calibration_profile(
  const BNO055CalibrationProfile & profile,
  std::string * error)
{
  auto fail = [error](const std::string & message) {
      if (error != nullptr) {
        *error = message;
      }
      return false;
    };

  if (!value_within(profile.accel_offset_x, ACCEL_OFFSET_LIMIT_LSB) ||
    !value_within(profile.accel_offset_y, ACCEL_OFFSET_LIMIT_LSB) ||
    !value_within(profile.accel_offset_z, ACCEL_OFFSET_LIMIT_LSB))
  {
    return fail("accelerometer offset exceeds BNO055 +/-500 LSB range");
  }
  if (!value_within(profile.mag_offset_x, MAG_OFFSET_LIMIT_LSB) ||
    !value_within(profile.mag_offset_y, MAG_OFFSET_LIMIT_LSB) ||
    !value_within(profile.mag_offset_z, MAG_OFFSET_LIMIT_LSB))
  {
    return fail("magnetometer offset exceeds BNO055 +/-6400 LSB range");
  }
  if (!value_within(profile.gyro_offset_x, GYRO_OFFSET_LIMIT_LSB) ||
    !value_within(profile.gyro_offset_y, GYRO_OFFSET_LIMIT_LSB) ||
    !value_within(profile.gyro_offset_z, GYRO_OFFSET_LIMIT_LSB))
  {
    return fail("gyroscope offset exceeds BNO055 +/-2000 LSB range");
  }
  if (profile.accel_radius > ACCEL_RADIUS_LIMIT_LSB) {
    return fail("accelerometer radius exceeds BNO055 2048 LSB range");
  }
  if (profile.mag_radius < MAG_RADIUS_MIN_LSB ||
    profile.mag_radius > MAG_RADIUS_MAX_LSB)
  {
    return fail("magnetometer radius is outside BNO055 144..1280 LSB range");
  }

  if (error != nullptr) {
    error->clear();
  }
  return true;
}

std::array<uint8_t, bno055_registers::CALIBRATION_BLOCK_LENGTH>
encode_bno055_calibration_profile(const BNO055CalibrationProfile & profile)
{
  std::array<uint8_t, bno055_registers::CALIBRATION_BLOCK_LENGTH> bytes{};
  encode_s16_le(profile.accel_offset_x, bytes, 0U);
  encode_s16_le(profile.accel_offset_y, bytes, 2U);
  encode_s16_le(profile.accel_offset_z, bytes, 4U);
  encode_s16_le(profile.mag_offset_x, bytes, 6U);
  encode_s16_le(profile.mag_offset_y, bytes, 8U);
  encode_s16_le(profile.mag_offset_z, bytes, 10U);
  encode_s16_le(profile.gyro_offset_x, bytes, 12U);
  encode_s16_le(profile.gyro_offset_y, bytes, 14U);
  encode_s16_le(profile.gyro_offset_z, bytes, 16U);
  encode_u16_le(profile.accel_radius, bytes, 18U);
  encode_u16_le(profile.mag_radius, bytes, 20U);
  return bytes;
}

BNO055CalibrationProfile decode_bno055_calibration_profile(
  const std::array<uint8_t, bno055_registers::CALIBRATION_BLOCK_LENGTH> & bytes)
{
  return BNO055CalibrationProfile{
    decode_s16_le(bytes[0], bytes[1]),
    decode_s16_le(bytes[2], bytes[3]),
    decode_s16_le(bytes[4], bytes[5]),
    decode_s16_le(bytes[6], bytes[7]),
    decode_s16_le(bytes[8], bytes[9]),
    decode_s16_le(bytes[10], bytes[11]),
    decode_s16_le(bytes[12], bytes[13]),
    decode_s16_le(bytes[14], bytes[15]),
    decode_s16_le(bytes[16], bytes[17]),
    decode_u16_le(bytes[18], bytes[19]),
    decode_u16_le(bytes[20], bytes[21]),
  };
}

BNO055Driver::BNO055Driver(int i2c_bus, uint8_t address)
: bus_(std::make_unique<I2CBus>(i2c_bus)),
  address_(address)
{
}

BNO055Driver::BNO055Driver(
  std::unique_ptr<I2CBusInterface> bus,
  uint8_t address)
: bus_(std::move(bus)),
  address_(address)
{
  if (!bus_) {
    throw std::invalid_argument("BNO055 I2C bus cannot be null");
  }
}

bool BNO055Driver::open()
{
  if (bus_->is_open()) {
    select_device();
    return true;
  }

  if (!bus_->open()) {
    return false;
  }

  select_device();
  return true;
}

void BNO055Driver::close()
{
  bus_->close();
  mode_ = BNO055Mode::CONFIG;
}

bool BNO055Driver::is_open() const
{
  return bus_->is_open();
}

bool BNO055Driver::initialize(BNO055Mode mode, bool reset_on_start)
{
  if (!initialize_config_mode(reset_on_start)) {
    return false;
  }

  set_mode(mode);
  return true;
}

bool BNO055Driver::initialize_config_mode(bool reset_on_start)
{
  if (!open()) {
    return false;
  }

  if (reset_on_start) {
    reset();
  }

  use_page(0);
  set_power_mode(BNO055PowerMode::NORMAL);
  set_mode(BNO055Mode::CONFIG);

  if (!chip_ok()) {
    return false;
  }

  return true;
}

bool BNO055Driver::chip_ok()
{
  return read_chip_id() == BNO055_CHIP_ID;
}

uint8_t BNO055Driver::read_chip_id()
{
  return read_register(REG_CHIP_ID);
}

void BNO055Driver::reset()
{
  select_device();

  write_register(REG_SYS_TRIGGER, SYS_TRIGGER_RESET);
  std::this_thread::sleep_for(RESET_DELAY);

  use_page(0);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    try {
      if (read_chip_id() == BNO055_CHIP_ID) {
        std::this_thread::sleep_for(STARTUP_DELAY);
        return;
      }
    } catch (const std::exception &) {
      // The device can NACK briefly while rebooting.
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  throw std::runtime_error("BNO055 did not return a valid chip ID after reset");
}

void BNO055Driver::set_mode(BNO055Mode mode)
{
  select_device();
  write_register(REG_OPR_MODE, mode_to_u8(mode));
  mode_ = mode;
  std::this_thread::sleep_for(MODE_SWITCH_DELAY);
}

void BNO055Driver::set_power_mode(BNO055PowerMode mode)
{
  select_device();
  write_register(REG_PWR_MODE, power_mode_to_u8(mode));
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void BNO055Driver::use_page(uint8_t page_id)
{
  select_device();
  write_register(REG_PAGE_ID, page_id);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

BNO055Mode BNO055Driver::mode() const
{
  return mode_;
}

uint8_t BNO055Driver::address() const
{
  return address_;
}

int BNO055Driver::bus_number() const
{
  return bus_->bus_number();
}

Vector3 BNO055Driver::read_accel_mps2()
{
  return read_vector3_scaled(REG_ACCEL_DATA_X_LSB, BNO055_ACCEL_SCALE_MPS2);
}

Vector3 BNO055Driver::read_gyro_dps()
{
  return read_vector3_scaled(REG_GYRO_DATA_X_LSB, BNO055_GYRO_SCALE_DPS);
}

Vector3 BNO055Driver::read_mag_ut()
{
  return read_vector3_scaled(REG_MAG_DATA_X_LSB, BNO055_MAG_SCALE_UT);
}

EulerAngles BNO055Driver::read_euler_deg()
{
  const auto raw = read_vector3_raw(REG_EULER_H_LSB);

  return EulerAngles{
    static_cast<double>(raw[0]) * BNO055_EULER_SCALE_DEG,
    static_cast<double>(raw[1]) * BNO055_EULER_SCALE_DEG,
    static_cast<double>(raw[2]) * BNO055_EULER_SCALE_DEG,
    true,
  };
}

int BNO055Driver::read_temperature_c()
{
  return static_cast<int>(read_register(REG_TEMP));
}

ImuCalibration BNO055Driver::read_calibration()
{
  const uint8_t value = read_register(REG_CALIB_STAT);

  return ImuCalibration{
    static_cast<int>((value >> 6) & 0x03),
    static_cast<int>((value >> 4) & 0x03),
    static_cast<int>((value >> 2) & 0x03),
    static_cast<int>(value & 0x03),
  };
}

uint8_t BNO055Driver::read_system_status()
{
  return read_register(REG_SYS_STATUS);
}

uint8_t BNO055Driver::read_system_error()
{
  return read_register(REG_SYS_ERR);
}

BNO055Status BNO055Driver::read_status()
{
  return BNO055Status{
    read_chip_id(),
    read_system_status(),
    read_system_error(),
    read_calibration(),
  };
}

BNO055CalibrationProfile BNO055Driver::read_calibration_profile()
{
  require_ready();
  const BNO055Mode original_mode = mode_;
  const bool restore_mode = original_mode != BNO055Mode::CONFIG;

  if (restore_mode) {
    set_mode(BNO055Mode::CONFIG);
  }

  try {
    use_page(0);
    const std::vector<uint8_t> values = bus_->read_block(
      bno055_registers::ACC_OFFSET_X_LSB,
      bno055_registers::CALIBRATION_BLOCK_LENGTH);
    if (values.size() != bno055_registers::CALIBRATION_BLOCK_LENGTH) {
      throw std::runtime_error("BNO055 calibration block read was incomplete");
    }
    std::array<uint8_t, bno055_registers::CALIBRATION_BLOCK_LENGTH> bytes{};
    std::copy(values.begin(), values.end(), bytes.begin());
    const auto profile = decode_bno055_calibration_profile(bytes);
    if (restore_mode) {
      set_mode(original_mode);
    }
    return profile;
  } catch (...) {
    const std::exception_ptr failure = std::current_exception();
    if (restore_mode) {
      try {
        set_mode(original_mode);
      } catch (const std::exception & restore_error) {
        throw std::runtime_error(
                std::string("BNO055 calibration read failed and operational mode restore failed: ") +
                restore_error.what());
      }
    }
    std::rethrow_exception(failure);
  }
}

void BNO055Driver::write_calibration_profile(
  const BNO055CalibrationProfile & profile)
{
  std::string validation_error;
  if (!validate_bno055_calibration_profile(profile, &validation_error)) {
    throw std::invalid_argument("invalid BNO055 calibration profile: " + validation_error);
  }

  require_ready();
  const BNO055Mode original_mode = mode_;
  const bool restore_mode = original_mode != BNO055Mode::CONFIG;

  if (restore_mode) {
    set_mode(BNO055Mode::CONFIG);
  }

  try {
    use_page(0);
    const auto encoded = encode_bno055_calibration_profile(profile);
    bus_->write_block(
      bno055_registers::ACC_OFFSET_X_LSB,
      std::vector<uint8_t>(encoded.begin(), encoded.end()));
    const std::vector<uint8_t> readback = bus_->read_block(
      bno055_registers::ACC_OFFSET_X_LSB,
      bno055_registers::CALIBRATION_BLOCK_LENGTH);
    if (readback.size() != encoded.size() ||
      !std::equal(readback.begin(), readback.end(), encoded.begin()))
    {
      throw std::runtime_error("BNO055 calibration profile readback mismatch");
    }
    if (restore_mode) {
      set_mode(original_mode);
    }
  } catch (...) {
    const std::exception_ptr failure = std::current_exception();
    if (restore_mode) {
      try {
        set_mode(original_mode);
      } catch (const std::exception & restore_error) {
        throw std::runtime_error(
                std::string("BNO055 calibration write failed and operational mode restore failed: ") +
                restore_error.what());
      }
    }
    std::rethrow_exception(failure);
  }
}

BNO055Sample BNO055Driver::read_sample(
  bool read_magnetic,
  bool read_euler,
  bool read_temperature)
{
  require_ready();

  BNO055Sample sample{};
  sample.accel_mps2 = read_accel_mps2();
  sample.gyro_dps = read_gyro_dps();

  if (read_magnetic) {
    sample.mag_ut = read_mag_ut();
    sample.magnetic_available = true;
  }

  if (read_euler && mode_ == BNO055Mode::NDOF) {
    sample.euler_deg = read_euler_deg();
  }

  if (read_temperature) {
    sample.temperature_c = read_temperature_c();
    sample.temperature_available = true;
  }

  sample.status = read_status();
  return sample;
}

std::string BNO055Driver::mode_name() const
{
  return mode_name(mode_);
}

std::string BNO055Driver::mode_name(BNO055Mode mode)
{
  switch (mode) {
    case BNO055Mode::CONFIG:
      return "config";
    case BNO055Mode::IMU:
      return "imu";
    case BNO055Mode::NDOF:
      return "ndof";
  }

  return "unknown";
}

void BNO055Driver::select_device()
{
  bus_->set_slave_address(address_);
}

void BNO055Driver::require_ready()
{
  if (!is_open()) {
    throw std::runtime_error("BNO055 driver is not open");
  }

  select_device();

  if (!chip_ok()) {
    throw std::runtime_error("BNO055 chip ID check failed");
  }
}

Vector3 BNO055Driver::read_vector3_scaled(
  uint8_t start_register,
  double scale)
{
  const auto raw = read_vector3_raw(start_register);

  return Vector3{
    static_cast<double>(raw[0]) * scale,
    static_cast<double>(raw[1]) * scale,
    static_cast<double>(raw[2]) * scale,
  };
}

std::array<int16_t, 3> BNO055Driver::read_vector3_raw(uint8_t start_register)
{
  const auto bytes = bus_->read_block(start_register, 6);

  auto read_axis = [&bytes](std::size_t index) -> int16_t {
    const uint16_t value =
      static_cast<uint16_t>(bytes[index]) |
      (static_cast<uint16_t>(bytes[index + 1]) << 8);

    return static_cast<int16_t>(value);
  };

  return {
    read_axis(0),
    read_axis(2),
    read_axis(4),
  };
}

uint8_t BNO055Driver::read_register(uint8_t reg)
{
  select_device();
  return bus_->read_u8(reg);
}

void BNO055Driver::write_register(uint8_t reg, uint8_t value)
{
  select_device();
  bus_->write_u8(reg, value);
}

uint8_t BNO055Driver::mode_to_u8(BNO055Mode mode)
{
  return static_cast<uint8_t>(mode);
}

uint8_t BNO055Driver::power_mode_to_u8(BNO055PowerMode mode)
{
  return static_cast<uint8_t>(mode);
}

}  // namespace savo_localization
