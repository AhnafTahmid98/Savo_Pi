#include "savo_localization/bno055_driver.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

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

}  // namespace

bool ImuCalibration::motion_ready() const
{
  return gyro >= 2 && accel >= 2;
}

bool ImuCalibration::fully_calibrated() const
{
  return system >= 3 && gyro >= 3 && accel >= 3 && mag >= 3;
}

BNO055Driver::BNO055Driver(int i2c_bus, uint8_t address)
: bus_(i2c_bus),
  address_(address)
{
}

bool BNO055Driver::open()
{
  if (bus_.is_open()) {
    select_device();
    return true;
  }

  if (!bus_.open()) {
    return false;
  }

  select_device();
  return true;
}

void BNO055Driver::close()
{
  bus_.close();
  mode_ = BNO055Mode::CONFIG;
}

bool BNO055Driver::is_open() const
{
  return bus_.is_open();
}

bool BNO055Driver::initialize(BNO055Mode mode, bool reset_on_start)
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

  set_mode(mode);
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
  return bus_.bus_number();
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
  bus_.set_slave_address(address_);
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
  const auto bytes = bus_.read_block(start_register, 6);

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
  return bus_.read_u8(reg);
}

void BNO055Driver::write_register(uint8_t reg, uint8_t value)
{
  select_device();
  bus_.write_u8(reg, value);
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