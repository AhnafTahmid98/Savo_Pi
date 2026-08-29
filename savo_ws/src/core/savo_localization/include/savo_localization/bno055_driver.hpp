#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "savo_localization/i2c_bus.hpp"

namespace savo_localization
{

constexpr uint8_t BNO055_DEFAULT_ADDRESS = 0x28;
constexpr uint8_t BNO055_CHIP_ID = 0xA0;

constexpr double BNO055_ACCEL_SCALE_MPS2 = 1.0 / 100.0;
constexpr double BNO055_GYRO_SCALE_DPS = 1.0 / 16.0;
constexpr double BNO055_EULER_SCALE_DEG = 1.0 / 16.0;
constexpr double BNO055_MAG_SCALE_UT = 1.0 / 16.0;

namespace bno055_registers
{

constexpr uint8_t ACC_OFFSET_X_LSB = 0x55;
constexpr uint8_t ACC_OFFSET_X_MSB = 0x56;
constexpr uint8_t ACC_OFFSET_Y_LSB = 0x57;
constexpr uint8_t ACC_OFFSET_Y_MSB = 0x58;
constexpr uint8_t ACC_OFFSET_Z_LSB = 0x59;
constexpr uint8_t ACC_OFFSET_Z_MSB = 0x5A;
constexpr uint8_t MAG_OFFSET_X_LSB = 0x5B;
constexpr uint8_t MAG_OFFSET_X_MSB = 0x5C;
constexpr uint8_t MAG_OFFSET_Y_LSB = 0x5D;
constexpr uint8_t MAG_OFFSET_Y_MSB = 0x5E;
constexpr uint8_t MAG_OFFSET_Z_LSB = 0x5F;
constexpr uint8_t MAG_OFFSET_Z_MSB = 0x60;
constexpr uint8_t GYR_OFFSET_X_LSB = 0x61;
constexpr uint8_t GYR_OFFSET_X_MSB = 0x62;
constexpr uint8_t GYR_OFFSET_Y_LSB = 0x63;
constexpr uint8_t GYR_OFFSET_Y_MSB = 0x64;
constexpr uint8_t GYR_OFFSET_Z_LSB = 0x65;
constexpr uint8_t GYR_OFFSET_Z_MSB = 0x66;
constexpr uint8_t ACC_RADIUS_LSB = 0x67;
constexpr uint8_t ACC_RADIUS_MSB = 0x68;
constexpr uint8_t MAG_RADIUS_LSB = 0x69;
constexpr uint8_t MAG_RADIUS_MSB = 0x6A;
constexpr std::size_t CALIBRATION_BLOCK_LENGTH = 22U;

}  // namespace bno055_registers

enum class BNO055Mode : uint8_t
{
  CONFIG = 0x00,
  IMU = 0x08,
  NDOF = 0x0C,
};

enum class BNO055PowerMode : uint8_t
{
  NORMAL = 0x00,
  LOW_POWER = 0x01,
  SUSPEND = 0x02,
};

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct EulerAngles
{
  double yaw_deg{0.0};
  double roll_deg{0.0};
  double pitch_deg{0.0};
  bool available{false};
};

struct ImuCalibration
{
  int system{0};
  int gyro{0};
  int accel{0};
  int mag{0};

  bool motion_ready() const;
  bool fully_calibrated() const;
};

struct BNO055CalibrationProfile
{
  int16_t accel_offset_x{0};
  int16_t accel_offset_y{0};
  int16_t accel_offset_z{0};
  int16_t mag_offset_x{0};
  int16_t mag_offset_y{0};
  int16_t mag_offset_z{0};
  int16_t gyro_offset_x{0};
  int16_t gyro_offset_y{0};
  int16_t gyro_offset_z{0};
  uint16_t accel_radius{0};
  uint16_t mag_radius{0};
};

bool operator==(
  const BNO055CalibrationProfile & lhs,
  const BNO055CalibrationProfile & rhs);
bool operator!=(
  const BNO055CalibrationProfile & lhs,
  const BNO055CalibrationProfile & rhs);

bool validate_bno055_calibration_profile(
  const BNO055CalibrationProfile & profile,
  std::string * error = nullptr);

std::array<uint8_t, bno055_registers::CALIBRATION_BLOCK_LENGTH>
encode_bno055_calibration_profile(const BNO055CalibrationProfile & profile);

BNO055CalibrationProfile decode_bno055_calibration_profile(
  const std::array<uint8_t, bno055_registers::CALIBRATION_BLOCK_LENGTH> & bytes);

struct BNO055Status
{
  uint8_t chip_id{0};
  uint8_t system_status{0};
  uint8_t system_error{0};
  ImuCalibration calibration{};
};

struct BNO055Sample
{
  Vector3 accel_mps2{};
  Vector3 gyro_dps{};
  Vector3 mag_ut{};
  EulerAngles euler_deg{};

  int temperature_c{0};
  bool temperature_available{false};
  bool magnetic_available{false};

  BNO055Status status{};
};

class BNO055Driver
{
public:
  explicit BNO055Driver(
    int i2c_bus = 1,
    uint8_t address = BNO055_DEFAULT_ADDRESS);
  BNO055Driver(
    std::unique_ptr<I2CBusInterface> bus,
    uint8_t address = BNO055_DEFAULT_ADDRESS);

  ~BNO055Driver() = default;

  BNO055Driver(const BNO055Driver &) = delete;
  BNO055Driver & operator=(const BNO055Driver &) = delete;

  BNO055Driver(BNO055Driver &&) noexcept = default;
  BNO055Driver & operator=(BNO055Driver &&) noexcept = default;

  bool open();
  void close();
  bool is_open() const;

  bool initialize(
    BNO055Mode mode = BNO055Mode::NDOF,
    bool reset_on_start = true);
  bool initialize_config_mode(bool reset_on_start = true);

  bool chip_ok();
  uint8_t read_chip_id();

  void reset();
  void set_mode(BNO055Mode mode);
  void set_power_mode(BNO055PowerMode mode);
  void use_page(uint8_t page_id);

  BNO055Mode mode() const;
  uint8_t address() const;
  int bus_number() const;

  Vector3 read_accel_mps2();
  Vector3 read_gyro_dps();
  Vector3 read_mag_ut();
  EulerAngles read_euler_deg();

  int read_temperature_c();
  ImuCalibration read_calibration();
  uint8_t read_system_status();
  uint8_t read_system_error();
  BNO055Status read_status();
  BNO055CalibrationProfile read_calibration_profile();
  void write_calibration_profile(const BNO055CalibrationProfile & profile);

  BNO055Sample read_sample(
    bool read_magnetic = true,
    bool read_euler = true,
    bool read_temperature = true);

  std::string mode_name() const;
  static std::string mode_name(BNO055Mode mode);

private:
  std::unique_ptr<I2CBusInterface> bus_;
  uint8_t address_{BNO055_DEFAULT_ADDRESS};
  BNO055Mode mode_{BNO055Mode::CONFIG};

  void select_device();
  void require_ready();

  Vector3 read_vector3_scaled(
    uint8_t start_register,
    double scale);

  std::array<int16_t, 3> read_vector3_raw(uint8_t start_register);

  uint8_t read_register(uint8_t reg);
  void write_register(uint8_t reg, uint8_t value);

  static uint8_t mode_to_u8(BNO055Mode mode);
  static uint8_t power_mode_to_u8(BNO055PowerMode mode);
};

}  // namespace savo_localization
