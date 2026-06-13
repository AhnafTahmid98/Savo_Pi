#pragma once

#include <array>
#include <cstdint>
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

  BNO055Sample read_sample(
    bool read_magnetic = true,
    bool read_euler = true,
    bool read_temperature = true);

  std::string mode_name() const;
  static std::string mode_name(BNO055Mode mode);

private:
  I2CBus bus_;
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