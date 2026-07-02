#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "savo_head/core/servo_calibration.hpp"

namespace savo_head
{

inline constexpr std::uint8_t kPca9685Mode1Reg = 0x00;
inline constexpr std::uint8_t kPca9685PrescaleReg = 0xFE;

inline constexpr std::uint8_t kPca9685Led0OnLReg = 0x06;
inline constexpr std::uint8_t kPca9685Led0OnHReg = 0x07;
inline constexpr std::uint8_t kPca9685Led0OffLReg = 0x08;
inline constexpr std::uint8_t kPca9685Led0OffHReg = 0x09;

inline constexpr std::uint8_t kPca9685Mode1Sleep = 0x10;
inline constexpr std::uint8_t kPca9685Mode1Restart = 0x80;

inline constexpr double kPca9685OscillatorHz = 25'000'000.0;

struct I2cWrite
{
  std::uint8_t reg{0};
  std::uint8_t value{0};
};

struct Pca9685Config
{
  int i2c_bus{kI2cBusDefault};
  int address{kPca9685AddressDefault};
  double pwm_frequency_hz{kPca9685PwmFrequencyHzDefault};
  bool dryrun{false};

  [[nodiscard]] Pca9685Config normalized() const
  {
    Pca9685Config out = *this;

    if (out.i2c_bus < 0) {
      out.i2c_bus = kI2cBusDefault;
    }

    if (out.address <= 0 || out.address > 0x7f) {
      out.address = kPca9685AddressDefault;
    }

    if (out.pwm_frequency_hz <= 0.0) {
      out.pwm_frequency_hz = kPca9685PwmFrequencyHzDefault;
    }

    return out;
  }

  [[nodiscard]] std::string device_path() const
  {
    return "/dev/i2c-" + std::to_string(normalized().i2c_bus);
  }
};

class Pca9685Driver
{
public:
  explicit Pca9685Driver(Pca9685Config config = Pca9685Config{});
  ~Pca9685Driver();

  Pca9685Driver(const Pca9685Driver &) = delete;
  Pca9685Driver & operator=(const Pca9685Driver &) = delete;

  Pca9685Driver(Pca9685Driver && other) noexcept;
  Pca9685Driver & operator=(Pca9685Driver && other) noexcept;

  void open();
  void close();

  [[nodiscard]] bool opened() const;
  [[nodiscard]] bool dryrun() const;
  [[nodiscard]] int file_descriptor() const;
  [[nodiscard]] const Pca9685Config & config() const;
  [[nodiscard]] std::optional<std::string> last_error() const;
  [[nodiscard]] const std::vector<I2cWrite> & writes() const;

  void clear_error();
  void clear_writes();

  void write_byte(std::uint8_t reg, std::uint8_t value);
  [[nodiscard]] std::uint8_t read_byte(std::uint8_t reg);

  [[nodiscard]] int set_pwm_frequency(double frequency_hz);

  void set_pwm(int channel, int on_ticks, int off_ticks);
  [[nodiscard]] int set_servo_pulse_us(int channel, double pulse_us);

  void stop_channel(int channel);
  void stop_all();

private:
  void ensure_open() const;
  void record_write(std::uint8_t reg, std::uint8_t value);
  void set_error(std::string message);

  Pca9685Config config_{};
  int fd_{-1};
  bool opened_{false};
  std::optional<std::string> last_error_{};
  std::vector<I2cWrite> writes_{};
};

[[nodiscard]] inline int pca9685_prescale_for_frequency(double frequency_hz)
{
  if (frequency_hz <= 0.0) {
    throw std::invalid_argument("frequency_hz must be positive");
  }

  auto prescale = kPca9685OscillatorHz;
  prescale /= static_cast<double>(kPca9685TicksPerCycle);
  prescale /= frequency_hz;
  prescale -= 1.0;

  return static_cast<int>(prescale + 0.5);
}

[[nodiscard]] inline bool valid_pca9685_channel(int channel)
{
  return channel >= 0 && channel <= 15;
}

[[nodiscard]] inline bool valid_servo_pulse_us(double pulse_us)
{
  return pulse_us >= kServoPulseMinUs && pulse_us <= kServoPulseMaxUs;
}

[[nodiscard]] inline int clamp_pca9685_ticks(int ticks)
{
  return std::clamp(ticks, 0, kPca9685TicksPerCycle - 1);
}

}  // namespace savo_head
