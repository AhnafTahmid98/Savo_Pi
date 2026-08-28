#pragma once

#include "savo_base/base_types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace savo_base
{

namespace freenove_detail
{

inline constexpr int kPcaMode1 = 0x00;
inline constexpr int kPcaMode2 = 0x01;
inline constexpr int kPcaPrescale = 0xFE;
inline constexpr int kPcaAllLedOffH = 0xFD;
inline constexpr int kLed0OnL = 0x06;

inline constexpr int kMode1Restart = 0x80;
inline constexpr int kMode1Sleep = 0x10;
inline constexpr int kMode1AllCall = 0x01;
inline constexpr int kMode1AutoIncrement = 0x20;
inline constexpr int kMode2OutDrv = 0x04;
inline constexpr int kPcaFullOff = 0x10;

inline constexpr double kOscillatorHz = 25000000.0;
inline constexpr int kPwmSteps = 4096;

struct MotorPairPwm
{
  int channel_a{0};
  int channel_b{0};
};

inline MotorPairPwm motor_pair_pwm(const int value)
{
  const int duty = std::clamp(value, -4095, 4095);
  if (duty > 0) {
    return {0, duty};
  }
  if (duty < 0) {
    return {-duty, 0};
  }
  return {4095, 4095};
}

struct PwmChannelRegisters
{
  std::uint8_t on_l{0};
  std::uint8_t on_h{0};
  std::uint8_t off_l{0};
  std::uint8_t off_h{0};
};

inline PwmChannelRegisters pwm_channel_registers(const int duty)
{
  const int value = std::clamp(duty, 0, 4095);
  if (value >= 4095) {
    return {0x00, 0x10, 0x00, 0x00};
  }
  return {
    0x00,
    0x00,
    static_cast<std::uint8_t>(value & 0xFF),
    static_cast<std::uint8_t>((value >> 8) & 0x0F)};
}

inline std::array<std::uint8_t, 64> startup_brake_registers(
  const WheelChannels & channels)
{
  std::array<std::uint8_t, 64> registers{};

  // Keep every unused PCA9685 output fully off.
  for (int channel = 0; channel < 16; ++channel) {
    registers[4 * channel + 3] = kPcaFullOff;
  }

  const std::array<std::array<int, 2>, 4> motor_channels{
    channels.fl, channels.rl, channels.fr, channels.rr};
  for (const auto & pair : motor_channels) {
    for (const int channel : pair) {
      if (channel < 0 || channel > 15) {
        throw std::runtime_error("PCA9685 channel out of range");
      }
      registers[4 * channel + 1] = 0x10;
      registers[4 * channel + 3] = 0x00;
    }
  }

  return registers;
}

template<
  typename WriteRegister,
  typename ReadRegister,
  typename WriteBrakeFrame,
  typename Sleep>
void initialize_pca9685_fail_safe(
  const double frequency_hz,
  WriteRegister write_register,
  ReadRegister read_register,
  WriteBrakeFrame write_brake_frame,
  Sleep sleep)
{
  try {
    // This is a single-register write and is safe before AUTO_INCREMENT is enabled.
    write_register(kPcaAllLedOffH, kPcaFullOff);

    write_register(kPcaMode2, kMode2OutDrv);
    write_register(kPcaMode1, kMode1AllCall | kMode1AutoIncrement);
    sleep(5);

    const double safe_frequency = std::clamp(frequency_hz, 24.0, 1526.0);
    const double prescale_value =
      (kOscillatorHz / (kPwmSteps * safe_frequency)) - 1.0;
    const int prescale = static_cast<int>(std::lround(prescale_value));

    const int old_mode =
      read_register(kPcaMode1) | kMode1AutoIncrement | kMode1AllCall;
    const int sleep_mode = (old_mode & 0x7F) | kMode1Sleep;

    write_register(kPcaMode1, sleep_mode);
    write_register(kPcaPrescale, prescale);
    write_register(kPcaMode1, old_mode);
    sleep(5);
    write_register(kPcaMode1, old_mode | kMode1Restart);

    // AUTO_INCREMENT is now enabled and MODE2 OCH remains zero. The production
    // callback loads all channel registers in one transaction, so the STOP
    // condition atomically replaces full-off with the established brake state.
    write_brake_frame();
  } catch (...) {
    try {
      write_register(kPcaAllLedOffH, kPcaFullOff);
    } catch (...) {
    }
    throw;
  }
}

template<typename Stop, typename ForceOutputsOff, typename Close>
void shutdown_fail_safe(Stop stop, ForceOutputsOff force_outputs_off, Close close) noexcept
{
  try {
    stop();
  } catch (...) {
    try {
      force_outputs_off();
    } catch (...) {
    }
  }

  try {
    close();
  } catch (...) {
  }
}

}  // namespace freenove_detail

class FreenoveMotorBoard
{
public:
  FreenoveMotorBoard(
    const BoardConfig & board_config,
    const WheelChannels & channels,
    const WheelInverts & inverts);

  ~FreenoveMotorBoard();

  FreenoveMotorBoard(const FreenoveMotorBoard &) = delete;
  FreenoveMotorBoard & operator=(const FreenoveMotorBoard &) = delete;

  bool connected() const;
  bool ping() const;

  void write(const WheelDuty & duty);
  void stop();
  void close();

private:
  static int clamp_duty(int value);
  static int apply_invert(int value, bool invert);
  static int sign_of(int value);

  void open_bus();
  void configure_pwm();
  void write_startup_brake_frame();

  void set_wheel(
    const std::array<int, 2> & channels,
    int value,
    bool invert,
    int wheel_index);

  void set_motor_pair(int channel_a, int channel_b, int value);
  void set_pwm_channel(int channel, int duty);

  void write_register(int reg, int value) const;
  int read_register(int reg) const;

  BoardConfig board_config_;
  WheelChannels channels_;
  WheelInverts inverts_;

  int fd_{-1};
  std::array<int, 4> last_output_{0, 0, 0, 0};
};

}  // namespace savo_base
