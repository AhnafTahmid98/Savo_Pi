#pragma once

#include "savo_base/base_types.hpp"

#include <array>
#include <string>

namespace savo_base
{

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
  void set_pwm_frequency(double frequency_hz);

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
