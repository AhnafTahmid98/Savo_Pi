#include "savo_base/freenove_motor_board.hpp"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <thread>

namespace savo_base
{

FreenoveMotorBoard::FreenoveMotorBoard(
  const BoardConfig & board_config,
  const WheelChannels & channels,
  const WheelInverts & inverts)
: board_config_(board_config),
  channels_(channels),
  inverts_(inverts)
{
  if (board_config_.dryrun) {
    return;
  }

  open_bus();
  try {
    configure_pwm();
  } catch (...) {
    close();
    throw;
  }
}

FreenoveMotorBoard::~FreenoveMotorBoard()
{
  freenove_detail::shutdown_fail_safe(
    [this]() {stop();},
    [this]() {
      if (fd_ >= 0) {
        write_register(freenove_detail::kPcaAllLedOffH, freenove_detail::kPcaFullOff);
      }
    },
    [this]() {close();});
}

bool FreenoveMotorBoard::connected() const
{
  return board_config_.dryrun || fd_ >= 0;
}

bool FreenoveMotorBoard::ping() const
{
  if (board_config_.dryrun) {
    return true;
  }

  if (fd_ < 0) {
    return false;
  }

  try {
    static_cast<void>(read_register(freenove_detail::kPcaMode1));
    return true;
  } catch (...) {
    return false;
  }
}

void FreenoveMotorBoard::write(const WheelDuty & duty)
{
  if (board_config_.dryrun) {
    return;
  }

  if (fd_ < 0) {
    throw std::runtime_error("Freenove board is not open");
  }

  set_wheel(channels_.fl, duty.fl, inverts_.fl, 0);
  set_wheel(channels_.rl, duty.rl, inverts_.rl, 1);
  set_wheel(channels_.fr, duty.fr, inverts_.fr, 2);
  set_wheel(channels_.rr, duty.rr, inverts_.rr, 3);
}

void FreenoveMotorBoard::stop()
{
  if (board_config_.dryrun) {
    last_output_ = {0, 0, 0, 0};
    return;
  }

  if (fd_ < 0) {
    return;
  }

  set_motor_pair(channels_.fl[0], channels_.fl[1], 0);
  set_motor_pair(channels_.rl[0], channels_.rl[1], 0);
  set_motor_pair(channels_.fr[0], channels_.fr[1], 0);
  set_motor_pair(channels_.rr[0], channels_.rr[1], 0);

  last_output_ = {0, 0, 0, 0};
}

void FreenoveMotorBoard::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

int FreenoveMotorBoard::clamp_duty(const int value)
{
  return std::clamp(value, -4095, 4095);
}

int FreenoveMotorBoard::apply_invert(const int value, const bool invert)
{
  return invert ? -value : value;
}

int FreenoveMotorBoard::sign_of(const int value)
{
  if (value > 0) {
    return 1;
  }
  if (value < 0) {
    return -1;
  }
  return 0;
}

void FreenoveMotorBoard::open_bus()
{
  const std::string device = "/dev/i2c-" + std::to_string(board_config_.i2c_bus);

  fd_ = ::open(device.c_str(), O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error("Failed to open " + device);
  }

  if (::ioctl(fd_, I2C_SLAVE, board_config_.address) < 0) {
    close();
    throw std::runtime_error("Failed to select I2C address");
  }
}

void FreenoveMotorBoard::configure_pwm()
{
  freenove_detail::initialize_pca9685_fail_safe(
    board_config_.pwm_freq_hz,
    [this](const int reg, const int value) {write_register(reg, value);},
    [this](const int reg) {return read_register(reg);},
    [this]() {write_startup_brake_frame();},
    [](const int milliseconds) {
      std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    });
}

void FreenoveMotorBoard::write_startup_brake_frame()
{
  const auto registers = freenove_detail::startup_brake_registers(channels_);
  std::array<std::uint8_t, 65> buffer{};
  buffer[0] = freenove_detail::kLed0OnL;
  std::copy(registers.begin(), registers.end(), buffer.begin() + 1);

  if (::write(fd_, buffer.data(), buffer.size()) !=
    static_cast<ssize_t>(buffer.size()))
  {
    throw std::runtime_error("Failed to write PCA9685 startup brake frame");
  }

  last_output_ = {0, 0, 0, 0};
}

void FreenoveMotorBoard::set_wheel(
  const std::array<int, 2> & channels,
  const int value,
  const bool invert,
  const int wheel_index)
{
  const int output = clamp_duty(apply_invert(value, invert));

  const bool direction_flip =
    sign_of(output) != 0 &&
    sign_of(last_output_[wheel_index]) != 0 &&
    sign_of(output) != sign_of(last_output_[wheel_index]);

  if (direction_flip && board_config_.quench_ms > 0) {
    set_motor_pair(channels[0], channels[1], 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(board_config_.quench_ms));
  }

  set_motor_pair(channels[0], channels[1], output);
  last_output_[wheel_index] = output;
}

void FreenoveMotorBoard::set_motor_pair(
  const int channel_a,
  const int channel_b,
  const int value)
{
  const auto pwm = freenove_detail::motor_pair_pwm(value);
  set_pwm_channel(channel_a, pwm.channel_a);
  set_pwm_channel(channel_b, pwm.channel_b);
}

void FreenoveMotorBoard::set_pwm_channel(const int channel, const int duty)
{
  if (channel < 0 || channel > 15) {
    throw std::runtime_error("PCA9685 channel out of range");
  }

  const int reg = freenove_detail::kLed0OnL + 4 * channel;
  const auto registers = freenove_detail::pwm_channel_registers(duty);

  uint8_t buffer[5]{};
  buffer[0] = static_cast<uint8_t>(reg);
  buffer[1] = registers.on_l;
  buffer[2] = registers.on_h;
  buffer[3] = registers.off_l;
  buffer[4] = registers.off_h;

  if (::write(fd_, buffer, sizeof(buffer)) != static_cast<ssize_t>(sizeof(buffer))) {
    throw std::runtime_error("Failed to write PCA9685 PWM channel");
  }
}

void FreenoveMotorBoard::write_register(const int reg, const int value) const
{
  uint8_t buffer[2]{
    static_cast<uint8_t>(reg & 0xFF),
    static_cast<uint8_t>(value & 0xFF)
  };

  if (::write(fd_, buffer, sizeof(buffer)) != static_cast<ssize_t>(sizeof(buffer))) {
    throw std::runtime_error("Failed to write PCA9685 register");
  }
}

int FreenoveMotorBoard::read_register(const int reg) const
{
  const uint8_t register_address = static_cast<uint8_t>(reg & 0xFF);

  if (::write(fd_, &register_address, 1) != 1) {
    throw std::runtime_error("Failed to select PCA9685 register");
  }

  uint8_t value = 0;
  if (::read(fd_, &value, 1) != 1) {
    throw std::runtime_error("Failed to read PCA9685 register");
  }

  return static_cast<int>(value);
}

}  // namespace savo_base
