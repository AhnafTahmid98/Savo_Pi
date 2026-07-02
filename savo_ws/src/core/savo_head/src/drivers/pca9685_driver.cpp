#include "savo_head/drivers/pca9685_driver.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace savo_head
{

namespace
{

std::string errno_message(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

}  // namespace

Pca9685Driver::Pca9685Driver(Pca9685Config config)
: config_(config.normalized())
{
}

Pca9685Driver::~Pca9685Driver()
{
  close();
}

Pca9685Driver::Pca9685Driver(Pca9685Driver && other) noexcept
{
  *this = std::move(other);
}

Pca9685Driver & Pca9685Driver::operator=(Pca9685Driver && other) noexcept
{
  if (this == &other) {
    return *this;
  }

  close();

  config_ = other.config_;
  fd_ = other.fd_;
  opened_ = other.opened_;
  last_error_ = std::move(other.last_error_);
  writes_ = std::move(other.writes_);

  other.fd_ = -1;
  other.opened_ = false;
  other.last_error_.reset();

  return *this;
}

void Pca9685Driver::open()
{
  if (opened_) {
    return;
  }

  config_ = config_.normalized();
  clear_error();

  if (config_.dryrun) {
    opened_ = true;
    return;
  }

  const auto path = config_.device_path();
  fd_ = ::open(path.c_str(), O_RDWR);

  if (fd_ < 0) {
    set_error(errno_message("failed to open " + path));
    throw std::runtime_error(*last_error_);
  }

  if (::ioctl(fd_, I2C_SLAVE, config_.address) < 0) {
    set_error(errno_message("failed to select PCA9685 I2C address"));
    close();
    throw std::runtime_error(*last_error_);
  }

  opened_ = true;

  try {
    write_byte(kPca9685Mode1Reg, 0x00);
    const auto prescale = set_pwm_frequency(config_.pwm_frequency_hz);
    (void)prescale;
  } catch (...) {
    close();
    throw;
  }
}

void Pca9685Driver::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }

  opened_ = false;
}

bool Pca9685Driver::opened() const
{
  return opened_;
}

bool Pca9685Driver::dryrun() const
{
  return config_.dryrun;
}

int Pca9685Driver::file_descriptor() const
{
  return fd_;
}

const Pca9685Config & Pca9685Driver::config() const
{
  return config_;
}

std::optional<std::string> Pca9685Driver::last_error() const
{
  return last_error_;
}

const std::vector<I2cWrite> & Pca9685Driver::writes() const
{
  return writes_;
}

void Pca9685Driver::clear_error()
{
  last_error_.reset();
}

void Pca9685Driver::clear_writes()
{
  writes_.clear();
}

void Pca9685Driver::write_byte(std::uint8_t reg, std::uint8_t value)
{
  record_write(reg, value);

  if (config_.dryrun) {
    return;
  }

  ensure_open();

  const std::uint8_t data[2] = {reg, value};
  const auto written = ::write(fd_, data, sizeof(data));

  if (written != static_cast<ssize_t>(sizeof(data))) {
    set_error(errno_message("failed to write PCA9685 register"));
    throw std::runtime_error(*last_error_);
  }
}

std::uint8_t Pca9685Driver::read_byte(std::uint8_t reg)
{
  if (config_.dryrun) {
    return 0x00;
  }

  ensure_open();

  const auto reg_write = ::write(fd_, &reg, 1);
  if (reg_write != 1) {
    set_error(errno_message("failed to select PCA9685 register"));
    throw std::runtime_error(*last_error_);
  }

  std::uint8_t value = 0;
  const auto read_count = ::read(fd_, &value, 1);

  if (read_count != 1) {
    set_error(errno_message("failed to read PCA9685 register"));
    throw std::runtime_error(*last_error_);
  }

  return value;
}

int Pca9685Driver::set_pwm_frequency(double frequency_hz)
{
  const auto prescale = pca9685_prescale_for_frequency(frequency_hz);

  const auto old_mode = read_byte(kPca9685Mode1Reg);
  const auto sleep_mode = static_cast<std::uint8_t>((old_mode & 0x7fU) | kPca9685Mode1Sleep);

  write_byte(kPca9685Mode1Reg, sleep_mode);
  write_byte(kPca9685PrescaleReg, static_cast<std::uint8_t>(prescale));
  write_byte(kPca9685Mode1Reg, old_mode);
  write_byte(kPca9685Mode1Reg, static_cast<std::uint8_t>(old_mode | kPca9685Mode1Restart));

  return prescale;
}

void Pca9685Driver::set_pwm(int channel, int on_ticks, int off_ticks)
{
  if (!valid_pca9685_channel(channel)) {
    throw std::out_of_range("PCA9685 channel must be 0..15");
  }

  const auto on = clamp_pca9685_ticks(on_ticks);
  const auto off = clamp_pca9685_ticks(off_ticks);

  const auto base = static_cast<std::uint8_t>(kPca9685Led0OnLReg + 4 * channel);

  write_byte(base, static_cast<std::uint8_t>(on & 0xff));
  write_byte(static_cast<std::uint8_t>(base + 1), static_cast<std::uint8_t>(on >> 8));
  write_byte(static_cast<std::uint8_t>(base + 2), static_cast<std::uint8_t>(off & 0xff));
  write_byte(static_cast<std::uint8_t>(base + 3), static_cast<std::uint8_t>(off >> 8));
}

int Pca9685Driver::set_servo_pulse_us(int channel, double pulse_us)
{
  const auto ticks = pulse_us_to_ticks(pulse_us);
  set_pwm(channel, 0, ticks);
  return ticks;
}

void Pca9685Driver::stop_channel(int channel)
{
  set_pwm(channel, 0, 0);
}

void Pca9685Driver::stop_all()
{
  for (int channel = 0; channel <= 15; ++channel) {
    stop_channel(channel);
  }
}

void Pca9685Driver::ensure_open() const
{
  if (!opened_) {
    throw std::runtime_error("PCA9685 driver is not open");
  }

  if (!config_.dryrun && fd_ < 0) {
    throw std::runtime_error("PCA9685 I2C file descriptor is invalid");
  }
}

void Pca9685Driver::record_write(std::uint8_t reg, std::uint8_t value)
{
  writes_.push_back(I2cWrite{reg, value});
}

void Pca9685Driver::set_error(std::string message)
{
  last_error_ = std::move(message);
}

}  // namespace savo_head
