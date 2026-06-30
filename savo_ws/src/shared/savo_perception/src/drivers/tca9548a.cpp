#include "savo_perception/tca9548a.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <sstream>
#include <utility>

namespace savo_perception
{

bool valid_tca_channel(const int channel)
{
  return channel >= 0 && channel <= 7;
}

std::uint8_t tca_channel_mask(const int channel)
{
  if (!valid_tca_channel(channel)) {
    return 0x00;
  }

  return static_cast<std::uint8_t>(1U << static_cast<unsigned int>(channel));
}

Tca9548a::Tca9548a(Tca9548aConfig config)
: config_(config)
{
}

Tca9548a::~Tca9548a()
{
  close();
}

Tca9548a::Tca9548a(Tca9548a && other) noexcept
{
  move_from(std::move(other));
}

Tca9548a & Tca9548a::operator=(Tca9548a && other) noexcept
{
  if (this != &other) {
    close();
    move_from(std::move(other));
  }

  return *this;
}

const Tca9548aConfig & Tca9548a::config() const
{
  return config_;
}

bool Tca9548a::is_open() const
{
  return fd_ >= 0;
}

int Tca9548a::fd() const
{
  return fd_;
}

std::string Tca9548a::bus_device() const
{
  std::ostringstream out;
  out << "/dev/i2c-" << config_.bus;
  return out.str();
}

bool Tca9548a::open()
{
  if (is_open()) {
    return true;
  }

  const auto device = bus_device();
  fd_ = ::open(device.c_str(), O_RDWR);

  if (fd_ < 0) {
    last_error_ = "open_failed:" + device + ":" + std::strerror(errno);
    return false;
  }

  if (ioctl(fd_, I2C_SLAVE, config_.address) < 0) {
    last_error_ = "ioctl_i2c_slave_failed:" + std::string(std::strerror(errno));
    close();
    return false;
  }

  last_error_.clear();
  return true;
}

void Tca9548a::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool Tca9548a::select_channel(const int channel)
{
  if (!valid_tca_channel(channel)) {
    last_error_ = "invalid_channel";
    return false;
  }

  return write_mask(tca_channel_mask(channel));
}

bool Tca9548a::disable_all()
{
  return write_mask(0x00);
}

std::string Tca9548a::last_error() const
{
  return last_error_;
}

bool Tca9548a::write_mask(const std::uint8_t mask)
{
  if (!is_open() && !open()) {
    return false;
  }

  const auto written = ::write(fd_, &mask, 1);

  if (written != 1) {
    last_error_ = "write_failed:" + std::string(std::strerror(errno));
    return false;
  }

  last_error_.clear();
  return true;
}

void Tca9548a::move_from(Tca9548a && other) noexcept
{
  config_ = other.config_;
  fd_ = other.fd_;
  last_error_ = std::move(other.last_error_);

  other.fd_ = -1;
}

}  // namespace savo_perception