#include "savo_localization/i2c_bus.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sstream>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>

namespace savo_localization
{

namespace
{

std::runtime_error make_system_error(const std::string & message)
{
  std::ostringstream oss;
  oss << message << ": " << std::strerror(errno);
  return std::runtime_error(oss.str());
}

void write_exact(int fd, const uint8_t * data, std::size_t length)
{
  const uint8_t * cursor = data;
  std::size_t remaining = length;

  while (remaining > 0) {
    const ssize_t written = ::write(fd, cursor, remaining);

    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw make_system_error("I2C write failed");
    }

    if (written == 0) {
      throw std::runtime_error("I2C write returned zero bytes");
    }

    cursor += written;
    remaining -= static_cast<std::size_t>(written);
  }
}

void read_exact(int fd, uint8_t * data, std::size_t length)
{
  uint8_t * cursor = data;
  std::size_t remaining = length;

  while (remaining > 0) {
    const ssize_t bytes_read = ::read(fd, cursor, remaining);

    if (bytes_read < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw make_system_error("I2C read failed");
    }

    if (bytes_read == 0) {
      throw std::runtime_error("I2C read returned zero bytes");
    }

    cursor += bytes_read;
    remaining -= static_cast<std::size_t>(bytes_read);
  }
}

}  // namespace

I2CBus::I2CBus(int bus_number)
: bus_number_(bus_number)
{
}

I2CBus::~I2CBus()
{
  close();
}

I2CBus::I2CBus(I2CBus && other) noexcept
: bus_number_(other.bus_number_),
  fd_(other.fd_),
  active_address_(other.active_address_)
{
  other.fd_ = -1;
  other.active_address_ = 0;
}

I2CBus & I2CBus::operator=(I2CBus && other) noexcept
{
  if (this == &other) {
    return *this;
  }

  close();

  bus_number_ = other.bus_number_;
  fd_ = other.fd_;
  active_address_ = other.active_address_;

  other.fd_ = -1;
  other.active_address_ = 0;

  return *this;
}

bool I2CBus::open()
{
  if (is_open()) {
    return true;
  }

  fd_ = ::open(device_path().c_str(), O_RDWR | O_CLOEXEC);

  if (fd_ < 0) {
    return false;
  }

  return true;
}

void I2CBus::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }

  active_address_ = 0;
}

bool I2CBus::is_open() const
{
  return fd_ >= 0;
}

int I2CBus::bus_number() const
{
  return bus_number_;
}

int I2CBus::fd() const
{
  return fd_;
}

std::string I2CBus::device_path() const
{
  return "/dev/i2c-" + std::to_string(bus_number_);
}

void I2CBus::set_slave_address(uint8_t address)
{
  require_open();

  if (active_address_ == address) {
    return;
  }

  if (::ioctl(fd_, I2C_SLAVE, address) < 0) {
    std::ostringstream oss;
    oss << "Failed to select I2C slave address 0x"
        << std::hex << static_cast<int>(address);
    throw make_system_error(oss.str());
  }

  active_address_ = address;
}

uint8_t I2CBus::read_u8(uint8_t reg)
{
  require_open();

  write_exact(fd_, &reg, 1);

  uint8_t value = 0;
  read_exact(fd_, &value, 1);

  return value;
}

int16_t I2CBus::read_s16_le(uint8_t reg)
{
  const uint16_t value = read_u16_le(reg);
  return static_cast<int16_t>(value);
}

uint16_t I2CBus::read_u16_le(uint8_t reg)
{
  const auto bytes = read_block(reg, 2);

  return static_cast<uint16_t>(
    static_cast<uint16_t>(bytes[0]) |
    (static_cast<uint16_t>(bytes[1]) << 8));
}

std::vector<uint8_t> I2CBus::read_block(uint8_t start_reg, std::size_t length)
{
  require_open();

  if (length == 0) {
    return {};
  }

  write_exact(fd_, &start_reg, 1);

  std::vector<uint8_t> values(length, 0);
  read_exact(fd_, values.data(), values.size());

  return values;
}

void I2CBus::write_u8(uint8_t reg, uint8_t value)
{
  require_open();

  const uint8_t buffer[2] = {reg, value};
  write_exact(fd_, buffer, sizeof(buffer));
}

void I2CBus::write_block(uint8_t start_reg, const std::vector<uint8_t> & values)
{
  require_open();

  std::vector<uint8_t> buffer;
  buffer.reserve(values.size() + 1);
  buffer.push_back(start_reg);
  buffer.insert(buffer.end(), values.begin(), values.end());

  write_exact(fd_, buffer.data(), buffer.size());
}

void I2CBus::require_open() const
{
  if (!is_open()) {
    throw std::runtime_error("I2C bus is not open");
  }
}

}  // namespace savo_localization