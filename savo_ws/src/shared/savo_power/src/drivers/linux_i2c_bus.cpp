#include "savo_power/linux_i2c_bus.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sstream>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>

namespace savo_power
{
namespace
{

std::string errno_message(const std::string & prefix)
{
  std::ostringstream out;
  out << prefix << ": " << std::strerror(errno);
  return out.str();
}

std::string address_to_string(std::uint8_t address)
{
  std::ostringstream out;
  out << "0x" << std::hex << static_cast<int>(address);
  return out.str();
}

int smbus_access(
  int fd,
  char read_write,
  std::uint8_t command,
  int size,
  union i2c_smbus_data * data)
{
  struct i2c_smbus_ioctl_data args;
  args.read_write = read_write;
  args.command = command;
  args.size = size;
  args.data = data;

  return ioctl(fd, I2C_SMBUS, &args);
}

void close_fd_noexcept(int & fd)
{
  if (fd >= 0) {
    static_cast<void>(::close(fd));
    fd = -1;
  }
}

}  // namespace

LinuxI2cBus::LinuxI2cBus(int bus_id)
: bus_id_(bus_id),
  device_path_(make_linux_i2c_device_path(bus_id))
{
  open_device();
}

LinuxI2cBus::LinuxI2cBus(
  int bus_id,
  std::string device_path)
: bus_id_(bus_id),
  device_path_(std::move(device_path))
{
  open_device();
}

LinuxI2cBus::~LinuxI2cBus()
{
  close_fd_noexcept(fd_);
}

LinuxI2cBus::LinuxI2cBus(LinuxI2cBus && other) noexcept
: bus_id_(other.bus_id_),
  device_path_(std::move(other.device_path_)),
  fd_(other.fd_)
{
  other.fd_ = -1;
}

LinuxI2cBus & LinuxI2cBus::operator=(LinuxI2cBus && other) noexcept
{
  if (this != &other) {
    close_fd_noexcept(fd_);

    bus_id_ = other.bus_id_;
    device_path_ = std::move(other.device_path_);
    fd_ = other.fd_;

    other.fd_ = -1;
  }

  return *this;
}

int LinuxI2cBus::bus_id() const
{
  return bus_id_;
}

bool LinuxI2cBus::is_open() const
{
  return fd_ >= 0;
}

const std::string & LinuxI2cBus::device_path() const
{
  return device_path_;
}

void LinuxI2cBus::open_device()
{
  if (is_open()) {
    return;
  }

  fd_ = ::open(device_path_.c_str(), O_RDWR);

  if (fd_ < 0) {
    throw I2cException(errno_message("Failed to open " + device_path_));
  }
}

void LinuxI2cBus::select_device(std::uint8_t device_address)
{
  if (!is_usable_7bit_i2c_address(device_address)) {
    throw I2cException(
      "Invalid or reserved I2C address " + address_to_string(device_address));
  }

  if (!is_open()) {
    throw I2cException("I2C bus is not open: " + device_path_);
  }

  if (ioctl(fd_, I2C_SLAVE, device_address) < 0) {
    throw I2cException(
      errno_message(
        "Failed to select I2C device " + address_to_string(device_address) +
        " on " + device_path_));
  }
}

std::uint16_t LinuxI2cBus::read_word_data(
  std::uint8_t device_address,
  std::uint8_t register_address)
{
  select_device(device_address);

  union i2c_smbus_data data;

  if (
    smbus_access(
      fd_,
      I2C_SMBUS_READ,
      register_address,
      I2C_SMBUS_WORD_DATA,
      &data) < 0)
  {
    throw I2cException(
      errno_message(
        "Failed to read word from " + address_to_string(device_address)));
  }

  return static_cast<std::uint16_t>(data.word);
}

std::uint8_t LinuxI2cBus::read_byte(std::uint8_t device_address)
{
  select_device(device_address);

  union i2c_smbus_data data;

  if (
    smbus_access(
      fd_,
      I2C_SMBUS_READ,
      0,
      I2C_SMBUS_BYTE,
      &data) < 0)
  {
    throw I2cException(
      errno_message(
        "Failed to read byte from " + address_to_string(device_address)));
  }

  return static_cast<std::uint8_t>(data.byte & 0xFFU);
}

void LinuxI2cBus::write_byte(
  std::uint8_t device_address,
  std::uint8_t value)
{
  select_device(device_address);

  if (
    smbus_access(
      fd_,
      I2C_SMBUS_WRITE,
      value,
      I2C_SMBUS_BYTE,
      nullptr) < 0)
  {
    throw I2cException(
      errno_message(
        "Failed to write byte to " + address_to_string(device_address)));
  }
}

void LinuxI2cBus::close()
{
  close_fd_noexcept(fd_);
}

}  // namespace savo_power
