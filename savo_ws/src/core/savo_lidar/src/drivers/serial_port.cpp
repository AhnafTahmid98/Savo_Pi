#include "savo_lidar/serial_port.hpp"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace savo_lidar
{
namespace
{

std::string errno_message(const std::string & prefix)
{
  return prefix + ": " + std::string(strerror(errno));
}

speed_t baudrate_to_termios(int baudrate)
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
#ifdef B230400
    case 230400:
      return B230400;
#endif
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
    default:
      throw std::invalid_argument("unsupported serial baudrate: " + std::to_string(baudrate));
  }
}

int timeout_to_ms(double timeout_s)
{
  if (timeout_s <= 0.0) {
    return 0;
  }

  const auto timeout_ms = static_cast<int>(timeout_s * 1000.0);
  return std::max(1, timeout_ms);
}

}  // namespace

SerialPort::SerialPort() = default;

SerialPort::SerialPort(const SerialConfig & config)
{
  open(config);
}

SerialPort::~SerialPort()
{
  close();
}

SerialPort::SerialPort(SerialPort && other) noexcept
: fd_(other.fd_),
  config_(std::move(other.config_))
{
  other.fd_ = -1;
}

SerialPort & SerialPort::operator=(SerialPort && other) noexcept
{
  if (this != &other) {
    close();

    fd_ = other.fd_;
    config_ = std::move(other.config_);
    other.fd_ = -1;
  }

  return *this;
}

void SerialPort::open(const SerialConfig & config)
{
  config.validate();
  close();

  config_ = config;

  fd_ = ::open(config_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error(errno_message("failed to open serial port " + config_.port));
  }

  try {
    configure_port();
    flush();
  } catch (...) {
    close();
    throw;
  }
}

void SerialPort::close() noexcept
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const noexcept
{
  return fd_ >= 0;
}

int SerialPort::fd() const noexcept
{
  return fd_;
}

const SerialConfig & SerialPort::config() const noexcept
{
  return config_;
}

void SerialPort::flush()
{
  ensure_open();

  if (::tcflush(fd_, TCIOFLUSH) != 0) {
    throw std::runtime_error(errno_message("failed to flush serial port"));
  }
}

std::size_t SerialPort::write_bytes(const std::uint8_t * data, std::size_t size)
{
  ensure_open();

  if (data == nullptr && size > 0) {
    throw std::invalid_argument("cannot write from null buffer");
  }

  std::size_t total_written = 0;

  while (total_written < size) {
    const auto remaining = size - total_written;
    const auto result = ::write(fd_, data + total_written, remaining);

    if (result > 0) {
      total_written += static_cast<std::size_t>(result);
      continue;
    }

    if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    throw std::runtime_error(errno_message("failed to write serial bytes"));
  }

  return total_written;
}

std::size_t SerialPort::write_bytes(const std::vector<std::uint8_t> & data)
{
  if (data.empty()) {
    return 0;
  }

  return write_bytes(data.data(), data.size());
}

std::size_t SerialPort::read_some(std::uint8_t * data, std::size_t max_size)
{
  ensure_open();

  if (data == nullptr && max_size > 0) {
    throw std::invalid_argument("cannot read into null buffer");
  }

  if (max_size == 0) {
    return 0;
  }

  const auto result = ::read(fd_, data, max_size);

  if (result > 0) {
    return static_cast<std::size_t>(result);
  }

  if (result == 0) {
    return 0;
  }

  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
    return 0;
  }

  throw std::runtime_error(errno_message("failed to read serial bytes"));
}

std::vector<std::uint8_t> SerialPort::read_available(std::size_t max_size)
{
  std::vector<std::uint8_t> buffer(max_size);
  const auto count = read_some(buffer.data(), buffer.size());
  buffer.resize(count);
  return buffer;
}

std::vector<std::uint8_t> SerialPort::read_exact(std::size_t size, double timeout_s)
{
  ensure_open();

  std::vector<std::uint8_t> output;
  output.resize(size);

  if (size == 0) {
    return output;
  }

  const auto start = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::duration<double>(timeout_s);

  std::size_t offset = 0;

  while (offset < size) {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = now - start;

    if (elapsed >= timeout) {
      throw std::runtime_error("timeout while reading exact serial bytes");
    }

    const auto remaining_time = std::chrono::duration<double>(timeout - elapsed).count();

    if (!wait_readable(remaining_time)) {
      throw std::runtime_error("timeout while waiting for serial bytes");
    }

    const auto count = read_some(output.data() + offset, size - offset);
    offset += count;
  }

  return output;
}

bool SerialPort::wait_readable(double timeout_s) const
{
  ensure_open();

  pollfd pfd;
  pfd.fd = fd_;
  pfd.events = POLLIN;
  pfd.revents = 0;

  const auto result = ::poll(&pfd, 1, timeout_to_ms(timeout_s));

  if (result > 0) {
    return (pfd.revents & POLLIN) != 0;
  }

  if (result == 0) {
    return false;
  }

  if (errno == EINTR) {
    return false;
  }

  throw std::runtime_error(errno_message("failed while polling serial port"));
}

bool SerialPort::path_exists(const std::string & port)
{
  struct stat info;
  return ::stat(port.c_str(), &info) == 0;
}

bool SerialPort::path_readable_writable(const std::string & port)
{
  return ::access(port.c_str(), R_OK | W_OK) == 0;
}

void SerialPort::configure_port()
{
  ensure_open();

  termios tty;
  if (::tcgetattr(fd_, &tty) != 0) {
    throw std::runtime_error(errno_message("failed to get serial attributes"));
  }

  ::cfmakeraw(&tty);

  const auto speed = baudrate_to_termios(config_.baudrate);
  if (::cfsetispeed(&tty, speed) != 0 || ::cfsetospeed(&tty, speed) != 0) {
    throw std::runtime_error(errno_message("failed to set serial baudrate"));
  }

  tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
  tty.c_cflag |= CS8;
  tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
  tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
  tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);

  tty.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
    throw std::runtime_error(errno_message("failed to apply serial attributes"));
  }
}

void SerialPort::ensure_open() const
{
  if (!is_open()) {
    throw std::runtime_error("serial port is not open");
  }
}

}  // namespace savo_lidar
