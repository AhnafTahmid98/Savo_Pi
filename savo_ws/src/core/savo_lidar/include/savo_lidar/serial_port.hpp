#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "savo_lidar/serial_config.hpp"
#include "savo_lidar/visibility_control.hpp"

namespace savo_lidar
{

class SAVO_LIDAR_PUBLIC SerialPort
{
public:
  SerialPort();
  explicit SerialPort(const SerialConfig & config);
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  SerialPort(SerialPort && other) noexcept;
  SerialPort & operator=(SerialPort && other) noexcept;

  void open(const SerialConfig & config);
  void close() noexcept;

  bool is_open() const noexcept;
  int fd() const noexcept;

  const SerialConfig & config() const noexcept;

  void flush();

  std::size_t write_bytes(const std::uint8_t * data, std::size_t size);
  std::size_t write_bytes(const std::vector<std::uint8_t> & data);

  std::size_t read_some(std::uint8_t * data, std::size_t max_size);
  std::vector<std::uint8_t> read_available(std::size_t max_size);
  std::vector<std::uint8_t> read_exact(std::size_t size, double timeout_s);

  bool wait_readable(double timeout_s) const;

  static bool path_exists(const std::string & port);
  static bool path_readable_writable(const std::string & port);

private:
  void configure_port();
  void ensure_open() const;

  int fd_{-1};
  SerialConfig config_;
};

}  // namespace savo_lidar