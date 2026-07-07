#ifndef SAVO_POWER__LINUX_I2C_BUS_HPP_
#define SAVO_POWER__LINUX_I2C_BUS_HPP_

#include "savo_power/constants.hpp"
#include "savo_power/i2c_bus.hpp"

#include <cstdint>
#include <string>

namespace savo_power
{

inline std::string make_linux_i2c_device_path(int bus_id)
{
  return "/dev/i2c-" + std::to_string(bus_id);
}

class LinuxI2cBus final : public I2cBus
{
public:
  explicit LinuxI2cBus(int bus_id = constants::kI2cBusDefault);

  LinuxI2cBus(
    int bus_id,
    std::string device_path);

  ~LinuxI2cBus() override;

  LinuxI2cBus(const LinuxI2cBus &) = delete;
  LinuxI2cBus & operator=(const LinuxI2cBus &) = delete;

  LinuxI2cBus(LinuxI2cBus && other) noexcept;
  LinuxI2cBus & operator=(LinuxI2cBus && other) noexcept;

  int bus_id() const override;

  bool is_open() const override;

  const std::string & device_path() const;

  std::uint16_t read_word_data(
    std::uint8_t device_address,
    std::uint8_t register_address) override;

  std::uint8_t read_byte(
    std::uint8_t device_address) override;

  void write_byte(
    std::uint8_t device_address,
    std::uint8_t value) override;

  void close() override;

private:
  void open_device();

  void select_device(std::uint8_t device_address);

  int bus_id_{constants::kI2cBusDefault};
  std::string device_path_{make_linux_i2c_device_path(constants::kI2cBusDefault)};
  int fd_{-1};
};

}  // namespace savo_power

#endif  // SAVO_POWER__LINUX_I2C_BUS_HPP_
