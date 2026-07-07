#ifndef SAVO_POWER__UPS_HAT_DRIVER_HPP_
#define SAVO_POWER__UPS_HAT_DRIVER_HPP_

#include "savo_power/battery_reading.hpp"
#include "savo_power/constants.hpp"
#include "savo_power/i2c_bus.hpp"

#include <cstdint>
#include <stdexcept>

namespace savo_power
{

inline constexpr std::uint16_t swap_word_bytes(std::uint16_t value)
{
  return static_cast<std::uint16_t>(
    ((value & 0x00FFU) << 8U) |
    ((value & 0xFF00U) >> 8U)
  );
}

inline constexpr double ups_voltage_from_raw_word(std::uint16_t raw_word)
{
  return static_cast<double>(swap_word_bytes(raw_word)) * 1.25 / 1000.0 / 16.0;
}

inline constexpr double ups_capacity_from_raw_word(std::uint16_t raw_word)
{
  return static_cast<double>(swap_word_bytes(raw_word)) / 256.0;
}

inline constexpr bool is_ups_source(BatterySource source)
{
  return source == BatterySource::CORE_UPS ||
         source == BatterySource::EDGE_UPS;
}

class UpsHatDriver
{
public:
  UpsHatDriver(
    I2cBus & bus,
    BatterySource source,
    std::uint8_t device_address = constants::kUpsHatAddrDefault)
  : bus_(bus),
    source_(source),
    device_address_(device_address)
  {
    if (!is_ups_source(source_)) {
      throw std::invalid_argument("UpsHatDriver source must be CORE_UPS or EDGE_UPS");
    }

    if (!is_usable_7bit_i2c_address(device_address_)) {
      throw std::invalid_argument("UpsHatDriver received invalid I2C address");
    }
  }

  int bus_id() const
  {
    return bus_.bus_id();
  }

  bool is_open() const
  {
    return bus_.is_open();
  }

  BatterySource source() const
  {
    return source_;
  }

  std::uint8_t device_address() const
  {
    return device_address_;
  }

  std::uint16_t read_raw_voltage_word();

  std::uint16_t read_raw_capacity_word();

  double read_voltage_v();

  double read_capacity_pct();

  BatteryReading read();

private:
  I2cBus & bus_;
  BatterySource source_{BatterySource::UNKNOWN};
  std::uint8_t device_address_{constants::kUpsHatAddrDefault};
};

}  // namespace savo_power

#endif  // SAVO_POWER__UPS_HAT_DRIVER_HPP_
