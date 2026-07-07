#ifndef SAVO_POWER__ADS7830_DRIVER_HPP_
#define SAVO_POWER__ADS7830_DRIVER_HPP_

#include "savo_power/battery_reading.hpp"
#include "savo_power/constants.hpp"
#include "savo_power/i2c_bus.hpp"

#include <cstdint>
#include <stdexcept>

namespace savo_power
{

enum class Ads7830PcbVersion
{
  V1,
  V2
};

inline constexpr bool is_valid_ads7830_channel(std::uint8_t channel)
{
  return channel <= 7;
}

inline constexpr std::uint8_t ads7830_channel_command(std::uint8_t channel)
{
  return static_cast<std::uint8_t>(
    constants::kAds7830CmdBase |
    (((((channel << 2U) | (channel >> 1U)) & 0x07U) << 4U))
  );
}

inline constexpr double ads7830_reference_voltage(Ads7830PcbVersion pcb_version)
{
  return pcb_version == Ads7830PcbVersion::V1 ? 3.3 : 5.2;
}

inline constexpr double ads7830_battery_multiplier(Ads7830PcbVersion pcb_version)
{
  return pcb_version == Ads7830PcbVersion::V1 ? 3.0 : 2.0;
}

inline constexpr double ads7830_adc_voltage_from_byte(
  std::uint8_t raw_value,
  Ads7830PcbVersion pcb_version)
{
  return static_cast<double>(raw_value) / 255.0 *
         ads7830_reference_voltage(pcb_version);
}

inline constexpr double ads7830_battery_voltage_from_byte(
  std::uint8_t raw_value,
  Ads7830PcbVersion pcb_version)
{
  return ads7830_adc_voltage_from_byte(raw_value, pcb_version) *
         ads7830_battery_multiplier(pcb_version);
}

inline constexpr double estimate_linear_soc_pct(
  double voltage_v,
  double empty_voltage_v = constants::kBaseBatteryEmptyVoltageDefault,
  double full_voltage_v = constants::kBaseBatteryFullVoltageDefault)
{
  if (voltage_v <= empty_voltage_v) {
    return 0.0;
  }

  if (voltage_v >= full_voltage_v) {
    return 100.0;
  }

  return (voltage_v - empty_voltage_v) /
         (full_voltage_v - empty_voltage_v) *
         100.0;
}

class Ads7830Driver
{
public:
  Ads7830Driver(
    I2cBus & bus,
    std::uint8_t device_address = constants::kAds7830AddrDefault,
    std::uint8_t channel = constants::kBaseBatteryAdcChannelDefault,
    Ads7830PcbVersion pcb_version = Ads7830PcbVersion::V2)
  : bus_(bus),
    device_address_(device_address),
    channel_(channel),
    pcb_version_(pcb_version)
  {
    if (!is_usable_7bit_i2c_address(device_address_)) {
      throw std::invalid_argument("Ads7830Driver received invalid I2C address");
    }

    if (!is_valid_ads7830_channel(channel_)) {
      throw std::invalid_argument("Ads7830Driver channel must be 0..7");
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

  std::uint8_t device_address() const
  {
    return device_address_;
  }

  std::uint8_t channel() const
  {
    return channel_;
  }

  Ads7830PcbVersion pcb_version() const
  {
    return pcb_version_;
  }

  std::uint8_t command() const
  {
    return ads7830_channel_command(channel_);
  }

  std::uint8_t read_raw_byte();

  std::uint8_t read_stable_byte();

  double read_adc_voltage_v();

  double read_battery_voltage_v();

  double estimate_soc_pct(double voltage_v) const;

  BatteryReading read();

private:
  I2cBus & bus_;
  std::uint8_t device_address_{constants::kAds7830AddrDefault};
  std::uint8_t channel_{constants::kBaseBatteryAdcChannelDefault};
  Ads7830PcbVersion pcb_version_{Ads7830PcbVersion::V2};
};

}  // namespace savo_power

#endif  // SAVO_POWER__ADS7830_DRIVER_HPP_
