#include "savo_power/ads7830_driver.hpp"

#include "savo_power/power_policy.hpp"

#include <exception>

namespace savo_power
{

std::uint8_t Ads7830Driver::read_raw_byte()
{
  bus_.write_byte(device_address_, command());
  return bus_.read_byte(device_address_);
}

std::uint8_t Ads7830Driver::read_stable_byte()
{
  static_cast<void>(read_raw_byte());
  return read_raw_byte();
}

double Ads7830Driver::read_adc_voltage_v()
{
  return ads7830_adc_voltage_from_byte(
    read_stable_byte(),
    pcb_version_);
}

double Ads7830Driver::read_battery_voltage_v()
{
  return ads7830_battery_voltage_from_byte(
    read_stable_byte(),
    pcb_version_);
}

double Ads7830Driver::estimate_soc_pct(double voltage_v) const
{
  return estimate_linear_soc_pct(
    voltage_v,
    constants::kBaseBatteryEmptyVoltageDefault,
    constants::kBaseBatteryFullVoltageDefault);
}

BatteryReading Ads7830Driver::read()
{
  try {
    const double voltage = read_battery_voltage_v();

    BatteryReading reading;
    reading.source = BatterySource::BASE_BATTERY;
    reading.ok = true;
    reading.voltage_v = voltage;
    reading.soc_pct = estimate_soc_pct(voltage);

    return apply_power_policy(reading);
  } catch (const std::exception & error) {
    return make_error_reading(
      BatterySource::BASE_BATTERY,
      error.what());
  }
}

}  // namespace savo_power
