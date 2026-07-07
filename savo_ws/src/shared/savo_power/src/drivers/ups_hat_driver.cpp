#include "savo_power/ups_hat_driver.hpp"

#include "savo_power/power_policy.hpp"

#include <exception>

namespace savo_power
{

std::uint16_t UpsHatDriver::read_raw_voltage_word()
{
  return bus_.read_word_data(
    device_address_,
    constants::kUpsHatVoltageRegister);
}

std::uint16_t UpsHatDriver::read_raw_capacity_word()
{
  return bus_.read_word_data(
    device_address_,
    constants::kUpsHatCapacityRegister);
}

double UpsHatDriver::read_voltage_v()
{
  return ups_voltage_from_raw_word(read_raw_voltage_word());
}

double UpsHatDriver::read_capacity_pct()
{
  return ups_capacity_from_raw_word(read_raw_capacity_word());
}

BatteryReading UpsHatDriver::read()
{
  try {
    BatteryReading reading;
    reading.source = source_;
    reading.ok = true;
    reading.voltage_v = read_voltage_v();
    reading.capacity_pct = read_capacity_pct();

    return apply_power_policy(reading);
  } catch (const std::exception & error) {
    return make_error_reading(source_, error.what());
  }
}

}  // namespace savo_power
