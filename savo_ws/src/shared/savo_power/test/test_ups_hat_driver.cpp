#include "gtest/gtest.h"

#include "savo_power/i2c_bus.hpp"
#include "savo_power/ups_hat_driver.hpp"

#include <cstdint>

namespace
{

class AlwaysFailingBus final : public savo_power::I2cBus
{
public:
  int bus_id() const override
  {
    return 1;
  }

  bool is_open() const override
  {
    return true;
  }

  std::uint16_t read_word_data(std::uint8_t, std::uint8_t) override
  {
    ++read_attempts;
    throw savo_power::I2cException("simulated_i2c_read_failure");
  }

  std::uint8_t read_byte(std::uint8_t) override
  {
    throw savo_power::I2cException("unexpected_byte_read");
  }

  void write_byte(std::uint8_t, std::uint8_t) override
  {
    throw savo_power::I2cException("unexpected_byte_write");
  }

  void close() override
  {
  }

  int read_attempts{0};
};

TEST(UpsHatDriverFailure, RepeatedReadFailuresReturnExplicitErrorReadings)
{
  AlwaysFailingBus bus;
  savo_power::UpsHatDriver driver(
    bus,
    savo_power::BatterySource::CORE_UPS);

  for (int attempt = 0; attempt < 3; ++attempt) {
    const auto reading = driver.read();
    EXPECT_EQ(reading.source, savo_power::BatterySource::CORE_UPS);
    EXPECT_EQ(reading.state, savo_power::PowerState::ERROR);
    EXPECT_FALSE(reading.ok);
    EXPECT_EQ(reading.error_message, "simulated_i2c_read_failure");
  }

  EXPECT_EQ(bus.read_attempts, 3);
}

}  // namespace
