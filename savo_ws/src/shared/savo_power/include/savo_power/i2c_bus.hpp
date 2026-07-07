#ifndef SAVO_POWER__I2C_BUS_HPP_
#define SAVO_POWER__I2C_BUS_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>

namespace savo_power
{

class I2cException : public std::runtime_error
{
public:
  explicit I2cException(const std::string & message)
  : std::runtime_error(message)
  {
  }
};

class I2cBus
{
public:
  virtual ~I2cBus() = default;

  virtual int bus_id() const = 0;
  virtual bool is_open() const = 0;

  virtual std::uint16_t read_word_data(
    std::uint8_t device_address,
    std::uint8_t register_address) = 0;

  virtual std::uint8_t read_byte(
    std::uint8_t device_address) = 0;

  virtual void write_byte(
    std::uint8_t device_address,
    std::uint8_t value) = 0;

  virtual void close() = 0;
};

inline constexpr bool is_valid_7bit_i2c_address(std::uint8_t address)
{
  return address <= 0x7F;
}

inline constexpr bool is_reserved_7bit_i2c_address(std::uint8_t address)
{
  return address < 0x08 || address > 0x77;
}

inline constexpr bool is_usable_7bit_i2c_address(std::uint8_t address)
{
  return is_valid_7bit_i2c_address(address) &&
         !is_reserved_7bit_i2c_address(address);
}

}  // namespace savo_power

#endif  // SAVO_POWER__I2C_BUS_HPP_
