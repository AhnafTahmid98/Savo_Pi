#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace savo_localization
{

class I2CBus
{
public:
  explicit I2CBus(int bus_number);
  ~I2CBus();

  I2CBus(const I2CBus &) = delete;
  I2CBus & operator=(const I2CBus &) = delete;

  I2CBus(I2CBus && other) noexcept;
  I2CBus & operator=(I2CBus && other) noexcept;

  bool open();
  void close();

  bool is_open() const;
  int bus_number() const;
  int fd() const;
  std::string device_path() const;

  void set_slave_address(uint8_t address);

  uint8_t read_u8(uint8_t reg);
  int16_t read_s16_le(uint8_t reg);
  uint16_t read_u16_le(uint8_t reg);

  std::vector<uint8_t> read_block(uint8_t start_reg, std::size_t length);

  void write_u8(uint8_t reg, uint8_t value);
  void write_block(uint8_t start_reg, const std::vector<uint8_t> & values);

private:
  int bus_number_{1};
  int fd_{-1};
  uint8_t active_address_{0};

  void require_open() const;
};

}  // namespace savo_localization