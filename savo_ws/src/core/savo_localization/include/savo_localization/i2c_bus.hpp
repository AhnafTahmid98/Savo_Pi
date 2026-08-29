#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace savo_localization
{

class I2CBusInterface
{
public:
  virtual ~I2CBusInterface() = default;

  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;
  virtual int bus_number() const = 0;
  virtual void set_slave_address(uint8_t address) = 0;
  virtual uint8_t read_u8(uint8_t reg) = 0;
  virtual std::vector<uint8_t> read_block(
    uint8_t start_reg,
    std::size_t length) = 0;
  virtual void write_u8(uint8_t reg, uint8_t value) = 0;
  virtual void write_block(
    uint8_t start_reg,
    const std::vector<uint8_t> & values) = 0;
};

class I2CBus final : public I2CBusInterface
{
public:
  explicit I2CBus(int bus_number);
  ~I2CBus() override;

  I2CBus(const I2CBus &) = delete;
  I2CBus & operator=(const I2CBus &) = delete;

  I2CBus(I2CBus && other) noexcept;
  I2CBus & operator=(I2CBus && other) noexcept;

  bool open() override;
  void close() override;

  bool is_open() const override;
  int bus_number() const override;
  int fd() const;
  std::string device_path() const;

  void set_slave_address(uint8_t address) override;

  uint8_t read_u8(uint8_t reg) override;
  int16_t read_s16_le(uint8_t reg);
  uint16_t read_u16_le(uint8_t reg);

  std::vector<uint8_t> read_block(
    uint8_t start_reg,
    std::size_t length) override;

  void write_u8(uint8_t reg, uint8_t value) override;
  void write_block(
    uint8_t start_reg,
    const std::vector<uint8_t> & values) override;

private:
  int bus_number_{1};
  int fd_{-1};
  uint8_t active_address_{0};

  void require_open() const;
};

}  // namespace savo_localization
