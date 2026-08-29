#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include "savo_localization/i2c_bus.hpp"

namespace savo_localization::test
{

struct BlockOperation
{
  uint8_t start_register{0};
  std::vector<uint8_t> values;
};

class FakeI2CBus final : public I2CBusInterface
{
public:
  explicit FakeI2CBus(const int bus_number = 1)
  : bus_number_(bus_number)
  {
    registers[0x00] = 0xA0;
  }

  bool open() override
  {
    open_ = open_result;
    return open_;
  }

  void close() override
  {
    open_ = false;
  }

  bool is_open() const override
  {
    return open_;
  }

  int bus_number() const override
  {
    return bus_number_;
  }

  void set_slave_address(const uint8_t address) override
  {
    require_open();
    active_address = address;
  }

  uint8_t read_u8(const uint8_t reg) override
  {
    require_open();
    return registers[reg];
  }

  std::vector<uint8_t> read_block(
    const uint8_t start_reg,
    const std::size_t length) override
  {
    require_open();
    if (fail_next_block_read) {
      fail_next_block_read = false;
      throw std::runtime_error("injected block read failure");
    }
    std::vector<uint8_t> values(length, 0U);
    for (std::size_t index = 0; index < length; ++index) {
      values[index] = registers[static_cast<uint8_t>(start_reg + index)];
    }
    if (short_next_block_read_by > 0U) {
      const std::size_t short_by = std::min(short_next_block_read_by, values.size());
      short_next_block_read_by = 0U;
      values.resize(values.size() - short_by);
    }
    if (corrupt_next_calibration_readback && start_reg == 0x55 && !values.empty()) {
      corrupt_next_calibration_readback = false;
      values.back() ^= 0x01U;
    }
    block_reads.push_back(BlockOperation{start_reg, values});
    return values;
  }

  void write_u8(const uint8_t reg, const uint8_t value) override
  {
    require_open();
    registers[reg] = value;
    byte_writes.emplace_back(reg, value);
  }

  void write_block(
    const uint8_t start_reg,
    const std::vector<uint8_t> & values) override
  {
    require_open();
    block_writes.push_back(BlockOperation{start_reg, values});
    const std::size_t write_count = fail_block_write_after_bytes > 0U ?
      std::min(fail_block_write_after_bytes, values.size()) : values.size();
    for (std::size_t index = 0; index < write_count; ++index) {
      registers[static_cast<uint8_t>(start_reg + index)] = values[index];
    }
    if (fail_block_write_after_bytes > 0U) {
      fail_block_write_after_bytes = 0U;
      throw std::runtime_error("injected partial block write failure");
    }
  }

  void load_block(
    const uint8_t start_reg,
    const std::vector<uint8_t> & values)
  {
    for (std::size_t index = 0; index < values.size(); ++index) {
      registers[static_cast<uint8_t>(start_reg + index)] = values[index];
    }
  }

  void require_open() const
  {
    if (!open_) {
      throw std::runtime_error("fake I2C bus is not open");
    }
  }

  std::array<uint8_t, 256> registers{};
  std::vector<std::pair<uint8_t, uint8_t>> byte_writes;
  std::vector<BlockOperation> block_reads;
  std::vector<BlockOperation> block_writes;
  uint8_t active_address{0};
  bool open_result{true};
  bool fail_next_block_read{false};
  bool corrupt_next_calibration_readback{false};
  std::size_t short_next_block_read_by{0U};
  std::size_t fail_block_write_after_bytes{0U};

private:
  int bus_number_{1};
  bool open_{false};
};

}  // namespace savo_localization::test
