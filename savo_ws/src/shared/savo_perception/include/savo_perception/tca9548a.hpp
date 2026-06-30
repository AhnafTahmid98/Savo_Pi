#ifndef SAVO_PERCEPTION__TCA9548A_HPP_
#define SAVO_PERCEPTION__TCA9548A_HPP_

#include <cstdint>
#include <string>

#include "savo_perception/constants.hpp"
#include "savo_perception/visibility_control.hpp"

namespace savo_perception
{

struct SAVO_PERCEPTION_PUBLIC Tca9548aConfig
{
  int bus{constants::kI2cBusDefault};
  std::uint8_t address{constants::kTca9548aAddrDefault};
};

class SAVO_PERCEPTION_PUBLIC Tca9548a
{
public:
  explicit Tca9548a(Tca9548aConfig config = Tca9548aConfig{});
  ~Tca9548a();

  Tca9548a(const Tca9548a &) = delete;
  Tca9548a & operator=(const Tca9548a &) = delete;

  Tca9548a(Tca9548a && other) noexcept;
  Tca9548a & operator=(Tca9548a && other) noexcept;

  [[nodiscard]] const Tca9548aConfig & config() const;
  [[nodiscard]] bool is_open() const;
  [[nodiscard]] int fd() const;
  [[nodiscard]] std::string bus_device() const;

  bool open();
  void close();

  bool select_channel(int channel);
  bool disable_all();

  [[nodiscard]] std::string last_error() const;

private:
  bool write_mask(std::uint8_t mask);
  void move_from(Tca9548a && other) noexcept;

  Tca9548aConfig config_{};
  int fd_{-1};
  std::string last_error_;
};

SAVO_PERCEPTION_PUBLIC bool valid_tca_channel(int channel);
SAVO_PERCEPTION_PUBLIC std::uint8_t tca_channel_mask(int channel);

}  // namespace savo_perception

#endif  // SAVO_PERCEPTION__TCA9548A_HPP_