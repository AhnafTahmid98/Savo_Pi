#pragma once

#include <array>
#include <cstdint>

#include "savo_localization/encoder_reader.hpp"
#include "savo_localization/encoder_state.hpp"

namespace savo_localization
{

class GpioEncoderBackend final : public EncoderBackend
{
public:
  GpioEncoderBackend() = default;
  ~GpioEncoderBackend() override;

  GpioEncoderBackend(const GpioEncoderBackend &) = delete;
  GpioEncoderBackend & operator=(const GpioEncoderBackend &) = delete;

  GpioEncoderBackend(GpioEncoderBackend &&) = delete;
  GpioEncoderBackend & operator=(GpioEncoderBackend &&) = delete;

  bool open(const EncoderHardwareConfig & config) override;
  void close() override;
  bool is_open() const override;

  void poll() override;

  std::array<std::int64_t, 4> counts() const override;
  std::array<std::uint64_t, 4> illegal_transitions() const override;

  void reset_counts() override;

private:
  struct WheelChannel
  {
    int a_gpio{-1};
    int b_gpio{-1};

    int last_state{0};
    bool initialized{false};

    std::int64_t count{0};
    std::uint64_t illegal_transitions{0};
  };

  EncoderHardwareConfig config_{};
  int gpiochip_{-1};
  bool open_{false};

  std::array<WheelChannel, 4> wheels_{};

  bool open_gpiochip();
  void close_gpiochip();

  void configure_input(int gpio);
  int read_gpio(int gpio) const;

  void poll_wheel(WheelChannel & wheel);

  static int state_from_ab(int a_value, int b_value);
  static int quadrature_delta(int previous_state, int current_state);

  static WheelChannel make_channel(const WheelEncoderConfig & config);
  static std::array<WheelChannel, 4> make_channels(
    const EncoderHardwareConfig & config);
};

}  // namespace savo_localization