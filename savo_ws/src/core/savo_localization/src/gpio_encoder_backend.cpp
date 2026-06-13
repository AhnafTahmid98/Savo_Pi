#include "savo_localization/gpio_encoder_backend.hpp"

#include <array>
#include <stdexcept>
#include <string>

#if defined(__has_include)
#  if __has_include(<lgpio.h>)
#    include <lgpio.h>
#    define SAVO_LOCALIZATION_HAVE_LGPIO 1
#  else
#    define SAVO_LOCALIZATION_HAVE_LGPIO 0
#  endif
#else
#  define SAVO_LOCALIZATION_HAVE_LGPIO 0
#endif

namespace savo_localization
{

namespace
{

constexpr std::array<int, 6> GPIOCHIP_CANDIDATES = {4, 0, 1, 2, 3, 5};

#if SAVO_LOCALIZATION_HAVE_LGPIO
int pull_flags(bool use_internal_pullup)
{
  return use_internal_pullup ? LG_SET_PULL_UP : 0;
}
#endif

}  // namespace

GpioEncoderBackend::~GpioEncoderBackend()
{
  close();
}

bool GpioEncoderBackend::open(const EncoderHardwareConfig & config)
{
  if (is_open()) {
    return true;
  }

  if (!config.valid()) {
    return false;
  }

#if SAVO_LOCALIZATION_HAVE_LGPIO
  config_ = config;
  wheels_ = make_channels(config_);

  try {
    if (!open_gpiochip()) {
      return false;
    }

    for (auto & wheel : wheels_) {
      configure_input(wheel.a_gpio);
      configure_input(wheel.b_gpio);

      const int a_value = read_gpio(wheel.a_gpio);
      const int b_value = read_gpio(wheel.b_gpio);

      wheel.last_state = state_from_ab(a_value, b_value);
      wheel.initialized = true;
    }

    open_ = true;
    return true;
  } catch (const std::exception &) {
    close();
    return false;
  }
#else
  (void)config;
  return false;
#endif
}

void GpioEncoderBackend::close()
{
#if SAVO_LOCALIZATION_HAVE_LGPIO
  close_gpiochip();
#endif

  open_ = false;
}

bool GpioEncoderBackend::is_open() const
{
  return open_;
}

void GpioEncoderBackend::poll()
{
  if (!is_open()) {
    throw std::runtime_error("GPIO encoder backend is not open");
  }

  for (auto & wheel : wheels_) {
    poll_wheel(wheel);
  }
}

std::array<std::int64_t, 4> GpioEncoderBackend::counts() const
{
  return {
    wheels_[0].count,
    wheels_[1].count,
    wheels_[2].count,
    wheels_[3].count,
  };
}

std::array<std::uint64_t, 4> GpioEncoderBackend::illegal_transitions() const
{
  return {
    wheels_[0].illegal_transitions,
    wheels_[1].illegal_transitions,
    wheels_[2].illegal_transitions,
    wheels_[3].illegal_transitions,
  };
}

void GpioEncoderBackend::reset_counts()
{
  for (auto & wheel : wheels_) {
    wheel.count = 0;
    wheel.illegal_transitions = 0;
  }
}

bool GpioEncoderBackend::open_gpiochip()
{
#if SAVO_LOCALIZATION_HAVE_LGPIO
  if (gpiochip_ >= 0) {
    return true;
  }

  if (config_.gpiochip >= 0) {
    gpiochip_ = lgGpiochipOpen(config_.gpiochip);
    return gpiochip_ >= 0;
  }

  for (const int candidate : GPIOCHIP_CANDIDATES) {
    gpiochip_ = lgGpiochipOpen(candidate);

    if (gpiochip_ >= 0) {
      config_.gpiochip = candidate;
      return true;
    }
  }

  return false;
#else
  return false;
#endif
}

void GpioEncoderBackend::close_gpiochip()
{
#if SAVO_LOCALIZATION_HAVE_LGPIO
  if (gpiochip_ >= 0) {
    lgGpiochipClose(gpiochip_);
    gpiochip_ = -1;
  }
#endif
}

void GpioEncoderBackend::configure_input(int gpio)
{
#if SAVO_LOCALIZATION_HAVE_LGPIO
  if (gpiochip_ < 0) {
    throw std::runtime_error("GPIO chip is not open");
  }

  const int claim_result = lgGpioClaimInput(
    gpiochip_,
    pull_flags(config_.use_internal_pullup),
    gpio);

  if (claim_result < 0) {
    throw std::runtime_error("Failed to claim GPIO input " + std::to_string(gpio));
  }

  if (config_.use_hw_debounce && config_.debounce_s > 0.0) {
    const int debounce_us = static_cast<int>(config_.debounce_s * 1'000'000.0);
    lgGpioSetDebounce(gpiochip_, gpio, debounce_us);
  }
#else
  (void)gpio;
  throw std::runtime_error("lgpio support was not available at build time");
#endif
}

int GpioEncoderBackend::read_gpio(int gpio) const
{
#if SAVO_LOCALIZATION_HAVE_LGPIO
  if (gpiochip_ < 0) {
    throw std::runtime_error("GPIO chip is not open");
  }

  const int value = lgGpioRead(gpiochip_, gpio);

  if (value < 0) {
    throw std::runtime_error("Failed to read GPIO " + std::to_string(gpio));
  }

  return value ? 1 : 0;
#else
  (void)gpio;
  throw std::runtime_error("lgpio support was not available at build time");
#endif
}

void GpioEncoderBackend::poll_wheel(WheelChannel & wheel)
{
  const int a_value = read_gpio(wheel.a_gpio);
  const int b_value = read_gpio(wheel.b_gpio);
  const int current_state = state_from_ab(a_value, b_value);

  if (!wheel.initialized) {
    wheel.last_state = current_state;
    wheel.initialized = true;
    return;
  }

  if (current_state == wheel.last_state) {
    return;
  }

  const int delta = quadrature_delta(wheel.last_state, current_state);

  if (delta == 2) {
    ++wheel.illegal_transitions;
  } else {
    wheel.count += delta;
  }

  wheel.last_state = current_state;
}

int GpioEncoderBackend::state_from_ab(int a_value, int b_value)
{
  const int a = a_value ? 1 : 0;
  const int b = b_value ? 1 : 0;

  return (a << 1) | b;
}

int GpioEncoderBackend::quadrature_delta(int previous_state, int current_state)
{
  static constexpr int table[4][4] = {
    {0, +1, -1, 2},
    {-1, 0, 2, +1},
    {+1, 2, 0, -1},
    {2, -1, +1, 0},
  };

  if (previous_state < 0 || previous_state > 3 ||
      current_state < 0 || current_state > 3) {
    return 2;
  }

  return table[previous_state][current_state];
}

GpioEncoderBackend::WheelChannel GpioEncoderBackend::make_channel(
  const WheelEncoderConfig & config)
{
  WheelChannel channel{};
  channel.a_gpio = config.a_gpio;
  channel.b_gpio = config.b_gpio;
  return channel;
}

std::array<GpioEncoderBackend::WheelChannel, 4> GpioEncoderBackend::make_channels(
  const EncoderHardwareConfig & config)
{
  return {
    make_channel(config.fl),
    make_channel(config.fr),
    make_channel(config.rl),
    make_channel(config.rr),
  };
}

}  // namespace savo_localization