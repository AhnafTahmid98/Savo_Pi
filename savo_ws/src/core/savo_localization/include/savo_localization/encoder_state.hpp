#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <string>

namespace savo_localization
{

constexpr const char * WHEEL_FL = "FL";
constexpr const char * WHEEL_FR = "FR";
constexpr const char * WHEEL_RL = "RL";
constexpr const char * WHEEL_RR = "RR";

constexpr std::array<const char *, 4> WHEEL_ORDER = {
  WHEEL_FL,
  WHEEL_FR,
  WHEEL_RL,
  WHEEL_RR,
};

enum class WheelId : std::uint8_t
{
  FL = 0,
  FR = 1,
  RL = 2,
  RR = 3,
};

struct WheelEncoderPins
{
  int a_gpio{-1};
  int b_gpio{-1};
};

struct WheelEncoderConfig
{
  WheelId wheel_id{WheelId::FL};
  std::string name{WHEEL_FL};

  int a_gpio{-1};
  int b_gpio{-1};
  bool inverted{false};

  bool valid() const;
};

struct EncoderHardwareConfig
{
  WheelEncoderConfig fl{
    WheelId::FL,
    WHEEL_FL,
    20,
    21,
    false,
  };

  WheelEncoderConfig fr{
    WheelId::FR,
    WHEEL_FR,
    13,
    25,
    false,
  };

  WheelEncoderConfig rl{
    WheelId::RL,
    WHEEL_RL,
    23,
    24,
    false,
  };

  WheelEncoderConfig rr{
    WheelId::RR,
    WHEEL_RR,
    12,
    26,
    false,
  };

  int gpiochip{-1};
  double poll_s{0.001};
  double debounce_s{0.0003};

  bool use_internal_pullup{false};
  bool use_hw_debounce{true};

  std::array<WheelEncoderConfig, 4> wheels() const;
  bool valid() const;
};

struct WheelEncoderState
{
  WheelId wheel_id{WheelId::FL};
  std::string name{WHEEL_FL};

  std::int64_t count{0};
  std::int64_t previous_count{0};
  std::int64_t delta_count{0};

  double counts_per_second{0.0};
  double speed_mps{0.0};

  int direction{0};
  std::uint64_t illegal_transitions{0};

  bool active{false};
  double last_update_s{0.0};

  void update(
    std::int64_t new_count,
    double dt_s,
    double new_speed_mps,
    double stamp_s,
    std::uint64_t new_illegal_transitions);

  bool moving_forward() const;
  bool moving_reverse() const;
  bool stopped() const;
};

struct EncoderSample
{
  double stamp_s{0.0};
  double dt_s{0.0};

  WheelEncoderState fl{};
  WheelEncoderState fr{};
  WheelEncoderState rl{};
  WheelEncoderState rr{};

  double vx_mps{0.0};
  double vy_mps{0.0};
  double omega_rad_s{0.0};

  std::array<WheelEncoderState, 4> wheels() const;

  int active_wheel_count() const;
  std::uint64_t total_illegal_transitions() const;
};

int direction_from_delta(std::int64_t delta_count);
std::string wheel_id_to_name(WheelId wheel_id);
WheelId wheel_id_from_name(const std::string & name);

bool wheel_name_valid(const std::string & name);
bool gpio_pin_valid(int gpio_pin);

std::map<std::string, std::int64_t> counts_to_map(const EncoderSample & sample);
std::map<std::string, double> speeds_to_map(const EncoderSample & sample);

}  // namespace savo_localization
