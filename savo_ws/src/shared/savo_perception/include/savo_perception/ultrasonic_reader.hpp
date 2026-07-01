#ifndef SAVO_PERCEPTION__ULTRASONIC_READER_HPP_
#define SAVO_PERCEPTION__ULTRASONIC_READER_HPP_

#include <chrono>
#include <deque>
#include <optional>
#include <string>

#include "savo_perception/constants.hpp"
#include "savo_perception/range_sample.hpp"
#include "savo_perception/visibility_control.hpp"

namespace savo_perception
{

struct SAVO_PERCEPTION_PUBLIC UltrasonicConfig
{
  int trig_pin{constants::kUltrasonicTrigPinDefault};
  int echo_pin{constants::kUltrasonicEchoPinDefault};

  double max_distance_m{constants::kUltrasonicMaxDistanceMDefault};
  double valid_min_m{constants::kUltrasonicValidMinMDefault};
  double valid_max_m{constants::kUltrasonicValidMaxMDefault};

  int queue_len{1};
  std::string pin_factory{"lgpio"};

  // Raspberry Pi 5 usually exposes the 40-pin GPIO header on gpiochip4.
  // Use -1 for auto-detect.
  int gpiochip{4};

  int trigger_pulse_us{10};
  int echo_timeout_us{30000};
  int echo_idle_timeout_us{30000};

  double speed_of_sound_mps{343.0};
  std::string sensor_name{"ultrasonic_front"};
  std::string source{"ultrasonic"};
};

struct SAVO_PERCEPTION_PUBLIC UltrasonicReading
{
  std::string sensor_name{"ultrasonic_front"};
  std::optional<double> raw_m;
  std::optional<double> filtered_m;

  int trig_pin{-1};
  int echo_pin{-1};

  bool valid{false};
  std::string error;

  [[nodiscard]] RangeSample to_sample() const
  {
    if (!valid || !filtered_m.has_value()) {
      return make_invalid_range_sample(
        sensor_name,
        error.empty() ? "invalid_or_no_echo" : error,
        "ultrasonic");
    }

    return make_valid_range_sample(sensor_name, *filtered_m, "ultrasonic");
  }
};

class SAVO_PERCEPTION_PUBLIC UltrasonicReader
{
public:
  explicit UltrasonicReader(UltrasonicConfig config = UltrasonicConfig{});
  ~UltrasonicReader();

  UltrasonicReader(const UltrasonicReader &) = delete;
  UltrasonicReader & operator=(const UltrasonicReader &) = delete;

  UltrasonicReader(UltrasonicReader && other) noexcept;
  UltrasonicReader & operator=(UltrasonicReader && other) noexcept;

  [[nodiscard]] const UltrasonicConfig & config() const;
  [[nodiscard]] bool started() const;
  [[nodiscard]] int gpiochip_number() const;
  [[nodiscard]] std::string last_error() const;

  bool start();
  void stop();

  [[nodiscard]] UltrasonicReading read_once();
  [[nodiscard]] RangeSample read_sample();

private:
  bool configure_gpio();
  bool try_configure_gpiochip(int gpiochip);
  bool wait_for_echo_idle();
  bool trigger_pulse();

  [[nodiscard]] std::optional<double> read_echo_distance_m();
  [[nodiscard]] std::optional<double> pulse_duration_to_distance_m(
    std::chrono::microseconds pulse_duration) const;
  [[nodiscard]] bool valid_distance(double distance_m) const;
  [[nodiscard]] std::optional<double> filtered_distance_m() const;

  void append_valid_reading(double distance_m);
  void set_error(std::string error);
  void move_from(UltrasonicReader && other) noexcept;

  UltrasonicConfig config_{};
  int gpiochip_handle_{-1};
  int gpiochip_number_{-1};
  bool started_{false};
  std::deque<double> history_;
  std::string last_error_;
};

SAVO_PERCEPTION_PUBLIC bool valid_ultrasonic_pins(int trig_pin, int echo_pin);

SAVO_PERCEPTION_PUBLIC bool valid_ultrasonic_distance(
  double distance_m,
  double valid_min_m,
  double valid_max_m);

SAVO_PERCEPTION_PUBLIC std::optional<double> ultrasonic_echo_us_to_distance_m(
  int echo_duration_us,
  double speed_of_sound_mps = 343.0);

}  // namespace savo_perception

#endif  // SAVO_PERCEPTION__ULTRASONIC_READER_HPP_
