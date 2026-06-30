#include "savo_perception/ultrasonic_reader.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace savo_perception
{
namespace
{

constexpr const char * kGpioRoot = "/sys/class/gpio";

std::string gpio_path(const int pin, const std::string & file)
{
  return std::string(kGpioRoot) + "/gpio" + std::to_string(pin) + "/" + file;
}

bool write_text_file(const std::string & path, const std::string & value)
{
  std::ofstream out(path);

  if (!out) {
    return false;
  }

  out << value;
  return static_cast<bool>(out);
}

std::optional<std::string> read_text_file(const std::string & path)
{
  std::ifstream in(path);

  if (!in) {
    return std::nullopt;
  }

  std::string value;
  in >> value;

  if (value.empty()) {
    return std::nullopt;
  }

  return value;
}

bool gpio_exported(const int pin)
{
  return read_text_file(gpio_path(pin, "direction")).has_value();
}

bool export_gpio(const int pin)
{
  if (gpio_exported(pin)) {
    return true;
  }

  if (!write_text_file(std::string(kGpioRoot) + "/export", std::to_string(pin))) {
    return false;
  }

  for (int i = 0; i < 20; ++i) {
    if (gpio_exported(pin)) {
      return true;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  return gpio_exported(pin);
}

bool set_direction(const int pin, const std::string & direction)
{
  return write_text_file(gpio_path(pin, "direction"), direction);
}

bool write_gpio_value(const int pin, const int value)
{
  return write_text_file(gpio_path(pin, "value"), value ? "1" : "0");
}

std::optional<int> read_gpio_value(const int pin)
{
  const auto value = read_text_file(gpio_path(pin, "value"));

  if (!value.has_value()) {
    return std::nullopt;
  }

  return (*value == "1") ? 1 : 0;
}

std::optional<double> median_from_history(const std::deque<double> & history)
{
  if (history.empty()) {
    return std::nullopt;
  }

  std::vector<double> values(history.begin(), history.end());
  std::sort(values.begin(), values.end());

  return values[values.size() / 2];
}

}  // namespace

bool valid_ultrasonic_pins(const int trig_pin, const int echo_pin)
{
  return trig_pin >= 0 && echo_pin >= 0 && trig_pin != echo_pin;
}

bool valid_ultrasonic_distance(
  const double distance_m,
  const double valid_min_m,
  const double valid_max_m)
{
  return std::isfinite(distance_m) &&
    distance_m >= valid_min_m &&
    distance_m <= valid_max_m;
}

std::optional<double> ultrasonic_echo_us_to_distance_m(
  const int echo_duration_us,
  const double speed_of_sound_mps)
{
  if (echo_duration_us <= 0 || !std::isfinite(speed_of_sound_mps) || speed_of_sound_mps <= 0.0) {
    return std::nullopt;
  }

  const auto duration_s = static_cast<double>(echo_duration_us) / 1'000'000.0;
  return (duration_s * speed_of_sound_mps) / 2.0;
}

UltrasonicReader::UltrasonicReader(UltrasonicConfig config)
: config_(std::move(config))
{
  config_.queue_len = std::max(1, config_.queue_len);
}

UltrasonicReader::~UltrasonicReader()
{
  stop();
}

UltrasonicReader::UltrasonicReader(UltrasonicReader && other) noexcept
{
  move_from(std::move(other));
}

UltrasonicReader & UltrasonicReader::operator=(UltrasonicReader && other) noexcept
{
  if (this != &other) {
    stop();
    move_from(std::move(other));
  }

  return *this;
}

const UltrasonicConfig & UltrasonicReader::config() const
{
  return config_;
}

bool UltrasonicReader::started() const
{
  return started_;
}

std::string UltrasonicReader::last_error() const
{
  return last_error_;
}

bool UltrasonicReader::start()
{
  if (started_) {
    return true;
  }

  if (!valid_ultrasonic_pins(config_.trig_pin, config_.echo_pin)) {
    set_error("invalid_gpio_pins");
    return false;
  }

  if (!configure_gpio()) {
    return false;
  }

  started_ = true;
  last_error_.clear();
  return true;
}

void UltrasonicReader::stop()
{
  if (started_) {
    write_gpio_value(config_.trig_pin, 0);
  }

  started_ = false;
}

UltrasonicReading UltrasonicReader::read_once()
{
  UltrasonicReading reading;
  reading.sensor_name = config_.sensor_name;
  reading.trig_pin = config_.trig_pin;
  reading.echo_pin = config_.echo_pin;

  if (!started_) {
    reading.error = "not_started";
    return reading;
  }

  const auto raw_m = read_echo_distance_m();
  reading.raw_m = raw_m;

  if (!raw_m.has_value()) {
    reading.error = last_error_.empty() ? "no_echo" : last_error_;
    return reading;
  }

  if (!valid_distance(*raw_m)) {
    reading.error = "distance_out_of_valid_range";
    return reading;
  }

  append_valid_reading(*raw_m);

  reading.filtered_m = filtered_distance_m();
  reading.valid = reading.filtered_m.has_value();

  if (!reading.valid) {
    reading.error = "filter_failed";
  }

  return reading;
}

RangeSample UltrasonicReader::read_sample()
{
  return read_once().to_sample();
}

bool UltrasonicReader::configure_gpio()
{
  if (!export_gpio(config_.trig_pin)) {
    set_error("trig_export_failed");
    return false;
  }

  if (!export_gpio(config_.echo_pin)) {
    set_error("echo_export_failed");
    return false;
  }

  if (!set_direction(config_.trig_pin, "out")) {
    set_error("trig_direction_failed");
    return false;
  }

  if (!set_direction(config_.echo_pin, "in")) {
    set_error("echo_direction_failed");
    return false;
  }

  if (!write_gpio_value(config_.trig_pin, 0)) {
    set_error("trig_initial_low_failed");
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  last_error_.clear();
  return true;
}

bool UltrasonicReader::trigger_pulse()
{
  if (!write_gpio_value(config_.trig_pin, 0)) {
    set_error("trig_low_failed");
    return false;
  }

  std::this_thread::sleep_for(std::chrono::microseconds(2));

  if (!write_gpio_value(config_.trig_pin, 1)) {
    set_error("trig_high_failed");
    return false;
  }

  std::this_thread::sleep_for(std::chrono::microseconds(config_.trigger_pulse_us));

  if (!write_gpio_value(config_.trig_pin, 0)) {
    set_error("trig_final_low_failed");
    return false;
  }

  return true;
}

std::optional<double> UltrasonicReader::read_echo_distance_m()
{
  if (!trigger_pulse()) {
    return std::nullopt;
  }

  const auto timeout = std::chrono::microseconds(config_.echo_timeout_us);
  const auto wait_start = std::chrono::steady_clock::now();

  std::optional<std::chrono::steady_clock::time_point> echo_start;

  while (std::chrono::steady_clock::now() - wait_start < timeout) {
    const auto value = read_gpio_value(config_.echo_pin);

    if (!value.has_value()) {
      set_error("echo_read_failed");
      return std::nullopt;
    }

    if (*value == 1) {
      echo_start = std::chrono::steady_clock::now();
      break;
    }
  }

  if (!echo_start.has_value()) {
    set_error("echo_start_timeout");
    return std::nullopt;
  }

  while (std::chrono::steady_clock::now() - *echo_start < timeout) {
    const auto value = read_gpio_value(config_.echo_pin);

    if (!value.has_value()) {
      set_error("echo_read_failed");
      return std::nullopt;
    }

    if (*value == 0) {
      const auto echo_end = std::chrono::steady_clock::now();
      const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        echo_end - *echo_start);

      return pulse_duration_to_distance_m(duration);
    }
  }

  set_error("echo_end_timeout");
  return std::nullopt;
}

std::optional<double> UltrasonicReader::pulse_duration_to_distance_m(
  const std::chrono::microseconds pulse_duration) const
{
  return ultrasonic_echo_us_to_distance_m(
    static_cast<int>(pulse_duration.count()),
    config_.speed_of_sound_mps);
}

bool UltrasonicReader::valid_distance(const double distance_m) const
{
  return valid_ultrasonic_distance(
    distance_m,
    config_.valid_min_m,
    config_.valid_max_m);
}

std::optional<double> UltrasonicReader::filtered_distance_m() const
{
  return median_from_history(history_);
}

void UltrasonicReader::append_valid_reading(const double distance_m)
{
  if (!valid_distance(distance_m)) {
    set_error("distance_out_of_valid_range");
    return;
  }

  history_.push_back(distance_m);

  while (static_cast<int>(history_.size()) > config_.queue_len) {
    history_.pop_front();
  }

  last_error_.clear();
}

void UltrasonicReader::set_error(std::string error)
{
  last_error_ = std::move(error);
}

void UltrasonicReader::move_from(UltrasonicReader && other) noexcept
{
  config_ = std::move(other.config_);
  started_ = other.started_;
  history_ = std::move(other.history_);
  last_error_ = std::move(other.last_error_);

  other.started_ = false;
}

}  // namespace savo_perception