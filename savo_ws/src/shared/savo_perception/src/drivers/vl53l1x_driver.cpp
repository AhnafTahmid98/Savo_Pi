#include "savo_perception/vl53l1x_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <utility>
#include <vector>

#if __has_include("VL53L1X_api.h")
  extern "C" {
    #include "VL53L1X_api.h"
  }
  #define SAVO_PERCEPTION_HAS_VL53L1X_ULD 1
#else
  #define SAVO_PERCEPTION_HAS_VL53L1X_ULD 0
#endif

namespace savo_perception
{
namespace
{

std::uint16_t api_address(const std::uint8_t address_7bit)
{
  return static_cast<std::uint16_t>(address_7bit << 1U);
}

void sleep_seconds(const double seconds)
{
  if (seconds <= 0.0) {
    return;
  }

  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

}  // namespace

bool valid_vl53_median_window(const int median_window)
{
  return median_window >= 1 && (median_window % 2) == 1;
}

bool valid_vl53_distance_mm(const int distance_mm)
{
  return distance_mm > 0 && distance_mm < 4000;
}

std::optional<double> vl53_mm_to_m(
  const int distance_mm,
  const double valid_min_m,
  const double valid_max_m)
{
  if (!valid_vl53_distance_mm(distance_mm)) {
    return std::nullopt;
  }

  const auto distance_m = static_cast<double>(distance_mm) / 1000.0;

  if (!is_valid_distance(distance_m, valid_min_m, valid_max_m)) {
    return std::nullopt;
  }

  return distance_m;
}

Vl53l1xDriver::Vl53l1xDriver(
  Vl53l1xConfig config,
  std::shared_ptr<Tca9548a> mux)
: config_(std::move(config)),
  mux_(std::move(mux))
{
  if (!valid_vl53_median_window(config_.median_window)) {
    config_.median_window = constants::kVl53MedianWindowDefault;
  }
}

Vl53l1xDriver::~Vl53l1xDriver()
{
  stop();
}

Vl53l1xDriver::Vl53l1xDriver(Vl53l1xDriver && other) noexcept
{
  move_from(std::move(other));
}

Vl53l1xDriver & Vl53l1xDriver::operator=(Vl53l1xDriver && other) noexcept
{
  if (this != &other) {
    stop();
    move_from(std::move(other));
  }

  return *this;
}

const Vl53l1xConfig & Vl53l1xDriver::config() const
{
  return config_;
}

bool Vl53l1xDriver::started() const
{
  return started_;
}

std::string Vl53l1xDriver::last_error() const
{
  return last_error_;
}

bool Vl53l1xDriver::start()
{
  if (started_) {
    return true;
  }

  if (!mux_) {
    set_error("mux_not_set");
    return false;
  }

  if (!mux_->open()) {
    set_error("mux_open_failed:" + mux_->last_error());
    return false;
  }

  if (!select_mux_channel()) {
    return false;
  }

  sleep_seconds(config_.init_settle_s);

  if (!initialize_sensor()) {
    return false;
  }

  if (!start_ranging()) {
    return false;
  }

  started_ = true;
  last_error_.clear();
  return true;
}

void Vl53l1xDriver::stop()
{
  if (started_) {
    stop_ranging();
  }

  started_ = false;
}

Vl53l1xReading Vl53l1xDriver::read_once()
{
  Vl53l1xReading reading;
  reading.sensor_name = config_.sensor_name;
  reading.mux_channel = config_.mux_channel;

  if (!started_) {
    reading.error = "not_started";
    return reading;
  }

  if (!select_mux_channel()) {
    reading.error = last_error_;
    return reading;
  }

  sleep_seconds(config_.settle_s);

  const auto raw_m = read_distance_m();
  reading.raw_m = raw_m;

  if (!raw_m.has_value()) {
    reading.error = last_error_.empty() ? "read_failed" : last_error_;
    return reading;
  }

  if (!append_valid_reading(*raw_m)) {
    reading.error = last_error_;
    return reading;
  }

  reading.filtered_m = median_distance_m();
  reading.valid = reading.filtered_m.has_value();

  if (!reading.valid) {
    reading.error = "filter_failed";
  }

  return reading;
}

RangeSample Vl53l1xDriver::read_sample()
{
  return read_once().to_sample();
}

bool Vl53l1xDriver::select_mux_channel()
{
  if (!mux_) {
    set_error("mux_not_set");
    return false;
  }

  if (!mux_->select_channel(config_.mux_channel)) {
    set_error("mux_select_failed:" + mux_->last_error());
    return false;
  }

  return true;
}

bool Vl53l1xDriver::initialize_sensor()
{
#if SAVO_PERCEPTION_HAS_VL53L1X_ULD
  const auto dev = api_address(config_.sensor_address);

  auto status = VL53L1X_SensorInit(dev);
  if (status != 0) {
    set_error("sensor_init_failed");
    return false;
  }

  status = VL53L1X_SetDistanceMode(dev, 1);
  if (status != 0) {
    set_error("set_distance_mode_failed");
    return false;
  }

  status = VL53L1X_SetTimingBudgetInMs(dev, 50);
  if (status != 0) {
    set_error("set_timing_budget_failed");
    return false;
  }

  status = VL53L1X_SetInterMeasurementInMs(dev, 70);
  if (status != 0) {
    set_error("set_inter_measurement_failed");
    return false;
  }

  last_error_.clear();
  return true;
#else
  set_error("vl53l1x_uld_header_not_available");
  return false;
#endif
}

bool Vl53l1xDriver::start_ranging()
{
#if SAVO_PERCEPTION_HAS_VL53L1X_ULD
  const auto dev = api_address(config_.sensor_address);
  const auto status = VL53L1X_StartRanging(dev);

  if (status != 0) {
    set_error("start_ranging_failed");
    return false;
  }

  last_error_.clear();
  return true;
#else
  set_error("vl53l1x_uld_header_not_available");
  return false;
#endif
}

bool Vl53l1xDriver::stop_ranging()
{
#if SAVO_PERCEPTION_HAS_VL53L1X_ULD
  const auto dev = api_address(config_.sensor_address);
  const auto status = VL53L1X_StopRanging(dev);

  if (status != 0) {
    set_error("stop_ranging_failed");
    return false;
  }

  last_error_.clear();
  return true;
#else
  return true;
#endif
}

std::optional<int> Vl53l1xDriver::read_distance_mm()
{
#if SAVO_PERCEPTION_HAS_VL53L1X_ULD
  const auto dev = api_address(config_.sensor_address);

  std::uint8_t ready = 0;

  for (int attempt = 0; attempt < 10; ++attempt) {
    const auto status = VL53L1X_CheckForDataReady(dev, &ready);

    if (status != 0) {
      set_error("check_data_ready_failed");
      return std::nullopt;
    }

    if (ready != 0) {
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  if (ready == 0) {
    set_error("data_not_ready");
    return std::nullopt;
  }

  std::uint16_t distance_mm = 0;
  const auto distance_status = VL53L1X_GetDistance(dev, &distance_mm);

  if (distance_status != 0) {
    set_error("get_distance_failed");
    return std::nullopt;
  }

  const auto clear_status = VL53L1X_ClearInterrupt(dev);
  if (clear_status != 0) {
    set_error("clear_interrupt_failed");
    return std::nullopt;
  }

  last_error_.clear();
  return static_cast<int>(distance_mm);
#else
  set_error("vl53l1x_uld_header_not_available");
  return std::nullopt;
#endif
}

std::optional<double> Vl53l1xDriver::read_distance_m()
{
  const auto distance_mm = read_distance_mm();

  if (!distance_mm.has_value()) {
    return std::nullopt;
  }

  const auto distance_m = vl53_mm_to_m(
    *distance_mm,
    config_.valid_min_m,
    config_.valid_max_m);

  if (!distance_m.has_value()) {
    set_error("distance_out_of_valid_range");
    return std::nullopt;
  }

  return distance_m;
}

std::optional<double> Vl53l1xDriver::median_distance_m() const
{
  if (history_.empty()) {
    return std::nullopt;
  }

  std::vector<double> sorted(history_.begin(), history_.end());
  std::sort(sorted.begin(), sorted.end());

  return sorted[sorted.size() / 2];
}

bool Vl53l1xDriver::append_valid_reading(const double distance_m)
{
  if (!is_valid_distance(distance_m, config_.valid_min_m, config_.valid_max_m)) {
    set_error("distance_out_of_valid_range");
    return false;
  }

  history_.push_back(distance_m);

  while (static_cast<int>(history_.size()) > config_.median_window) {
    history_.pop_front();
  }

  last_error_.clear();
  return true;
}

void Vl53l1xDriver::set_error(std::string error)
{
  last_error_ = std::move(error);
}

void Vl53l1xDriver::move_from(Vl53l1xDriver && other) noexcept
{
  config_ = std::move(other.config_);
  mux_ = std::move(other.mux_);
  started_ = other.started_;
  history_ = std::move(other.history_);
  last_error_ = std::move(other.last_error_);

  other.started_ = false;
}

Vl53MuxPairDriver::Vl53MuxPairDriver(Vl53MuxPairConfig config)
: config_(config)
{
}

Vl53MuxPairDriver::~Vl53MuxPairDriver()
{
  stop();
}

Vl53MuxPairDriver::Vl53MuxPairDriver(Vl53MuxPairDriver && other) noexcept
{
  move_from(std::move(other));
}

Vl53MuxPairDriver & Vl53MuxPairDriver::operator=(Vl53MuxPairDriver && other) noexcept
{
  if (this != &other) {
    stop();
    move_from(std::move(other));
  }

  return *this;
}

const Vl53MuxPairConfig & Vl53MuxPairDriver::config() const
{
  return config_;
}

bool Vl53MuxPairDriver::started() const
{
  return started_;
}

std::string Vl53MuxPairDriver::last_error() const
{
  return last_error_;
}

bool Vl53MuxPairDriver::start()
{
  if (started_) {
    return true;
  }

  if (!valid_tca_channel(config_.right_channel)) {
    set_error("invalid_right_channel");
    return false;
  }

  if (!valid_tca_channel(config_.left_channel)) {
    set_error("invalid_left_channel");
    return false;
  }

  mux_ = std::make_shared<Tca9548a>(
    Tca9548aConfig{
      config_.bus,
      config_.tca_address});

  if (!mux_->open()) {
    set_error("mux_open_failed:" + mux_->last_error());
    return false;
  }

  Vl53l1xConfig right_config;
  right_config.bus = config_.bus;
  right_config.sensor_address = config_.sensor_address;
  right_config.mux_channel = config_.right_channel;
  right_config.settle_s = config_.settle_s;
  right_config.init_settle_s = config_.init_settle_s;
  right_config.valid_min_m = config_.valid_min_m;
  right_config.valid_max_m = config_.valid_max_m;
  right_config.median_window = config_.median_window;
  right_config.sensor_name = "tof_right";
  right_config.source = "vl53l1x";

  Vl53l1xConfig left_config = right_config;
  left_config.mux_channel = config_.left_channel;
  left_config.sensor_name = "tof_left";

  right_ = std::make_unique<Vl53l1xDriver>(right_config, mux_);
  left_ = std::make_unique<Vl53l1xDriver>(left_config, mux_);

  if (!right_->start()) {
    set_error("right_start_failed:" + right_->last_error());
    stop();
    return false;
  }

  if (!left_->start()) {
    set_error("left_start_failed:" + left_->last_error());
    stop();
    return false;
  }

  started_ = true;
  last_error_.clear();
  return true;
}

void Vl53MuxPairDriver::stop()
{
  if (left_) {
    left_->stop();
  }

  if (right_) {
    right_->stop();
  }

  if (mux_) {
    mux_->disable_all();
    mux_->close();
  }

  started_ = false;
}

Vl53MuxPairReading Vl53MuxPairDriver::read_once()
{
  Vl53MuxPairReading reading;

  if (!started_ || !left_ || !right_) {
    reading.left.sensor_name = "tof_left";
    reading.right.sensor_name = "tof_right";
    reading.left.mux_channel = config_.left_channel;
    reading.right.mux_channel = config_.right_channel;
    reading.left.error = "not_started";
    reading.right.error = "not_started";
    return reading;
  }

  reading.right = right_->read_once();
  reading.left = left_->read_once();

  if (!reading.right.valid && !reading.right.error.empty()) {
    set_error("right_read_failed:" + reading.right.error);
  } else if (!reading.left.valid && !reading.left.error.empty()) {
    set_error("left_read_failed:" + reading.left.error);
  } else {
    last_error_.clear();
  }

  return reading;
}

Vl53MuxPairSamples Vl53MuxPairDriver::read_samples()
{
  const auto reading = read_once();

  Vl53MuxPairSamples samples;
  samples.left = reading.left.to_sample();
  samples.right = reading.right.to_sample();

  return samples;
}

void Vl53MuxPairDriver::set_error(std::string error)
{
  last_error_ = std::move(error);
}

void Vl53MuxPairDriver::move_from(Vl53MuxPairDriver && other) noexcept
{
  config_ = other.config_;
  mux_ = std::move(other.mux_);
  left_ = std::move(other.left_);
  right_ = std::move(other.right_);
  started_ = other.started_;
  last_error_ = std::move(other.last_error_);

  other.started_ = false;
}

}  // namespace savo_perception