#include "savo_perception/vl53l1x_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifndef SAVO_PERCEPTION_HAVE_VL53L1X_ULD
#define SAVO_PERCEPTION_HAVE_VL53L1X_ULD 0
#endif

#if SAVO_PERCEPTION_HAVE_VL53L1X_ULD
#include "VL53L1X_api.h"
#include "VL53L1X_platform.h"
#endif

namespace savo_perception
{
namespace
{

std::uint16_t api_address(const std::uint8_t address_7bit)
{
  return static_cast<std::uint16_t>(address_7bit << 1);
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

bool valid_vl53_median_window(const int median_window)
{
  return median_window > 0 && median_window <= 25;
}

bool valid_vl53_distance_mm(const int distance_mm)
{
  return distance_mm > 0 && distance_mm < 8000;
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

  if (!std::isfinite(distance_m) || distance_m < valid_min_m || distance_m > valid_max_m) {
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

#if SAVO_PERCEPTION_HAVE_VL53L1X_ULD
  if (!select_mux_channel()) {
    return false;
  }

  VL53L1X_SetI2CBus(config_.bus);

  if (config_.init_settle_s > 0.0) {
    std::this_thread::sleep_for(std::chrono::duration<double>(config_.init_settle_s));
  }

  if (!initialize_sensor()) {
    return false;
  }

  if (!start_ranging()) {
    return false;
  }

  started_ = true;
  last_error_.clear();
  return true;
#else
  set_error("vl53l1x_uld_header_not_available");
  return false;
#endif
}

void Vl53l1xDriver::stop()
{
#if SAVO_PERCEPTION_HAVE_VL53L1X_ULD
  if (started_) {
    (void)select_mux_channel();
    (void)stop_ranging();
  }
#endif

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

  const auto distance_m = read_distance_m();
  reading.raw_m = distance_m;

  if (!distance_m.has_value()) {
    reading.error = last_error_.empty() ? "read_failed" : last_error_;
    return reading;
  }

  if (!append_valid_reading(*distance_m)) {
    reading.error = last_error_.empty() ? "distance_out_of_valid_range" : last_error_;
    return reading;
  }

  reading.filtered_m = median_distance_m();
  reading.valid = reading.filtered_m.has_value();

  if (!reading.valid) {
    reading.error = "median_filter_empty";
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
    set_error("mux_not_configured");
    return false;
  }

  if (!mux_->is_open() && !mux_->open()) {
    set_error("mux_open_failed:" + mux_->last_error());
    return false;
  }

  if (!mux_->select_channel(config_.mux_channel)) {
    set_error("mux_select_failed:" + mux_->last_error());
    return false;
  }

  if (config_.settle_s > 0.0) {
    std::this_thread::sleep_for(std::chrono::duration<double>(config_.settle_s));
  }

  return true;
}

bool Vl53l1xDriver::initialize_sensor()
{
#if SAVO_PERCEPTION_HAVE_VL53L1X_ULD
  const auto dev = api_address(config_.sensor_address);

  const auto status = VL53L1X_SensorInit(dev);
  if (status != 0) {
    set_error("sensor_init_failed:" + std::to_string(static_cast<int>(status)));
    return false;
  }

  return true;
#else
  set_error("vl53l1x_uld_header_not_available");
  return false;
#endif
}

bool Vl53l1xDriver::start_ranging()
{
#if SAVO_PERCEPTION_HAVE_VL53L1X_ULD
  const auto dev = api_address(config_.sensor_address);
  const auto status = VL53L1X_StartRanging(dev);

  if (status != 0) {
    set_error("start_ranging_failed:" + std::to_string(static_cast<int>(status)));
    return false;
  }

  return true;
#else
  set_error("vl53l1x_uld_header_not_available");
  return false;
#endif
}

bool Vl53l1xDriver::stop_ranging()
{
#if SAVO_PERCEPTION_HAVE_VL53L1X_ULD
  const auto dev = api_address(config_.sensor_address);
  const auto status = VL53L1X_StopRanging(dev);

  if (status != 0) {
    set_error("stop_ranging_failed:" + std::to_string(static_cast<int>(status)));
    return false;
  }

  return true;
#else
  return true;
#endif
}

std::optional<int> Vl53l1xDriver::read_distance_mm()
{
#if SAVO_PERCEPTION_HAVE_VL53L1X_ULD
  if (!select_mux_channel()) {
    return std::nullopt;
  }

  const auto dev = api_address(config_.sensor_address);

  std::uint8_t data_ready = 0;
  bool ready = false;

  for (int attempt = 0; attempt < 30; ++attempt) {
    const auto ready_status = VL53L1X_CheckForDataReady(dev, &data_ready);
    if (ready_status != 0) {
      set_error("data_ready_check_failed:" + std::to_string(static_cast<int>(ready_status)));
      return std::nullopt;
    }

    if (data_ready != 0U) {
      ready = true;
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  if (!ready) {
    set_error("data_not_ready");
    return std::nullopt;
  }

  std::uint16_t distance_mm = 0;
  const auto distance_status = VL53L1X_GetDistance(dev, &distance_mm);
  if (distance_status != 0) {
    set_error("get_distance_failed:" + std::to_string(static_cast<int>(distance_status)));
    return std::nullopt;
  }

  const auto clear_status = VL53L1X_ClearInterrupt(dev);
  if (clear_status != 0) {
    set_error("clear_interrupt_failed:" + std::to_string(static_cast<int>(clear_status)));
    return std::nullopt;
  }

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
  return median_from_history(history_);
}

bool Vl53l1xDriver::append_valid_reading(const double distance_m)
{
  if (!std::isfinite(distance_m) ||
    distance_m < config_.valid_min_m ||
    distance_m > config_.valid_max_m)
  {
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
: config_(std::move(config))
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

  mux_ = std::make_shared<Tca9548a>(
    Tca9548aConfig{
      config_.bus,
      config_.tca_address,
    });

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
    (void)mux_->disable_all();
    mux_->close();
  }

  left_.reset();
  right_.reset();
  mux_.reset();

  started_ = false;
}

Vl53MuxPairReading Vl53MuxPairDriver::read_once()
{
  Vl53MuxPairReading reading;

  if (!started_ || !left_ || !right_) {
    reading.left.sensor_name = "tof_left";
    reading.left.error = "not_started";
    reading.right.sensor_name = "tof_right";
    reading.right.error = "not_started";
    return reading;
  }

  reading.right = right_->read_once();
  reading.left = left_->read_once();

  if (!reading.right.valid && !reading.left.valid) {
    set_error("both_tof_reads_failed:" + reading.right.error + "," + reading.left.error);
  } else {
    last_error_.clear();
  }

  return reading;
}

Vl53MuxPairSamples Vl53MuxPairDriver::read_samples()
{
  const auto reading = read_once();

  return Vl53MuxPairSamples{
    reading.left.to_sample(),
    reading.right.to_sample(),
  };
}

void Vl53MuxPairDriver::set_error(std::string error)
{
  last_error_ = std::move(error);
}

void Vl53MuxPairDriver::move_from(Vl53MuxPairDriver && other) noexcept
{
  config_ = std::move(other.config_);
  mux_ = std::move(other.mux_);
  left_ = std::move(other.left_);
  right_ = std::move(other.right_);
  started_ = other.started_;
  last_error_ = std::move(other.last_error_);

  other.started_ = false;
}

}  // namespace savo_perception
