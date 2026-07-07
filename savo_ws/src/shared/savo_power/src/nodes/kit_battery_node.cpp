#include "savo_power/kit_battery_node.hpp"

#include "savo_power/constants.hpp"
#include "savo_power/power_format.hpp"
#include "savo_power/power_policy.hpp"
#include "savo_power/topic_names.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <stdexcept>
#include <utility>

namespace savo_power
{
namespace
{

double clamp_rate_hz(double value)
{
  return std::clamp(value, 0.1, 10.0);
}

}  // namespace

KitBatteryNode::KitBatteryNode(
  const rclcpp::NodeOptions & options)
: KitBatteryNode(
    constants::kBaseBatteryNodeName,
    topics::kBaseBattery,
    options)
{
}

KitBatteryNode::KitBatteryNode(
  std::string node_name,
  std::string topic_name,
  const rclcpp::NodeOptions & options)
: rclcpp::Node(std::move(node_name), options),
  topic_name_(std::move(topic_name))
{
  load_params();

  publisher_ = create_publisher<std_msgs::msg::String>(
    topic_name_,
    rclcpp::QoS(10).reliable());

  const double rate_hz = clamp_rate_hz(params_.publish_rate_hz);
  const auto period = std::chrono::duration<double>(1.0 / rate_hz);

  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    [this]() {
      publish_reading();
    });

  RCLCPP_INFO(
    get_logger(),
    "Started %s on I2C bus %d address 0x%02X channel %u pcb=%s publishing %s",
    get_name(),
    params_.i2c_bus,
    static_cast<unsigned int>(params_.device_address),
    static_cast<unsigned int>(params_.channel),
    std::string(to_string(params_.pcb_version)).c_str(),
    topic_name_.c_str());
}

void KitBatteryNode::load_params()
{
  params_ = make_base_battery_node_params();

  params_.i2c_bus = declare_parameter<int>(
    params::kI2cBus,
    params_.i2c_bus);

  const int ads7830_address = declare_parameter<int>(
    params::kAds7830Address,
    static_cast<int>(params_.device_address));

  params_.device_address = checked_i2c_address(
    ads7830_address,
    params::kAds7830Address);

  const int ads7830_channel = declare_parameter<int>(
    params::kAds7830Channel,
    static_cast<int>(params_.channel));

  params_.channel = checked_ads7830_channel(
    ads7830_channel,
    params::kAds7830Channel);

  const std::string pcb_version = declare_parameter<std::string>(
    params::kAds7830PcbVersion,
    std::string(to_string(params_.pcb_version)));

  params_.pcb_version = ads7830_pcb_version_from_string(pcb_version);

  params_.sample_rate_hz = declare_parameter<double>(
    params::kSampleRateHz,
    params_.sample_rate_hz);

  params_.publish_rate_hz = declare_parameter<double>(
    params::kPublishRateHz,
    params_.publish_rate_hz);

  params_.thresholds.base_empty_voltage_v = declare_parameter<double>(
    params::kBaseEmptyVoltage,
    params_.thresholds.base_empty_voltage_v);

  params_.thresholds.base_low_voltage_v = declare_parameter<double>(
    params::kBaseLowVoltage,
    params_.thresholds.base_low_voltage_v);

  params_.thresholds.base_full_voltage_v = declare_parameter<double>(
    params::kBaseFullVoltage,
    params_.thresholds.base_full_voltage_v);

  params_.thresholds.base_low_soc_pct = declare_parameter<double>(
    params::kBaseLowSoc,
    params_.thresholds.base_low_soc_pct);

  params_.thresholds.base_full_soc_pct = declare_parameter<double>(
    params::kBaseFullSoc,
    params_.thresholds.base_full_soc_pct);

  params_.thresholds.automatic_shutdown_enabled = declare_parameter<bool>(
    params::kAutomaticShutdownEnabled,
    params_.thresholds.automatic_shutdown_enabled);
}

bool KitBatteryNode::ensure_driver()
{
  if (driver_) {
    return true;
  }

  try {
    bus_ = std::make_unique<LinuxI2cBus>(params_.i2c_bus);
    driver_ = std::make_unique<Ads7830Driver>(
      *bus_,
      params_.device_address,
      params_.channel,
      params_.pcb_version);

    last_error_.clear();
    return true;
  } catch (const std::exception & error) {
    driver_.reset();
    bus_.reset();

    last_error_ = error.what();

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Kit battery driver not ready: %s",
      last_error_.c_str());

    return false;
  }
}

BatteryReading KitBatteryNode::read_battery()
{
  if (!ensure_driver()) {
    return make_error_reading(BatterySource::BASE_BATTERY, last_error_);
  }

  auto reading = driver_->read();
  reading.state = evaluate_base_battery_state(reading, params_.thresholds);
  reading.ok = reading.state != PowerState::ERROR &&
               reading.state != PowerState::UNKNOWN &&
               reading.state != PowerState::STALE;

  return reading;
}

void KitBatteryNode::publish_reading()
{
  const auto reading = read_battery();

  std_msgs::msg::String message;
  message.data = format_battery_reading_line(reading);

  publisher_->publish(message);

  if (reading.state == PowerState::CRITICAL) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Kit/base battery critical: %s",
      message.data.c_str());
    return;
  }

  if (reading.state == PowerState::LOW) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Kit/base battery low: %s",
      message.data.c_str());
    return;
  }

  if (reading.state == PowerState::ERROR) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Kit/base battery read error: %s",
      message.data.c_str());
  }
}

std::uint8_t KitBatteryNode::checked_i2c_address(
  int value,
  const std::string & name)
{
  if (value < 0 || value > 0x7F) {
    throw std::invalid_argument(name + " must be a 7-bit I2C address");
  }

  const auto address = static_cast<std::uint8_t>(value);

  if (!is_usable_7bit_i2c_address(address)) {
    throw std::invalid_argument(name + " is reserved or not usable");
  }

  return address;
}

std::uint8_t KitBatteryNode::checked_ads7830_channel(
  int value,
  const std::string & name)
{
  if (value < 0 || value > 7) {
    throw std::invalid_argument(name + " must be in range 0..7");
  }

  return static_cast<std::uint8_t>(value);
}

}  // namespace savo_power
