#include "savo_power/ups_hat_node.hpp"

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

UpsHatNode::UpsHatNode(
  PowerNodeRole role,
  const rclcpp::NodeOptions & options)
: UpsHatNode(
    role,
    default_node_name(role),
    default_topic_name(role),
    options)
{
}

UpsHatNode::UpsHatNode(
  PowerNodeRole role,
  std::string node_name,
  std::string topic_name,
  const rclcpp::NodeOptions & options)
: rclcpp::Node(std::move(node_name), options),
  role_(role),
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
    "Started %s role=%s on I2C bus %d address 0x%02X publishing %s",
    get_name(),
    std::string(to_string(role_)).c_str(),
    params_.i2c_bus,
    static_cast<unsigned int>(params_.device_address),
    topic_name_.c_str());
}

void UpsHatNode::load_params()
{
  if (role_ == PowerNodeRole::CORE_UPS) {
    params_ = make_core_ups_node_params();
  } else if (role_ == PowerNodeRole::EDGE_UPS) {
    params_ = make_edge_ups_node_params();
  } else {
    throw std::invalid_argument(
      "UpsHatNode role must be CORE_UPS or EDGE_UPS");
  }

  params_.i2c_bus = declare_parameter<int>(
    params::kI2cBus,
    params_.i2c_bus);

  const int ups_address = declare_parameter<int>(
    params::kUpsAddress,
    static_cast<int>(params_.device_address));

  params_.device_address = checked_i2c_address(
    ups_address,
    params::kUpsAddress);

  params_.sample_rate_hz = declare_parameter<double>(
    params::kSampleRateHz,
    params_.sample_rate_hz);

  params_.publish_rate_hz = declare_parameter<double>(
    params::kPublishRateHz,
    params_.publish_rate_hz);

  params_.thresholds.ups_low_voltage_v = declare_parameter<double>(
    params::kUpsLowVoltage,
    params_.thresholds.ups_low_voltage_v);

  params_.thresholds.ups_critical_voltage_v = declare_parameter<double>(
    params::kUpsCriticalVoltage,
    params_.thresholds.ups_critical_voltage_v);

  params_.thresholds.full_capacity_pct = declare_parameter<double>(
    params::kFullCapacity,
    params_.thresholds.full_capacity_pct);

  params_.thresholds.automatic_shutdown_enabled = declare_parameter<bool>(
    params::kAutomaticShutdownEnabled,
    params_.thresholds.automatic_shutdown_enabled);

  params_.source = source_for_role(role_);
}

bool UpsHatNode::ensure_driver()
{
  if (driver_) {
    return true;
  }

  try {
    bus_ = std::make_unique<LinuxI2cBus>(params_.i2c_bus);
    driver_ = std::make_unique<UpsHatDriver>(
      *bus_,
      params_.source,
      params_.device_address);

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
      "UPS HAT driver not ready for role=%s: %s",
      std::string(to_string(role_)).c_str(),
      last_error_.c_str());

    return false;
  }
}

BatteryReading UpsHatNode::read_ups()
{
  if (!ensure_driver()) {
    return make_error_reading(params_.source, last_error_);
  }

  auto reading = driver_->read();
  reading.state = evaluate_ups_state(reading, params_.thresholds);
  reading.ok = reading.state != PowerState::ERROR &&
               reading.state != PowerState::UNKNOWN &&
               reading.state != PowerState::STALE;

  return reading;
}

void UpsHatNode::publish_reading()
{
  const auto reading = read_ups();

  std_msgs::msg::String message;
  message.data = format_battery_reading_line(reading);

  publisher_->publish(message);

  if (reading.state == PowerState::CRITICAL) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "UPS HAT critical role=%s: %s",
      std::string(to_string(role_)).c_str(),
      message.data.c_str());
    return;
  }

  if (reading.state == PowerState::LOW) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "UPS HAT low role=%s: %s",
      std::string(to_string(role_)).c_str(),
      message.data.c_str());
    return;
  }

  if (reading.state == PowerState::ERROR) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "UPS HAT read error role=%s: %s",
      std::string(to_string(role_)).c_str(),
      message.data.c_str());
  }
}

std::string UpsHatNode::default_node_name(PowerNodeRole role)
{
  switch (role) {
    case PowerNodeRole::CORE_UPS:
      return constants::kCoreUpsNodeName;
    case PowerNodeRole::EDGE_UPS:
      return constants::kEdgeUpsNodeName;
    case PowerNodeRole::BASE_BATTERY:
    case PowerNodeRole::AGGREGATOR:
    case PowerNodeRole::HEALTH:
    case PowerNodeRole::DASHBOARD:
    case PowerNodeRole::UNKNOWN:
      break;
  }

  return "ups_hat_node";
}

std::string UpsHatNode::default_topic_name(PowerNodeRole role)
{
  switch (role) {
    case PowerNodeRole::CORE_UPS:
      return topics::kCoreUps;
    case PowerNodeRole::EDGE_UPS:
      return topics::kEdgeUps;
    case PowerNodeRole::BASE_BATTERY:
    case PowerNodeRole::AGGREGATOR:
    case PowerNodeRole::HEALTH:
    case PowerNodeRole::DASHBOARD:
    case PowerNodeRole::UNKNOWN:
      break;
  }

  return "/savo_power/ups";
}

BatterySource UpsHatNode::source_for_role(PowerNodeRole role)
{
  switch (role) {
    case PowerNodeRole::CORE_UPS:
      return BatterySource::CORE_UPS;
    case PowerNodeRole::EDGE_UPS:
      return BatterySource::EDGE_UPS;
    case PowerNodeRole::BASE_BATTERY:
    case PowerNodeRole::AGGREGATOR:
    case PowerNodeRole::HEALTH:
    case PowerNodeRole::DASHBOARD:
    case PowerNodeRole::UNKNOWN:
      break;
  }

  return BatterySource::UNKNOWN;
}

std::uint8_t UpsHatNode::checked_i2c_address(
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

}  // namespace savo_power
