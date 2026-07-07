#include "savo_power/constants.hpp"
#include "savo_power/linux_i2c_bus.hpp"
#include "savo_power/power_format.hpp"
#include "savo_power/power_node_params.hpp"
#include "savo_power/topic_names.hpp"
#include "savo_power/ups_hat_driver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

namespace savo_power
{
namespace
{

double clamp_rate_hz(double value)
{
  return std::clamp(value, 0.1, 10.0);
}

std::uint8_t checked_i2c_address(int value, const std::string & name)
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

}  // namespace

class EdgeUpsNode final : public rclcpp::Node
{
public:
  EdgeUpsNode()
  : rclcpp::Node(constants::kEdgeUpsNodeName)
  {
    load_params();

    publisher_ = create_publisher<std_msgs::msg::String>(
      topics::kEdgeUps,
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
      "Started %s on I2C bus %d address 0x%02X publishing %s",
      constants::kEdgeUpsNodeName,
      params_.i2c_bus,
      params_.device_address,
      topics::kEdgeUps);
  }

private:
  void load_params()
  {
    params_ = make_edge_ups_node_params();

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

    params_.source = BatterySource::EDGE_UPS;
  }

  bool ensure_driver()
  {
    if (driver_) {
      return true;
    }

    try {
      bus_ = std::make_unique<LinuxI2cBus>(params_.i2c_bus);
      driver_ = std::make_unique<UpsHatDriver>(
        *bus_,
        BatterySource::EDGE_UPS,
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
        "Edge UPS driver not ready: %s",
        last_error_.c_str());

      return false;
    }
  }

  BatteryReading read_edge_ups()
  {
    if (!ensure_driver()) {
      return make_error_reading(BatterySource::EDGE_UPS, last_error_);
    }

    auto reading = driver_->read();
    reading.state = evaluate_ups_state(reading, params_.thresholds);
    reading.ok = reading.state != PowerState::ERROR &&
                 reading.state != PowerState::UNKNOWN &&
                 reading.state != PowerState::STALE;

    return reading;
  }

  void publish_reading()
  {
    const auto reading = read_edge_ups();

    std_msgs::msg::String message;
    message.data = format_battery_reading_line(reading);

    publisher_->publish(message);

    if (reading.state == PowerState::CRITICAL) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Edge UPS critical: %s",
        message.data.c_str());
      return;
    }

    if (reading.state == PowerState::LOW) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Edge UPS low: %s",
        message.data.c_str());
      return;
    }

    if (reading.state == PowerState::ERROR) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Edge UPS read error: %s",
        message.data.c_str());
    }
  }

  UpsNodeParams params_{};
  std::unique_ptr<LinuxI2cBus> bus_{};
  std::unique_ptr<UpsHatDriver> driver_{};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_{};
  rclcpp::TimerBase::SharedPtr timer_{};

  std::string last_error_{};
};

}  // namespace savo_power

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<savo_power::EdgeUpsNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
