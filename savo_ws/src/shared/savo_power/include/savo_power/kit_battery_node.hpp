#ifndef SAVO_POWER__KIT_BATTERY_NODE_HPP_
#define SAVO_POWER__KIT_BATTERY_NODE_HPP_

#include "savo_power/ads7830_driver.hpp"
#include "savo_power/battery_reading.hpp"
#include "savo_power/linux_i2c_bus.hpp"
#include "savo_power/power_node_params.hpp"
#include "savo_power/visibility_control.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace savo_power
{

class SAVO_POWER_PUBLIC KitBatteryNode : public rclcpp::Node
{
public:
  explicit KitBatteryNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  KitBatteryNode(
    std::string node_name,
    std::string topic_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  KitBatteryNode(const KitBatteryNode &) = delete;
  KitBatteryNode & operator=(const KitBatteryNode &) = delete;

private:
  void load_params();

  bool ensure_driver();

  BatteryReading read_battery();

  void publish_reading();

  static std::uint8_t checked_i2c_address(
    int value,
    const std::string & name);

  static std::uint8_t checked_ads7830_channel(
    int value,
    const std::string & name);

  BaseBatteryNodeParams params_{};

  std::string topic_name_{};

  std::unique_ptr<LinuxI2cBus> bus_{};
  std::unique_ptr<Ads7830Driver> driver_{};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_{};
  rclcpp::TimerBase::SharedPtr timer_{};

  std::string last_error_{};
};

}  // namespace savo_power

#endif  // SAVO_POWER__KIT_BATTERY_NODE_HPP_
