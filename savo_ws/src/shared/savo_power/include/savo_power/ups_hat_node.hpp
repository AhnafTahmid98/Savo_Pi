#ifndef SAVO_POWER__UPS_HAT_NODE_HPP_
#define SAVO_POWER__UPS_HAT_NODE_HPP_

#include "savo_power/battery_reading.hpp"
#include "savo_power/linux_i2c_bus.hpp"
#include "savo_power/power_node_params.hpp"
#include "savo_power/ups_hat_driver.hpp"
#include "savo_power/visibility_control.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace savo_power
{

class SAVO_POWER_PUBLIC UpsHatNode : public rclcpp::Node
{
public:
  explicit UpsHatNode(
    PowerNodeRole role,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  UpsHatNode(
    PowerNodeRole role,
    std::string node_name,
    std::string topic_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  UpsHatNode(const UpsHatNode &) = delete;
  UpsHatNode & operator=(const UpsHatNode &) = delete;

private:
  void load_params();

  bool ensure_driver();

  BatteryReading read_ups();

  void publish_reading();

  static std::string default_node_name(PowerNodeRole role);

  static std::string default_topic_name(PowerNodeRole role);

  static BatterySource source_for_role(PowerNodeRole role);

  static std::uint8_t checked_i2c_address(
    int value,
    const std::string & name);

  PowerNodeRole role_{PowerNodeRole::UNKNOWN};
  UpsNodeParams params_{};

  std::string topic_name_{};

  std::unique_ptr<LinuxI2cBus> bus_{};
  std::unique_ptr<UpsHatDriver> driver_{};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_{};
  rclcpp::TimerBase::SharedPtr timer_{};

  std::string last_error_{};
};

}  // namespace savo_power

#endif  // SAVO_POWER__UPS_HAT_NODE_HPP_
