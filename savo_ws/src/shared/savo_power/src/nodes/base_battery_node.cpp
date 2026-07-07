#include "savo_power/kit_battery_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<savo_power::KitBatteryNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
