#include "savo_ui/app/ui_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<savo_ui::UiNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
