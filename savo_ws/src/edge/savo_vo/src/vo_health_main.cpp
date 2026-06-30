#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "savo_vo/vo_health_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<savo_vo::VOHealthNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
