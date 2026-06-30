#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "savo_vo/vo_republisher_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<savo_vo::VORepublisherNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
