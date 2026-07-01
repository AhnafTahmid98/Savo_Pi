#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "savo_vo/rgbd_odometry_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<savo_vo::RGBDOdometryNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
