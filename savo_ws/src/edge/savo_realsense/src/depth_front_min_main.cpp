// Copyright 2026 Ahnaf Tahmid

#include <exception>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "savo_realsense/depth_front_min_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<savo_realsense::DepthFrontMinNode>());
  } catch (const std::exception & exc) {
    std::cerr << "depth_front_min_node failed: " << exc.what() << std::endl;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
