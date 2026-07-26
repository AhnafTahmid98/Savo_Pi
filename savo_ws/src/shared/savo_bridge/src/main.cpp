// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "savo_bridge/bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<savo_bridge::BridgeNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
