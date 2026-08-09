// Copyright 2026 Ahnaf Tahmid
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "savo_ui/runtime/ui_runtime_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<savo_ui::UiRuntimeNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
