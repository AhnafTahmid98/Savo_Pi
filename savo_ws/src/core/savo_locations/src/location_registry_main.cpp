// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "savo_locations/location_registry_node.hpp"


int main(
  int argc,
  char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<
      savo_locations::LocationRegistryNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
