// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#ifndef SAVO_BRIDGE__ROS_GRAPH_DISCOVERY_HPP_
#define SAVO_BRIDGE__ROS_GRAPH_DISCOVERY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "savo_bridge/graph_evidence.hpp"

namespace savo_bridge
{

[[nodiscard]] GraphSnapshot collect_ros_graph_snapshot(
  rclcpp::Node & node);

}  // namespace savo_bridge

#endif  // SAVO_BRIDGE__ROS_GRAPH_DISCOVERY_HPP_
