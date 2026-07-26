// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include "savo_bridge/qos_profiles.hpp"

#include "rclcpp/qos.hpp"

namespace savo_bridge
{

rclcpp::QoS bridge_latched_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1U)).reliable().transient_local();
}

rclcpp::QoS bridge_stream_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(10U)).reliable().durability_volatile();
}

}  // namespace savo_bridge
