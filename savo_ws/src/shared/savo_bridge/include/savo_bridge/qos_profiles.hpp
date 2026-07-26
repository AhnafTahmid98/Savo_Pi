// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#ifndef SAVO_BRIDGE__QOS_PROFILES_HPP_
#define SAVO_BRIDGE__QOS_PROFILES_HPP_

#include "rclcpp/qos.hpp"

namespace savo_bridge
{

[[nodiscard]] rclcpp::QoS bridge_latched_qos();
[[nodiscard]] rclcpp::QoS bridge_stream_qos();

}  // namespace savo_bridge

#endif  // SAVO_BRIDGE__QOS_PROFILES_HPP_
