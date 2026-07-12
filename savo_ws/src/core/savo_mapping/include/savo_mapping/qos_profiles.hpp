#pragma once

#include <rclcpp/qos.hpp>

#include <cstddef>

namespace savo_mapping::qos
{

inline constexpr std::size_t STATUS_DEPTH = 10;
inline constexpr std::size_t STATE_DEPTH = 1;
inline constexpr std::size_t COMMAND_DEPTH = 10;
inline constexpr std::size_t MAP_DEPTH = 1;
inline constexpr std::size_t EVENT_DEPTH = 20;

rclcpp::QoS status_qos();
rclcpp::QoS state_qos();
rclcpp::QoS command_qos();
rclcpp::QoS scan_qos();
rclcpp::QoS pointcloud_qos();
rclcpp::QoS map_qos();
rclcpp::QoS event_qos();

}  // namespace savo_mapping::qos
