#include "savo_mapping/qos_profiles.hpp"

namespace savo_mapping::qos
{

rclcpp::QoS status_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(STATUS_DEPTH))
           .reliable()
           .durability_volatile();
}

rclcpp::QoS state_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(STATE_DEPTH))
           .reliable()
           .transient_local();
}

rclcpp::QoS command_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(COMMAND_DEPTH))
           .reliable()
           .durability_volatile();
}

rclcpp::QoS scan_qos()
{
  return rclcpp::SensorDataQoS();
}

rclcpp::QoS pointcloud_qos()
{
  return rclcpp::SensorDataQoS();
}

rclcpp::QoS map_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(MAP_DEPTH))
           .reliable()
           .transient_local();
}

rclcpp::QoS event_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(EVENT_DEPTH))
           .reliable()
           .durability_volatile();
}

}  // namespace savo_mapping::qos
