#pragma once

#include <cstddef>

#include <rclcpp/qos.hpp>

namespace savo_speech::qos
{

inline rclcpp::QoS state()
{
  auto profile = rclcpp::QoS(rclcpp::KeepLast(1));
  profile.reliable();
  profile.transient_local();
  return profile;
}

inline rclcpp::QoS event()
{
  auto profile = rclcpp::QoS(rclcpp::KeepLast(10));
  profile.reliable();
  profile.durability_volatile();
  return profile;
}

inline rclcpp::QoS status()
{
  auto profile = rclcpp::QoS(rclcpp::KeepLast(10));
  profile.reliable();
  profile.durability_volatile();
  return profile;
}

inline rclcpp::QoS health()
{
  auto profile = rclcpp::QoS(rclcpp::KeepLast(10));
  profile.reliable();
  profile.durability_volatile();
  return profile;
}

inline rclcpp::QoS heartbeat()
{
  auto profile = rclcpp::QoS(rclcpp::KeepLast(5));
  profile.best_effort();
  profile.durability_volatile();
  return profile;
}

}  // namespace savo_speech::qos
