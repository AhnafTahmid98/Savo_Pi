// wheel_odom.hpp
// Robot SAVO — Wheel Odometry (C++)
// ------------------------------------------------------------
// Header for a lightweight wheel odometry node that reads two
// rear-wheel quadrature encoders (A/B per wheel) and publishes
// nav_msgs/Odometry on /wheel/odom.
//
// IMPORTANT (mecanum + 2 encoders):
// We read quadrature A/B on the two rear wheels (4 GPIO signals total), so we have
// direction + high-resolution counts for Left and Right. However, it is still only
// two-wheel odometry on a mecanum base.
// - We estimate vx and wz using a differential-style approximation from rear wheels.
// - We set vy = 0.0 because lateral motion is not observable without front wheel
//   encoders or additional sensing.
// Fuse with IMU + SLAM/AMCL in robot_localization for stable navigation.
//
// Notes:
// - This file is header-only style for clarity; you can split into .cpp later.
// - GPIO backend can be lgpio (recommended on Pi 5). Replace placeholders as needed.
//
// ------------------------------------------------------------

#pragma once

#include <cstdint>
#include <cmath>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace savo_localization
{

// ------------------------------------------------------------
// Utility: yaw -> quaternion
// ------------------------------------------------------------
inline geometry_msgs::msg::Quaternion yawToQuat(double yaw_rad)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_rad);
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

// ------------------------------------------------------------
// Quadrature decoding (Gray-code transition table)
// A is bit1, B is bit0
// ------------------------------------------------------------
inline int8_t quadDelta(uint8_t prev, uint8_t curr)
{
  // Valid transitions:
  // 00->01->11->10->00 : +1
  // 00->10->11->01->00 : -1
  if (prev == curr) return 0;

  // clockwise (+1) transitions
  if ((prev == 0b00 && curr == 0b01) ||
      (prev == 0b01 && curr == 0b11) ||
      (prev == 0b11 && curr == 0b10) ||
      (prev == 0b10 && curr == 0b00))
  {
    return +1;
  }

  // counter-clockwise (-1) transitions
  if ((prev == 0b00 && curr == 0b10) ||
      (prev == 0b10 && curr == 0b11) ||
      (prev == 0b11 && curr == 0b01) ||
      (prev == 0b01 && curr == 0b00))
  {
    return -1;
  }

  // illegal jump (noise / missed edge)
  return 0; // keep 0 here; count illegal separately if you want
}

// ------------------------------------------------------------
// Minimal encoder state (per side)
// ------------------------------------------------------------
struct EncoderSide
{
  int pin_a = -1;
  int pin_b = -1;
  int invert = +1;          // +1 or -1 (to make forward positive)
  int64_t count = 0;        // signed count
  uint8_t prev_state = 0;   // 2-bit A/B
};

// ------------------------------------------------------------
// WheelOdom parameters (tuned for Robot SAVO defaults)
// ------------------------------------------------------------
struct WheelOdomParams
{
  // GPIO pins (BCM)
  int left_a  = 21;
  int left_b  = 20;
  int right_a = 12;
  int right_b = 26;

  // kinematics
  double wheel_dia_m = 0.065;
  int cpr = 20;              // counts per rev per channel
  int decoding = 4;          // 1/2/4
  double gear = 1.0;         // motor->wheel ratio
  double track_m = 0.165;    // used for omega

  // publish config
  double publish_hz = 50.0;
  std::string frame_odom = "odom";
  std::string frame_base = "base_link";

  // direction fix
  bool invert_left = true;   // based on your typical usage
  bool invert_right = true;

  // output covariance (simple sane defaults)
  double cov_x = 0.02;
  double cov_y = 0.50;       // large, because vy unknown
  double cov_yaw = 0.05;
  double cov_vx = 0.05;
  double cov_vy = 1.00;      // large, vy unknown
  double cov_wz = 0.05;
};

// ------------------------------------------------------------
// WheelOdomNode (skeleton interface)
// Replace readGPIO() with lgpio implementation in .cpp
// ------------------------------------------------------------
class WheelOdomNode : public rclcpp::Node
{
public:
  explicit WheelOdomNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("wheel_odom_node", options)
  {
    // Declare params (keep same names for YAML)
    p_.left_a  = declare_parameter<int>("left_a", p_.left_a);
    p_.left_b  = declare_parameter<int>("left_b", p_.left_b);
    p_.right_a = declare_parameter<int>("right_a", p_.right_a);
    p_.right_b = declare_parameter<int>("right_b", p_.right_b);

    p_.wheel_dia_m = declare_parameter<double>("wheel_dia_m", p_.wheel_dia_m);
    p_.cpr         = declare_parameter<int>("cpr", p_.cpr);
    p_.decoding    = declare_parameter<int>("decoding", p_.decoding);
    p_.gear        = declare_parameter<double>("gear", p_.gear);
    p_.track_m     = declare_parameter<double>("track_m", p_.track_m);

    p_.publish_hz  = declare_parameter<double>("publish_hz", p_.publish_hz);
    p_.frame_odom  = declare_parameter<std::string>("frame_odom", p_.frame_odom);
    p_.frame_base  = declare_parameter<std::string>("frame_base", p_.frame_base);

    p_.invert_left  = declare_parameter<bool>("invert_left", p_.invert_left);
    p_.invert_right = declare_parameter<bool>("invert_right", p_.invert_right);

    // Publisher
    pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", rclcpp::QoS(20));

    // Init encoder sides
    left_.pin_a = p_.left_a;
    left_.pin_b = p_.left_b;
    left_.invert = p_.invert_left ? -1 : +1;

    right_.pin_a = p_.right_a;
    right_.pin_b = p_.right_b;
    right_.invert = p_.invert_right ? -1 : +1;

    // TODO: initGPIO() with lgpio and set left_.prev_state/right_.prev_state from initial reads

    // Timer
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, p_.publish_hz));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&WheelOdomNode::onTimer, this)
    );

    last_time_ = now();
    last_L_ = left_.count;
    last_R_ = right_.count;

    RCLCPP_INFO(get_logger(),
      "[wheel_odom_node] rear encoders only (A/B per wheel). Publishing /wheel/odom at %.1f Hz",
      p_.publish_hz
    );
  }

private:
  // Placeholder: replace with lgpio read (0/1)
  inline uint8_t readGPIO(int /*bcm_pin*/) const
  {
    // TODO: implement using lgpio in .cpp
    return 0;
  }

  inline uint8_t readAB(int pin_a, int pin_b) const
  {
    const uint8_t a = readGPIO(pin_a) & 1U;
    const uint8_t b = readGPIO(pin_b) & 1U;
    return static_cast<uint8_t>((a << 1) | b);
  }

  void sampleEncoder(EncoderSide& s)
  {
    const uint8_t curr = readAB(s.pin_a, s.pin_b);
    const int8_t d = quadDelta(s.prev_state, curr);
    if (d != 0)
    {
      s.count += static_cast<int64_t>(d) * static_cast<int64_t>(s.invert);
    }
    s.prev_state = curr;
  }

  void onTimer()
  {
    // In a real implementation you will sample in a faster loop or interrupt/callback.
    // This timer-based skeleton is for structure; the .cpp should do tight sampling.
    sampleEncoder(left_);
    sampleEncoder(right_);

    const rclcpp::Time t = now();
    const double dt = std::max(1e-6, (t - last_time_).seconds());

    const int64_t L = left_.count;
    const int64_t R = right_.count;
    const int64_t dL = L - last_L_;
    const int64_t dR = R - last_R_;
    last_L_ = L;
    last_R_ = R;
    last_time_ = t;

    const double edges_per_rev = static_cast<double>(p_.cpr) * static_cast<double>(p_.decoding);
    const double counts_per_wrev = std::max(1.0, edges_per_rev * p_.gear);
    const double wheel_circ = M_PI * p_.wheel_dia_m;

    const double L_v = (static_cast<double>(dL) / dt) / counts_per_wrev * wheel_circ;
    const double R_v = (static_cast<double>(dR) / dt) / counts_per_wrev * wheel_circ;

    // Differential-style approximation (rear wheels only)
    const double vx = 0.5 * (L_v + R_v);
    const double wz = (R_v - L_v) / std::max(1e-9, p_.track_m);
    const double vy = 0.0; // not observable with rear-only encoders on mecanum

    // Integrate pose in odom frame (simple)
    yaw_ += wz * dt;
    x_ += vx * std::cos(yaw_) * dt;
    y_ += vx * std::sin(yaw_) * dt;

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = t;
    msg.header.frame_id = p_.frame_odom;
    msg.child_frame_id = p_.frame_base;

    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = yawToQuat(yaw_);

    msg.twist.twist.linear.x = vx;
    msg.twist.twist.linear.y = vy;
    msg.twist.twist.angular.z = wz;

    // Covariances (simple, conservative)
    for (int i = 0; i < 36; i++) {
      msg.pose.covariance[i]  = 0.0;
      msg.twist.covariance[i] = 0.0;
    }
    msg.pose.covariance[0]  = p_.cov_x;
    msg.pose.covariance[7]  = p_.cov_y;
    msg.pose.covariance[35] = p_.cov_yaw;

    msg.twist.covariance[0]  = p_.cov_vx;
    msg.twist.covariance[7]  = p_.cov_vy;
    msg.twist.covariance[35] = p_.cov_wz;

    pub_->publish(msg);
  }

private:
  WheelOdomParams p_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  EncoderSide left_;
  EncoderSide right_;

  rclcpp::Time last_time_;
  int64_t last_L_{0};
  int64_t last_R_{0};

  // integrated pose
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
};

} // namespace savo_localization