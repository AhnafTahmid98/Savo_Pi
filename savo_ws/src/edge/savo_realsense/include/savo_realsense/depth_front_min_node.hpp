// Copyright 2026 Ahnaf Tahmid

#ifndef SAVO_REALSENSE__DEPTH_FRONT_MIN_NODE_HPP_
#define SAVO_REALSENSE__DEPTH_FRONT_MIN_NODE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"

namespace savo_realsense
{

class DepthFrontMinNode final : public rclcpp::Node
{
public:
  explicit DepthFrontMinNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void sanitize_parameters();

  void on_depth_image(const sensor_msgs::msg::Image::SharedPtr msg);

  void collect_16uc1(
    const sensor_msgs::msg::Image & msg,
    uint32_t x0,
    uint32_t x1,
    uint32_t y0,
    uint32_t y1,
    std::vector<float> & out) const;

  void collect_32fc1(
    const sensor_msgs::msg::Image & msg,
    uint32_t x0,
    uint32_t x1,
    uint32_t y0,
    uint32_t y1,
    std::vector<float> & out) const;

  void add_if_valid(float depth_m, std::vector<float> & out) const;

  float percentile_value(std::vector<float> & values) const;

  void publish_depth(float depth_m);

  std::string input_topic_;
  std::string output_topic_;

  double depth_scale_{0.001};
  double min_valid_m_{0.02};
  double max_valid_m_{3.0};
  double percentile_{10.0};

  double x_min_ratio_{0.35};
  double x_max_ratio_{0.65};
  double y_min_ratio_{0.35};
  double y_max_ratio_{0.75};

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

}  // namespace savo_realsense

#endif  // SAVO_REALSENSE__DEPTH_FRONT_MIN_NODE_HPP_
