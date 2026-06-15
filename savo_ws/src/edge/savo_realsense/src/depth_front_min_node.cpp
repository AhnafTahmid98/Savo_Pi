// Copyright 2026 Ahnaf Tahmid

#include "savo_realsense/depth_front_min_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>

namespace
{

bool is_system_big_endian()
{
  const uint16_t value = 0x0102;
  return *(reinterpret_cast<const uint8_t *>(&value)) == 0x01;
}

uint16_t byte_swap_u16(uint16_t value)
{
  return static_cast<uint16_t>((value >> 8U) | (value << 8U));
}

uint32_t byte_swap_u32(uint32_t value)
{
  return ((value & 0x000000FFU) << 24U) |
         ((value & 0x0000FF00U) << 8U) |
         ((value & 0x00FF0000U) >> 8U) |
         ((value & 0xFF000000U) >> 24U);
}

double clamp_ratio(double value)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }

  return std::clamp(value, 0.0, 1.0);
}

}  // namespace

namespace savo_realsense
{

DepthFrontMinNode::DepthFrontMinNode(const rclcpp::NodeOptions & options)
: Node("depth_front_min_node", options)
{
  input_topic_ = declare_parameter<std::string>(
    "input_topic", "/camera/camera/depth/image_rect_raw");
  output_topic_ = declare_parameter<std::string>(
    "output_topic", "/depth/min_front_m");

  depth_scale_ = declare_parameter<double>("depth_scale", 0.001);
  min_valid_m_ = declare_parameter<double>("min_valid_m", 0.02);
  max_valid_m_ = declare_parameter<double>("max_valid_m", 3.0);
  percentile_ = declare_parameter<double>("percentile", 10.0);

  x_min_ratio_ = declare_parameter<double>("x_min_ratio", 0.35);
  x_max_ratio_ = declare_parameter<double>("x_max_ratio", 0.65);
  y_min_ratio_ = declare_parameter<double>("y_min_ratio", 0.35);
  y_max_ratio_ = declare_parameter<double>("y_max_ratio", 0.75);

  sanitize_parameters();

  publisher_ = create_publisher<std_msgs::msg::Float32>(
    output_topic_, rclcpp::QoS(10).reliable().durability_volatile());

  subscription_ = create_subscription<sensor_msgs::msg::Image>(
    input_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&DepthFrontMinNode::on_depth_image, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Depth front-min C++ node subscribed to %s, publishing %s",
    input_topic_.c_str(),
    output_topic_.c_str());
}

void DepthFrontMinNode::sanitize_parameters()
{
  depth_scale_ = std::max(depth_scale_, 0.000001);
  min_valid_m_ = std::max(min_valid_m_, 0.001);
  max_valid_m_ = std::max(max_valid_m_, min_valid_m_);
  percentile_ = std::clamp(percentile_, 0.0, 100.0);

  x_min_ratio_ = clamp_ratio(x_min_ratio_);
  x_max_ratio_ = clamp_ratio(x_max_ratio_);
  y_min_ratio_ = clamp_ratio(y_min_ratio_);
  y_max_ratio_ = clamp_ratio(y_max_ratio_);

  if (x_max_ratio_ <= x_min_ratio_) {
    x_min_ratio_ = 0.35;
    x_max_ratio_ = 0.65;
  }

  if (y_max_ratio_ <= y_min_ratio_) {
    y_min_ratio_ = 0.35;
    y_max_ratio_ = 0.75;
  }
}

void DepthFrontMinNode::on_depth_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (msg->width == 0U || msg->height == 0U || msg->data.empty()) {
    publish_depth(std::numeric_limits<float>::quiet_NaN());
    return;
  }

  const auto x0 = static_cast<uint32_t>(
    std::floor(static_cast<double>(msg->width) * x_min_ratio_));
  const auto x1 = static_cast<uint32_t>(
    std::ceil(static_cast<double>(msg->width) * x_max_ratio_));
  const auto y0 = static_cast<uint32_t>(
    std::floor(static_cast<double>(msg->height) * y_min_ratio_));
  const auto y1 = static_cast<uint32_t>(
    std::ceil(static_cast<double>(msg->height) * y_max_ratio_));

  if (x1 <= x0 || y1 <= y0 || x1 > msg->width || y1 > msg->height) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Invalid ROI for depth image: width=%u height=%u",
      msg->width, msg->height);
    publish_depth(std::numeric_limits<float>::quiet_NaN());
    return;
  }

  std::vector<float> valid_depths;
  valid_depths.reserve(static_cast<size_t>((x1 - x0) * (y1 - y0)));

  if (msg->encoding == "16UC1") {
    collect_16uc1(*msg, x0, x1, y0, y1, valid_depths);
  } else if (msg->encoding == "32FC1") {
    collect_32fc1(*msg, x0, x1, y0, y1, valid_depths);
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Unsupported depth encoding: %s",
      msg->encoding.c_str());
    publish_depth(std::numeric_limits<float>::quiet_NaN());
    return;
  }

  if (valid_depths.empty()) {
    publish_depth(std::numeric_limits<float>::quiet_NaN());
    return;
  }

  publish_depth(percentile_value(valid_depths));
}

void DepthFrontMinNode::collect_16uc1(
  const sensor_msgs::msg::Image & msg,
  uint32_t x0,
  uint32_t x1,
  uint32_t y0,
  uint32_t y1,
  std::vector<float> & out) const
{
  const bool need_swap = static_cast<bool>(msg.is_bigendian) != is_system_big_endian();

  if (msg.step < msg.width * sizeof(uint16_t)) {
    return;
  }

  for (uint32_t y = y0; y < y1; ++y) {
    const size_t row_offset = static_cast<size_t>(y) * msg.step;

    for (uint32_t x = x0; x < x1; ++x) {
      const size_t offset = row_offset + static_cast<size_t>(x) * sizeof(uint16_t);

      if (offset + sizeof(uint16_t) > msg.data.size()) {
        continue;
      }

      uint16_t raw = 0;
      std::memcpy(&raw, msg.data.data() + offset, sizeof(uint16_t));

      if (need_swap) {
        raw = byte_swap_u16(raw);
      }

      if (raw == 0U) {
        continue;
      }

      const float depth_m = static_cast<float>(static_cast<double>(raw) * depth_scale_);
      add_if_valid(depth_m, out);
    }
  }
}

void DepthFrontMinNode::collect_32fc1(
  const sensor_msgs::msg::Image & msg,
  uint32_t x0,
  uint32_t x1,
  uint32_t y0,
  uint32_t y1,
  std::vector<float> & out) const
{
  const bool need_swap = static_cast<bool>(msg.is_bigendian) != is_system_big_endian();

  if (msg.step < msg.width * sizeof(float)) {
    return;
  }

  for (uint32_t y = y0; y < y1; ++y) {
    const size_t row_offset = static_cast<size_t>(y) * msg.step;

    for (uint32_t x = x0; x < x1; ++x) {
      const size_t offset = row_offset + static_cast<size_t>(x) * sizeof(float);

      if (offset + sizeof(float) > msg.data.size()) {
        continue;
      }

      uint32_t bits = 0;
      std::memcpy(&bits, msg.data.data() + offset, sizeof(uint32_t));

      if (need_swap) {
        bits = byte_swap_u32(bits);
      }

      float depth_m = 0.0F;
      std::memcpy(&depth_m, &bits, sizeof(float));

      add_if_valid(depth_m, out);
    }
  }
}

void DepthFrontMinNode::add_if_valid(float depth_m, std::vector<float> & out) const
{
  if (!std::isfinite(depth_m)) {
    return;
  }

  if (depth_m < static_cast<float>(min_valid_m_) ||
      depth_m > static_cast<float>(max_valid_m_))
  {
    return;
  }

  out.push_back(depth_m);
}

float DepthFrontMinNode::percentile_value(std::vector<float> & values) const
{
  if (values.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  const double rank =
    (percentile_ / 100.0) * static_cast<double>(values.size() - 1U);
  const auto index = static_cast<size_t>(std::round(rank));
  const size_t safe_index = std::min(index, values.size() - 1U);

  std::nth_element(values.begin(), values.begin() + safe_index, values.end());
  return values[safe_index];
}

void DepthFrontMinNode::publish_depth(float depth_m)
{
  std_msgs::msg::Float32 out;
  out.data = depth_m;
  publisher_->publish(out);
}

}  // namespace savo_realsense
