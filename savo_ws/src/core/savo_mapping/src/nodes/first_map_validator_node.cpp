#include "savo_mapping/parameter_utils.hpp"
#include "savo_mapping/qos_profiles.hpp"
#include "savo_mapping/topic_names.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

namespace savo_mapping
{

class FirstMapValidatorNode final : public rclcpp::Node
{
public:
  FirstMapValidatorNode()
  : Node("first_map_validator_node"),
    started_at_(std::chrono::steady_clock::now())
  {
    timeout_s_ = params::require_positive_parameter(
      "validation.timeout_s",
      params::declare_or_get<double>(
        *this,
        "validation.timeout_s",
        20.0));

    expected_resolution_m_ =
      params::require_positive_parameter(
      "validation.expected_resolution_m",
      params::declare_or_get<double>(
        *this,
        "validation.expected_resolution_m",
        0.05));

    minimum_known_cells_ =
      params::require_positive_integer_parameter(
      "validation.minimum_known_cells",
      params::declare_or_get<std::int64_t>(
        *this,
        "validation.minimum_known_cells",
        10));

    map_topic_ = params::declare_or_get<std::string>(
      *this,
      "topics.map",
      std::string{topics::MAP});

    readiness_topic_ =
      params::declare_or_get<std::string>(
      *this,
      "topics.readiness",
      std::string{topics::READINESS});

    map_quality_topic_ =
      params::declare_or_get<std::string>(
      *this,
      "topics.map_quality",
      std::string{topics::MAP_QUALITY});

    map_subscription_ =
      create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      qos::map_qos(),
      std::bind(
        &FirstMapValidatorNode::on_map,
        this,
        std::placeholders::_1));

    readiness_subscription_ =
      create_subscription<std_msgs::msg::String>(
      readiness_topic_,
      qos::state_qos(),
      std::bind(
        &FirstMapValidatorNode::on_readiness,
        this,
        std::placeholders::_1));

    map_quality_subscription_ =
      create_subscription<std_msgs::msg::String>(
      map_quality_topic_,
      qos::state_qos(),
      std::bind(
        &FirstMapValidatorNode::on_map_quality,
        this,
        std::placeholders::_1));

    validation_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(
        &FirstMapValidatorNode::evaluate,
        this));

    RCLCPP_INFO(
      get_logger(),
      "waiting up to %.1fs for the first valid "
      "slam_toolbox occupancy map",
      timeout_s_);
  }

  bool success() const
  {
    return success_;
  }

private:
  void on_map(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
  {
    map_received_ = true;
    map_valid_ = false;

    if (message->header.frame_id != "map") {
      map_failure_reason_ = "map_frame_is_not_map";
      return;
    }

    if (!std::isfinite(message->info.resolution) ||
        message->info.resolution <= 0.0)
    {
      map_failure_reason_ = "invalid_map_resolution";
      return;
    }

    if (std::abs(
        static_cast<double>(message->info.resolution) -
        expected_resolution_m_) > 1.0e-5)
    {
      map_failure_reason_ =
        "unexpected_map_resolution";
      return;
    }

    if (message->info.width == 0 ||
        message->info.height == 0)
    {
      map_failure_reason_ = "empty_map_dimensions";
      return;
    }

    const std::uint64_t expected_cells =
      static_cast<std::uint64_t>(
      message->info.width) *
      static_cast<std::uint64_t>(
      message->info.height);

    if (expected_cells >
        static_cast<std::uint64_t>(
          std::numeric_limits<std::size_t>::max()))
    {
      map_failure_reason_ = "map_cell_count_overflow";
      return;
    }

    if (message->data.size() !=
        static_cast<std::size_t>(expected_cells))
    {
      map_failure_reason_ = "map_cell_count_mismatch";
      return;
    }

    std::uint64_t known_cells = 0;
    std::uint64_t free_cells = 0;
    std::uint64_t occupied_cells = 0;
    std::uint64_t unknown_cells = 0;

    for (const std::int8_t value : message->data) {
      if (value == -1) {
        ++unknown_cells;
        continue;
      }

      if (value < 0 || value > 100) {
        map_failure_reason_ =
          "invalid_occupancy_value";
        return;
      }

      ++known_cells;

      if (value <= 25) {
        ++free_cells;
      }

      if (value >= 65) {
        ++occupied_cells;
      }
    }

    if (known_cells <
        static_cast<std::uint64_t>(
          minimum_known_cells_))
    {
      map_failure_reason_ =
        "insufficient_known_map_cells";
      return;
    }

    map_width_ = message->info.width;
    map_height_ = message->info.height;
    map_resolution_m_ = message->info.resolution;

    known_cells_ = known_cells;
    free_cells_ = free_cells;
    occupied_cells_ = occupied_cells;
    unknown_cells_ = unknown_cells;

    map_failure_reason_.clear();
    map_valid_ = true;
  }

  void on_readiness(
    const std_msgs::msg::String::ConstSharedPtr message)
  {
    readiness_ready_ = message->data == "ready";
    latest_readiness_ = message->data;
  }

  void on_map_quality(
    const std_msgs::msg::String::ConstSharedPtr message)
  {
    const std::string & data = message->data;

    quality_valid_ =
      data.find("\"structurally_valid\":true") !=
        std::string::npos &&
      data.find("\"evaluated\":false") !=
        std::string::npos &&
      data.find(
        "\"navigation_handoff_ready\":false") !=
        std::string::npos;

    latest_quality_ = data;
  }

  bool slam_toolbox_publishes_map()
  {
    const auto publishers =
      get_publishers_info_by_topic(map_topic_);

    return std::any_of(
      publishers.begin(),
      publishers.end(),
      [](const auto & endpoint) {
        return endpoint.node_name() == "slam_toolbox";
      });
  }

  void evaluate()
  {
    if (finished_) {
      return;
    }

    const bool slam_publisher_present =
      slam_toolbox_publishes_map();

    if (map_valid_ &&
        readiness_ready_ &&
        quality_valid_ &&
        slam_publisher_present)
    {
      success_ = true;
      finished_ = true;

      RCLCPP_INFO(
        get_logger(),
        "FIRST OCCUPANCY MAP VALIDATION PASSED");

      RCLCPP_INFO(
        get_logger(),
        "map=%ux%u resolution=%.3fm "
        "known=%lu free=%lu occupied=%lu unknown=%lu",
        map_width_,
        map_height_,
        map_resolution_m_,
        static_cast<unsigned long>(known_cells_),
        static_cast<unsigned long>(free_cells_),
        static_cast<unsigned long>(occupied_cells_),
        static_cast<unsigned long>(unknown_cells_));

      RCLCPP_INFO(
        get_logger(),
        "publisher=slam_toolbox readiness=ready "
        "quality=structurally_valid "
        "navigation_handoff=false");

      rclcpp::shutdown();
      return;
    }

    const double elapsed_s =
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() -
      started_at_).count();

    if (elapsed_s < timeout_s_) {
      return;
    }

    success_ = false;
    finished_ = true;

    RCLCPP_ERROR(
      get_logger(),
      "FIRST OCCUPANCY MAP VALIDATION FAILED");

    RCLCPP_ERROR(
      get_logger(),
      "map_received=%s map_valid=%s "
      "readiness_ready=%s quality_valid=%s "
      "slam_map_publisher=%s",
      map_received_ ? "true" : "false",
      map_valid_ ? "true" : "false",
      readiness_ready_ ? "true" : "false",
      quality_valid_ ? "true" : "false",
      slam_publisher_present ? "true" : "false");

    RCLCPP_ERROR(
      get_logger(),
      "map_reason=%s readiness=%s",
      map_failure_reason_.empty() ?
      "none" :
      map_failure_reason_.c_str(),
      latest_readiness_.empty() ?
      "none" :
      latest_readiness_.c_str());

    rclcpp::shutdown();
  }

  double timeout_s_{20.0};
  double expected_resolution_m_{0.05};
  std::int64_t minimum_known_cells_{10};

  std::string map_topic_;
  std::string readiness_topic_;
  std::string map_quality_topic_;

  bool map_received_{false};
  bool map_valid_{false};
  bool readiness_ready_{false};
  bool quality_valid_{false};

  bool success_{false};
  bool finished_{false};

  std::string map_failure_reason_;
  std::string latest_readiness_;
  std::string latest_quality_;

  std::uint32_t map_width_{0};
  std::uint32_t map_height_{0};
  double map_resolution_m_{0.0};

  std::uint64_t known_cells_{0};
  std::uint64_t free_cells_{0};
  std::uint64_t occupied_cells_{0};
  std::uint64_t unknown_cells_{0};

  const std::chrono::steady_clock::time_point
    started_at_;

  rclcpp::Subscription<
    nav_msgs::msg::OccupancyGrid>::SharedPtr
    map_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    readiness_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    map_quality_subscription_;

  rclcpp::TimerBase::SharedPtr validation_timer_;
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node =
      std::make_shared<
      savo_mapping::FirstMapValidatorNode>();

    rclcpp::spin(node);

    const bool success = node->success();

    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }

    return success ? 0 : 1;
  } catch (const std::exception & exception) {
    std::cerr
      << "first_map_validator_node failed: "
      << exception.what()
      << '\n';

    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }

    return 1;
  }
}
