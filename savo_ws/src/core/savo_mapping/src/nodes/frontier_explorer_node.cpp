#include "savo_mapping/exploration_goal_handoff.hpp"
#include "savo_mapping/exploration_planner.hpp"
#include "savo_mapping/topic_names.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <savo_msgs/msg/frontier_exploration_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace savo_mapping
{
namespace
{

bool is_handoff_terminal(const std::string & state)
{
  return
    state == "succeeded" ||
    state == "rejected" ||
    state == "aborted" ||
    state == "canceled" ||
    state == "timed_out" ||
    state == "error";
}

bool handoff_allows_new_goal(const std::string & state)
{
  return state == "idle" ||
         is_handoff_terminal(state);
}

bool is_handoff_active(const std::string & state)
{
  return !state.empty() &&
         !handoff_allows_new_goal(state);
}

std::uint8_t exhaustion_kind_from_status(const std::string & status)
{
  using Status = savo_msgs::msg::FrontierExplorationStatus;

  if (status == "no_frontiers") {
    return Status::EXHAUSTION_NO_FRONTIERS;
  }
  if (status == "no_reachable_frontiers") {
    return Status::EXHAUSTION_NO_REACHABLE_FRONTIERS;
  }
  if (status == "no_selectable_frontier") {
    return Status::EXHAUSTION_NO_SELECTABLE_FRONTIER;
  }

  return Status::EXHAUSTION_NONE;
}

std::string json_escape(const std::string & input)
{
  std::ostringstream output;

  for (const char character : input) {
    switch (character) {
      case '\\':
        output << "\\\\";
        break;

      case '"':
        output << "\\\"";
        break;

      case '\n':
        output << "\\n";
        break;

      case '\r':
        output << "\\r";
        break;

      case '\t':
        output << "\\t";
        break;

      default:
        output << character;
        break;
    }
  }

  return output.str();
}

}  // namespace

class FrontierExplorerNode final : public rclcpp::Node
{
public:
  FrontierExplorerNode()
  : Node("frontier_explorer_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    enabled_ =
      declare_parameter<bool>(
      "enabled",
      false);

    require_handoff_state_ =
      declare_parameter<bool>(
      "require_handoff_state",
      true);

    map_topic_ =
      declare_parameter<std::string>(
      "map_topic",
      "/map");

    selected_goal_topic_ =
      declare_parameter<std::string>(
      "selected_goal_topic",
      exploration::kSelectedGoalTopic);

    handoff_state_topic_ =
      declare_parameter<std::string>(
      "handoff_state_topic",
      exploration::kGoalStateTopic);

    runtime_enabled_topic_ =
      declare_parameter<std::string>(
      "runtime_enabled_topic",
      std::string{
        topics::EXPLORATION_RUNTIME_ENABLED});

    state_topic_ =
      declare_parameter<std::string>(
      "state_topic",
      "/savo_mapping/frontier_explorer/state");

    status_topic_ =
      declare_parameter<std::string>(
      "status_topic",
      "/savo_mapping/frontier_explorer/status");

    typed_status_topic_ =
      declare_parameter<std::string>(
      "typed_status_topic",
      std::string{topics::FRONTIER_EXPLORER_STATUS});

    map_frame_ =
      declare_parameter<std::string>(
      "map_frame",
      "map");

    base_frame_ =
      declare_parameter<std::string>(
      "base_frame",
      "base_link");

    const std::int64_t planning_period_ms =
      declare_parameter<std::int64_t>(
      "planning_period_ms",
      1000);

    tf_timeout_sec_ =
      declare_parameter<double>(
      "tf_timeout_sec",
      0.20);

    goal_ack_timeout_sec_ =
      declare_parameter<double>(
      "goal_ack_timeout_sec",
      3.0);

    replan_delay_sec_ =
      declare_parameter<double>(
      "replan_delay_sec",
      2.0);

    repeat_goal_cooldown_sec_ =
      declare_parameter<double>(
      "repeat_goal_cooldown_sec",
      5.0);

    exhaustion_recheck_period_sec_ =
      declare_parameter<double>(
      "exhaustion_recheck_period_sec",
      2.0);

    map_orientation_tolerance_rad_ =
      declare_parameter<double>(
      "map_orientation_tolerance_rad",
      1.0e-6);

    exploration::ExplorationPlannerConfig config;

    const std::int64_t free_threshold =
      declare_parameter<std::int64_t>(
      "detector.free_threshold",
      0);

    const std::int64_t minimum_cluster_size =
      declare_parameter<std::int64_t>(
      "detector.minimum_cluster_size",
      3);

    config.detector.cluster_diagonally =
      declare_parameter<bool>(
      "detector.cluster_diagonally",
      true);

    config.selector.information_gain_weight =
      declare_parameter<double>(
      "selector.information_gain_weight",
      1.0);

    config.selector.distance_weight =
      declare_parameter<double>(
      "selector.distance_weight",
      1.0);

    config.selector.minimum_information_gain_m2 =
      declare_parameter<double>(
      "selector.minimum_information_gain_m2",
      0.0);

    config.selector.maximum_distance_m =
      declare_parameter<double>(
      "selector.maximum_distance_m",
      0.0);

    config.goal_id_prefix =
      declare_parameter<std::string>(
      "goal_id_prefix",
      "frontier");

    validate_parameters(
      planning_period_ms,
      free_threshold,
      minimum_cluster_size);

    config.detector.free_threshold =
      static_cast<std::int8_t>(
      free_threshold);

    config.detector.minimum_cluster_size =
      static_cast<std::size_t>(
      minimum_cluster_size);

    planner_ =
      std::make_unique<
      exploration::ExplorationPlanner>(
      std::move(config));

    const auto retained_qos =
      rclcpp::QoS(
      rclcpp::KeepLast(1))
      .reliable()
      .transient_local();

    state_publisher_ =
      create_publisher<
      std_msgs::msg::String>(
        state_topic_,
        retained_qos);

    status_publisher_ =
      create_publisher<
      std_msgs::msg::String>(
        status_topic_,
        retained_qos);

    typed_status_publisher_ =
      create_publisher<
      savo_msgs::msg::FrontierExplorationStatus>(
        typed_status_topic_,
        retained_qos);

    selected_goal_publisher_ =
      create_publisher<
      geometry_msgs::msg::PoseStamped>(
        selected_goal_topic_,
        rclcpp::QoS(1).reliable());

    map_subscription_ =
      create_subscription<
      nav_msgs::msg::OccupancyGrid>(
        map_topic_,
        retained_qos,
        std::bind(
          &FrontierExplorerNode::handle_map,
          this,
          std::placeholders::_1));

    handoff_state_subscription_ =
      create_subscription<
      std_msgs::msg::String>(
        handoff_state_topic_,
        retained_qos,
        std::bind(
          &FrontierExplorerNode::
        handle_handoff_state,
          this,
          std::placeholders::_1));

    runtime_enabled_subscription_ =
      create_subscription<
      std_msgs::msg::Bool>(
        runtime_enabled_topic_,
        retained_qos,
        std::bind(
          &FrontierExplorerNode::
        handle_runtime_enabled,
          this,
          std::placeholders::_1));

    planning_timer_ =
      create_wall_timer(
        std::chrono::milliseconds(
          planning_period_ms),
        std::bind(
          &FrontierExplorerNode::planning_tick,
          this));

    set_state(
      enabled_ ?
      "waiting_for_runtime_authority" :
      "disabled",
      enabled_ ?
      "waiting_for_runtime_authority" :
      "exploration_disabled_by_configuration");

    RCLCPP_INFO(
      get_logger(),
      "frontier explorer ready: configured_enabled=%s",
      enabled_ ? "true" : "false");

    RCLCPP_INFO(
      get_logger(),
      "selected goals publish only to %s",
      selected_goal_topic_.c_str());
  }

private:
  void validate_parameters(
    std::int64_t planning_period_ms,
    std::int64_t free_threshold,
    std::int64_t minimum_cluster_size) const
  {
    if (map_topic_.empty() ||
      selected_goal_topic_.empty() ||
      handoff_state_topic_.empty() ||
      runtime_enabled_topic_.empty() ||
      state_topic_.empty() ||
      status_topic_.empty() ||
      typed_status_topic_.empty() ||
      map_frame_.empty() ||
      base_frame_.empty())
    {
      throw std::invalid_argument(
              "topic_and_frame_parameters_must_not_be_empty");
    }

    if (planning_period_ms < 50 ||
      planning_period_ms > 10000)
    {
      throw std::invalid_argument(
              "planning_period_ms_out_of_range");
    }

    if (free_threshold < 0 ||
      free_threshold > 100)
    {
      throw std::invalid_argument(
              "free_threshold_out_of_range");
    }

    if (minimum_cluster_size <= 0) {
      throw std::invalid_argument(
              "minimum_cluster_size_must_be_positive");
    }

    if (!std::isfinite(tf_timeout_sec_) ||
      tf_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "tf_timeout_sec_must_be_positive");
    }

    if (!std::isfinite(goal_ack_timeout_sec_) ||
      goal_ack_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "goal_ack_timeout_sec_must_be_positive");
    }

    if (!std::isfinite(replan_delay_sec_) ||
      replan_delay_sec_ < 0.0)
    {
      throw std::invalid_argument(
              "replan_delay_sec_must_be_nonnegative");
    }

    if (!std::isfinite(
        repeat_goal_cooldown_sec_) ||
      repeat_goal_cooldown_sec_ < 0.0)
    {
      throw std::invalid_argument(
              "repeat_goal_cooldown_sec_must_be_nonnegative");
    }

    if (!std::isfinite(
        exhaustion_recheck_period_sec_) ||
      exhaustion_recheck_period_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "exhaustion_recheck_period_sec_must_be_positive");
    }

    if (!std::isfinite(
        map_orientation_tolerance_rad_) ||
      map_orientation_tolerance_rad_ < 0.0)
    {
      throw std::invalid_argument(
              "map_orientation_tolerance_must_be_nonnegative");
    }
  }

  void handle_map(
    const nav_msgs::msg::
    OccupancyGrid::SharedPtr message)
  {
    latest_map_ = message;
    ++map_generation_;
  }

  void handle_handoff_state(
    const std_msgs::msg::String::SharedPtr message)
  {
    handoff_state_ = message->data;
    handoff_state_received_ = true;
  }

  bool update_pending_goal(
    const rclcpp::Time & current_time)
  {
    if (!goal_pending_) {
      return false;
    }

    const double elapsed_sec =
      goal_published_at_.has_value() ?
      (current_time -
      goal_published_at_.value()).seconds() :
      0.0;

    if (is_handoff_active(handoff_state_)) {
      observed_active_handoff_ = true;

      set_state(
        "waiting_for_handoff",
        "exploration_goal_active");

      return true;
    }

    if (observed_active_handoff_ &&
      handoff_allows_new_goal(
          handoff_state_))
    {
      goal_pending_ = false;
      observed_active_handoff_ = false;
      last_goal_completed_at_ = current_time;

      set_state(
        "waiting_for_replan",
        "exploration_goal_terminal");

      return false;
    }

    if (elapsed_sec >= goal_ack_timeout_sec_) {
      set_state(
        "error",
        "handoff_ack_timeout");

      return true;
    }

    set_state(
      "waiting_for_handoff",
      "waiting_for_handoff_acknowledgement");

    return true;
  }

  frontier::OccupancyGrid convert_map(
    const nav_msgs::msg::OccupancyGrid & message) const
  {
    if (message.header.frame_id.empty()) {
      throw std::invalid_argument(
              "map_frame_empty");
    }

    if (message.header.frame_id != map_frame_) {
      throw std::invalid_argument(
              "map_frame_mismatch:" +
              message.header.frame_id);
    }

    const auto & origin =
      message.info.origin;

    const auto & orientation =
      origin.orientation;

    if (!std::isfinite(origin.position.x) ||
      !std::isfinite(origin.position.y) ||
      !std::isfinite(origin.position.z) ||
      !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w))
    {
      throw std::invalid_argument(
              "map_origin_not_finite");
    }

    const double norm =
      std::sqrt(
        orientation.x * orientation.x +
        orientation.y * orientation.y +
        orientation.z * orientation.z +
        orientation.w * orientation.w);

    if (norm < 1.0e-9) {
      throw std::invalid_argument(
              "map_origin_orientation_zero_norm");
    }

    const double qx = orientation.x / norm;
    const double qy = orientation.y / norm;
    const double qz = orientation.z / norm;
    const double qw = orientation.w / norm;

    const double roll =
      std::atan2(
      2.0 * (qw * qx + qy * qz),
      1.0 - 2.0 * (qx * qx + qy * qy));

    const double pitch_argument =
      std::clamp(
      2.0 * (qw * qy - qz * qx),
      -1.0,
      1.0);

    const double pitch =
      std::asin(pitch_argument);

    const double yaw =
      std::atan2(
      2.0 * (qw * qz + qx * qy),
      1.0 - 2.0 * (qy * qy + qz * qz));

    if (std::abs(roll) >
      map_orientation_tolerance_rad_ ||
      std::abs(pitch) >
      map_orientation_tolerance_rad_ ||
      std::abs(yaw) >
      map_orientation_tolerance_rad_)
    {
      throw std::invalid_argument(
              "rotated_map_origin_not_supported");
    }

    frontier::OccupancyGrid grid;

    grid.width =
      static_cast<std::size_t>(
      message.info.width);

    grid.height =
      static_cast<std::size_t>(
      message.info.height);

    grid.resolution_m =
      static_cast<double>(
      message.info.resolution);

    grid.origin_x_m =
      origin.position.x;

    grid.origin_y_m =
      origin.position.y;

    grid.cells.assign(
      message.data.begin(),
      message.data.end());

    return grid;
  }

  std::pair<double, double> robot_position() const
  {
    const auto transform =
      tf_buffer_.lookupTransform(
      map_frame_,
      base_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(
        tf_timeout_sec_));

    const double x_m =
      transform.transform.translation.x;

    const double y_m =
      transform.transform.translation.y;

    if (!std::isfinite(x_m) ||
      !std::isfinite(y_m))
    {
      throw std::runtime_error(
              "robot_tf_position_not_finite");
    }

    return {x_m, y_m};
  }

  void handle_runtime_enabled(
    const std_msgs::msg::Bool::ConstSharedPtr message)
  {
    runtime_enabled_received_ = true;
    runtime_enabled_ = message->data;

    if (!effective_enabled()) {
      set_state(
        "disabled",
        enabled_ ?
        "runtime_authority_disabled" :
        "exploration_disabled_by_configuration");

      return;
    }

    set_state(
      latest_map_ ?
      "waiting_for_map_update" :
      "waiting_for_map",
      latest_map_ ?
      "runtime_authority_enabled" :
      "waiting_for_first_map");
  }

  bool effective_enabled() const
  {
    return
      enabled_ &&
      runtime_enabled_received_ &&
      runtime_enabled_;
  }

  void planning_tick()
  {
    const rclcpp::Time current_time = now();

    if (!effective_enabled()) {
      set_state(
        enabled_ && !runtime_enabled_received_ ?
        "waiting_for_runtime_authority" :
        "disabled",
        enabled_ ?
        (runtime_enabled_received_ ?
        "runtime_authority_disabled" :
        "waiting_for_runtime_authority") :
        "exploration_disabled_by_configuration");

      return;
    }

    if (!latest_map_) {
      set_state(
        "waiting_for_map",
        "waiting_for_first_map");

      return;
    }

    if (require_handoff_state_ &&
      !handoff_state_received_)
    {
      set_state(
        "waiting_for_handoff",
        "waiting_for_handoff_state");

      return;
    }

    if (update_pending_goal(current_time)) {
      return;
    }

    if (!handoff_allows_new_goal(
        handoff_state_))
    {
      set_state(
        "waiting_for_handoff",
        "handoff_not_ready_for_new_goal");

      return;
    }

    if (last_goal_completed_at_.has_value()) {
      const double elapsed_sec =
        (current_time -
        last_goal_completed_at_.value()).seconds();

      if (elapsed_sec < replan_delay_sec_) {
        set_state(
          "waiting_for_replan",
          "replan_delay_active");

        return;
      }
    }

    if (last_planned_map_generation_ == map_generation_) {
      const bool exhaustion_recheck_due =
        exhaustion_kind_from_status(last_planning_status_) !=
        savo_msgs::msg::FrontierExplorationStatus::EXHAUSTION_NONE &&
        last_plan_at_.has_value() &&
        (current_time - last_plan_at_.value()).seconds() >=
        exhaustion_recheck_period_sec_;

      if (!exhaustion_recheck_due) {
        set_state(
          "waiting_for_map_update",
          "map_has_not_changed");

        return;
      }
    }

    try {
      const auto [robot_x_m, robot_y_m] =
        robot_position();

      const frontier::OccupancyGrid grid =
        convert_map(*latest_map_);

      last_planned_map_generation_ =
        map_generation_;

      set_state(
        "planning",
        "evaluating_frontiers");

      const exploration::ExplorationPlan plan =
        planner_->plan(
        grid,
        robot_x_m,
        robot_y_m);

      ++plan_sequence_;
      last_plan_at_ = current_time;
      last_planning_status_ =
        exploration::to_string(plan.status);
      last_planning_reason_ = plan.reason;

      last_detected_frontier_count_ =
        plan.detected_frontier_count;

      last_reachable_frontier_count_ =
        plan.reachable_frontier_count;

      if (!plan.goal.has_value()) {
        set_state(
          exploration::to_string(plan.status),
          plan.reason);

        return;
      }

      const exploration::ExplorationGoal & goal =
        plan.goal.value();

      if (last_goal_id_ == goal.goal_id &&
        last_goal_published_at_.has_value())
      {
        const double elapsed_sec =
          (current_time -
          last_goal_published_at_.value()).seconds();

        if (elapsed_sec <
          repeat_goal_cooldown_sec_)
        {
          set_state(
            "waiting_for_map_update",
            "repeat_goal_cooldown_active");

          return;
        }
      }

      geometry_msgs::msg::PoseStamped message;

      message.header.stamp = current_time;
      message.header.frame_id = map_frame_;

      message.pose.position.x = goal.x_m;
      message.pose.position.y = goal.y_m;
      message.pose.position.z = 0.0;

      const double half_yaw =
        goal.yaw_rad * 0.5;

      message.pose.orientation.x = 0.0;
      message.pose.orientation.y = 0.0;
      message.pose.orientation.z =
        std::sin(half_yaw);

      message.pose.orientation.w =
        std::cos(half_yaw);

      selected_goal_publisher_->publish(
        message);

      goal_pending_ = true;
      observed_active_handoff_ = false;
      goal_published_at_ = current_time;
      last_goal_published_at_ = current_time;
      last_goal_id_ = goal.goal_id;

      set_state(
        "goal_published",
        goal.goal_id);

      RCLCPP_INFO(
        get_logger(),
        "published frontier goal %s at "
        "(%.3f, %.3f), score=%.3f",
        goal.goal_id.c_str(),
        goal.x_m,
        goal.y_m,
        goal.score);
    } catch (const tf2::TransformException & error) {
      set_state(
        "waiting_for_tf",
        error.what());
    } catch (const std::exception & error) {
      set_state(
        "planning_error",
        error.what());
    }
  }

  void set_state(
    const std::string & state,
    const std::string & reason)
  {
    const bool changed =
      state != current_state_ ||
      reason != current_reason_;

    current_state_ = state;
    current_reason_ = reason;

    if (changed) {
      std_msgs::msg::String message;
      message.data = state;
      state_publisher_->publish(message);
    }

    publish_status();
  }

  void publish_status()
  {
    std::ostringstream output;

    output
      << "{\"state\":\""
      << json_escape(current_state_)
      << "\",\"reason\":\""
      << json_escape(current_reason_)
      << "\",\"configured_enabled\":"
      << (enabled_ ? "true" : "false")
      << ",\"runtime_authority_received\":"
      << (runtime_enabled_received_ ?
    "true" : "false")
      << ",\"runtime_enabled\":"
      << (runtime_enabled_ ? "true" : "false")
      << ",\"enabled\":"
      << (effective_enabled() ?
    "true" : "false")
      << ",\"map_received\":"
      << (latest_map_ ? "true" : "false")
      << ",\"map_generation\":"
      << map_generation_
      << ",\"handoff_state\":\""
      << json_escape(handoff_state_)
      << "\",\"goal_pending\":"
      << (goal_pending_ ? "true" : "false")
      << ",\"last_goal_id\":\""
      << json_escape(last_goal_id_)
      << "\",\"detected_frontiers\":"
      << last_detected_frontier_count_
      << ",\"reachable_frontiers\":"
      << last_reachable_frontier_count_
      << "}";

    std_msgs::msg::String message;
    message.data = output.str();

    status_publisher_->publish(message);

    savo_msgs::msg::FrontierExplorationStatus typed;
    typed.contract_version =
      savo_msgs::msg::FrontierExplorationStatus::CONTRACT_VERSION;
    typed.stamp = now();
    typed.state = current_state_;
    typed.reason = current_reason_;
    typed.configured_enabled = enabled_;
    typed.runtime_authority_received = runtime_enabled_received_;
    typed.runtime_enabled = runtime_enabled_;
    typed.enabled = effective_enabled();
    typed.map_received = static_cast<bool>(latest_map_);
    typed.map_generation = map_generation_;
    typed.planned_map_generation =
      last_planned_map_generation_ ==
      std::numeric_limits<std::uint64_t>::max() ?
      0U : last_planned_map_generation_;
    typed.plan_sequence = plan_sequence_;
    typed.planning_status = last_planning_status_;
    typed.planning_reason = last_planning_reason_;
    typed.exhaustion_kind =
      exhaustion_kind_from_status(last_planning_status_);
    typed.handoff_state = handoff_state_;
    typed.goal_pending = goal_pending_;
    typed.last_goal_id = last_goal_id_;
    typed.detected_frontiers =
      static_cast<std::uint32_t>(last_detected_frontier_count_);
    typed.reachable_frontiers =
      static_cast<std::uint32_t>(last_reachable_frontier_count_);

    typed_status_publisher_->publish(typed);
  }

  bool enabled_{false};
  bool require_handoff_state_{true};
  bool runtime_enabled_received_{false};
  bool runtime_enabled_{false};

  std::string map_topic_;
  std::string selected_goal_topic_;
  std::string handoff_state_topic_;
  std::string runtime_enabled_topic_;
  std::string state_topic_;
  std::string status_topic_;
  std::string typed_status_topic_;
  std::string map_frame_;
  std::string base_frame_;

  double tf_timeout_sec_{0.20};
  double goal_ack_timeout_sec_{3.0};
  double replan_delay_sec_{2.0};
  double repeat_goal_cooldown_sec_{5.0};
  double exhaustion_recheck_period_sec_{2.0};
  double map_orientation_tolerance_rad_{1.0e-6};

  std::unique_ptr<
    exploration::ExplorationPlanner> planner_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  nav_msgs::msg::OccupancyGrid::SharedPtr
    latest_map_;

  std::string handoff_state_;
  bool handoff_state_received_{false};

  std::uint64_t map_generation_{0};

  std::uint64_t last_planned_map_generation_{
    std::numeric_limits<std::uint64_t>::max()};

  bool goal_pending_{false};
  bool observed_active_handoff_{false};

  std::optional<rclcpp::Time>
  goal_published_at_;

  std::optional<rclcpp::Time>
  last_goal_published_at_;

  std::optional<rclcpp::Time>
  last_goal_completed_at_;

  std::optional<rclcpp::Time>
  last_plan_at_;

  std::string last_goal_id_;
  std::string current_state_;
  std::string current_reason_;

  std::size_t last_detected_frontier_count_{0};
  std::size_t last_reachable_frontier_count_{0};
  std::uint64_t plan_sequence_{0};
  std::string last_planning_status_{"unavailable"};
  std::string last_planning_reason_{"frontier_plan_not_available"};

  rclcpp::Publisher<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    selected_goal_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    state_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    status_publisher_;

  rclcpp::Publisher<
    savo_msgs::msg::FrontierExplorationStatus>::SharedPtr
    typed_status_publisher_;

  rclcpp::Subscription<
    nav_msgs::msg::OccupancyGrid>::SharedPtr
    map_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    handoff_state_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr
    runtime_enabled_subscription_;

  rclcpp::TimerBase::SharedPtr planning_timer_;
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<
        savo_mapping::FrontierExplorerNode>());
  } catch (const std::exception & error) {
    std::cerr
      << "frontier_explorer_node failed: "
      << error.what()
      << '\n';

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
