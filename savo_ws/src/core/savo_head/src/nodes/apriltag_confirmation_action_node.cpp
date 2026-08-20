// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "savo_msgs/action/confirm_april_tag.hpp"
#include "savo_msgs/msg/april_tag_observation.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "savo_head/core/apriltag_action_contract.hpp"

namespace savo_head
{
namespace
{

using ConfirmAprilTag = savo_msgs::action::ConfirmAprilTag;
using GoalHandle = rclcpp_action::ServerGoalHandle<ConfirmAprilTag>;
using Observation = savo_msgs::msg::AprilTagObservation;

constexpr double kNanosecondsPerSecond = 1.0e9;

[[nodiscard]] double duration_seconds(
  const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) / kNanosecondsPerSecond;
}

[[nodiscard]] bool finite_pose(
  const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) &&
         std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

[[nodiscard]] double angle_difference(
  const double lhs,
  const double rhs)
{
  return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

struct SpatialSample
{
  geometry_msgs::msg::PoseStamped pose{};
};

struct StabilitySummary
{
  geometry_msgs::msg::PoseStamped mean_pose{};
  float position_stddev_m{0.0F};
  float yaw_stddev_rad{0.0F};
};

[[nodiscard]] StabilitySummary summarize(
  const std::vector<SpatialSample> & samples,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  StabilitySummary summary;
  summary.mean_pose.header.frame_id = frame_id;
  summary.mean_pose.header.stamp = stamp;

  if (samples.empty()) {
    return summary;
  }

  double mean_x = 0.0;
  double mean_y = 0.0;
  double mean_z = 0.0;
  double mean_sin = 0.0;
  double mean_cos = 0.0;

  for (const auto & sample : samples) {
    mean_x += sample.pose.pose.position.x;
    mean_y += sample.pose.pose.position.y;
    mean_z += sample.pose.pose.position.z;
    const double yaw = tf2::getYaw(sample.pose.pose.orientation);
    mean_sin += std::sin(yaw);
    mean_cos += std::cos(yaw);
  }

  const double count = static_cast<double>(samples.size());
  mean_x /= count;
  mean_y /= count;
  mean_z /= count;
  const double mean_yaw = std::atan2(mean_sin / count, mean_cos / count);

  double position_variance = 0.0;
  double yaw_variance = 0.0;

  for (const auto & sample : samples) {
    const double dx = sample.pose.pose.position.x - mean_x;
    const double dy = sample.pose.pose.position.y - mean_y;
    position_variance += dx * dx + dy * dy;

    const double yaw = tf2::getYaw(sample.pose.pose.orientation);
    const double dyaw = angle_difference(yaw, mean_yaw);
    yaw_variance += dyaw * dyaw;
  }

  summary.position_stddev_m =
    static_cast<float>(std::sqrt(position_variance / count));
  summary.yaw_stddev_rad =
    static_cast<float>(std::sqrt(yaw_variance / count));

  summary.mean_pose.pose.position.x = mean_x;
  summary.mean_pose.pose.position.y = mean_y;
  summary.mean_pose.pose.position.z = mean_z;

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, mean_yaw);
  summary.mean_pose.pose.orientation = tf2::toMsg(orientation);
  return summary;
}

}  // namespace

class AprilTagConfirmationActionNode final : public rclcpp::Node
{
public:
  AprilTagConfirmationActionNode()
  : Node("apriltag_confirmation_action_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    observation_topic_ = declare_parameter<std::string>(
      "observation_topic",
      std::string(apriltag_contract::kTypedObservationTopic));
    action_name_ = declare_parameter<std::string>(
      "action_name",
      std::string(apriltag_contract::kConfirmAction));
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    minimum_observations_ = declare_parameter<int>("minimum_observations", 5);
    minimum_detection_quality_ = declare_parameter<double>(
      "minimum_detection_quality", 0.70);
    maximum_observation_age_s_ = declare_parameter<double>(
      "maximum_observation_age_s", 0.50);
    maximum_position_stddev_m_ = declare_parameter<double>(
      "maximum_position_stddev_m", 0.08);
    maximum_yaw_stddev_rad_ = declare_parameter<double>(
      "maximum_yaw_stddev_rad", 0.15);
    maximum_hamming_distance_ = declare_parameter<int>(
      "maximum_hamming_distance", 1);
    observation_buffer_size_ = declare_parameter<int>(
      "observation_buffer_size", 100);
    default_timeout_s_ = declare_parameter<double>("default_timeout_s", 12.0);
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.25);
    wrong_tag_grace_s_ = declare_parameter<double>("wrong_tag_grace_s", 1.0);

    require_stationary_signal_ = declare_parameter<bool>(
      "require_stationary_signal", false);
    motion_topic_ = declare_parameter<std::string>("motion_topic", "/cmd_vel");
    motion_state_timeout_s_ = declare_parameter<double>(
      "motion_state_timeout_s", 0.50);
    maximum_linear_speed_mps_ = declare_parameter<double>(
      "maximum_linear_speed_mps", 0.03);
    maximum_angular_speed_radps_ = declare_parameter<double>(
      "maximum_angular_speed_radps", 0.05);

    require_localization_signal_ = declare_parameter<bool>(
      "require_localization_signal", false);
    localization_ready_topic_ = declare_parameter<std::string>(
      "localization_ready_topic", "/savo_localization/ready");
    localization_state_timeout_s_ = declare_parameter<double>(
      "localization_state_timeout_s", 1.0);

    if (observation_topic_.empty() || action_name_.empty() || map_frame_.empty() ||
      minimum_observations_ <= 0 || observation_buffer_size_ <= 0 ||
      default_timeout_s_ <= 0.0 || maximum_observation_age_s_ <= 0.0 ||
      tf_timeout_s_ <= 0.0)
    {
      throw std::runtime_error("invalid AprilTag confirmation parameters");
    }

    observation_subscription_ = create_subscription<Observation>(
      observation_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &AprilTagConfirmationActionNode::on_observation,
        this,
        std::placeholders::_1));

    motion_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      motion_topic_,
      rclcpp::QoS(10).best_effort(),
      std::bind(
        &AprilTagConfirmationActionNode::on_motion,
        this,
        std::placeholders::_1));

    localization_subscription_ = create_subscription<std_msgs::msg::Bool>(
      localization_ready_topic_,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &AprilTagConfirmationActionNode::on_localization,
        this,
        std::placeholders::_1));

    server_ = rclcpp_action::create_server<ConfirmAprilTag>(
      this,
      action_name_,
      std::bind(
        &AprilTagConfirmationActionNode::handle_goal,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      std::bind(
        &AprilTagConfirmationActionNode::handle_cancel,
        this,
        std::placeholders::_1),
      std::bind(
        &AprilTagConfirmationActionNode::handle_accepted,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "typed AprilTag confirmation action ready action=%s observations=%s",
      action_name_.c_str(),
      observation_topic_.c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const ConfirmAprilTag::Goal> goal)
  {
    if (!apriltag_contract::IsValidDuty(goal->mode) ||
      goal->expected_family.empty())
    {
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->mode == ConfirmAprilTag::Goal::CONFIRM_ARRIVAL &&
      goal->expected_tag_id < 0)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->expected_tag_id < ConfirmAprilTag::Goal::ANY_TAG_ID) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->require_map_pose &&
      (goal->map_id.empty() || goal->map_revision == 0U))
    {
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle>)
  {
    observation_condition_.notify_all();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{
      [this, goal_handle]()
      {
        try {
          execute(goal_handle);
        } catch (const std::exception & exception) {
          finish(
            goal_handle,
            false,
            ConfirmAprilTag::Result::RESULT_INTERNAL_ERROR,
            std::string("apriltag_confirmation_exception: ") + exception.what(),
            {},
            0U,
            std::nullopt,
            false);
        }
      }}.detach();
  }

  void on_observation(const Observation::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);

    if (!observations_.empty() &&
      message->observation_sequence <=
      observations_.back().observation_sequence)
    {
      return;
    }

    observations_.push_back(*message);
    while (observations_.size() >
      static_cast<std::size_t>(observation_buffer_size_))
    {
      observations_.pop_front();
    }
    observation_condition_.notify_all();
  }

  void on_motion(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_motion_ = *message;
    latest_motion_stamp_ = now();
  }

  void on_localization(const std_msgs::msg::Bool::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    localization_ready_ = message->data;
    localization_stamp_ = now();
  }

  [[nodiscard]] std::optional<std::pair<std::uint8_t, std::string>>
  readiness_failure() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto current = now();

    if (require_stationary_signal_) {
      if (!latest_motion_.has_value() ||
        (current - latest_motion_stamp_).seconds() > motion_state_timeout_s_)
      {
        return std::make_pair(
          ConfirmAprilTag::Result::RESULT_ROBOT_MOVING,
          "stationary_motion_state_unavailable");
      }

      if (std::abs(latest_motion_->linear.x) > maximum_linear_speed_mps_ ||
        std::abs(latest_motion_->linear.y) > maximum_linear_speed_mps_ ||
        std::abs(latest_motion_->angular.z) > maximum_angular_speed_radps_)
      {
        return std::make_pair(
          ConfirmAprilTag::Result::RESULT_ROBOT_MOVING,
          "robot_not_stationary");
      }
    }

    if (require_localization_signal_) {
      if (!localization_ready_.has_value() ||
        (current - localization_stamp_).seconds() > localization_state_timeout_s_ ||
        !localization_ready_.value())
      {
        return std::make_pair(
          ConfirmAprilTag::Result::RESULT_LOCALIZATION_UNHEALTHY,
          "localization_not_ready");
      }
    }

    return std::nullopt;
  }

  [[nodiscard]] std::uint64_t latest_sequence() const
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    return observations_.empty() ? 0U : observations_.back().observation_sequence;
  }

  [[nodiscard]] std::vector<Observation> observations_after(
    const std::uint64_t sequence) const
  {
    std::vector<Observation> output;
    std::lock_guard<std::mutex> lock(observation_mutex_);
    for (const auto & observation : observations_) {
      if (observation.observation_sequence > sequence) {
        output.push_back(observation);
      }
    }
    return output;
  }

  [[nodiscard]] std::optional<geometry_msgs::msg::PoseStamped> map_pose(
    const Observation & observation,
    const bool require_map_pose)
  {
    if (!observation.pose_valid || !finite_pose(observation.pose.pose)) {
      return std::nullopt;
    }

    geometry_msgs::msg::PoseStamped source;
    source.header = observation.header;
    source.pose = observation.pose.pose;

    if (source.header.frame_id == map_frame_) {
      return source;
    }

    try {
      return tf_buffer_.transform(
        source,
        map_frame_,
        tf2::durationFromSec(tf_timeout_s_));
    } catch (const tf2::TransformException & exception) {
      if (require_map_pose) {
        RCLCPP_DEBUG(
          get_logger(),
          "AprilTag map transform unavailable: %s",
          exception.what());
      }
      return std::nullopt;
    }
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::uint8_t state,
    const std::string & text,
    const std::uint32_t accepted,
    const std::uint32_t rejected,
    const std::optional<Observation> & latest)
  {
    auto feedback = std::make_shared<ConfirmAprilTag::Feedback>();
    feedback->state = state;
    feedback->state_text = text;
    feedback->accepted_observations = accepted;
    feedback->rejected_observations = rejected;
    if (latest.has_value()) {
      feedback->current_tag_id = latest->tag_id;
      feedback->current_detection_quality = latest->detection_quality;
      feedback->latest_observation = latest.value();
    } else {
      feedback->current_tag_id = ConfirmAprilTag::Goal::ANY_TAG_ID;
    }
    goal_handle->publish_feedback(feedback);
  }

  void finish(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const bool confirmed,
    const std::uint8_t code,
    const std::string & reason,
    const std::vector<Observation> & accepted,
    const std::uint32_t rejected,
    const std::optional<StabilitySummary> & stability,
    const bool canceled)
  {
    auto result = std::make_shared<ConfirmAprilTag::Result>();
    result->confirmed = confirmed;
    result->result_code = code;
    result->reason = reason;
    result->accepted_observations =
      static_cast<std::uint32_t>(accepted.size());
    result->rejected_observations = rejected;

    if (!accepted.empty()) {
      result->final_observation = accepted.back();
    }

    if (stability.has_value()) {
      result->map_pose_valid =
        stability->mean_pose.header.frame_id == map_frame_;
      if (result->map_pose_valid) {
        result->tag_pose_map = stability->mean_pose;
      }
      result->position_stddev_m = stability->position_stddev_m;
      result->yaw_stddev_rad = stability->yaw_stddev_rad;
    }

    if (canceled) {
      goal_handle->canceled(result);
    } else if (confirmed) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const double requested_timeout = duration_seconds(goal->timeout);
    const double timeout_s = requested_timeout > 0.0 ?
      requested_timeout : default_timeout_s_;
    const auto start = now();
    const auto deadline = start + rclcpp::Duration::from_seconds(timeout_s);
    const bool spatial_evidence_required =
      apriltag_contract::RequiresSpatialEvidence(
        goal->mode, goal->require_map_pose);
    std::uint64_t consumed_sequence = latest_sequence();
    std::optional<std::int32_t> locked_tag_id;
    std::vector<Observation> accepted;
    std::vector<SpatialSample> spatial_samples;
    std::uint32_t rejected = 0U;
    bool saw_wrong_tag = false;
    bool saw_matching_unstable = false;

    publish_feedback(
      goal_handle,
      ConfirmAprilTag::Feedback::STATE_SEARCHING,
      "waiting_for_fresh_apriltag_observations",
      0U,
      0U,
      std::nullopt);

    while (rclcpp::ok() && now() < deadline) {
      if (goal_handle->is_canceling()) {
        finish(
          goal_handle,
          false,
          ConfirmAprilTag::Result::RESULT_CANCELED,
          "apriltag_confirmation_canceled",
          accepted,
          rejected,
          std::nullopt,
          true);
        return;
      }

      const auto readiness = readiness_failure();
      if (readiness.has_value()) {
        finish(
          goal_handle,
          false,
          readiness->first,
          readiness->second,
          accepted,
          rejected,
          std::nullopt,
          false);
        return;
      }

      const auto batch = observations_after(consumed_sequence);
      for (const auto & observation : batch) {
        consumed_sequence = std::max(
          consumed_sequence,
          observation.observation_sequence);

        const bool family_matches = observation.family == goal->expected_family;
        const std::int32_t expected_id =
          goal->expected_tag_id == ConfirmAprilTag::Goal::ANY_TAG_ID ?
          (locked_tag_id.has_value() ? locked_tag_id.value() : observation.tag_id) :
          goal->expected_tag_id;

        if (family_matches && !locked_tag_id.has_value() &&
          goal->expected_tag_id == ConfirmAprilTag::Goal::ANY_TAG_ID)
        {
          locked_tag_id = observation.tag_id;
        }

        const rclcpp::Time observation_stamp(observation.header.stamp);
        const double age_s = observation_stamp.nanoseconds() == 0 ?
          std::numeric_limits<double>::infinity() :
          (now() - observation_stamp).seconds();

        const auto identity_disposition =
          apriltag_contract::ClassifyIdentityEvidence(
            family_matches,
            observation.tag_id == expected_id,
            age_s >= 0.0 && age_s <= maximum_observation_age_s_,
            observation.detection_quality >= minimum_detection_quality_,
            observation.hamming_distance <= maximum_hamming_distance_);

        if (identity_disposition ==
          apriltag_contract::IdentityEvidenceDisposition::kWrongTag)
        {
          saw_wrong_tag = true;
          ++rejected;
          continue;
        }
        if (identity_disposition ==
          apriltag_contract::IdentityEvidenceDisposition::kUnstable)
        {
          saw_matching_unstable = true;
          ++rejected;
          continue;
        }

        if (spatial_evidence_required) {
          if (!observation.pose_valid) {
            saw_matching_unstable = true;
            ++rejected;
            continue;
          }

          const auto transformed = map_pose(observation, true);
          if (!transformed.has_value()) {
            saw_matching_unstable = true;
            ++rejected;
            continue;
          }

          SpatialSample sample;
          sample.pose = transformed.value();
          spatial_samples.push_back(std::move(sample));
        }

        accepted.push_back(observation);

        if (accepted.size() > static_cast<std::size_t>(minimum_observations_ * 3)) {
          accepted.erase(accepted.begin());
        }
        if (spatial_samples.size() >
          static_cast<std::size_t>(minimum_observations_ * 3))
        {
          spatial_samples.erase(spatial_samples.begin());
        }

        publish_feedback(
          goal_handle,
          ConfirmAprilTag::Feedback::STATE_COLLECTING,
          "collecting_stable_apriltag_observations",
          static_cast<std::uint32_t>(accepted.size()),
          rejected,
          observation);

        if (apriltag_contract::HasMinimumEvidence(
            accepted.size(), static_cast<std::size_t>(minimum_observations_)))
        {
          if (apriltag_contract::IsIdentityOnlyArrival(
              goal->mode, goal->require_map_pose))
          {
            publish_feedback(
              goal_handle,
              ConfirmAprilTag::Feedback::STATE_CONFIRMING,
              "apriltag_identity_confirmed",
              static_cast<std::uint32_t>(accepted.size()),
              rejected,
              observation);
            finish(
              goal_handle,
              true,
              ConfirmAprilTag::Result::RESULT_CONFIRMED,
              "apriltag_identity_confirmation_succeeded",
              accepted,
              rejected,
              std::nullopt,
              false);
            return;
          }

          const auto stability = summarize(
            spatial_samples,
            spatial_samples.back().pose.header.frame_id,
            now());

          if (stability.position_stddev_m <= maximum_position_stddev_m_ &&
            stability.yaw_stddev_rad <= maximum_yaw_stddev_rad_)
          {
            publish_feedback(
              goal_handle,
              ConfirmAprilTag::Feedback::STATE_CONFIRMING,
              "apriltag_confirmed",
              static_cast<std::uint32_t>(accepted.size()),
              rejected,
              observation);
            finish(
              goal_handle,
              true,
              ConfirmAprilTag::Result::RESULT_CONFIRMED,
              "stable_apriltag_confirmation_succeeded",
              accepted,
              rejected,
              stability,
              false);
            return;
          }
          saw_matching_unstable = true;
        }
      }

      if (saw_wrong_tag && accepted.empty() &&
        (now() - start).seconds() >= wrong_tag_grace_s_)
      {
        finish(
          goal_handle,
          false,
          ConfirmAprilTag::Result::RESULT_WRONG_TAG,
          "only_unexpected_apriltag_observed",
          accepted,
          rejected,
          std::nullopt,
          false);
        return;
      }

      std::unique_lock<std::mutex> lock(observation_mutex_);
      observation_condition_.wait_for(lock, std::chrono::milliseconds(50));
    }

    const std::uint8_t code = saw_matching_unstable ?
      ConfirmAprilTag::Result::RESULT_UNSTABLE :
      ConfirmAprilTag::Result::RESULT_TIMED_OUT;
    const std::string reason = saw_matching_unstable ?
      "matching_apriltag_never_became_stable" :
      "apriltag_confirmation_timed_out";
    finish(
      goal_handle,
      false,
      code,
      reason,
      accepted,
      rejected,
      std::nullopt,
      false);
  }

  std::string observation_topic_{};
  std::string action_name_{};
  std::string map_frame_{};
  int minimum_observations_{5};
  double minimum_detection_quality_{0.70};
  double maximum_observation_age_s_{0.50};
  double maximum_position_stddev_m_{0.08};
  double maximum_yaw_stddev_rad_{0.15};
  int maximum_hamming_distance_{1};
  int observation_buffer_size_{100};
  double default_timeout_s_{12.0};
  double tf_timeout_s_{0.25};
  double wrong_tag_grace_s_{1.0};

  bool require_stationary_signal_{false};
  std::string motion_topic_{};
  double motion_state_timeout_s_{0.50};
  double maximum_linear_speed_mps_{0.03};
  double maximum_angular_speed_radps_{0.05};
  bool require_localization_signal_{false};
  std::string localization_ready_topic_{};
  double localization_state_timeout_s_{1.0};

  mutable std::mutex observation_mutex_{};
  std::condition_variable observation_condition_{};
  std::deque<Observation> observations_{};

  mutable std::mutex state_mutex_{};
  std::optional<geometry_msgs::msg::Twist> latest_motion_{};
  rclcpp::Time latest_motion_stamp_{0, 0, RCL_ROS_TIME};
  std::optional<bool> localization_ready_{};
  rclcpp::Time localization_stamp_{0, 0, RCL_ROS_TIME};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<Observation>::SharedPtr observation_subscription_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_subscription_{};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_subscription_{};
  rclcpp_action::Server<ConfirmAprilTag>::SharedPtr server_{};
};

}  // namespace savo_head

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<savo_head::AprilTagConfirmationActionNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
