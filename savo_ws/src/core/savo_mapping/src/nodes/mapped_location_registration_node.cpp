// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "savo_msgs/action/confirm_april_tag.hpp"
#include "savo_msgs/action/register_mapped_location.hpp"
#include "savo_msgs/srv/authorize_location_operation.hpp"
#include "savo_msgs/srv/register_location_candidate.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "savo_mapping/semantic_landmark.hpp"
#include "savo_mapping/semantic_landmark_recorder.hpp"

namespace savo_mapping
{
namespace
{

using RegisterMappedLocation = savo_msgs::action::RegisterMappedLocation;
using RegisterGoalHandle =
  rclcpp_action::ServerGoalHandle<RegisterMappedLocation>;
using ConfirmAprilTag = savo_msgs::action::ConfirmAprilTag;
using ConfirmGoalHandle =
  rclcpp_action::ClientGoalHandle<ConfirmAprilTag>;
using Authorize = savo_msgs::srv::AuthorizeLocationOperation;
using RegisterCandidate = savo_msgs::srv::RegisterLocationCandidate;

constexpr double kNanosecondsPerSecond = 1.0e9;

[[nodiscard]] double duration_seconds(
  const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) / kNanosecondsPerSecond;
}

[[nodiscard]] builtin_interfaces::msg::Duration to_duration(
  const double seconds)
{
  builtin_interfaces::msg::Duration output;
  const double bounded = std::max(0.0, seconds);
  output.sec = static_cast<std::int32_t>(std::floor(bounded));
  output.nanosec = static_cast<std::uint32_t>(
    (bounded - static_cast<double>(output.sec)) * kNanosecondsPerSecond);
  return output;
}

[[nodiscard]] bool valid_map_pose(
  const geometry_msgs::msg::PoseStamped & pose)
{
  return pose.header.frame_id == "map" &&
         std::isfinite(pose.pose.position.x) &&
         std::isfinite(pose.pose.position.y) &&
         std::isfinite(pose.pose.position.z) &&
         std::isfinite(pose.pose.orientation.x) &&
         std::isfinite(pose.pose.orientation.y) &&
         std::isfinite(pose.pose.orientation.z) &&
         std::isfinite(pose.pose.orientation.w);
}

[[nodiscard]] SemanticPlanarPose planar_pose(
  const geometry_msgs::msg::PoseStamped & pose)
{
  SemanticPlanarPose output;
  output.frame_id = pose.header.frame_id;
  output.x = pose.pose.position.x;
  output.y = pose.pose.position.y;
  output.yaw = tf2::getYaw(pose.pose.orientation);
  return output;
}

[[nodiscard]] geometry_msgs::msg::PoseStamped derive_approach_pose(
  const geometry_msgs::msg::PoseStamped & tag_pose,
  const double stand_off_m,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseStamped output = tag_pose;
  output.header.stamp = stamp;
  const double yaw = tf2::getYaw(tag_pose.pose.orientation);
  output.pose.position.x -= stand_off_m * std::cos(yaw);
  output.pose.position.y -= stand_off_m * std::sin(yaw);
  return output;
}

}  // namespace

class MappedLocationRegistrationNode final : public rclcpp::Node
{
public:
  MappedLocationRegistrationNode()
  : Node("mapped_location_registration_node"),
    recorder_(declare_parameter<double>(
      "minimum_tag_to_approach_distance_m", 0.20))
  {
    action_name_ = declare_parameter<std::string>(
      "action_name", "/savo_mapping/locations/register");
    head_action_name_ = declare_parameter<std::string>(
      "head_confirmation_action", "/savo_head/apriltag/confirm");
    authorization_service_name_ = declare_parameter<std::string>(
      "authorization_service", "/savo_supervisor/authorize_location_operation");
    registration_service_name_ = declare_parameter<std::string>(
      "registration_service", "/savo_locations/candidates/register");
    default_timeout_s_ = declare_parameter<double>("default_timeout_s", 25.0);
    dependency_wait_timeout_s_ = declare_parameter<double>(
      "dependency_wait_timeout_s", 2.0);
    default_approach_distance_m_ = declare_parameter<double>(
      "default_approach_distance_m", 0.80);
    derive_missing_approach_pose_ = declare_parameter<bool>(
      "derive_missing_approach_pose", true);

    if (action_name_.empty() || head_action_name_.empty() ||
      authorization_service_name_.empty() || registration_service_name_.empty() ||
      default_timeout_s_ <= 0.0 || dependency_wait_timeout_s_ <= 0.0 ||
      default_approach_distance_m_ <= recorder_.minimum_tag_to_approach_distance_m())
    {
      throw std::runtime_error("invalid mapped-location registration parameters");
    }

    authorization_client_ = create_client<Authorize>(authorization_service_name_);
    registration_client_ = create_client<RegisterCandidate>(registration_service_name_);
    head_client_ = rclcpp_action::create_client<ConfirmAprilTag>(this, head_action_name_);

    server_ = rclcpp_action::create_server<RegisterMappedLocation>(
      this,
      action_name_,
      std::bind(
        &MappedLocationRegistrationNode::handle_goal,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      std::bind(
        &MappedLocationRegistrationNode::handle_cancel,
        this,
        std::placeholders::_1),
      std::bind(
        &MappedLocationRegistrationNode::handle_accepted,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "mapped-location registration action ready action=%s",
      action_name_.c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const RegisterMappedLocation::Goal> goal)
  {
    if (busy_.exchange(true)) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    const bool invalid =
      goal->request_id.empty() || goal->actor_id.empty() ||
      goal->candidate_id.empty() || goal->expected_family.empty() ||
      goal->expected_tag_id < RegisterMappedLocation::Goal::ANY_TAG_ID ||
      goal->map_id.empty() || goal->map_revision == 0U ||
      goal->source_session_id.empty() ||
      (goal->approach_pose_valid && !valid_map_pose(goal->approach_pose)) ||
      (goal->confirmation_pose_valid && !valid_map_pose(goal->confirmation_pose)) ||
      (!goal->approach_pose_valid && !derive_missing_approach_pose_);

    if (invalid) {
      busy_.store(false);
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<RegisterGoalHandle>)
  {
    return persistence_in_progress_.load() ?
           rclcpp_action::CancelResponse::REJECT :
           rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<RegisterGoalHandle> goal_handle)
  {
    std::thread{
      [this, goal_handle]()
      {
        try {
          execute(goal_handle);
        } catch (const std::exception & exception) {
          persistence_in_progress_.store(false);
          savo_msgs::msg::LocationCandidate empty_candidate{
            rosidl_runtime_cpp::MessageInitialization::ZERO};
          finish(
            goal_handle,
            false,
            RegisterMappedLocation::Result::RESULT_INTERNAL_ERROR,
            std::string("mapped_location_registration_exception: ") +
            exception.what(),
            empty_candidate);
        }
        busy_.store(false);
      }}.detach();
  }

  void feedback(
    const std::shared_ptr<RegisterGoalHandle> & goal_handle,
    const std::uint8_t state,
    const std::string & text,
    const std::uint32_t accepted = 0U,
    const float quality = 0.0F,
    const std::int32_t tag_id = RegisterMappedLocation::Goal::ANY_TAG_ID)
  {
    auto message = std::make_shared<RegisterMappedLocation::Feedback>();
    message->state = state;
    message->state_text = text;
    message->accepted_observations = accepted;
    message->detection_quality = quality;
    message->current_tag_id = tag_id;
    goal_handle->publish_feedback(message);
  }

  void finish(
    const std::shared_ptr<RegisterGoalHandle> & goal_handle,
    const bool registered,
    const std::uint8_t code,
    const std::string & reason,
    const savo_msgs::msg::LocationCandidate & candidate,
    const bool canceled = false)
  {
    busy_.store(false);
    auto result = std::make_shared<RegisterMappedLocation::Result>();
    result->registered = registered;
    result->result_code = code;
    result->reason = reason;
    result->candidate = candidate;

    if (canceled) {
      goal_handle->canceled(result);
    } else if (registered) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  [[nodiscard]] bool canceled_or_expired(
    const std::shared_ptr<RegisterGoalHandle> & goal_handle,
    const rclcpp::Time & deadline) const
  {
    return goal_handle->is_canceling() || !rclcpp::ok() || now() >= deadline;
  }

  template<typename FutureT>
  [[nodiscard]] bool wait_future(
    FutureT & future,
    const std::shared_ptr<RegisterGoalHandle> & goal_handle,
    const rclcpp::Time & deadline) const
  {
    while (!canceled_or_expired(goal_handle, deadline)) {
      if (future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
        return true;
      }
    }
    return false;
  }

  void execute(const std::shared_ptr<RegisterGoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const double requested_timeout = duration_seconds(goal->timeout);
    const double timeout_s = requested_timeout > 0.0 ?
      requested_timeout : default_timeout_s_;
    const auto deadline = now() + rclcpp::Duration::from_seconds(timeout_s);
    savo_msgs::msg::LocationCandidate empty_candidate;

    feedback(
      goal_handle,
      RegisterMappedLocation::Feedback::STATE_VALIDATING,
      "validating_mapping_request");

    if (!authorization_client_->wait_for_service(
        std::chrono::duration<double>(dependency_wait_timeout_s_)))
    {
      finish(
        goal_handle,
        false,
        RegisterMappedLocation::Result::RESULT_SUPERVISOR_DENIED,
        "supervisor_authorization_service_unavailable",
        empty_candidate);
      return;
    }

    feedback(
      goal_handle,
      RegisterMappedLocation::Feedback::STATE_REQUESTING_AUTHORIZATION,
      "requesting_registration_authorization");

    auto authorization_request = std::make_shared<Authorize::Request>();
    authorization_request->operation =
      Authorize::Request::OP_REGISTER_LOCATION_CANDIDATE;
    authorization_request->request_id = goal->request_id;
    authorization_request->actor_id = goal->actor_id;
    authorization_request->candidate_id = goal->candidate_id;
    authorization_request->map_id = goal->map_id;
    authorization_request->map_revision = goal->map_revision;
    authorization_request->motion_required = false;

    auto authorization_future =
      authorization_client_->async_send_request(authorization_request);
    if (!wait_future(authorization_future, goal_handle, deadline)) {
      const bool canceled = goal_handle->is_canceling();
      finish(
        goal_handle,
        false,
        canceled ? RegisterMappedLocation::Result::RESULT_CANCELED :
        RegisterMappedLocation::Result::RESULT_TIMED_OUT,
        canceled ? "mapping_registration_canceled" :
        "registration_authorization_timed_out",
        empty_candidate,
        canceled);
      return;
    }

    const auto authorization = authorization_future.get();
    if (!authorization->authorized) {
      finish(
        goal_handle,
        false,
        RegisterMappedLocation::Result::RESULT_SUPERVISOR_DENIED,
        authorization->reason,
        empty_candidate);
      return;
    }

    if (!head_client_->wait_for_action_server(
        std::chrono::duration<double>(dependency_wait_timeout_s_)))
    {
      finish(
        goal_handle,
        false,
        RegisterMappedLocation::Result::RESULT_HEAD_UNAVAILABLE,
        "head_confirmation_action_unavailable",
        empty_candidate);
      return;
    }

    feedback(
      goal_handle,
      RegisterMappedLocation::Feedback::STATE_CONFIRMING_TAG,
      "confirming_mapping_apriltag");

    ConfirmAprilTag::Goal head_goal;
    head_goal.mode = ConfirmAprilTag::Goal::REGISTER_LOCATION;
    head_goal.expected_family = goal->expected_family;
    head_goal.expected_tag_id = goal->expected_tag_id;
    head_goal.map_id = goal->map_id;
    head_goal.map_revision = goal->map_revision;
    head_goal.timeout = to_duration(
      std::max(0.1, (deadline - now()).seconds()));
    head_goal.require_map_pose = true;

    rclcpp_action::Client<ConfirmAprilTag>::SendGoalOptions head_options;
    head_options.feedback_callback =
      [this, goal_handle](
      ConfirmGoalHandle::SharedPtr,
      const std::shared_ptr<const ConfirmAprilTag::Feedback> head_feedback)
      {
        feedback(
          goal_handle,
          RegisterMappedLocation::Feedback::STATE_CONFIRMING_TAG,
          head_feedback->state_text,
          head_feedback->accepted_observations,
          head_feedback->current_detection_quality,
          head_feedback->current_tag_id);
      };

    auto head_goal_future = head_client_->async_send_goal(head_goal, head_options);
    if (!wait_future(head_goal_future, goal_handle, deadline)) {
      const bool canceled = goal_handle->is_canceling();
      finish(
        goal_handle,
        false,
        canceled ? RegisterMappedLocation::Result::RESULT_CANCELED :
        RegisterMappedLocation::Result::RESULT_TIMED_OUT,
        canceled ? "mapping_registration_canceled" :
        "head_goal_response_timed_out",
        empty_candidate,
        canceled);
      return;
    }

    const auto head_goal_handle = head_goal_future.get();
    if (!head_goal_handle) {
      finish(
        goal_handle,
        false,
        RegisterMappedLocation::Result::RESULT_CONFIRMATION_FAILED,
        "head_confirmation_goal_rejected",
        empty_candidate);
      return;
    }

    auto head_result_future = head_client_->async_get_result(head_goal_handle);
    if (!wait_future(head_result_future, goal_handle, deadline)) {
      head_client_->async_cancel_goal(head_goal_handle);
      const bool canceled = goal_handle->is_canceling();
      finish(
        goal_handle,
        false,
        canceled ? RegisterMappedLocation::Result::RESULT_CANCELED :
        RegisterMappedLocation::Result::RESULT_TIMED_OUT,
        canceled ? "mapping_registration_canceled" :
        "head_confirmation_timed_out",
        empty_candidate,
        canceled);
      return;
    }

    const auto wrapped_head_result = head_result_future.get();
    if (wrapped_head_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !wrapped_head_result.result ||
      !wrapped_head_result.result->confirmed ||
      !wrapped_head_result.result->map_pose_valid)
    {
      const std::string reason = wrapped_head_result.result ?
        wrapped_head_result.result->reason :
        "head_confirmation_failed_without_result";
      finish(
        goal_handle,
        false,
        RegisterMappedLocation::Result::RESULT_CONFIRMATION_FAILED,
        reason,
        empty_candidate);
      return;
    }

    feedback(
      goal_handle,
      RegisterMappedLocation::Feedback::STATE_BUILDING_CANDIDATE,
      "building_semantic_location_candidate",
      wrapped_head_result.result->accepted_observations,
      wrapped_head_result.result->final_observation.detection_quality,
      wrapped_head_result.result->final_observation.tag_id);

    const auto & tag_pose = wrapped_head_result.result->tag_pose_map;
    geometry_msgs::msg::PoseStamped approach_pose = goal->approach_pose_valid ?
      goal->approach_pose :
      derive_approach_pose(tag_pose, default_approach_distance_m_, now());
    geometry_msgs::msg::PoseStamped confirmation_pose =
      goal->confirmation_pose_valid ? goal->confirmation_pose : approach_pose;

    SemanticLandmarkDraft draft;
    draft.candidate_id = goal->candidate_id;
    draft.map_id = goal->map_id;
    draft.map_revision = goal->map_revision;
    draft.map_release_id = goal->map_release_id;
    draft.tag_family = wrapped_head_result.result->final_observation.family;
    draft.tag_id = wrapped_head_result.result->final_observation.tag_id;
    draft.tag_pose_map = planar_pose(tag_pose);
    draft.detection_quality =
      wrapped_head_result.result->final_observation.detection_quality;
    draft.accepted_observations =
      wrapped_head_result.result->accepted_observations;
    draft.position_stddev_m = wrapped_head_result.result->position_stddev_m;
    draft.yaw_stddev_rad = wrapped_head_result.result->yaw_stddev_rad;
    draft.approach_pose_valid = true;
    draft.approach_pose = planar_pose(approach_pose);
    draft.confirmation_pose_valid = true;
    draft.confirmation_pose = planar_pose(confirmation_pose);
    draft.suggested_location_id = goal->suggested_location_id;
    draft.suggested_display_name = goal->suggested_display_name;
    draft.suggested_aliases = goal->suggested_aliases;
    draft.suggested_semantic_type = goal->suggested_semantic_type;
    draft.building = goal->building;
    draft.floor = goal->floor;
    draft.area = goal->area;
    draft.notes = goal->notes;
    draft.source_session_id = goal->source_session_id;

    const auto validation = recorder_.Validate(draft);
    if (!validation.valid) {
      finish(
        goal_handle,
        false,
        RegisterMappedLocation::Result::RESULT_INVALID_REQUEST,
        validation.reason,
        empty_candidate);
      return;
    }

    savo_msgs::msg::LocationCandidate candidate;
    candidate.state = savo_msgs::msg::LocationCandidate::STATE_PENDING_REVIEW;
    candidate.candidate_revision = 0U;
    candidate.candidate_id = goal->candidate_id;
    candidate.map_id = goal->map_id;
    candidate.map_revision = goal->map_revision;
    candidate.map_release_id = goal->map_release_id;
    candidate.tag_family = draft.tag_family;
    candidate.tag_id = draft.tag_id;
    candidate.tag_pose_map = tag_pose;
    candidate.detection_quality = static_cast<float>(draft.detection_quality);
    candidate.accepted_observations = draft.accepted_observations;
    candidate.position_stddev_m = static_cast<float>(draft.position_stddev_m);
    candidate.yaw_stddev_rad = static_cast<float>(draft.yaw_stddev_rad);
    candidate.approach_pose_valid = true;
    candidate.approach_pose = approach_pose;
    candidate.confirmation_pose_valid = true;
    candidate.confirmation_pose = confirmation_pose;
    candidate.suggested_location_id = goal->suggested_location_id;
    candidate.suggested_display_name = goal->suggested_display_name;
    candidate.suggested_aliases = goal->suggested_aliases;
    candidate.suggested_semantic_type = goal->suggested_semantic_type;
    candidate.building = goal->building;
    candidate.floor = goal->floor;
    candidate.area = goal->area;
    candidate.notes = goal->notes;
    candidate.source_session_id = goal->source_session_id;
    candidate.source_component = "savo_mapping";
    candidate.review_reason.clear();
    candidate.created_at = now();
    candidate.updated_at = candidate.created_at;

    if (!registration_client_->wait_for_service(
        std::chrono::duration<double>(dependency_wait_timeout_s_)))
    {
      finish(
        goal_handle,
        false,
        RegisterMappedLocation::Result::RESULT_LOCATION_REGISTRY_UNAVAILABLE,
        "location_registration_service_unavailable",
        candidate);
      return;
    }

    feedback(
      goal_handle,
      RegisterMappedLocation::Feedback::STATE_PERSISTING_CANDIDATE,
      "persisting_location_candidate",
      candidate.accepted_observations,
      candidate.detection_quality,
      candidate.tag_id);

    auto registration_request = std::make_shared<RegisterCandidate::Request>();
    registration_request->candidate = candidate;
    registration_request->actor_id = goal->actor_id;
    if (goal_handle->is_canceling()) {
      finish(
        goal_handle,
        false,
        RegisterMappedLocation::Result::RESULT_CANCELED,
        "mapping_registration_canceled_before_persistence",
        candidate,
        true);
      return;
    }

    persistence_in_progress_.store(true);
    auto registration_future =
      registration_client_->async_send_request(registration_request);

    while (rclcpp::ok() &&
      registration_future.wait_for(std::chrono::milliseconds(50)) !=
      std::future_status::ready)
    {
      // Cancellation is rejected after persistence starts. Wait for the
      // authoritative commit acknowledgement so the action never reports
      // a canceled or timed-out result for a candidate that may have committed.
    }
    persistence_in_progress_.store(false);

    if (!rclcpp::ok()) {
      finish(
        goal_handle,
        false,
        RegisterMappedLocation::Result::RESULT_INTERNAL_ERROR,
        "shutdown_during_location_registration_commit",
        candidate);
      return;
    }

    const auto registration = registration_future.get();
    if (!registration->registered) {
      finish(
        goal_handle,
        false,
        RegisterMappedLocation::Result::RESULT_REGISTRATION_REJECTED,
        registration->reason,
        registration->stored_candidate.candidate_id.empty() ?
        candidate : registration->stored_candidate);
      return;
    }

    finish(
      goal_handle,
      true,
      RegisterMappedLocation::Result::RESULT_REGISTERED,
      registration->reason,
      registration->stored_candidate);
  }

  SemanticLandmarkRecorder recorder_;
  std::atomic<bool> busy_{false};
  std::atomic<bool> persistence_in_progress_{false};
  std::string action_name_{};
  std::string head_action_name_{};
  std::string authorization_service_name_{};
  std::string registration_service_name_{};
  double default_timeout_s_{25.0};
  double dependency_wait_timeout_s_{2.0};
  double default_approach_distance_m_{0.80};
  bool derive_missing_approach_pose_{true};

  rclcpp::Client<Authorize>::SharedPtr authorization_client_{};
  rclcpp::Client<RegisterCandidate>::SharedPtr registration_client_{};
  rclcpp_action::Client<ConfirmAprilTag>::SharedPtr head_client_{};
  rclcpp_action::Server<RegisterMappedLocation>::SharedPtr server_{};
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<savo_mapping::MappedLocationRegistrationNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
