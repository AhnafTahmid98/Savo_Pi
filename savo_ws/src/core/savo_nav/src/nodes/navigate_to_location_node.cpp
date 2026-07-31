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
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "savo_msgs/action/confirm_april_tag.hpp"
#include "savo_msgs/action/navigate_to_location.hpp"
#include "savo_msgs/srv/authorize_location_operation.hpp"
#include "savo_msgs/srv/resolve_location.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "savo_nav/action_names.hpp"
#include "savo_nav/location_navigation_contract.hpp"

namespace savo_nav
{
namespace
{

using NavigateToLocation = savo_msgs::action::NavigateToLocation;
using LocationGoalHandle =
  rclcpp_action::ServerGoalHandle<NavigateToLocation>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using ConfirmAprilTag = savo_msgs::action::ConfirmAprilTag;
using ConfirmGoalHandle = rclcpp_action::ClientGoalHandle<ConfirmAprilTag>;
using ResolveLocation = savo_msgs::srv::ResolveLocation;
using Authorize = savo_msgs::srv::AuthorizeLocationOperation;

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

[[nodiscard]] LocationPlanarPose planar_pose(
  const geometry_msgs::msg::PoseStamped & pose,
  const bool valid)
{
  LocationPlanarPose output;
  output.frame_id = pose.header.frame_id;
  output.x = pose.pose.position.x;
  output.y = pose.pose.position.y;
  output.yaw = tf2::getYaw(pose.pose.orientation);
  output.valid = valid;
  return output;
}

[[nodiscard]] LocationRecordView record_view(
  const savo_msgs::msg::LocationRecord & record)
{
  LocationRecordView output;
  output.approved =
    record.state == savo_msgs::msg::LocationRecord::STATE_APPROVED;
  output.retired =
    record.state == savo_msgs::msg::LocationRecord::STATE_RETIRED;
  output.enabled = record.enabled;
  output.location_id = record.location_id;
  output.map_id = record.map_id;
  output.map_revision = record.map_revision;
  output.approach_pose = planar_pose(record.approach_pose, true);
  output.tag_pose_map = planar_pose(
    record.tag_pose_map, record.tag_pose_map_valid);
  output.tag_pose_map_valid = record.tag_pose_map_valid;
  output.arrival_confirmation_required =
    record.arrival_confirmation_required;
  return output;
}

}  // namespace

class NavigateToLocationNode final : public rclcpp::Node
{
public:
  NavigateToLocationNode()
  : Node("navigate_to_location_node"),
    contract_(declare_parameter<double>(
      "direct_tag_pose_epsilon_m", 0.05))
  {
    action_name_ = declare_parameter<std::string>(
      "action_name", "/savo_nav/locations/navigate");
    resolution_service_name_ = declare_parameter<std::string>(
      "resolution_service", "/savo_locations/resolve");
    authorization_service_name_ = declare_parameter<std::string>(
      "authorization_service", "/savo_supervisor/authorize_location_operation");
    navigation_action_name_ = declare_parameter<std::string>(
      "navigation_action_name",
      std::string(actions::kNavigationNavigateToPose));
    head_action_name_ = declare_parameter<std::string>(
      "head_confirmation_action", "/savo_head/apriltag/confirm");
    default_timeout_s_ = declare_parameter<double>("default_timeout_s", 120.0);
    dependency_wait_timeout_s_ = declare_parameter<double>(
      "dependency_wait_timeout_s", 2.0);
    authorization_recheck_period_s_ = declare_parameter<double>(
      "authorization_recheck_period_s", 1.0);
    arrival_confirmation_timeout_s_ = declare_parameter<double>(
      "arrival_confirmation_timeout_s", 15.0);

    if (action_name_.empty() || resolution_service_name_.empty() ||
      authorization_service_name_.empty() || navigation_action_name_.empty() ||
      head_action_name_.empty() || default_timeout_s_ <= 0.0 ||
      dependency_wait_timeout_s_ <= 0.0 ||
      authorization_recheck_period_s_ <= 0.0 ||
      arrival_confirmation_timeout_s_ <= 0.0)
    {
      throw std::runtime_error("invalid navigate-to-location parameters");
    }

    resolution_client_ = create_client<ResolveLocation>(resolution_service_name_);
    authorization_client_ = create_client<Authorize>(authorization_service_name_);
    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, navigation_action_name_);
    head_client_ = rclcpp_action::create_client<ConfirmAprilTag>(
      this, head_action_name_);

    server_ = rclcpp_action::create_server<NavigateToLocation>(
      this,
      action_name_,
      std::bind(
        &NavigateToLocationNode::handle_goal,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      std::bind(
        &NavigateToLocationNode::handle_cancel,
        this,
        std::placeholders::_1),
      std::bind(
        &NavigateToLocationNode::handle_accepted,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "navigate-to-location action ready action=%s motion=%s",
      action_name_.c_str(),
      navigation_action_name_.c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const NavigateToLocation::Goal> goal)
  {
    if (busy_.exchange(true)) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    const bool invalid =
      goal->request_id.empty() || goal->actor_id.empty() || goal->query.empty() ||
      (goal->enforce_map_context &&
      (goal->map_id.empty() || goal->map_revision == 0U));

    if (invalid) {
      busy_.store(false);
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<LocationGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<LocationGoalHandle> goal_handle)
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
            NavigateToLocation::Result::RESULT_INTERNAL_ERROR,
            std::string("navigate_to_location_exception: ") + exception.what(),
            savo_msgs::msg::LocationRecord{},
            false,
            false,
            savo_msgs::msg::AprilTagObservation{});
        }
        busy_.store(false);
      }}.detach();
  }

  void feedback(
    const std::shared_ptr<LocationGoalHandle> & goal_handle,
    const std::uint8_t state,
    const std::string & text,
    const std::string & location_id = "",
    const float distance_remaining_m = 0.0F,
    const std::uint32_t recoveries = 0U,
    const std::uint32_t tag_observations = 0U)
  {
    auto message = std::make_shared<NavigateToLocation::Feedback>();
    message->state = state;
    message->state_text = text;
    message->location_id = location_id;
    message->distance_remaining_m = distance_remaining_m;
    message->navigation_recoveries = recoveries;
    message->accepted_tag_observations = tag_observations;
    goal_handle->publish_feedback(message);
  }

  void finish(
    const std::shared_ptr<LocationGoalHandle> & goal_handle,
    const bool succeeded,
    const std::uint8_t code,
    const std::string & reason,
    const savo_msgs::msg::LocationRecord & location,
    const bool navigation_succeeded,
    const bool arrival_confirmed,
    const savo_msgs::msg::AprilTagObservation & observation,
    const bool canceled = false)
  {
    busy_.store(false);
    auto result = std::make_shared<NavigateToLocation::Result>();
    result->succeeded = succeeded;
    result->result_code = code;
    result->reason = reason;
    result->location = location;
    result->navigation_succeeded = navigation_succeeded;
    result->arrival_confirmed = arrival_confirmed;
    result->final_observation = observation;

    if (canceled) {
      goal_handle->canceled(result);
    } else if (succeeded) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  [[nodiscard]] bool canceled_or_expired(
    const std::shared_ptr<LocationGoalHandle> & goal_handle,
    const rclcpp::Time & deadline) const
  {
    return goal_handle->is_canceling() || !rclcpp::ok() || now() >= deadline;
  }

  template<typename FutureT>
  [[nodiscard]] bool wait_future(
    FutureT & future,
    const std::shared_ptr<LocationGoalHandle> & goal_handle,
    const rclcpp::Time & deadline) const
  {
    while (!canceled_or_expired(goal_handle, deadline)) {
      if (future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] std::shared_ptr<Authorize::Response> authorize(
    const NavigateToLocation::Goal & goal,
    const savo_msgs::msg::LocationRecord & location,
    const std::uint8_t operation,
    const bool motion_required,
    const std::shared_ptr<LocationGoalHandle> & goal_handle,
    const rclcpp::Time & deadline)
  {
    if (!authorization_client_->wait_for_service(
        std::chrono::duration<double>(dependency_wait_timeout_s_)))
    {
      return nullptr;
    }

    auto request = std::make_shared<Authorize::Request>();
    request->operation = operation;
    request->request_id = goal.request_id;
    request->actor_id = goal.actor_id;
    request->location_id = location.location_id;
    request->map_id = location.map_id;
    request->map_revision = location.map_revision;
    request->motion_required = motion_required;

    auto future = authorization_client_->async_send_request(request);
    if (!wait_future(future, goal_handle, deadline)) {
      return nullptr;
    }
    return future.get();
  }

  void execute(const std::shared_ptr<LocationGoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const double requested_timeout = duration_seconds(goal->timeout);
    const double timeout_s = requested_timeout > 0.0 ?
      requested_timeout : default_timeout_s_;
    const auto deadline = now() + rclcpp::Duration::from_seconds(timeout_s);
    savo_msgs::msg::LocationRecord empty_location;
    savo_msgs::msg::AprilTagObservation empty_observation;

    feedback(
      goal_handle,
      NavigateToLocation::Feedback::STATE_RESOLVING_LOCATION,
      "resolving_semantic_location");

    if (!resolution_client_->wait_for_service(
        std::chrono::duration<double>(dependency_wait_timeout_s_)))
    {
      finish(
        goal_handle,
        false,
        NavigateToLocation::Result::RESULT_LOCATION_NOT_FOUND,
        "location_resolution_service_unavailable",
        empty_location,
        false,
        false,
        empty_observation);
      return;
    }

    auto resolve_request = std::make_shared<ResolveLocation::Request>();
    resolve_request->query = goal->query;
    resolve_request->enforce_map_context = goal->enforce_map_context;
    resolve_request->map_id = goal->map_id;
    resolve_request->map_revision = goal->map_revision;
    auto resolve_future = resolution_client_->async_send_request(resolve_request);

    if (!wait_future(resolve_future, goal_handle, deadline)) {
      const bool canceled = goal_handle->is_canceling();
      finish(
        goal_handle,
        false,
        canceled ? NavigateToLocation::Result::RESULT_CANCELED :
        NavigateToLocation::Result::RESULT_TIMED_OUT,
        canceled ? "navigate_to_location_canceled" :
        "location_resolution_timed_out",
        empty_location,
        false,
        false,
        empty_observation,
        canceled);
      return;
    }

    const auto resolved = resolve_future.get();
    if (!resolved->resolved) {
      std::uint8_t code = NavigateToLocation::Result::RESULT_LOCATION_NOT_FOUND;
      switch (resolved->result_code) {
        case ResolveLocation::Response::RESULT_AMBIGUOUS:
          code = NavigateToLocation::Result::RESULT_LOCATION_AMBIGUOUS;
          break;
        case ResolveLocation::Response::RESULT_DISABLED:
          code = NavigateToLocation::Result::RESULT_LOCATION_DISABLED;
          break;
        case ResolveLocation::Response::RESULT_MAP_MISMATCH:
          code = NavigateToLocation::Result::RESULT_MAP_MISMATCH;
          break;
        default:
          break;
      }
      finish(
        goal_handle,
        false,
        code,
        resolved->reason,
        resolved->location,
        false,
        false,
        empty_observation);
      return;
    }

    const auto location = resolved->location;
    LocationNavigationRequestView request_view;
    request_view.query = goal->query;
    request_view.enforce_map_context = goal->enforce_map_context;
    request_view.map_id = goal->map_id;
    request_view.map_revision = goal->map_revision;
    request_view.require_arrival_confirmation =
      goal->require_arrival_confirmation;
    const auto target = contract_.SelectTarget(
      request_view, record_view(location));

    if (!target.accepted) {
      std::uint8_t code = NavigateToLocation::Result::RESULT_NAVIGATION_REJECTED;
      if (target.code == LocationTargetCode::kDisabled) {
        code = NavigateToLocation::Result::RESULT_LOCATION_DISABLED;
      } else if (target.code == LocationTargetCode::kMapMismatch) {
        code = NavigateToLocation::Result::RESULT_MAP_MISMATCH;
      }
      finish(
        goal_handle,
        false,
        code,
        target.reason,
        location,
        false,
        false,
        empty_observation);
      return;
    }

    feedback(
      goal_handle,
      NavigateToLocation::Feedback::STATE_REQUESTING_AUTHORIZATION,
      "requesting_navigation_authorization",
      location.location_id);

    const auto authorization = authorize(
      *goal,
      location,
      Authorize::Request::OP_NAVIGATE_TO_LOCATION,
      true,
      goal_handle,
      deadline);
    if (!authorization || !authorization->authorized) {
      finish(
        goal_handle,
        false,
        NavigateToLocation::Result::RESULT_SUPERVISOR_DENIED,
        authorization ? authorization->reason :
        "supervisor_authorization_unavailable",
        location,
        false,
        false,
        empty_observation);
      return;
    }

    if (!navigation_client_->wait_for_action_server(
        std::chrono::duration<double>(dependency_wait_timeout_s_)))
    {
      finish(
        goal_handle,
        false,
        NavigateToLocation::Result::RESULT_NAVIGATION_UNAVAILABLE,
        "validated_navigation_action_unavailable",
        location,
        false,
        false,
        empty_observation);
      return;
    }

    feedback(
      goal_handle,
      NavigateToLocation::Feedback::STATE_NAVIGATING_TO_APPROACH_POSE,
      "navigating_to_approved_approach_pose",
      location.location_id);

    NavigateToPose::Goal navigation_goal;
    navigation_goal.pose = location.approach_pose;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions navigation_options;
    navigation_options.feedback_callback =
      [this, goal_handle, location](
      NavGoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> nav_feedback)
      {
        feedback(
          goal_handle,
          NavigateToLocation::Feedback::STATE_NAVIGATING_TO_APPROACH_POSE,
          "navigating_to_approved_approach_pose",
          location.location_id,
          nav_feedback->distance_remaining,
          nav_feedback->number_of_recoveries,
          0U);
      };

    auto navigation_goal_future =
      navigation_client_->async_send_goal(navigation_goal, navigation_options);
    if (!wait_future(navigation_goal_future, goal_handle, deadline)) {
      const bool canceled = goal_handle->is_canceling();
      finish(
        goal_handle,
        false,
        canceled ? NavigateToLocation::Result::RESULT_CANCELED :
        NavigateToLocation::Result::RESULT_TIMED_OUT,
        canceled ? "navigate_to_location_canceled" :
        "navigation_goal_response_timed_out",
        location,
        false,
        false,
        empty_observation,
        canceled);
      return;
    }

    const auto navigation_goal_handle = navigation_goal_future.get();
    if (!navigation_goal_handle) {
      finish(
        goal_handle,
        false,
        NavigateToLocation::Result::RESULT_NAVIGATION_REJECTED,
        "validated_navigation_goal_rejected",
        location,
        false,
        false,
        empty_observation);
      return;
    }

    auto navigation_result_future =
      navigation_client_->async_get_result(navigation_goal_handle);
    auto next_authorization_check = now() +
      rclcpp::Duration::from_seconds(authorization_recheck_period_s_);

    while (!canceled_or_expired(goal_handle, deadline) &&
      navigation_result_future.wait_for(std::chrono::milliseconds(50)) !=
      std::future_status::ready)
    {
      if (now() >= next_authorization_check) {
        const auto reauthorization = authorize(
          *goal,
          location,
          Authorize::Request::OP_NAVIGATE_TO_LOCATION,
          true,
          goal_handle,
          deadline);
        if (!reauthorization || !reauthorization->authorized) {
          navigation_client_->async_cancel_goal(navigation_goal_handle);
          finish(
            goal_handle,
            false,
            NavigateToLocation::Result::RESULT_SUPERVISOR_DENIED,
            reauthorization ? reauthorization->reason :
            "runtime_navigation_authorization_lost",
            location,
            false,
            false,
            empty_observation);
          return;
        }
        next_authorization_check = now() +
          rclcpp::Duration::from_seconds(authorization_recheck_period_s_);
      }
    }

    if (navigation_result_future.wait_for(std::chrono::milliseconds(0)) !=
      std::future_status::ready)
    {
      navigation_client_->async_cancel_goal(navigation_goal_handle);
      const bool canceled = goal_handle->is_canceling();
      finish(
        goal_handle,
        false,
        canceled ? NavigateToLocation::Result::RESULT_CANCELED :
        NavigateToLocation::Result::RESULT_TIMED_OUT,
        canceled ? "navigate_to_location_canceled" :
        "navigation_execution_timed_out",
        location,
        false,
        false,
        empty_observation,
        canceled);
      return;
    }

    const auto navigation_result = navigation_result_future.get();
    if (navigation_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      const bool canceled =
        navigation_result.code == rclcpp_action::ResultCode::CANCELED;
      finish(
        goal_handle,
        false,
        canceled ? NavigateToLocation::Result::RESULT_CANCELED :
        NavigateToLocation::Result::RESULT_NAVIGATION_FAILED,
        canceled ? "validated_navigation_canceled" :
        "validated_navigation_failed",
        location,
        false,
        false,
        empty_observation,
        canceled);
      return;
    }

    if (!target.arrival_confirmation_required) {
      feedback(
        goal_handle,
        NavigateToLocation::Feedback::STATE_COMPLETING,
        "navigation_completed_without_tag_confirmation",
        location.location_id);
      finish(
        goal_handle,
        true,
        NavigateToLocation::Result::RESULT_SUCCEEDED,
        "approved_approach_pose_reached",
        location,
        true,
        false,
        empty_observation);
      return;
    }

    const auto confirmation_authorization = authorize(
      *goal,
      location,
      Authorize::Request::OP_CONFIRM_LOCATION_ARRIVAL,
      false,
      goal_handle,
      deadline);
    if (!confirmation_authorization || !confirmation_authorization->authorized) {
      finish(
        goal_handle,
        false,
        NavigateToLocation::Result::RESULT_SUPERVISOR_DENIED,
        confirmation_authorization ? confirmation_authorization->reason :
        "arrival_confirmation_authorization_unavailable",
        location,
        true,
        false,
        empty_observation);
      return;
    }

    if (!head_client_->wait_for_action_server(
        std::chrono::duration<double>(dependency_wait_timeout_s_)))
    {
      finish(
        goal_handle,
        false,
        NavigateToLocation::Result::RESULT_HEAD_UNAVAILABLE,
        "head_confirmation_action_unavailable",
        location,
        true,
        false,
        empty_observation);
      return;
    }

    feedback(
      goal_handle,
      NavigateToLocation::Feedback::STATE_CONFIRMING_ARRIVAL,
      "confirming_expected_arrival_apriltag",
      location.location_id);

    ConfirmAprilTag::Goal confirmation_goal;
    confirmation_goal.mode = ConfirmAprilTag::Goal::CONFIRM_ARRIVAL;
    confirmation_goal.expected_family = location.tag_family;
    confirmation_goal.expected_tag_id = location.tag_id;
    confirmation_goal.location_id = location.location_id;
    confirmation_goal.map_id = location.map_id;
    confirmation_goal.map_revision = location.map_revision;
    confirmation_goal.timeout = to_duration(std::min(
      arrival_confirmation_timeout_s_,
      std::max(0.1, (deadline - now()).seconds())));
    confirmation_goal.require_map_pose = false;

    rclcpp_action::Client<ConfirmAprilTag>::SendGoalOptions confirmation_options;
    confirmation_options.feedback_callback =
      [this, goal_handle, location](
      ConfirmGoalHandle::SharedPtr,
      const std::shared_ptr<const ConfirmAprilTag::Feedback> head_feedback)
      {
        feedback(
          goal_handle,
          NavigateToLocation::Feedback::STATE_CONFIRMING_ARRIVAL,
          head_feedback->state_text,
          location.location_id,
          0.0F,
          0U,
          head_feedback->accepted_observations);
      };

    auto confirmation_goal_future =
      head_client_->async_send_goal(confirmation_goal, confirmation_options);
    if (!wait_future(confirmation_goal_future, goal_handle, deadline)) {
      const bool canceled = goal_handle->is_canceling();
      finish(
        goal_handle,
        false,
        canceled ? NavigateToLocation::Result::RESULT_CANCELED :
        NavigateToLocation::Result::RESULT_TIMED_OUT,
        canceled ? "arrival_confirmation_canceled" :
        "arrival_confirmation_goal_timed_out",
        location,
        true,
        false,
        empty_observation,
        canceled);
      return;
    }

    const auto confirmation_goal_handle = confirmation_goal_future.get();
    if (!confirmation_goal_handle) {
      finish(
        goal_handle,
        false,
        NavigateToLocation::Result::RESULT_ARRIVAL_CONFIRMATION_FAILED,
        "arrival_confirmation_goal_rejected",
        location,
        true,
        false,
        empty_observation);
      return;
    }

    auto confirmation_result_future =
      head_client_->async_get_result(confirmation_goal_handle);
    if (!wait_future(confirmation_result_future, goal_handle, deadline)) {
      head_client_->async_cancel_goal(confirmation_goal_handle);
      const bool canceled = goal_handle->is_canceling();
      finish(
        goal_handle,
        false,
        canceled ? NavigateToLocation::Result::RESULT_CANCELED :
        NavigateToLocation::Result::RESULT_TIMED_OUT,
        canceled ? "arrival_confirmation_canceled" :
        "arrival_confirmation_timed_out",
        location,
        true,
        false,
        empty_observation,
        canceled);
      return;
    }

    const auto confirmation_result = confirmation_result_future.get();
    if (confirmation_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !confirmation_result.result || !confirmation_result.result->confirmed)
    {
      finish(
        goal_handle,
        false,
        NavigateToLocation::Result::RESULT_ARRIVAL_CONFIRMATION_FAILED,
        confirmation_result.result ? confirmation_result.result->reason :
        "arrival_confirmation_failed_without_result",
        location,
        true,
        false,
        confirmation_result.result ?
        confirmation_result.result->final_observation : empty_observation);
      return;
    }

    feedback(
      goal_handle,
      NavigateToLocation::Feedback::STATE_COMPLETING,
      "location_arrival_confirmed",
      location.location_id,
      0.0F,
      0U,
      confirmation_result.result->accepted_observations);
    finish(
      goal_handle,
      true,
      NavigateToLocation::Result::RESULT_SUCCEEDED,
      "approved_approach_pose_reached_and_expected_tag_confirmed",
      location,
      true,
      true,
      confirmation_result.result->final_observation);
  }

  LocationNavigationContract contract_;
  std::atomic<bool> busy_{false};
  std::string action_name_{};
  std::string resolution_service_name_{};
  std::string authorization_service_name_{};
  std::string navigation_action_name_{};
  std::string head_action_name_{};
  double default_timeout_s_{120.0};
  double dependency_wait_timeout_s_{2.0};
  double authorization_recheck_period_s_{1.0};
  double arrival_confirmation_timeout_s_{15.0};

  rclcpp::Client<ResolveLocation>::SharedPtr resolution_client_{};
  rclcpp::Client<Authorize>::SharedPtr authorization_client_{};
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_{};
  rclcpp_action::Client<ConfirmAprilTag>::SharedPtr head_client_{};
  rclcpp_action::Server<NavigateToLocation>::SharedPtr server_{};
};

}  // namespace savo_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<savo_nav::NavigateToLocationNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
