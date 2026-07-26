// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <action_msgs/srv/cancel_goal.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_nav/action_names.hpp"
#include "savo_nav/goal_gateway.hpp"
#include "savo_nav/topic_names.hpp"

namespace
{

using NavigateToPose =
  nav2_msgs::action::NavigateToPose;

using ExternalGoalHandle =
  rclcpp_action::ServerGoalHandle<NavigateToPose>;

using InternalGoalHandle =
  rclcpp_action::ClientGoalHandle<NavigateToPose>;

using SteadyClock =
  std::chrono::steady_clock;

std::string GoalIdToString(
  const rclcpp_action::GoalUUID & uuid)
{
  std::ostringstream stream;

  stream << std::hex << std::setfill('0');

  for (const auto value : uuid) {
    stream << std::setw(2)
           << static_cast<int>(value);
  }

  return stream.str();
}

bool PoseToPlanarYaw(
  const geometry_msgs::msg::Pose & pose,
  double & yaw)
{
  const auto & orientation = pose.orientation;

  const std::array<double, 7> values{
    pose.position.x,
    pose.position.y,
    pose.position.z,
    orientation.x,
    orientation.y,
    orientation.z,
    orientation.w
  };

  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }

  if (std::abs(pose.position.z) > 1.0e-3) {
    return false;
  }

  if (
    std::abs(orientation.x) > 1.0e-3 ||
    std::abs(orientation.y) > 1.0e-3)
  {
    return false;
  }

  const double norm =
    orientation.x * orientation.x +
    orientation.y * orientation.y +
    orientation.z * orientation.z +
    orientation.w * orientation.w;

  if (
    !std::isfinite(norm) ||
    std::abs(norm - 1.0) > 1.0e-3)
  {
    return false;
  }

  yaw = std::atan2(
    2.0 * orientation.w * orientation.z,
    1.0 -
    2.0 * orientation.z * orientation.z);

  return std::isfinite(yaw);
}

savo_nav::NavigationReadinessState
ReadinessStateFromString(
  const std::string & state)
{
  if (state == "ready") {
    return
      savo_nav::NavigationReadinessState::kReady;
  }

  if (state == "degraded") {
    return
      savo_nav::NavigationReadinessState::kDegraded;
  }

  if (state == "fault") {
    return
      savo_nav::NavigationReadinessState::kFault;
  }

  if (state == "offline") {
    return
      savo_nav::NavigationReadinessState::kOffline;
  }

  return
    savo_nav::NavigationReadinessState::kBlocked;
}

class GoalGatewayNode final : public rclcpp::Node
{
public:
  GoalGatewayNode()
  : Node("goal_gateway_node")
  {
    const std::string navigation_action =
      declare_parameter<std::string>(
      "navigation_action_name",
      std::string(
        savo_nav::actions::
        kNavigationNavigateToPose));

    const std::string exploration_action =
      declare_parameter<std::string>(
      "exploration_action_name",
      std::string(
        savo_nav::actions::
        kExplorationNavigateToPose));

    const std::string nav2_action =
      declare_parameter<std::string>(
      "nav2_action_name",
      std::string(
        savo_nav::actions::
        kNav2NavigateToPose));

    const std::string map_mode =
      declare_parameter<std::string>(
      "map_mode",
      "saved_map");

    const std::string active_map_id =
      declare_parameter<std::string>(
      "active_map_id",
      "saved_map");

    const int map_revision =
      declare_parameter<int>(
      "map_revision",
      1);

    const int history_capacity =
      declare_parameter<int>(
      "recent_history_capacity",
      32);

    if (history_capacity <= 0) {
      throw std::invalid_argument(
              "recent_history_capacity must be positive");
    }

    if (map_revision <= 0) {
      throw std::invalid_argument(
              "map_revision must be positive");
    }

    allow_behavior_tree_override_ =
      declare_parameter<bool>(
      "allow_behavior_tree_override",
      false);

    execution_timeout_seconds_ =
      declare_parameter<double>(
      "execution_timeout_seconds",
      300.0);

    feedback_stale_timeout_seconds_ =
      declare_parameter<double>(
      "feedback_stale_timeout_seconds",
      10.0);

    if (
      !std::isfinite(execution_timeout_seconds_) ||
      execution_timeout_seconds_ <= 0.0)
    {
      throw std::invalid_argument(
              "execution_timeout_seconds must be positive");
    }

    if (
      !std::isfinite(
        feedback_stale_timeout_seconds_) ||
      feedback_stale_timeout_seconds_ <= 0.0)
    {
      throw std::invalid_argument(
              "feedback_stale_timeout_seconds "
              "must be positive");
    }

    savo_nav::GoalValidationPolicy policy;

    policy.max_abs_coordinate_m =
      declare_parameter<double>(
      "max_abs_coordinate_m",
      1000.0);

    policy.require_readiness = true;

    policy.allow_degraded_readiness =
      declare_parameter<bool>(
      "allow_degraded_readiness",
      false);

    policy.require_map_id_match = true;

    gateway_ =
      std::make_unique<savo_nav::GoalGateway>(
      policy,
      static_cast<std::size_t>(
        history_capacity));

    ConfigureMapContext(
      map_mode,
      active_map_id,
      map_revision);

    auto state_qos =
      rclcpp::QoS(rclcpp::KeepLast(1));

    state_qos.reliable();
    state_qos.transient_local();

    readiness_subscription_ =
      create_subscription<std_msgs::msg::String>(
      std::string(
        savo_nav::topics::kReadiness),
      state_qos,
      [this](
        const std_msgs::msg::String::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);

        readiness_.state =
        ReadinessStateFromString(
          message->data);

        readiness_.goal_acceptance_allowed =
        message->data == "ready" ||
        (
          message->data == "degraded" &&
          allow_degraded_readiness_
        );

        readiness_.reason =
        "readiness_state_" + message->data;
      });

    readiness_reason_subscription_ =
      create_subscription<std_msgs::msg::String>(
      std::string(
        savo_nav::topics::kReadinessReason),
      state_qos,
      [this](
        const std_msgs::msg::String::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        readiness_.reason = message->data;
      });

    state_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string(
        savo_nav::topics::kNavigationState),
      state_qos);

    status_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string(
        savo_nav::topics::kNavigationStatus),
      state_qos);

    feedback_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string(
        savo_nav::topics::kNavigationFeedback),
      rclcpp::QoS(10).reliable());

    result_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string(
        savo_nav::topics::kNavigationResult),
      state_qos);

    nav2_client_ =
      rclcpp_action::create_client<NavigateToPose>(
      this,
      nav2_action);

    navigation_server_ =
      CreatePublicServer(
      navigation_action,
      savo_nav::GoalSource::kNavigation);

    exploration_server_ =
      CreatePublicServer(
      exploration_action,
      savo_nav::GoalSource::kExploration);

    watchdog_timer_ =
      create_wall_timer(
      std::chrono::milliseconds(250),
      [this]()
      {
        CheckWatchdogs();
      });

    PublishState("gateway_started");

    RCLCPP_INFO(
      get_logger(),
      "Goal gateway started: navigation=%s "
      "exploration=%s internal_nav2=%s",
      navigation_action.c_str(),
      exploration_action.c_str(),
      nav2_action.c_str());
  }

private:
  using PublicServer =
    rclcpp_action::Server<NavigateToPose>;

  PublicServer::SharedPtr CreatePublicServer(
    const std::string & action_name,
    const savo_nav::GoalSource source)
  {
    return rclcpp_action::create_server<NavigateToPose>(
      this,
      action_name,
      [this, source](
        const rclcpp_action::GoalUUID & uuid,
        const std::shared_ptr<
          const NavigateToPose::Goal> goal)
      {
        return HandleGoal(
          uuid,
          goal,
          source);
      },
      [this](
        const std::shared_ptr<
          ExternalGoalHandle> goal_handle)
      {
        return HandleCancel(goal_handle);
      },
      [this](
        const std::shared_ptr<
          ExternalGoalHandle> goal_handle)
      {
        HandleAccepted(goal_handle);
      });
  }

  void ConfigureMapContext(
    const std::string & map_mode,
    const std::string & active_map_id,
    const int map_revision)
  {
    map_context_.frame_id = "map";
    map_context_.revision =
      static_cast<std::uint64_t>(map_revision);

    map_context_.available = true;
    map_context_.localization_ready = true;

    if (map_mode == "saved_map") {
      if (active_map_id.empty()) {
        throw std::invalid_argument(
                "saved_map mode requires active_map_id");
      }

      map_context_.mode =
        savo_nav::NavigationMapMode::kSavedMap;

      map_context_.authority =
        savo_nav::MapToOdomAuthority::kAmcl;

      map_context_.map_id = active_map_id;
      map_context_.mapping_active = false;

      return;
    }

    if (map_mode == "live_mapping") {
      map_context_.mode =
        savo_nav::NavigationMapMode::kLiveMapping;

      map_context_.authority =
        savo_nav::MapToOdomAuthority::kSlamToolbox;

      map_context_.map_id.clear();
      map_context_.mapping_active = true;

      return;
    }

    throw std::invalid_argument(
            "map_mode must be saved_map "
            "or live_mapping");
  }

  rclcpp_action::GoalResponse HandleGoal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<
      const NavigateToPose::Goal> goal,
    const savo_nav::GoalSource source)
  {
    const std::string goal_id =
      GoalIdToString(uuid);

    std::lock_guard<std::mutex> lock(mutex_);

    if (
      !allow_behavior_tree_override_ &&
      !goal->behavior_tree.empty())
    {
      PublishStatusLocked(
        "goal_rejected_behavior_tree_override");

      return rclcpp_action::GoalResponse::REJECT;
    }

    if (
      !nav2_client_->wait_for_action_server(
        std::chrono::seconds(0)))
    {
      PublishStatusLocked(
        "goal_rejected_nav2_server_unavailable");

      return rclcpp_action::GoalResponse::REJECT;
    }

    double yaw =
      std::numeric_limits<double>::quiet_NaN();

    if (!PoseToPlanarYaw(goal->pose.pose, yaw)) {
      PublishStatusLocked(
        "goal_rejected_invalid_planar_pose");

      return rclcpp_action::GoalResponse::REJECT;
    }

    savo_nav::GoalValidationRequest request;

    request.context.goal_id = goal_id;
    request.context.source = source;
    request.context.target_frame =
      goal->pose.header.frame_id;

    request.context.map_id =
      map_context_.map_id;

    request.context.sequence =
      NextSequence(source);

    request.pose.x = goal->pose.pose.position.x;
    request.pose.y = goal->pose.pose.position.y;
    request.pose.yaw = yaw;

    request.map_context = map_context_;
    request.readiness = readiness_;

    const auto decision =
      gateway_->Admit(request);

    if (!decision.accepted) {
      PublishStatusLocked(
        "goal_rejected_" + decision.reason);

      return rclcpp_action::GoalResponse::REJECT;
    }

    active_context_ = request.context;
    active_source_ = source;

    pending_goal_id_ = goal_id;
    cancel_forwarded_ = false;

    PublishStateLocked(
      "goal_reserved");

    return
      rclcpp_action::GoalResponse::
      ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse HandleCancel(
    const std::shared_ptr<
      ExternalGoalHandle> goal_handle)
  {
    const std::string goal_id =
      GoalIdToString(
      goal_handle->get_goal_id());

    InternalGoalHandle::SharedPtr internal_handle;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (
        !active_context_ ||
        active_context_->goal_id != goal_id)
      {
        return
          rclcpp_action::CancelResponse::REJECT;
      }

      const auto decision =
        gateway_->RequestCancel(
        active_context_->goal_id,
        active_context_->sequence,
        "external_cancel_requested");

      if (!decision.accepted) {
        return
          rclcpp_action::CancelResponse::REJECT;
      }

      PublishStateLocked(
        "external_cancel_requested");

      if (
        internal_goal_handle_ &&
        !cancel_forwarded_)
      {
        cancel_forwarded_ = true;
        internal_handle =
          internal_goal_handle_;
      }
    }

    if (internal_handle) {
      ForwardCancelToNav2(internal_handle);
    }

    return
      rclcpp_action::CancelResponse::ACCEPT;
  }

  void HandleAccepted(
    const std::shared_ptr<
      ExternalGoalHandle> goal_handle)
  {
    const std::string goal_id =
      GoalIdToString(
      goal_handle->get_goal_id());

    NavigateToPose::Goal nav2_goal;

    std::uint64_t sequence = 0;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (
        !active_context_ ||
        active_context_->goal_id != goal_id)
      {
        auto result =
          MakeErrorResult(
          "accepted_goal_context_mismatch");

        goal_handle->abort(result);
        return;
      }

      external_goal_handle_ = goal_handle;
      sequence = active_context_->sequence;

      if (
        !gateway_->MarkForwarding(
          goal_id,
          sequence))
      {
        auto result =
          MakeErrorResult(
          "gateway_failed_to_enter_forwarding");

        goal_handle->abort(result);
        ReleaseActiveLocked(
          "forwarding_state_failure");

        return;
      }

      nav2_goal =
        *goal_handle->get_goal();

      forwarding_started_ =
        SteadyClock::now();

      last_feedback_time_ =
        forwarding_started_;

      PublishStateLocked(
        "forwarding_goal_to_nav2");
    }

    typename rclcpp_action::Client<
      NavigateToPose>::SendGoalOptions options;

    options.goal_response_callback =
      [this, goal_id, sequence](
      const InternalGoalHandle::SharedPtr &
      internal_goal_handle)
      {
        OnNav2GoalResponse(
          goal_id,
          sequence,
          internal_goal_handle);
      };

    options.feedback_callback =
      [this, goal_id, sequence](
      InternalGoalHandle::SharedPtr,
      const std::shared_ptr<
        const NavigateToPose::Feedback> feedback)
      {
        OnNav2Feedback(
          goal_id,
          sequence,
          feedback);
      };

    options.result_callback =
      [this, goal_id, sequence](
      const InternalGoalHandle::WrappedResult &
      result)
      {
        OnNav2Result(
          goal_id,
          sequence,
          result);
      };

    try {
      nav2_client_->async_send_goal(
        nav2_goal,
        options);
    } catch (const std::exception & exception) {
      std::lock_guard<std::mutex> lock(mutex_);

      if (
        active_context_ &&
        active_context_->goal_id == goal_id &&
        external_goal_handle_)
      {
        auto result =
          MakeErrorResult(
          std::string(
            "nav2_send_goal_exception:") +
          exception.what());

        external_goal_handle_->abort(result);

        ReleaseActiveLocked(
          "nav2_send_goal_exception");
      }
    }
  }

  void OnNav2GoalResponse(
    const std::string & goal_id,
    const std::uint64_t sequence,
    const InternalGoalHandle::SharedPtr &
    internal_goal_handle)
  {
    bool forward_pending_cancel = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (!MatchesActiveLocked(
          goal_id,
          sequence))
      {
        return;
      }

      if (!internal_goal_handle) {
        if (external_goal_handle_) {
          external_goal_handle_->abort(
            MakeErrorResult(
              "nav2_rejected_goal"));
        }

        ReleaseActiveLocked(
          "nav2_rejected_goal");

        return;
      }

      internal_goal_handle_ =
        internal_goal_handle;

      if (
        !gateway_->MarkAcceptedByNav2(
          goal_id,
          sequence))
      {
        if (external_goal_handle_) {
          external_goal_handle_->abort(
            MakeErrorResult(
              "nav2_acceptance_state_mismatch"));
        }

        ReleaseActiveLocked(
          "nav2_acceptance_state_mismatch");

        return;
      }

      nav2_accepted_time_ =
        SteadyClock::now();

      last_feedback_time_ =
        nav2_accepted_time_;

      PublishStateLocked(
        "nav2_goal_active");

      if (
        gateway_->CancelRequested() &&
        !cancel_forwarded_)
      {
        cancel_forwarded_ = true;
        forward_pending_cancel = true;
      }
    }

    if (forward_pending_cancel) {
      ForwardCancelToNav2(
        internal_goal_handle);
    }
  }

  void OnNav2Feedback(
    const std::string & goal_id,
    const std::uint64_t sequence,
    const std::shared_ptr<
      const NavigateToPose::Feedback> feedback)
  {
    std::shared_ptr<ExternalGoalHandle>
    external_goal_handle;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (!MatchesActiveLocked(
          goal_id,
          sequence))
      {
        return;
      }

      last_feedback_time_ =
        SteadyClock::now();

      external_goal_handle =
        external_goal_handle_;

      std_msgs::msg::String message;

      std::ostringstream stream;

      stream
        << "goal_id=" << goal_id
        << ";distance_remaining="
        << feedback->distance_remaining
        << ";recoveries="
        << feedback->number_of_recoveries;

      message.data = stream.str();

      feedback_publisher_->publish(message);
    }

    if (external_goal_handle) {
      auto forwarded_feedback =
        std::make_shared<
        NavigateToPose::Feedback>(
        *feedback);

      external_goal_handle->
      publish_feedback(
        forwarded_feedback);
    }
  }

  void OnNav2Result(
    const std::string & goal_id,
    const std::uint64_t sequence,
    const InternalGoalHandle::WrappedResult &
    wrapped_result)
  {
    std::shared_ptr<ExternalGoalHandle>
    external_goal_handle;

    std::shared_ptr<NavigateToPose::Result>
    external_result;

    rclcpp_action::ResultCode result_code;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (!MatchesActiveLocked(
          goal_id,
          sequence))
      {
        return;
      }

      external_goal_handle =
        external_goal_handle_;

      external_result =
        std::make_shared<
        NavigateToPose::Result>();

      if (wrapped_result.result) {
        *external_result =
          *wrapped_result.result;
      }

      result_code = wrapped_result.code;

      if (
        result_code ==
        rclcpp_action::ResultCode::CANCELED &&
        gateway_->CancelRequested())
      {
        static_cast<void>(
          gateway_->AcknowledgeCancellation(
            goal_id,
            sequence));
      } else {
        static_cast<void>(
          gateway_->Complete(
            goal_id,
            sequence,
            ResultCodeToReason(
              result_code)));
      }

      PublishTerminalResultLocked(
        goal_id,
        result_code,
        external_result);

      ClearSessionLocked();
    }

    if (!external_goal_handle) {
      return;
    }

    switch (result_code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        external_goal_handle->succeed(
          external_result);
        return;

      case rclcpp_action::ResultCode::CANCELED:
        external_goal_handle->canceled(
          external_result);
        return;

      case rclcpp_action::ResultCode::ABORTED:
      case rclcpp_action::ResultCode::UNKNOWN:
        external_goal_handle->abort(
          external_result);
        return;
    }
  }

  void ForwardCancelToNav2(
    const InternalGoalHandle::SharedPtr &
    internal_goal_handle)
  {
    try {
      nav2_client_->async_cancel_goal(
        internal_goal_handle,
        [this](
          const typename rclcpp_action::Client<
            NavigateToPose>::CancelResponse::
          SharedPtr response)
        {
          std::lock_guard<std::mutex> lock(mutex_);

          if (
            response->return_code ==
            action_msgs::srv::CancelGoal::
            Response::ERROR_NONE)
          {
            PublishStateLocked(
              "nav2_cancel_request_accepted");
          } else {
            PublishStatusLocked(
              "nav2_cancel_request_rejected");
          }
        });
    } catch (const std::exception & exception) {
      std::lock_guard<std::mutex> lock(mutex_);

      PublishStatusLocked(
        std::string(
          "nav2_cancel_exception:") +
        exception.what());
    }
  }

  void CheckWatchdogs()
  {
    InternalGoalHandle::SharedPtr handle_to_cancel;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (
        !active_context_ ||
        !internal_goal_handle_ ||
        gateway_->CancelRequested())
      {
        return;
      }

      const auto snapshot =
        gateway_->Snapshot();

      if (
        snapshot.state !=
        savo_nav::GoalGatewayState::kActive)
      {
        return;
      }

      const auto now = SteadyClock::now();

      const double execution_seconds =
        std::chrono::duration<double>(
        now - nav2_accepted_time_).count();

      const double feedback_age_seconds =
        std::chrono::duration<double>(
        now - last_feedback_time_).count();

      std::string cancellation_reason;

      if (
        execution_seconds >
        execution_timeout_seconds_)
      {
        cancellation_reason =
          "nav2_execution_timeout";
      }

      if (
        cancellation_reason.empty() &&
        feedback_age_seconds >
        feedback_stale_timeout_seconds_)
      {
        cancellation_reason =
          "nav2_feedback_stale";
      }

      if (cancellation_reason.empty()) {
        return;
      }

      const auto decision =
        gateway_->RequestCancel(
        active_context_->goal_id,
        active_context_->sequence,
        cancellation_reason);

      if (!decision.accepted) {
        return;
      }

      cancel_forwarded_ = true;
      handle_to_cancel = internal_goal_handle_;

      PublishStateLocked(
        cancellation_reason);
    }

    if (handle_to_cancel) {
      ForwardCancelToNav2(handle_to_cancel);
    }
  }

  bool MatchesActiveLocked(
    const std::string & goal_id,
    const std::uint64_t sequence) const
  {
    return
      active_context_.has_value() &&
      active_context_->goal_id == goal_id &&
      active_context_->sequence == sequence;
  }

  std::uint64_t NextSequence(
    const savo_nav::GoalSource source)
  {
    if (
      source ==
      savo_nav::GoalSource::kExploration)
    {
      return ++exploration_sequence_;
    }

    return ++navigation_sequence_;
  }

  std::shared_ptr<NavigateToPose::Result>
  MakeErrorResult(
    const std::string & reason) const
  {
    auto result =
      std::make_shared<
      NavigateToPose::Result>();

    result->error_code =
      std::numeric_limits<std::uint16_t>::max();

    result->error_msg = reason;

    return result;
  }

  static std::string ResultCodeToReason(
    const rclcpp_action::ResultCode code)
  {
    switch (code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        return "nav2_goal_succeeded";

      case rclcpp_action::ResultCode::CANCELED:
        return "nav2_goal_canceled";

      case rclcpp_action::ResultCode::ABORTED:
        return "nav2_goal_aborted";

      case rclcpp_action::ResultCode::UNKNOWN:
        return "nav2_goal_unknown_result";
    }

    return "nav2_goal_unknown_result";
  }

  void PublishState(
    const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    PublishStateLocked(reason);
  }

  void PublishStateLocked(
    const std::string & reason)
  {
    const auto snapshot =
      gateway_->Snapshot();

    std_msgs::msg::String state_message;

    state_message.data =
      std::string(
      savo_nav::GoalGateway::ToString(
        snapshot.state));

    state_publisher_->publish(state_message);

    PublishStatusLocked(reason);
  }

  void PublishStatusLocked(
    const std::string & reason)
  {
    const auto snapshot =
      gateway_->Snapshot();

    std_msgs::msg::String message;

    std::ostringstream stream;

    stream
      << "state="
      << savo_nav::GoalGateway::ToString(
      snapshot.state)
      << ";goal_id="
      << snapshot.active_goal_id
      << ";source="
      << savo_nav::GoalContextContract::ToString(
      snapshot.source)
      << ";sequence="
      << snapshot.sequence
      << ";reason="
      << reason;

    message.data = stream.str();

    status_publisher_->publish(message);
  }

  void PublishTerminalResultLocked(
    const std::string & goal_id,
    const rclcpp_action::ResultCode code,
    const std::shared_ptr<
      NavigateToPose::Result> & result)
  {
    std_msgs::msg::String message;

    std::ostringstream stream;

    stream
      << "goal_id=" << goal_id
      << ";result="
      << ResultCodeToReason(code)
      << ";error_code="
      << result->error_code
      << ";error_msg="
      << result->error_msg;

    message.data = stream.str();

    result_publisher_->publish(message);
  }

  void ReleaseActiveLocked(
    const std::string & reason)
  {
    if (active_context_) {
      static_cast<void>(
        gateway_->Complete(
          active_context_->goal_id,
          active_context_->sequence,
          reason));
    }

    PublishTerminalResultLocked(
      active_context_ ?
      active_context_->goal_id :
      std::string{},
      rclcpp_action::ResultCode::ABORTED,
      MakeErrorResult(reason));

    ClearSessionLocked();
  }

  void ClearSessionLocked()
  {
    active_context_.reset();

    active_source_ =
      savo_nav::GoalSource::kUnknown;

    pending_goal_id_.clear();

    external_goal_handle_.reset();
    internal_goal_handle_.reset();

    cancel_forwarded_ = false;

    PublishStateLocked(
      "gateway_idle");
  }

  std::mutex mutex_;

  std::unique_ptr<savo_nav::GoalGateway>
  gateway_;

  savo_nav::NavigationReadinessResult
    readiness_{};

  savo_nav::MapContext map_context_{};

  std::optional<savo_nav::GoalContext>
  active_context_{};

  savo_nav::GoalSource active_source_{
    savo_nav::GoalSource::kUnknown};

  std::string pending_goal_id_{};

  bool allow_behavior_tree_override_{false};
  bool allow_degraded_readiness_{false};
  bool cancel_forwarded_{false};

  double execution_timeout_seconds_{300.0};
  double feedback_stale_timeout_seconds_{10.0};

  std::uint64_t navigation_sequence_{0};
  std::uint64_t exploration_sequence_{0};

  SteadyClock::time_point forwarding_started_{};
  SteadyClock::time_point nav2_accepted_time_{};
  SteadyClock::time_point last_feedback_time_{};

  std::shared_ptr<ExternalGoalHandle>
  external_goal_handle_;

  InternalGoalHandle::SharedPtr
    internal_goal_handle_;

  rclcpp_action::Client<
    NavigateToPose>::SharedPtr nav2_client_;

  PublicServer::SharedPtr navigation_server_;
  PublicServer::SharedPtr exploration_server_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    readiness_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    readiness_reason_subscription_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    state_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    status_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    feedback_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    result_publisher_;

  rclcpp::TimerBase::SharedPtr
    watchdog_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int exit_code = 0;

  try {
    auto node =
      std::make_shared<GoalGatewayNode>();

    rclcpp::executors::MultiThreadedExecutor
      executor;

    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "goal_gateway_node"),
      "Fatal error: %s",
      exception.what());

    exit_code = 1;
  }

  rclcpp::shutdown();

  return exit_code;
}
