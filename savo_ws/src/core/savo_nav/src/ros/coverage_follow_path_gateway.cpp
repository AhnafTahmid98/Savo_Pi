// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/coverage_follow_path_gateway.hpp"

#include <chrono>
#include <exception>
#include <stdexcept>
#include <utility>

#include <action_msgs/srv/cancel_goal.hpp>

namespace savo_nav
{

CoverageFollowPathGateway::CoverageFollowPathGateway(
  rclcpp::Node & node,
  std::mutex & shared_mutex,
  GoalGateway & gateway,
  MapContext & map_context,
  NavigationReadinessResult & readiness,
  std::string coverage_action,
  std::string follow_path_action,
  std::string controller_id,
  std::string goal_checker_id,
  std::string progress_checker_id,
  CoverageActionAdapterPolicy adapter_policy,
  CoverageExecutionPolicy execution_policy,
  PublishCallback publish)
: node_(node),
  mutex_(shared_mutex),
  gateway_(gateway),
  map_context_(map_context),
  readiness_(readiness),
  controller_id_(std::move(controller_id)),
  goal_checker_id_(std::move(goal_checker_id)),
  progress_checker_id_(std::move(progress_checker_id)),
  adapter_policy_(std::move(adapter_policy)),
  execution_model_(execution_policy),
  publish_(std::move(publish))
{
  if (
    coverage_action.empty() ||
    follow_path_action.empty() ||
    !publish_)
  {
    throw std::invalid_argument(
            "Coverage FollowPath gateway configuration is invalid");
  }

  follow_path_client_ =
    rclcpp_action::create_client<FollowPath>(
    &node_,
    follow_path_action);

  coverage_server_ =
    rclcpp_action::create_server<Action>(
    &node_,
    coverage_action,
    [this](
      const rclcpp_action::GoalUUID & uuid,
      const std::shared_ptr<const Action::Goal> goal)
    {
      return HandleGoal(uuid, goal);
    },
    [this](
      const std::shared_ptr<ExternalGoalHandle> goal_handle)
    {
      return HandleCancel(goal_handle);
    },
    [this](
      const std::shared_ptr<ExternalGoalHandle> goal_handle)
    {
      HandleAccepted(goal_handle);
    });
}

bool CoverageFollowPathGateway::Active() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_;
}

rclcpp_action::GoalResponse
CoverageFollowPathGateway::HandleGoal(
  const rclcpp_action::GoalUUID &,
  const std::shared_ptr<const Action::Goal> & goal)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (active_) {
    publish_("coverage_rejected_goal_gateway_busy");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!follow_path_client_->wait_for_action_server(
      std::chrono::seconds(0)))
  {
    publish_("coverage_rejected_follow_path_unavailable");
    return rclcpp_action::GoalResponse::REJECT;
  }

  auto adapted =
    CoverageActionAdapter::AdaptGoal(
    *goal,
    map_context_,
    readiness_,
    ++sequence_counter_,
    adapter_policy_);

  if (!adapted.validation.IsValid()) {
    publish_(
      "coverage_rejected_" +
      adapted.validation.validation.reason);

    return rclcpp_action::GoalResponse::REJECT;
  }

  progress_tracker_.Reset();
  execution_model_.Reset();

  if (
    !progress_tracker_.Configure(
      adapted.validation_request.points) ||
    !execution_model_.Start(
      MonotonicSeconds(),
      adapted.requested_execution_timeout_seconds))
  {
    publish_("coverage_rejected_runtime_initialization");

    return rclcpp_action::GoalResponse::REJECT;
  }

  const auto decision =
    gateway_.AdmitValidated(
    adapted.context,
    adapted.validation.validation);

  if (!decision.accepted) {
    progress_tracker_.Reset();
    execution_model_.Reset();

    publish_(
      "coverage_rejected_" +
      decision.reason);

    return rclcpp_action::GoalResponse::REJECT;
  }

  active_ = true;
  active_context_ = adapted.context;

  cancel_forwarded_ = false;
  public_terminal_sent_ = false;

  external_goal_handle_.reset();
  internal_goal_handle_.reset();

  publish_("coverage_goal_reserved");

  return
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
CoverageFollowPathGateway::HandleCancel(
  const std::shared_ptr<ExternalGoalHandle> & goal_handle)
{
  InternalGoalHandle::SharedPtr internal_handle;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (
      !active_ ||
      external_goal_handle_ != goal_handle ||
      !active_context_)
    {
      return rclcpp_action::CancelResponse::REJECT;
    }

    const auto decision =
      gateway_.RequestCancel(
      active_context_->goal_id,
      active_context_->sequence,
      "external_cancel_requested");

    if (!decision.accepted) {
      return rclcpp_action::CancelResponse::REJECT;
    }

    static_cast<void>(
      execution_model_.RequestCancel(
        MonotonicSeconds(),
        "external_cancel"));

    if (
      internal_goal_handle_ &&
      !cancel_forwarded_)
    {
      cancel_forwarded_ = true;
      internal_handle = internal_goal_handle_;
    }

    publish_("coverage_external_cancel_requested");
    PublishFeedbackLocked();
  }

  if (internal_handle) {
    ForwardBackendCancel(internal_handle);
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void CoverageFollowPathGateway::HandleAccepted(
  const std::shared_ptr<ExternalGoalHandle> & goal_handle)
{
  FollowPath::Goal backend_goal;
  std::string goal_id;
  std::uint64_t sequence = 0;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (
      !active_ ||
      !active_context_ ||
      goal_handle->get_goal()->mission_id !=
      active_context_->goal_id)
    {
      auto execution = execution_model_.Snapshot();
      execution.state = CoverageExecutionState::kFailed;
      execution.terminal_code =
        CoverageTerminalCode::kInternalError;

      execution.reason =
        "accepted_coverage_context_mismatch";

      goal_handle->abort(
        std::make_shared<Action::Result>(
          CoverageActionAdapter::MakeResult(
            execution,
            progress_tracker_.Snapshot())));

      return;
    }

    external_goal_handle_ = goal_handle;

    goal_id = active_context_->goal_id;
    sequence = active_context_->sequence;

    if (
      !gateway_.MarkForwarding(
        goal_id,
        sequence) ||
      !progress_tracker_.Start(
        MonotonicSeconds()))
    {
      static_cast<void>(
        execution_model_.FailBeforeBackend(
          CoverageTerminalCode::kInternalError,
          "coverage_forwarding_state_failure"));

      static_cast<void>(
        gateway_.Complete(
          goal_id,
          sequence,
          "coverage_forwarding_state_failure"));

      auto result =
        CoverageActionAdapter::MakeResult(
        execution_model_.Snapshot(),
        progress_tracker_.Snapshot());

      goal_handle->abort(
        std::make_shared<Action::Result>(result));

      ClearLocked();
      return;
    }

    backend_goal.path =
      goal_handle->get_goal()->path;

    backend_goal.controller_id = controller_id_;
    backend_goal.goal_checker_id = goal_checker_id_;

    backend_goal.progress_checker_id =
      progress_checker_id_;

    publish_("coverage_forwarding_to_follow_path");
  }

  rclcpp_action::Client<FollowPath>::SendGoalOptions options;

  options.goal_response_callback =
    [this, goal_id, sequence](
    const InternalGoalHandle::SharedPtr & internal_handle)
    {
      OnBackendGoalResponse(
        goal_id,
        sequence,
        internal_handle);
    };

  options.feedback_callback =
    [this, goal_id, sequence](
    InternalGoalHandle::SharedPtr,
    const std::shared_ptr<
      const FollowPath::Feedback> feedback)
    {
      OnBackendFeedback(
        goal_id,
        sequence,
        feedback);
    };

  options.result_callback =
    [this, goal_id, sequence](
    const InternalGoalHandle::WrappedResult & result)
    {
      OnBackendResult(
        goal_id,
        sequence,
        result);
    };

  try {
    static_cast<void>(
      follow_path_client_->async_send_goal(
        backend_goal,
        options));
  } catch (const std::exception & exception) {
    std::shared_ptr<ExternalGoalHandle> external_handle;
    Action::Result result;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (!MatchesLocked(goal_id, sequence)) {
        return;
      }

      static_cast<void>(
        execution_model_.FailBeforeBackend(
          CoverageTerminalCode::kInternalError,
          std::string("follow_path_send_exception:") +
          exception.what()));

      static_cast<void>(
        gateway_.Complete(
          goal_id,
          sequence,
          "follow_path_send_exception"));

      external_handle = external_goal_handle_;

      result = CoverageActionAdapter::MakeResult(
        execution_model_.Snapshot(),
        progress_tracker_.Snapshot());

      ClearLocked();
    }

    if (external_handle) {
      external_handle->abort(
        std::make_shared<Action::Result>(result));
    }
  }
}

void CoverageFollowPathGateway::OnBackendGoalResponse(
  const std::string & goal_id,
  const std::uint64_t sequence,
  const InternalGoalHandle::SharedPtr & internal_handle)
{
  std::shared_ptr<ExternalGoalHandle> external_handle;
  Action::Result rejected_result;

  bool abort_external = false;
  bool forward_cancel = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!MatchesLocked(goal_id, sequence)) {
      return;
    }

    if (!internal_handle) {
      static_cast<void>(
        execution_model_.FailBeforeBackend(
          CoverageTerminalCode::kBackendRejected,
          "follow_path_rejected_goal"));

      static_cast<void>(
        gateway_.Complete(
          goal_id,
          sequence,
          "follow_path_rejected_goal"));

      external_handle = external_goal_handle_;

      rejected_result =
        CoverageActionAdapter::MakeResult(
        execution_model_.Snapshot(),
        progress_tracker_.Snapshot());

      abort_external = true;
      ClearLocked();
    } else {
      internal_goal_handle_ = internal_handle;

      if (
        !gateway_.MarkAcceptedByNav2(
          goal_id,
          sequence) ||
        !execution_model_.MarkBackendAccepted(
          MonotonicSeconds()))
      {
        static_cast<void>(
          gateway_.RequestCancel(
            goal_id,
            sequence,
            "follow_path_acceptance_state_mismatch"));

        static_cast<void>(
          execution_model_.RequestCancel(
            MonotonicSeconds(),
            "follow_path_acceptance_state_mismatch"));

        cancel_forwarded_ = true;
        forward_cancel = true;
      } else {
        forward_cancel =
          gateway_.CancelRequested() &&
          !cancel_forwarded_;

        if (forward_cancel) {
          cancel_forwarded_ = true;
        }

        publish_("coverage_follow_path_active");
        PublishFeedbackLocked();
      }
    }
  }

  if (abort_external && external_handle) {
    external_handle->abort(
      std::make_shared<Action::Result>(
        rejected_result));

    return;
  }

  if (forward_cancel && internal_handle) {
    ForwardBackendCancel(internal_handle);
  }
}

void CoverageFollowPathGateway::OnBackendFeedback(
  const std::string & goal_id,
  const std::uint64_t sequence,
  const std::shared_ptr<const FollowPath::Feedback> & feedback)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (
    !MatchesLocked(goal_id, sequence) ||
    feedback == nullptr)
  {
    return;
  }

  const double now_seconds = MonotonicSeconds();

  static_cast<void>(
    execution_model_.MarkFeedback(now_seconds));

  static_cast<void>(
    progress_tracker_.UpdateRemainingDistance(
      feedback->distance_to_goal,
      now_seconds));

  PublishFeedbackLocked();
  publish_("coverage_follow_path_feedback");
}

void CoverageFollowPathGateway::OnBackendResult(
  const std::string & goal_id,
  const std::uint64_t sequence,
  const InternalGoalHandle::WrappedResult & wrapped)
{
  std::shared_ptr<ExternalGoalHandle> external_handle;
  Action::Result result;

  CoverageExecutionSnapshot execution;
  bool public_terminal_already_sent = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!MatchesLocked(goal_id, sequence)) {
      return;
    }

    const double now_seconds = MonotonicSeconds();

    if (
      wrapped.code ==
      rclcpp_action::ResultCode::SUCCEEDED)
    {
      static_cast<void>(
        progress_tracker_.MarkSucceeded(
          now_seconds));
    }

    static_cast<void>(
      execution_model_.ObserveBackendTerminal(
        BackendTerminalFrom(wrapped.code),
        BackendReason(
          wrapped.code,
          wrapped.result)));

    if (
      wrapped.code ==
      rclcpp_action::ResultCode::CANCELED &&
      gateway_.CancelRequested())
    {
      static_cast<void>(
        gateway_.AcknowledgeCancellation(
          goal_id,
          sequence));
    } else {
      static_cast<void>(
        gateway_.Complete(
          goal_id,
          sequence,
          BackendReason(
            wrapped.code,
            wrapped.result)));
    }

    external_handle = external_goal_handle_;
    public_terminal_already_sent =
      public_terminal_sent_;

    execution = execution_model_.Snapshot();

    result = CoverageActionAdapter::MakeResult(
      execution,
      progress_tracker_.Snapshot());

    ClearLocked();
  }

  if (
    external_handle == nullptr ||
    public_terminal_already_sent)
  {
    return;
  }

  auto result_ptr =
    std::make_shared<Action::Result>(result);

  if (
    execution.state ==
    CoverageExecutionState::kSucceeded)
  {
    external_handle->succeed(result_ptr);
    return;
  }

  if (
    execution.state ==
    CoverageExecutionState::kCanceled &&
    external_handle->is_canceling())
  {
    external_handle->canceled(result_ptr);
    return;
  }

  external_handle->abort(result_ptr);
}

void CoverageFollowPathGateway::CheckWatchdogs()
{
  InternalGoalHandle::SharedPtr handle_to_cancel;

  std::shared_ptr<ExternalGoalHandle>
  external_to_finish;

  std::optional<Action::Result>
  public_timeout_result;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (
      !active_ ||
      !active_context_)
    {
      return;
    }

    const auto event =
      execution_model_.CheckWatchdogs(
      MonotonicSeconds());

    if (event == CoverageWatchdogEvent::kNone) {
      return;
    }

    const auto execution =
      execution_model_.Snapshot();

    if (
      event ==
      CoverageWatchdogEvent::kExecutionTimeout ||
      event ==
      CoverageWatchdogEvent::kFeedbackStale)
    {
      static_cast<void>(
        gateway_.RequestCancel(
          active_context_->goal_id,
          active_context_->sequence,
          execution.reason));

      if (
        internal_goal_handle_ &&
        !cancel_forwarded_)
      {
        cancel_forwarded_ = true;
        handle_to_cancel = internal_goal_handle_;
      }

      publish_(
        "coverage_watchdog_" +
        execution.reason);

      PublishFeedbackLocked();
    }

    if (
      event ==
      CoverageWatchdogEvent::kCancelTimeout &&
      !public_terminal_sent_ &&
      external_goal_handle_)
    {
      public_terminal_sent_ = true;
      external_to_finish = external_goal_handle_;

      public_timeout_result =
        CoverageActionAdapter::MakeResult(
        execution,
        progress_tracker_.Snapshot());

      publish_("coverage_cancel_ack_timeout_quarantined");
    }
  }

  if (handle_to_cancel) {
    ForwardBackendCancel(handle_to_cancel);
  }

  if (
    external_to_finish &&
    public_timeout_result)
  {
    external_to_finish->abort(
      std::make_shared<Action::Result>(
        *public_timeout_result));
  }
}

void CoverageFollowPathGateway::ForwardBackendCancel(
  const InternalGoalHandle::SharedPtr & internal_handle)
{
  try {
    static_cast<void>(
      follow_path_client_->async_cancel_goal(
        internal_handle,
        [this](
          const typename rclcpp_action::Client<
            FollowPath>::CancelResponse::SharedPtr response)
        {
          std::lock_guard<std::mutex> lock(mutex_);

          const bool accepted =
          response->return_code ==
          action_msgs::srv::CancelGoal::
          Response::ERROR_NONE;

          publish_(
            accepted ?
            "coverage_follow_path_cancel_accepted" :
            "coverage_follow_path_cancel_rejected");
        }));
  } catch (const std::exception & exception) {
    std::lock_guard<std::mutex> lock(mutex_);

    publish_(
      std::string("coverage_follow_path_cancel_exception:") +
      exception.what());
  }
}

void CoverageFollowPathGateway::PublishFeedbackLocked()
{
  if (!external_goal_handle_) {
    return;
  }

  const auto feedback =
    CoverageActionAdapter::MakeFeedback(
    execution_model_.Snapshot(),
    progress_tracker_.Snapshot());

  external_goal_handle_->publish_feedback(
    std::make_shared<Action::Feedback>(
      feedback));
}

bool CoverageFollowPathGateway::MatchesLocked(
  const std::string & goal_id,
  const std::uint64_t sequence) const
{
  return
    active_ &&
    active_context_.has_value() &&
    active_context_->goal_id == goal_id &&
    active_context_->sequence == sequence;
}

void CoverageFollowPathGateway::ClearLocked()
{
  active_ = false;
  active_context_.reset();

  external_goal_handle_.reset();
  internal_goal_handle_.reset();

  cancel_forwarded_ = false;
  public_terminal_sent_ = false;

  execution_model_.Reset();
  progress_tracker_.Reset();

  publish_("coverage_gateway_idle");
}

double CoverageFollowPathGateway::MonotonicSeconds()
{
  const auto now =
    std::chrono::steady_clock::now().time_since_epoch();

  return std::chrono::duration<double>(now).count();
}

CoverageBackendTerminal
CoverageFollowPathGateway::BackendTerminalFrom(
  const rclcpp_action::ResultCode code)
{
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return CoverageBackendTerminal::kSucceeded;

    case rclcpp_action::ResultCode::CANCELED:
      return CoverageBackendTerminal::kCanceled;

    case rclcpp_action::ResultCode::ABORTED:
      return CoverageBackendTerminal::kAborted;

    case rclcpp_action::ResultCode::UNKNOWN:
      return CoverageBackendTerminal::kUnknown;
  }

  return CoverageBackendTerminal::kUnknown;
}

std::string CoverageFollowPathGateway::BackendReason(
  const rclcpp_action::ResultCode code,
  const std::shared_ptr<FollowPath::Result> & result)
{
  std::string reason;

  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      reason = "follow_path_succeeded";
      break;

    case rclcpp_action::ResultCode::CANCELED:
      reason = "follow_path_canceled";
      break;

    case rclcpp_action::ResultCode::ABORTED:
      reason = "follow_path_aborted";
      break;

    case rclcpp_action::ResultCode::UNKNOWN:
      reason = "follow_path_unknown_result";
      break;
  }

  if (
    result != nullptr &&
    !result->error_msg.empty())
  {
    reason += ":" + result->error_msg;
  }

  return reason;
}

}  // namespace savo_nav
