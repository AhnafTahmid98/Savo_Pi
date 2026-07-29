// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/coverage_admission_proxy.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

namespace savo_nav
{

CoverageAdmissionProxy::CoverageAdmissionProxy(
  rclcpp::Node & node,
  std::mutex & shared_mutex,
  bool & slot_reserved,
  bool & active_goal,
  bool & cancellation_requested,
  bool & internal_cancel_sent,
  std::string & cancellation_reason,
  std::uint64_t & generation_counter,
  const double internal_server_timeout_seconds,
  std::string public_action,
  std::string internal_action,
  EvaluateCallback evaluate,
  PublishCallback publish)
: node_(node),
  mutex_(shared_mutex),
  slot_reserved_(slot_reserved),
  active_goal_(active_goal),
  cancellation_requested_(cancellation_requested),
  internal_cancel_sent_(internal_cancel_sent),
  cancellation_reason_(cancellation_reason),
  generation_counter_(generation_counter),
  internal_server_timeout_seconds_(
    internal_server_timeout_seconds),
  evaluate_(std::move(evaluate)),
  publish_(std::move(publish))
{
  if (
    public_action.empty() ||
    internal_action.empty() ||
    !evaluate_ ||
    !publish_)
  {
    throw std::invalid_argument(
            "Coverage admission proxy configuration is invalid");
  }

  client_ = rclcpp_action::create_client<Action>(
    &node_,
    internal_action);

  server_ = rclcpp_action::create_server<Action>(
    &node_,
    public_action,
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

bool CoverageAdmissionProxy::Active() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_;
}

bool CoverageAdmissionProxy::ActiveLocked() const noexcept
{
  return active_;
}

rclcpp_action::GoalResponse
CoverageAdmissionProxy::HandleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const Action::Goal>)
{
  GoalAdmissionDecision decision;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    decision = evaluate_();

    if (decision.accept_new_goal) {
      slot_reserved_ = true;
    }
  }

  if (!decision.accept_new_goal) {
    publish_("rejected", decision.reason);
    return rclcpp_action::GoalResponse::REJECT;
  }

  publish_("reserved", "goal_reserved_coverage");

  return
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
CoverageAdmissionProxy::HandleCancel(
  const std::shared_ptr<ExternalGoalHandle> & goal_handle)
{
  std::uint64_t generation = 0;
  bool forward_cancel = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (
      !active_ ||
      external_goal_handle_ != goal_handle)
    {
      return rclcpp_action::CancelResponse::REJECT;
    }

    if (!cancellation_requested_) {
      cancellation_requested_ = true;
      cancellation_reason_ = "external_cancel";
    }

    generation = generation_;

    forward_cancel =
      internal_goal_handle_ != nullptr &&
      !internal_cancel_sent_;
  }

  publish_("canceling", "external_cancel");

  if (forward_cancel) {
    RequestInternalCancel(generation);
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void CoverageAdmissionProxy::HandleAccepted(
  const std::shared_ptr<ExternalGoalHandle> & goal_handle)
{
  std::uint64_t generation = 0;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    slot_reserved_ = false;
    active_goal_ = true;
    active_ = true;

    external_goal_handle_ = goal_handle;
    internal_goal_handle_.reset();

    cancellation_requested_ = false;
    internal_cancel_sent_ = false;
    cancellation_reason_.clear();

    generation = ++generation_counter_;
    generation_ = generation;
  }

  publish_("forwarding", "forwarding_coverage");

  std::thread(
    [this, generation, goal_handle]()
    {
      ForwardGoal(generation, goal_handle);
    }).detach();
}

void CoverageAdmissionProxy::ForwardGoal(
  const std::uint64_t generation,
  const std::shared_ptr<ExternalGoalHandle> & external_handle)
{
  if (!client_->wait_for_action_server(
      std::chrono::duration<double>(
        internal_server_timeout_seconds_)))
  {
    FinishLocally(
      generation,
      "internal_coverage_gateway_unavailable",
      false);

    return;
  }

  bool finish_before_forwarding = false;
  bool canceled_before_forwarding = false;
  std::string reason;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (
      !active_ ||
      generation != generation_)
    {
      return;
    }

    const auto decision = evaluate_();

    if (cancellation_requested_) {
      finish_before_forwarding = true;

      canceled_before_forwarding =
        cancellation_reason_ == "external_cancel";

      reason = cancellation_reason_;
    }

    if (
      !finish_before_forwarding &&
      !decision.accept_new_goal &&
      decision.reason != "goal_gateway_busy")
    {
      cancellation_requested_ = true;
      cancellation_reason_ = decision.reason;

      finish_before_forwarding = true;
      reason = decision.reason;
    }
  }

  if (finish_before_forwarding) {
    FinishLocally(
      generation,
      reason,
      canceled_before_forwarding);

    return;
  }

  rclcpp_action::Client<Action>::SendGoalOptions options;

  options.goal_response_callback =
    [this, generation](
    const InternalGoalHandle::SharedPtr & internal_handle)
    {
      OnInternalGoalResponse(
        generation,
        internal_handle);
    };

  options.feedback_callback =
    [this, generation](
    InternalGoalHandle::SharedPtr,
    const std::shared_ptr<const Action::Feedback> feedback)
    {
      OnInternalFeedback(generation, feedback);
    };

  options.result_callback =
    [this, generation](
    const InternalGoalHandle::WrappedResult & result)
    {
      OnInternalResult(generation, result);
    };

  try {
    static_cast<void>(
      client_->async_send_goal(
        *external_handle->get_goal(),
        options));
  } catch (const std::exception & exception) {
    FinishLocally(
      generation,
      std::string("internal_coverage_send_error:") +
      exception.what(),
      false);
  }
}

void CoverageAdmissionProxy::OnInternalGoalResponse(
  const std::uint64_t generation,
  const InternalGoalHandle::SharedPtr & internal_handle)
{
  bool forward_cancel = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (
      !active_ ||
      generation != generation_)
    {
      return;
    }

    if (internal_handle != nullptr) {
      internal_goal_handle_ = internal_handle;

      forward_cancel =
        cancellation_requested_ &&
        !internal_cancel_sent_;
    }
  }

  if (internal_handle == nullptr) {
    FinishLocally(
      generation,
      "internal_coverage_gateway_rejected_goal",
      false);

    return;
  }

  publish_("active", "internal_coverage_goal_accepted");

  if (forward_cancel) {
    RequestInternalCancel(generation);
  }
}

void CoverageAdmissionProxy::OnInternalFeedback(
  const std::uint64_t generation,
  const std::shared_ptr<const Action::Feedback> & feedback)
{
  std::shared_ptr<ExternalGoalHandle> external_handle;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (
      !active_ ||
      generation != generation_)
    {
      return;
    }

    external_handle = external_goal_handle_;
  }

  if (
    external_handle != nullptr &&
    feedback != nullptr)
  {
    external_handle->publish_feedback(
      std::make_shared<Action::Feedback>(*feedback));
  }
}

void CoverageAdmissionProxy::RequestCancel(
  std::string reason)
{
  std::uint64_t generation = 0;
  bool forward_cancel = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!active_ || reason.empty()) {
      return;
    }

    if (!cancellation_requested_) {
      cancellation_requested_ = true;
      cancellation_reason_ = std::move(reason);
    } else {
      if (
        cancellation_reason_ != "external_cancel")
      {
        cancellation_reason_ = std::move(reason);
      }
    }

    generation = generation_;

    forward_cancel =
      internal_goal_handle_ != nullptr &&
      !internal_cancel_sent_;
  }

  publish_("canceling", cancellation_reason_);

  if (forward_cancel) {
    RequestInternalCancel(generation);
  }
}

void CoverageAdmissionProxy::RequestInternalCancel(
  const std::uint64_t generation)
{
  InternalGoalHandle::SharedPtr internal_handle;
  std::string reason;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (
      !active_ ||
      generation != generation_ ||
      internal_goal_handle_ == nullptr ||
      internal_cancel_sent_)
    {
      return;
    }

    internal_cancel_sent_ = true;
    internal_handle = internal_goal_handle_;
    reason = cancellation_reason_;
  }

  publish_("canceling", reason);

  try {
    static_cast<void>(
      client_->async_cancel_goal(
        internal_handle,
        [this, generation](
          const typename rclcpp_action::Client<
            Action>::CancelResponse::SharedPtr)
        {
          bool still_active = false;

          {
            std::lock_guard<std::mutex> lock(mutex_);

            still_active =
            active_ &&
            generation == generation_;
          }

          if (still_active) {
            publish_(
              "canceling",
              "internal_coverage_cancel_requested");
          }
        }));
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "Coverage internal cancellation failed: %s",
      exception.what());
  }
}

void CoverageAdmissionProxy::OnInternalResult(
  const std::uint64_t generation,
  const InternalGoalHandle::WrappedResult & wrapped)
{
  std::shared_ptr<ExternalGoalHandle> external_handle;
  bool external_cancel = false;
  std::string interruption_reason;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (
      !active_ ||
      generation != generation_)
    {
      return;
    }

    external_handle = external_goal_handle_;

    external_cancel =
      cancellation_reason_ == "external_cancel";

    interruption_reason = cancellation_reason_;

    ReleaseLocked();
  }

  if (external_handle == nullptr) {
    return;
  }

  auto result = wrapped.result;

  if (result == nullptr) {
    result = MakeErrorResult(
      "internal_coverage_returned_null_result",
      Action::Result::RESULT_INTERNAL_ERROR);
  }

  switch (wrapped.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      external_handle->succeed(result);
      publish_("succeeded", "internal_coverage_succeeded");
      return;

    case rclcpp_action::ResultCode::CANCELED:
      if (
        external_cancel &&
        external_handle->is_canceling())
      {
        external_handle->canceled(result);

        publish_(
          "canceled",
          "external_coverage_cancel_acknowledged");
      } else {
        const std::string reason =
          interruption_reason.empty() ?
          "control_recovery_interruption" :
          interruption_reason;

        external_handle->abort(
          MakeErrorResult(
            reason,
            Action::Result::RESULT_ABORTED));

        publish_("interrupted", reason);
      }

      return;

    case rclcpp_action::ResultCode::ABORTED:
      external_handle->abort(result);
      publish_("aborted", "internal_coverage_aborted");
      return;

    case rclcpp_action::ResultCode::UNKNOWN:
      external_handle->abort(
        MakeErrorResult(
          "internal_coverage_unknown_result",
          Action::Result::RESULT_INTERNAL_ERROR));

      publish_("error", "internal_coverage_unknown_result");
      return;
  }
}

void CoverageAdmissionProxy::FinishLocally(
  const std::uint64_t generation,
  const std::string & reason,
  const bool canceled)
{
  std::shared_ptr<ExternalGoalHandle> external_handle;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (
      !active_ ||
      generation != generation_)
    {
      return;
    }

    external_handle = external_goal_handle_;
    ReleaseLocked();
  }

  if (external_handle == nullptr) {
    return;
  }

  if (
    canceled &&
    external_handle->is_canceling())
  {
    external_handle->canceled(
      MakeErrorResult(
        reason,
        Action::Result::RESULT_CANCELED));

    publish_("canceled", reason);
    return;
  }

  external_handle->abort(
    MakeErrorResult(
      reason,
      Action::Result::RESULT_BACKEND_UNAVAILABLE));

  publish_("aborted", reason);
}

void CoverageAdmissionProxy::ReleaseLocked()
{
  active_ = false;
  active_goal_ = false;
  slot_reserved_ = false;

  external_goal_handle_.reset();
  internal_goal_handle_.reset();

  cancellation_requested_ = false;
  internal_cancel_sent_ = false;
  cancellation_reason_.clear();
}

std::shared_ptr<CoverageAdmissionProxy::Action::Result>
CoverageAdmissionProxy::MakeErrorResult(
  const std::string & reason,
  const std::uint8_t result_code)
{
  auto result = std::make_shared<Action::Result>();

  result->success = false;
  result->result_code = result_code;
  result->terminal_state = "failed";
  result->reason = reason;

  return result;
}

}  // namespace savo_nav
