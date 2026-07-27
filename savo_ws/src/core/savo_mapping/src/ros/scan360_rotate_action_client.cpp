#include "savo_mapping/scan360_rotate_action_client.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace savo_mapping
{
namespace
{

std::chrono::steady_clock::duration
seconds_to_duration(const double seconds)
{
  return std::chrono::duration_cast<
    std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(seconds));
}

bool valid_positive_timeout(const double seconds)
{
  return std::isfinite(seconds) &&
         seconds > 0.0;
}

}  // namespace

Scan360RotateActionClient::SharedPtr
Scan360RotateActionClient::create(
  const rclcpp::Node::SharedPtr & node)
{
  return create(node, Options{});
}

Scan360RotateActionClient::SharedPtr
Scan360RotateActionClient::create(
  const rclcpp::Node::SharedPtr & node,
  Options options)
{
  if (!node) {
    throw std::invalid_argument(
            "Scan360RotateActionClient requires a node");
  }

  if (options.action_name.empty()) {
    throw std::invalid_argument(
            "RotateToHeading action name must not be empty");
  }

  if (
    !valid_positive_timeout(
      options.server_wait_timeout_sec) ||
    !valid_positive_timeout(
      options.goal_response_timeout_sec) ||
    !valid_positive_timeout(
      options.feedback_stale_timeout_sec) ||
    !valid_positive_timeout(
      options.cancel_timeout_sec) ||
    !std::isfinite(
      options.execution_grace_sec) ||
    options.execution_grace_sec < 0.0)
  {
    throw std::invalid_argument(
            "RotateToHeading timeout options are invalid");
  }

  auto instance = SharedPtr(
    new Scan360RotateActionClient(
      node,
      std::move(options)));

  instance->initialize();
  return instance;
}

Scan360RotateActionClient::Scan360RotateActionClient(
  rclcpp::Node::SharedPtr node,
  Options options)
: node_(std::move(node)),
  options_(std::move(options))
{
}

Scan360RotateActionClient::
~Scan360RotateActionClient()
{
  GoalHandle::SharedPtr goal_handle;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle = active_goal_;
  }

  if (!client_ || !goal_handle) {
    return;
  }

  try {
    (void)client_->async_cancel_goal(goal_handle);
  } catch (const std::exception &) {
  }

  try {
    client_->stop_callbacks(goal_handle);
  } catch (const std::exception &) {
  }
}

void Scan360RotateActionClient::initialize()
{
  client_ = rclcpp_action::create_client<Action>(
    node_,
    options_.action_name);
}

bool Scan360RotateActionClient::request_rotation(
  const double target_yaw_rad,
  const double max_duration_sec)
{
  if (
    !std::isfinite(target_yaw_rad) ||
    !valid_positive_timeout(max_duration_sec))
  {
    return false;
  }

  const auto now = SteadyClock::now();

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (busy_locked()) {
      return false;
    }

    active_goal_.reset();

    requested_max_duration_sec_ =
      max_duration_sec;

    state_deadline_ =
      now +
      seconds_to_duration(
      options_.server_wait_timeout_sec);

    execution_deadline_ = TimePoint{};
    last_feedback_at_ = TimePoint{};

    cancel_terminal_state_ = State::kCanceled;
    cancel_reason_ = "scan360_cancel_requested";

    snapshot_ = Snapshot{};
    snapshot_.state = State::kWaitingForServer;
    snapshot_.active = true;

    snapshot_.target_yaw_rad =
      normalize_yaw(target_yaw_rad);

    snapshot_.reason =
      "scan360_rotation_requested";
  }

  notify_update();
  tick();
  return true;
}

bool Scan360RotateActionClient::request_cancel(
  const std::string & reason)
{
  GoalHandle::SharedPtr goal_handle;
  bool terminal_without_goal = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (
      !busy_locked() ||
      snapshot_.state == State::kCanceling)
    {
      return false;
    }

    cancel_reason_ =
      reason.empty() ?
      "scan360_cancel_requested" :
      reason;

    cancel_terminal_state_ = State::kCanceled;

    snapshot_.cancel_requested = true;
    snapshot_.reason = cancel_reason_;

    if (
      snapshot_.state ==
      State::kWaitingForServer)
    {
      set_terminal_locked(
        State::kCanceled,
        cancel_reason_);

      terminal_without_goal = true;
    } else {
      snapshot_.state = State::kCanceling;

      state_deadline_ =
        SteadyClock::now() +
        seconds_to_duration(
        options_.cancel_timeout_sec);

      goal_handle = active_goal_;
    }
  }

  notify_update();

  if (terminal_without_goal) {
    return true;
  }

  if (goal_handle) {
    send_cancel_request(goal_handle);
  }

  return true;
}

void Scan360RotateActionClient::tick()
{
  const auto now = SteadyClock::now();

  bool send_goal = false;
  bool notify = false;
  bool cancel_for_feedback = false;
  bool cancel_for_execution = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    switch (snapshot_.state) {
      case State::kWaitingForServer:
        if (client_->action_server_is_ready()) {
          send_goal = true;
        } else if (now >= state_deadline_) {
          set_terminal_locked(
            State::kTimedOut,
            "scan360_action_server_unavailable");

          notify = true;
        }
        break;

      case State::kWaitingForGoalResponse:
        if (now >= state_deadline_) {
          set_terminal_locked(
            State::kTimedOut,
            "scan360_goal_response_timeout");

          notify = true;
        }
        break;

      case State::kActive:
        if (now >= execution_deadline_) {
          cancel_for_execution = true;
        } else if (  // NOLINT(readability/braces)
          now - last_feedback_at_ >=
          seconds_to_duration(
            options_.feedback_stale_timeout_sec))
        {
          cancel_for_feedback = true;
        }
        break;

      case State::kCanceling:
        if (now >= state_deadline_) {
          set_terminal_locked(
            State::kTimedOut,
            "scan360_cancel_timeout");

          notify = true;
        }
        break;

      default:
        break;
    }
  }

  if (notify) {
    notify_update();
  }

  if (send_goal) {
    send_pending_goal();
  } else if (cancel_for_execution) {
    begin_cancel(
      "scan360_rotation_timeout",
      State::kTimedOut);
  } else if (cancel_for_feedback) {
    begin_cancel(
      "scan360_feedback_stale",
      State::kFailed);
  }
}

Scan360RotateActionClient::Snapshot
Scan360RotateActionClient::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

void Scan360RotateActionClient::set_update_callback(UpdateCallback callback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  update_callback_ = std::move(callback);
}

double Scan360RotateActionClient::normalize_yaw(
  const double yaw_rad)
{
  if (!std::isfinite(yaw_rad)) {
    return yaw_rad;
  }

  return std::atan2(
    std::sin(yaw_rad),
    std::cos(yaw_rad));
}

const char * Scan360RotateActionClient::to_string(
  const State state) noexcept
{
  switch (state) {
    case State::kIdle:
      return "idle";

    case State::kWaitingForServer:
      return "waiting_for_server";

    case State::kWaitingForGoalResponse:
      return "waiting_for_goal_response";

    case State::kActive:
      return "active";

    case State::kCanceling:
      return "canceling";

    case State::kSucceeded:
      return "succeeded";

    case State::kCanceled:
      return "canceled";

    case State::kRejected:
      return "rejected";

    case State::kAborted:
      return "aborted";

    case State::kTimedOut:
      return "timed_out";

    case State::kFailed:
      return "failed";
  }

  return "unknown";
}

void Scan360RotateActionClient::send_pending_goal()
{
  Action::Goal goal;

  rclcpp_action::Client<Action>::
  SendGoalOptions send_options;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (
      snapshot_.state !=
      State::kWaitingForServer)
    {
      return;
    }

    snapshot_.state =
      State::kWaitingForGoalResponse;

    snapshot_.reason = "scan360_goal_sent";

    state_deadline_ =
      SteadyClock::now() +
      seconds_to_duration(
      options_.goal_response_timeout_sec);

    goal.target_yaw_rad =
      snapshot_.target_yaw_rad;

    goal.max_duration_sec =
      requested_max_duration_sec_;
  }

  const auto weak_self = weak_from_this();

  send_options.goal_response_callback =
    [weak_self](
    const GoalHandle::SharedPtr & goal_handle)
    {
      if (const auto self = weak_self.lock()) {
        self->handle_goal_response(
          goal_handle);
      }
    };

  send_options.feedback_callback =
    [weak_self](
    GoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<
      const Action::Feedback> feedback)
    {
      if (const auto self = weak_self.lock()) {
        self->handle_feedback(
          std::move(goal_handle),
          feedback);
      }
    };

  send_options.result_callback =
    [weak_self](
    const GoalHandle::WrappedResult &
    wrapped_result)
    {
      if (const auto self = weak_self.lock()) {
        self->handle_result(wrapped_result);
      }
    };

  notify_update();

  try {
    (void)client_->async_send_goal(
      goal,
      send_options);
  } catch (const std::exception &) {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      set_terminal_locked(
        State::kFailed,
        "scan360_goal_send_failed");
    }

    notify_update();
  }
}

void Scan360RotateActionClient::begin_cancel(
  const std::string & reason,
  const State terminal_state)
{
  GoalHandle::SharedPtr goal_handle;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (
      snapshot_.state != State::kActive ||
      !active_goal_)
    {
      return;
    }

    cancel_reason_ = reason;
    cancel_terminal_state_ = terminal_state;

    snapshot_.state = State::kCanceling;
    snapshot_.cancel_requested = true;
    snapshot_.reason = cancel_reason_;

    state_deadline_ =
      SteadyClock::now() +
      seconds_to_duration(
      options_.cancel_timeout_sec);

    goal_handle = active_goal_;
  }

  notify_update();
  send_cancel_request(goal_handle);
}

void Scan360RotateActionClient::send_cancel_request(
  const GoalHandle::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    return;
  }

  const auto weak_self = weak_from_this();

  try {
    (void)client_->async_cancel_goal(
      goal_handle,
      [weak_self](
        const rclcpp_action::Client<Action>::
        CancelResponse::SharedPtr & response)
      {
        if (const auto self = weak_self.lock()) {
          self->handle_cancel_response(
            response);
        }
      });
  } catch (const std::exception &) {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      snapshot_.reason =
        "scan360_cancel_request_failed";
    }

    notify_update();
  }
}

void Scan360RotateActionClient::handle_goal_response(
  const GoalHandle::SharedPtr & goal_handle)
{
  bool send_cancel = false;
  bool cancel_orphan = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!goal_handle) {
      if (
        snapshot_.state == State::kCanceling &&
        snapshot_.cancel_requested)
      {
        set_terminal_locked(
          State::kCanceled,
          cancel_reason_);
      } else if (  // NOLINT(readability/braces)
        snapshot_.state ==
        State::kWaitingForGoalResponse)
      {
        set_terminal_locked(
          State::kRejected,
          "scan360_goal_rejected");
      }
    } else if (  // NOLINT(readability/braces)
      snapshot_.state ==
      State::kWaitingForGoalResponse)
    {
      active_goal_ = goal_handle;

      snapshot_.state = State::kActive;
      snapshot_.reason =
        "scan360_rotation_active";

      const auto now = SteadyClock::now();

      last_feedback_at_ = now;

      execution_deadline_ =
        now +
        seconds_to_duration(
        requested_max_duration_sec_ +
        options_.execution_grace_sec);
    } else if (  // NOLINT(readability/braces)
      snapshot_.state == State::kCanceling &&
      snapshot_.cancel_requested)
    {
      active_goal_ = goal_handle;
      send_cancel = true;
    } else {
      cancel_orphan = true;
    }
  }

  notify_update();

  if (send_cancel) {
    send_cancel_request(goal_handle);
  } else if (cancel_orphan) {
    try {
      (void)client_->async_cancel_goal(
        goal_handle);
    } catch (const std::exception &) {
    }
  }
}

void Scan360RotateActionClient::handle_feedback(
  GoalHandle::SharedPtr goal_handle,
  const std::shared_ptr<
    const Action::Feedback> feedback)
{
  if (!goal_handle || !feedback) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (
      snapshot_.state != State::kActive ||
      !active_goal_ ||
      goal_handle != active_goal_)
    {
      return;
    }

    snapshot_.current_yaw_rad =
      feedback->current_yaw_rad;

    snapshot_.target_yaw_rad =
      feedback->target_yaw_rad;

    snapshot_.error_rad =
      feedback->error_rad;

    snapshot_.commanded_wz_rad_s =
      feedback->commanded_wz_rad_s;

    snapshot_.elapsed_sec =
      feedback->elapsed_sec;

    snapshot_.safety_stop_active =
      feedback->safety_stop_active;

    snapshot_.reason =
      feedback->state.empty() ?
      "scan360_rotation_active" :
      feedback->state;

    last_feedback_at_ = SteadyClock::now();
  }

  notify_update();
}

void Scan360RotateActionClient::handle_result(
  const GoalHandle::WrappedResult &
  wrapped_result)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (snapshot_.terminal) {
      return;
    }

    if (wrapped_result.result) {
      snapshot_.current_yaw_rad =
        wrapped_result.result->final_yaw_rad;

      snapshot_.error_rad =
        wrapped_result.result->final_error_rad;
    }

    const std::string action_reason =
      wrapped_result.result &&
      !wrapped_result.result->reason.empty() ?
      wrapped_result.result->reason :
      "scan360_action_result_missing_reason";

    switch (wrapped_result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        if (
          wrapped_result.result &&
          wrapped_result.result->success &&
          action_reason == "goal_reached")
        {
          set_terminal_locked(
            State::kSucceeded,
            action_reason);
        } else {
          set_terminal_locked(
            State::kAborted,
            action_reason);
        }
        break;

      case rclcpp_action::ResultCode::ABORTED:
        set_terminal_locked(
          State::kAborted,
          action_reason);
        break;

      case rclcpp_action::ResultCode::CANCELED:
        if (
          snapshot_.cancel_requested &&
          cancel_terminal_state_ !=
          State::kCanceled)
        {
          set_terminal_locked(
            cancel_terminal_state_,
            cancel_reason_);
        } else {
          set_terminal_locked(
            State::kCanceled,
            action_reason);
        }
        break;

      case rclcpp_action::ResultCode::UNKNOWN:
      default:
        set_terminal_locked(
          State::kFailed,
          "scan360_action_unknown_result");
        break;
    }
  }

  notify_update();
}

void
Scan360RotateActionClient::handle_cancel_response(
  const rclcpp_action::Client<Action>::
  CancelResponse::SharedPtr & response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (
      snapshot_.state != State::kCanceling ||
      snapshot_.terminal)
    {
      return;
    }

    if (
      response &&
      !response->goals_canceling.empty())
    {
      snapshot_.cancel_acknowledged = true;
      snapshot_.reason = cancel_reason_;
    } else {
      snapshot_.cancel_acknowledged = false;
      snapshot_.reason =
        "scan360_cancel_rejected";
    }
  }

  notify_update();
}

void Scan360RotateActionClient::set_terminal_locked(
  const State state,
  const std::string & reason)
{
  snapshot_.state = state;
  snapshot_.active = false;
  snapshot_.terminal = true;
  snapshot_.reason = reason;

  active_goal_.reset();
}

bool Scan360RotateActionClient::busy_locked()
const noexcept
{
  return
    snapshot_.state ==
    State::kWaitingForServer ||

    snapshot_.state ==
    State::kWaitingForGoalResponse ||

    snapshot_.state ==
    State::kActive ||

    snapshot_.state ==
    State::kCanceling;
}

void Scan360RotateActionClient::notify_update()
{
  UpdateCallback callback;
  Snapshot current_snapshot;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    callback = update_callback_;
    current_snapshot = snapshot_;
  }

  if (callback) {
    callback(current_snapshot);
  }
}

}  // namespace savo_mapping
