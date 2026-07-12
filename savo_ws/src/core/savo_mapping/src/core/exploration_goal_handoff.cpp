#include "savo_mapping/exploration_goal_handoff.hpp"

#include <utility>

namespace savo_mapping::exploration
{

std::string to_string(
  const GoalHandoffState state)
{
  switch (state) {
    case GoalHandoffState::kIdle:
      return "idle";

    case GoalHandoffState::kWaitingForServer:
      return "waiting_for_savo_nav";

    case GoalHandoffState::kSending:
      return "sending";

    case GoalHandoffState::kAccepted:
      return "accepted";

    case GoalHandoffState::kExecuting:
      return "executing";

    case GoalHandoffState::kCanceling:
      return "canceling";

    case GoalHandoffState::kSucceeded:
      return "succeeded";

    case GoalHandoffState::kRejected:
      return "rejected";

    case GoalHandoffState::kAborted:
      return "aborted";

    case GoalHandoffState::kCanceled:
      return "canceled";

    case GoalHandoffState::kTimedOut:
      return "timed_out";

    case GoalHandoffState::kError:
      return "error";
  }

  return "unknown";
}

bool is_active(
  const GoalHandoffState state)
{
  switch (state) {
    case GoalHandoffState::kWaitingForServer:
    case GoalHandoffState::kSending:
    case GoalHandoffState::kAccepted:
    case GoalHandoffState::kExecuting:
    case GoalHandoffState::kCanceling:
      return true;

    default:
      return false;
  }
}

bool is_terminal(
  const GoalHandoffState state)
{
  switch (state) {
    case GoalHandoffState::kSucceeded:
    case GoalHandoffState::kRejected:
    case GoalHandoffState::kAborted:
    case GoalHandoffState::kCanceled:
    case GoalHandoffState::kTimedOut:
    case GoalHandoffState::kError:
      return true;

    default:
      return false;
  }
}

bool transition_allowed(
  const GoalHandoffState from,
  const GoalHandoffState to)
{
  if (from == to) {
    return true;
  }

  if ((from == GoalHandoffState::kIdle ||
       is_terminal(from)) &&
      to == GoalHandoffState::kWaitingForServer)
  {
    return true;
  }

  if (is_terminal(from) &&
      to == GoalHandoffState::kIdle)
  {
    return true;
  }

  switch (from) {
    case GoalHandoffState::kIdle:
      return to == GoalHandoffState::kError;

    case GoalHandoffState::kWaitingForServer:
      return
        to == GoalHandoffState::kSending ||
        to == GoalHandoffState::kCanceled ||
        to == GoalHandoffState::kTimedOut ||
        to == GoalHandoffState::kError;

    case GoalHandoffState::kSending:
      return
        to == GoalHandoffState::kAccepted ||
        to == GoalHandoffState::kRejected ||
        to == GoalHandoffState::kCanceled ||
        to == GoalHandoffState::kTimedOut ||
        to == GoalHandoffState::kError;

    case GoalHandoffState::kAccepted:
      return
        to == GoalHandoffState::kExecuting ||
        to == GoalHandoffState::kCanceling ||
        to == GoalHandoffState::kSucceeded ||
        to == GoalHandoffState::kAborted ||
        to == GoalHandoffState::kCanceled ||
        to == GoalHandoffState::kTimedOut ||
        to == GoalHandoffState::kError;

    case GoalHandoffState::kExecuting:
      return
        to == GoalHandoffState::kCanceling ||
        to == GoalHandoffState::kSucceeded ||
        to == GoalHandoffState::kAborted ||
        to == GoalHandoffState::kCanceled ||
        to == GoalHandoffState::kTimedOut ||
        to == GoalHandoffState::kError;

    case GoalHandoffState::kCanceling:
      return
        to == GoalHandoffState::kCanceled ||
        to == GoalHandoffState::kAborted ||
        to == GoalHandoffState::kTimedOut ||
        to == GoalHandoffState::kError;

    case GoalHandoffState::kSucceeded:
    case GoalHandoffState::kRejected:
    case GoalHandoffState::kAborted:
    case GoalHandoffState::kCanceled:
    case GoalHandoffState::kTimedOut:
    case GoalHandoffState::kError:
      return false;
  }

  return false;
}

GoalHandoffState
GoalHandoffMachine::state() const noexcept
{
  return state_;
}

const std::string &
GoalHandoffMachine::request_id() const noexcept
{
  return request_id_;
}

const std::string &
GoalHandoffMachine::reason() const noexcept
{
  return reason_;
}

std::uint64_t
GoalHandoffMachine::sequence() const noexcept
{
  return sequence_;
}

GoalTransition GoalHandoffMachine::begin(
  const std::string & request_id)
{
  if (request_id.empty()) {
    return rejected_transition(
      "request_id_empty");
  }

  if (is_active(state_)) {
    return rejected_transition(
      "goal_already_active");
  }

  request_id_ = request_id;
  ++sequence_;

  return transition(
    GoalHandoffState::kWaitingForServer,
    "goal_received");
}

GoalTransition
GoalHandoffMachine::mark_server_available()
{
  return transition(
    GoalHandoffState::kSending,
    "savo_nav_available");
}

GoalTransition
GoalHandoffMachine::mark_accepted()
{
  return transition(
    GoalHandoffState::kAccepted,
    "goal_accepted");
}

GoalTransition
GoalHandoffMachine::mark_executing()
{
  return transition(
    GoalHandoffState::kExecuting,
    "goal_executing");
}

GoalTransition
GoalHandoffMachine::request_cancel(
  const std::string & reason)
{
  if (state_ ==
      GoalHandoffState::kWaitingForServer ||
      state_ ==
      GoalHandoffState::kSending)
  {
    return transition(
      GoalHandoffState::kCanceled,
      "canceled_before_acceptance");
  }

  return transition(
    GoalHandoffState::kCanceling,
    reason.empty() ?
    "cancel_requested" :
    reason);
}

GoalTransition
GoalHandoffMachine::mark_succeeded()
{
  return transition(
    GoalHandoffState::kSucceeded,
    "goal_succeeded");
}

GoalTransition
GoalHandoffMachine::mark_rejected(
  const std::string & reason)
{
  return transition(
    GoalHandoffState::kRejected,
    reason.empty() ?
    "goal_rejected" :
    reason);
}

GoalTransition
GoalHandoffMachine::mark_aborted(
  const std::string & reason)
{
  return transition(
    GoalHandoffState::kAborted,
    reason.empty() ?
    "goal_aborted" :
    reason);
}

GoalTransition
GoalHandoffMachine::mark_canceled(
  const std::string & reason)
{
  return transition(
    GoalHandoffState::kCanceled,
    reason.empty() ?
    "goal_canceled" :
    reason);
}

GoalTransition
GoalHandoffMachine::mark_timed_out(
  const std::string & reason)
{
  return transition(
    GoalHandoffState::kTimedOut,
    reason.empty() ?
    "goal_timed_out" :
    reason);
}

GoalTransition
GoalHandoffMachine::mark_error(
  const std::string & reason)
{
  return transition(
    GoalHandoffState::kError,
    reason.empty() ?
    "goal_handoff_error" :
    reason);
}

GoalTransition GoalHandoffMachine::reset()
{
  if (state_ == GoalHandoffState::kIdle) {
    return GoalTransition{
      true,
      state_,
      state_,
      request_id_,
      "already_idle"};
  }

  if (!is_terminal(state_)) {
    return rejected_transition(
      "cannot_reset_active_goal");
  }

  const GoalHandoffState previous =
    state_;

  state_ = GoalHandoffState::kIdle;
  request_id_.clear();
  reason_ = "idle";

  return GoalTransition{
    true,
    previous,
    state_,
    request_id_,
    reason_};
}

GoalTransition GoalHandoffMachine::transition(
  const GoalHandoffState next,
  const std::string & reason)
{
  if (!transition_allowed(
      state_,
      next))
  {
    return rejected_transition(
      "invalid_transition:" +
      to_string(state_) +
      "_to_" +
      to_string(next));
  }

  const GoalHandoffState previous =
    state_;

  state_ = next;
  reason_ = reason;

  return GoalTransition{
    true,
    previous,
    state_,
    request_id_,
    reason_};
}

GoalTransition
GoalHandoffMachine::rejected_transition(
  const std::string & reason) const
{
  return GoalTransition{
    false,
    state_,
    state_,
    request_id_,
    reason};
}

}  // namespace savo_mapping::exploration
