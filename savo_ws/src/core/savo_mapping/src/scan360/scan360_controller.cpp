#include "savo_mapping/scan360_controller.hpp"

#include <cmath>
#include <utility>

namespace savo_mapping::scan360
{

std::string_view to_string(
  ControllerState state)
{
  switch (state) {
    case ControllerState::Idle:
      return "idle";

    case ControllerState::Ready:
      return "ready";

    case ControllerState::CommandPending:
      return "command_pending";

    case ControllerState::Rotating:
      return "rotating";

    case ControllerState::Settling:
      return "settling";

    case ControllerState::Canceling:
      return "canceling";

    case ControllerState::Paused:
      return "paused";

    case ControllerState::Complete:
      return "complete";

    case ControllerState::Canceled:
      return "canceled";

    case ControllerState::Failed:
      return "failed";
  }

  return "failed";
}

std::string_view to_string(
  ControllerAction action)
{
  switch (action) {
    case ControllerAction::None:
      return "none";

    case ControllerAction::
      IssueRotationRequest:
      return "issue_rotation_request";

    case ControllerAction::RequestCancel:
      return "request_cancel";

    case ControllerAction::
      StartSettleTimer:
      return "start_settle_timer";

    case ControllerAction::ScanComplete:
      return "scan_complete";
  }

  return "none";
}

ControllerDecision
Scan360Controller::load_plan(
  Scan360Plan plan)
{
  if (!plan_is_valid(plan)) {
    return fail("invalid_plan");
  }

  plan_ = std::move(plan);

  state_ = ControllerState::Ready;
  current_target_index_ = 0;

  plan_loaded_ = true;
  resume_after_cancel_ = false;

  return make_decision(
    ControllerAction::None,
    "plan_loaded");
}

ControllerDecision
Scan360Controller::handle(
  ControllerEvent event)
{
  switch (event) {
    case ControllerEvent::Reset:
      current_target_index_ = 0;
      resume_after_cancel_ = false;

      state_ =
        plan_loaded_ ?
        ControllerState::Ready :
        ControllerState::Idle;

      return make_decision(
        ControllerAction::None,
        plan_loaded_ ?
        "reset_ready" :
        "reset_idle");

    case ControllerEvent::Start:
      if (
        state_ == ControllerState::Ready ||
        state_ == ControllerState::Paused)
      {
        state_ =
          ControllerState::CommandPending;

        return make_decision(
          ControllerAction::
          IssueRotationRequest,
          "rotation_request_required");
      }

      break;

    case ControllerEvent::MotionAccepted:
      if (
        state_ ==
        ControllerState::CommandPending)
      {
        state_ =
          ControllerState::Rotating;

        return make_decision(
          ControllerAction::None,
          "rotation_active");
      }

      break;

    case ControllerEvent::TargetReached:
      if (
        state_ ==
        ControllerState::CommandPending ||
        state_ ==
        ControllerState::Rotating)
      {
        state_ =
          ControllerState::Settling;

        return make_decision(
          ControllerAction::
          StartSettleTimer,
          "target_reached");
      }

      break;

    case ControllerEvent::SettleComplete:
      if (
        state_ ==
        ControllerState::Settling)
      {
        if (
          current_target_index_ + 1 >=
          plan_.targets.size())
        {
          state_ =
            ControllerState::Complete;

          return make_decision(
            ControllerAction::ScanComplete,
            "scan_complete");
        }

        ++current_target_index_;

        state_ =
          ControllerState::CommandPending;

        return make_decision(
          ControllerAction::
          IssueRotationRequest,
          "next_rotation_request_required");
      }

      break;

    case ControllerEvent::AuthorityLost:
      return pause_for_interlock(
        "authority_lost");

    case ControllerEvent::SafetyStop:
      return pause_for_interlock(
        "safety_stop");

    case ControllerEvent::OperatorCancel:
      resume_after_cancel_ = false;

      if (motion_may_be_active()) {
        state_ =
          ControllerState::Canceling;

        return make_decision(
          ControllerAction::RequestCancel,
          "operator_cancel_requested");
      }

      state_ =
        ControllerState::Canceled;

      return make_decision(
        ControllerAction::None,
        "operator_canceled");

    case ControllerEvent::
      CancelAcknowledged:
      if (
        state_ ==
        ControllerState::Canceling)
      {
        state_ =
          resume_after_cancel_ ?
          ControllerState::Paused :
          ControllerState::Canceled;

        resume_after_cancel_ = false;

        return make_decision(
          ControllerAction::None,
          state_ ==
          ControllerState::Paused ?
          "cancel_acknowledged_paused" :
          "cancel_acknowledged_canceled");
      }

      break;

    case ControllerEvent::CancelRejected:
      if (
        state_ ==
        ControllerState::Canceling)
      {
        return fail(
          "cancel_rejected");
      }

      break;

    case ControllerEvent::MotionRejected:
      if (
        state_ ==
        ControllerState::CommandPending)
      {
        return fail(
          "motion_rejected");
      }

      break;

    case ControllerEvent::MotionFailed:
      if (
        state_ ==
        ControllerState::CommandPending ||
        state_ ==
        ControllerState::Rotating)
      {
        return fail(
          "motion_failed");
      }

      break;
  }

  return make_decision(
    ControllerAction::None,
    "event_ignored_in_state:" +
    std::string{to_string(state_)});
}

ControllerState
Scan360Controller::state() const
{
  return state_;
}

std::size_t
Scan360Controller::current_target_index()
const
{
  return current_target_index_;
}

bool Scan360Controller::has_plan() const
{
  return plan_loaded_;
}

bool Scan360Controller::plan_is_valid(
  const Scan360Plan & plan) const
{
  if (plan.targets.empty()) {
    return false;
  }

  for (
    std::size_t index = 0;
    index < plan.targets.size();
    ++index)
  {
    const auto & target =
      plan.targets[index];

    if (
      target.index != index ||
      !std::isfinite(
        target.relative_yaw_rad) ||
      !std::isfinite(
        target.normalized_yaw_rad))
    {
      return false;
    }

    const bool should_be_final =
      index + 1 ==
      plan.targets.size();

    if (
      target.final_target !=
      should_be_final)
    {
      return false;
    }
  }

  return true;
}

bool Scan360Controller::motion_may_be_active() const
{
  return
    state_ ==
    ControllerState::CommandPending ||
    state_ ==
    ControllerState::Rotating;
}

ControllerDecision
Scan360Controller::make_decision(
  ControllerAction action,
  std::string reason) const
{
  ControllerDecision decision;

  decision.state = state_;
  decision.action = action;
  decision.reason = std::move(reason);

  if (
    plan_loaded_ &&
    current_target_index_ <
    plan_.targets.size() &&
    state_ != ControllerState::Complete)
  {
    decision.target =
      plan_.targets[
      current_target_index_];
  }

  return decision;
}

ControllerDecision
Scan360Controller::pause_for_interlock(
  std::string reason)
{
  resume_after_cancel_ = true;

  if (motion_may_be_active()) {
    state_ =
      ControllerState::Canceling;

    return make_decision(
      ControllerAction::RequestCancel,
      std::move(reason));
  }

  if (
    state_ == ControllerState::Ready ||
    state_ == ControllerState::Settling ||
    state_ == ControllerState::Paused)
  {
    state_ =
      ControllerState::Paused;

    return make_decision(
      ControllerAction::None,
      std::move(reason));
  }

  resume_after_cancel_ = false;

  return make_decision(
    ControllerAction::None,
    "interlock_ignored_in_state:" +
    std::string{to_string(state_)});
}

ControllerDecision
Scan360Controller::fail(
  std::string reason)
{
  state_ = ControllerState::Failed;
  resume_after_cancel_ = false;

  return make_decision(
    ControllerAction::None,
    std::move(reason));
}

}  // namespace savo_mapping::scan360
