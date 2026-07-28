#include "savo_mapping/scan360_orchestrator.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace savo_mapping::scan360
{

namespace
{

std::string terminal_reason_or(
  const RotationClientSnapshot & snapshot,
  const std::string & fallback)
{
  if (
    snapshot.reason.empty() ||
    snapshot.reason == "idle")
  {
    return fallback;
  }

  return snapshot.reason;
}

}  // namespace

Scan360Orchestrator::Scan360Orchestrator(
  Scan360RotationCallbacks callbacks,
  const double rotation_duration_sec)
: callbacks_(std::move(callbacks)),
  rotation_duration_sec_(rotation_duration_sec)
{
  if (
    !callbacks_.request_rotation ||
    !callbacks_.request_cancel ||
    !callbacks_.tick ||
    !callbacks_.snapshot)
  {
    throw std::invalid_argument(
            "scan360 rotation callbacks are incomplete");
  }

  if (
    !std::isfinite(rotation_duration_sec_) ||
    rotation_duration_sec_ <= 0.0)
  {
    throw std::invalid_argument(
            "scan360 rotation duration must be positive and finite");
  }
}

void Scan360Orchestrator::dispatch(
  const ControllerDecision & decision)
{
  switch (decision.action) {
    case ControllerAction::IssueRotationRequest:
      dispatch_rotation(decision);
      break;

    case ControllerAction::RequestCancel:
      dispatch_cancel();
      break;

    case ControllerAction::None:
    case ControllerAction::StartSettleTimer:
    case ControllerAction::ScanComplete:
      break;
  }
}

void Scan360Orchestrator::tick()
{
  callbacks_.tick();
  observe(callbacks_.snapshot());
}

std::vector<ControllerEvent>
Scan360Orchestrator::take_events()
{
  std::vector<ControllerEvent> events;

  events.swap(events_);
  return events;
}

void Scan360Orchestrator::reset()
{
  pending_operation_ = PendingOperation::None;
  motion_accepted_emitted_ = false;
  terminal_event_emitted_ = false;
  events_.clear();
  last_reason_ = "idle";
}

std::string_view
Scan360Orchestrator::last_reason() const
{
  return last_reason_;
}

void Scan360Orchestrator::dispatch_rotation(
  const ControllerDecision & decision)
{
  pending_operation_ = PendingOperation::Rotation;
  motion_accepted_emitted_ = false;
  terminal_event_emitted_ = false;
  last_reason_ = "rotation_pending";

  if (!decision.target.has_value()) {
    emit_terminal(
      ControllerEvent::MotionRejected,
      "scan360_target_missing");
    return;
  }

  const double target_yaw_rad =
    decision.target->normalized_yaw_rad;

  if (!std::isfinite(target_yaw_rad)) {
    emit_terminal(
      ControllerEvent::MotionRejected,
      "scan360_target_yaw_not_finite");
    return;
  }

  if (
    !callbacks_.request_rotation(
      target_yaw_rad,
      rotation_duration_sec_))
  {
    const auto snapshot = callbacks_.snapshot();

    emit_terminal(
      ControllerEvent::MotionRejected,
      terminal_reason_or(
        snapshot,
        "scan360_rotation_request_rejected"));
    return;
  }

  observe(callbacks_.snapshot());
}

void Scan360Orchestrator::dispatch_cancel()
{
  pending_operation_ = PendingOperation::Cancel;
  terminal_event_emitted_ = false;
  last_reason_ = "cancel_pending";

  if (!callbacks_.request_cancel()) {
    const auto snapshot = callbacks_.snapshot();

    emit_terminal(
      ControllerEvent::CancelRejected,
      terminal_reason_or(
        snapshot,
        "scan360_cancel_request_rejected"));
    return;
  }

  observe(callbacks_.snapshot());
}

void Scan360Orchestrator::observe(
  const RotationClientSnapshot & snapshot)
{
  if (
    pending_operation_ == PendingOperation::None ||
    terminal_event_emitted_)
  {
    return;
  }

  switch (snapshot.state) {
    case RotationClientState::Idle:
    case RotationClientState::Pending:
    case RotationClientState::Canceling:
      break;

    case RotationClientState::Active:
      if (
        pending_operation_ ==
        PendingOperation::Rotation)
      {
        emit_motion_accepted();
      }
      break;

    case RotationClientState::Succeeded:
      if (
        pending_operation_ ==
        PendingOperation::Rotation)
      {
        emit_motion_accepted();
        emit_terminal(
          ControllerEvent::TargetReached,
          terminal_reason_or(
            snapshot,
            "scan360_target_reached"));
      }
      break;

    case RotationClientState::Canceled:
      if (
        pending_operation_ ==
        PendingOperation::Cancel)
      {
        emit_terminal(
          ControllerEvent::CancelAcknowledged,
          terminal_reason_or(
            snapshot,
            "scan360_cancel_acknowledged"));
      }
      break;

    case RotationClientState::Rejected:
      if (
        pending_operation_ ==
        PendingOperation::Cancel)
      {
        emit_terminal(
          ControllerEvent::CancelRejected,
          terminal_reason_or(
            snapshot,
            "scan360_cancel_rejected"));
      } else {
        emit_terminal(
          ControllerEvent::MotionRejected,
          terminal_reason_or(
            snapshot,
            "scan360_motion_rejected"));
      }
      break;

    case RotationClientState::Failed:
      if (
        pending_operation_ ==
        PendingOperation::Cancel)
      {
        emit_terminal(
          ControllerEvent::CancelRejected,
          terminal_reason_or(
            snapshot,
            "scan360_cancel_failed"));
      } else {
        emit_terminal(
          ControllerEvent::MotionFailed,
          terminal_reason_or(
            snapshot,
            "scan360_motion_failed"));
      }
      break;
  }
}

void Scan360Orchestrator::emit_motion_accepted()
{
  if (motion_accepted_emitted_) {
    return;
  }

  events_.push_back(
    ControllerEvent::MotionAccepted);

  motion_accepted_emitted_ = true;
}

void Scan360Orchestrator::emit_terminal(
  const ControllerEvent event,
  const std::string & reason)
{
  if (terminal_event_emitted_) {
    return;
  }

  events_.push_back(event);
  terminal_event_emitted_ = true;
  last_reason_ = reason;
}

}  // namespace savo_mapping::scan360
