#pragma once

#include "savo_mapping/scan360_planner.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

namespace savo_mapping::scan360
{

enum class ControllerState
{
  Idle,
  Ready,
  CommandPending,
  Rotating,
  Settling,
  Canceling,
  Paused,
  Complete,
  Canceled,
  Failed
};

enum class ControllerAction
{
  None,
  IssueRotationRequest,
  RequestCancel,
  StartSettleTimer,
  ScanComplete
};

enum class ControllerEvent
{
  Start,
  MotionAccepted,
  TargetReached,
  SettleComplete,
  AuthorityLost,
  SafetyStop,
  OperatorCancel,
  CancelAcknowledged,
  CancelRejected,
  MotionRejected,
  MotionFailed,
  Reset
};

struct ControllerDecision
{
  ControllerState state{
    ControllerState::Idle};

  ControllerAction action{
    ControllerAction::None};

  std::optional<Scan360Target> target;

  std::string reason{"idle"};
};

std::string_view to_string(
  ControllerState state);

std::string_view to_string(
  ControllerAction action);

class Scan360Controller
{
public:
  ControllerDecision load_plan(
    Scan360Plan plan);

  ControllerDecision handle(
    ControllerEvent event);

  ControllerState state() const;

  std::size_t current_target_index() const;

  bool has_plan() const;

private:
  bool plan_is_valid(
    const Scan360Plan & plan) const;

  bool motion_may_be_active() const;

  ControllerDecision make_decision(
    ControllerAction action,
    std::string reason) const;

  ControllerDecision pause_for_interlock(
    std::string reason);

  ControllerDecision fail(
    std::string reason);

  Scan360Plan plan_;

  ControllerState state_{
    ControllerState::Idle};

  std::size_t current_target_index_{0};

  bool plan_loaded_{false};
  bool resume_after_cancel_{false};
};

}  // namespace savo_mapping::scan360
