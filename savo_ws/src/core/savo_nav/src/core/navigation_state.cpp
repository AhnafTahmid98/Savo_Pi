// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/navigation_state.hpp"

namespace savo_nav
{

std::string_view NavigationStateContract::ToString(
  const NavigationState state) noexcept
{
  switch (state) {
    case NavigationState::kOffline:
      return "offline";

    case NavigationState::kStarting:
      return "starting";

    case NavigationState::kIdle:
      return "idle";

    case NavigationState::kValidatingGoal:
      return "validating_goal";

    case NavigationState::kWaitingForControl:
      return "waiting_for_control";

    case NavigationState::kPlanning:
      return "planning";

    case NavigationState::kNavigating:
      return "navigating";

    case NavigationState::kRecovering:
      return "recovering";

    case NavigationState::kCanceling:
      return "canceling";

    case NavigationState::kSucceeded:
      return "succeeded";

    case NavigationState::kCanceled:
      return "canceled";

    case NavigationState::kRejected:
      return "rejected";

    case NavigationState::kFailed:
      return "failed";
  }

  return "unknown";
}

bool NavigationStateContract::IsTerminal(
  const NavigationState state) noexcept
{
  return
    state == NavigationState::kSucceeded ||
    state == NavigationState::kCanceled ||
    state == NavigationState::kRejected ||
    state == NavigationState::kFailed;
}

bool NavigationStateContract::AcceptsNewGoal(
  const NavigationState state) noexcept
{
  return state == NavigationState::kIdle;
}

bool NavigationStateContract::HasActiveGoal(
  const NavigationState state) noexcept
{
  return
    state == NavigationState::kValidatingGoal ||
    state == NavigationState::kWaitingForControl ||
    state == NavigationState::kPlanning ||
    state == NavigationState::kNavigating ||
    state == NavigationState::kRecovering ||
    state == NavigationState::kCanceling;
}

bool NavigationStateContract::IsTransitionAllowed(
  const NavigationState from,
  const NavigationState to) noexcept
{
  if (from == to) {
    return true;
  }

  switch (from) {
    case NavigationState::kOffline:
      return to == NavigationState::kStarting;

    case NavigationState::kStarting:
      return
        to == NavigationState::kIdle ||
        to == NavigationState::kOffline ||
        to == NavigationState::kFailed;

    case NavigationState::kIdle:
      return
        to == NavigationState::kValidatingGoal ||
        to == NavigationState::kOffline ||
        to == NavigationState::kFailed;

    case NavigationState::kValidatingGoal:
      return
        to == NavigationState::kWaitingForControl ||
        to == NavigationState::kCanceling ||
        to == NavigationState::kRejected ||
        to == NavigationState::kFailed;

    case NavigationState::kWaitingForControl:
      return
        to == NavigationState::kPlanning ||
        to == NavigationState::kCanceling ||
        to == NavigationState::kRejected ||
        to == NavigationState::kFailed;

    case NavigationState::kPlanning:
      return
        to == NavigationState::kNavigating ||
        to == NavigationState::kRecovering ||
        to == NavigationState::kCanceling ||
        to == NavigationState::kFailed;

    case NavigationState::kNavigating:
      return
        to == NavigationState::kRecovering ||
        to == NavigationState::kCanceling ||
        to == NavigationState::kSucceeded ||
        to == NavigationState::kFailed;

    case NavigationState::kRecovering:
      return
        to == NavigationState::kPlanning ||
        to == NavigationState::kNavigating ||
        to == NavigationState::kCanceling ||
        to == NavigationState::kFailed;

    case NavigationState::kCanceling:
      return
        to == NavigationState::kCanceled ||
        to == NavigationState::kSucceeded ||
        to == NavigationState::kFailed;

    case NavigationState::kSucceeded:
    case NavigationState::kCanceled:
    case NavigationState::kRejected:
    case NavigationState::kFailed:
      return
        to == NavigationState::kIdle ||
        to == NavigationState::kOffline;
  }

  return false;
}

}  // namespace savo_nav
