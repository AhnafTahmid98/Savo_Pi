#pragma once

#include <cstdint>
#include <string>

namespace savo_mapping::exploration
{

inline constexpr char kExplorationActionName[] =
  "/savo_nav/exploration/navigate_to_pose";

inline constexpr char kSelectedGoalTopic[] =
  "/savo_mapping/exploration/selected_goal";

inline constexpr char kGoalStateTopic[] =
  "/savo_mapping/exploration_goal/state";

inline constexpr char kGoalStatusTopic[] =
  "/savo_mapping/exploration_goal/status";

inline constexpr char kGoalFeedbackTopic[] =
  "/savo_mapping/exploration_goal/feedback";

inline constexpr char kGoalCancelService[] =
  "/savo_mapping/exploration_goal/cancel";

enum class GoalHandoffState
{
  kIdle,
  kWaitingForServer,
  kSending,
  kAccepted,
  kExecuting,
  kCanceling,
  kSucceeded,
  kRejected,
  kAborted,
  kCanceled,
  kTimedOut,
  kError,
};

struct GoalTransition
{
  bool accepted{false};

  GoalHandoffState previous{
    GoalHandoffState::kIdle};

  GoalHandoffState current{
    GoalHandoffState::kIdle};

  std::string request_id;
  std::string reason;
};

std::string to_string(
  GoalHandoffState state);

bool is_active(
  GoalHandoffState state);

bool is_terminal(
  GoalHandoffState state);

bool transition_allowed(
  GoalHandoffState from,
  GoalHandoffState to);

class GoalHandoffMachine
{
public:
  GoalHandoffMachine() = default;

  GoalHandoffState state() const noexcept;

  const std::string & request_id() const noexcept;

  const std::string & reason() const noexcept;

  std::uint64_t sequence() const noexcept;

  GoalTransition begin(
    const std::string & request_id);

  GoalTransition mark_server_available();

  GoalTransition mark_accepted();

  GoalTransition mark_executing();

  GoalTransition request_cancel(
    const std::string & reason =
    "cancel_requested");

  GoalTransition mark_succeeded();

  GoalTransition mark_rejected(
    const std::string & reason);

  GoalTransition mark_aborted(
    const std::string & reason);

  GoalTransition mark_canceled(
    const std::string & reason);

  GoalTransition mark_timed_out(
    const std::string & reason);

  GoalTransition mark_error(
    const std::string & reason);

  GoalTransition reset();

private:
  GoalTransition transition(
    GoalHandoffState next,
    const std::string & reason);

  GoalTransition rejected_transition(
    const std::string & reason) const;

  GoalHandoffState state_{
    GoalHandoffState::kIdle};

  std::string request_id_;
  std::string reason_{"idle"};

  std::uint64_t sequence_{0};
};

}  // namespace savo_mapping::exploration
