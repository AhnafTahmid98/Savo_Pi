#pragma once

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include "savo_control/command_limiter.hpp"
#include "savo_control/control_math.hpp"
#include "savo_control/recovery_types.hpp"

namespace savo_control
{

struct RecoveryManagerConfig
{
  double backup_vx{-0.06};
  double rotate_wz{0.25};

  double backup_duration_s{1.0};
  double rotate_duration_s{1.0};
  double stop_hold_s{0.20};
  double max_recovery_duration_s{8.0};

  RecoveryAction default_action{RecoveryAction::BACKUP_THEN_LEFT};

  CommandLimits output_limits{};

  bool enabled{true};
  bool stop_before_motion{true};
  bool stop_after_motion{true};
  bool allow_rotate_left{true};
  bool allow_rotate_right{true};

  RecoveryManagerConfig sanitized() const
  {
    RecoveryManagerConfig out = *this;

    out.backup_vx = -std::abs(ControlMath::finite_or_zero(out.backup_vx));
    out.rotate_wz = std::abs(ControlMath::finite_or_zero(out.rotate_wz));

    if (!std::isfinite(out.backup_duration_s) || out.backup_duration_s < 0.0) {
      out.backup_duration_s = 1.0;
    }

    if (!std::isfinite(out.rotate_duration_s) || out.rotate_duration_s < 0.0) {
      out.rotate_duration_s = 1.0;
    }

    if (!std::isfinite(out.stop_hold_s) || out.stop_hold_s < 0.0) {
      out.stop_hold_s = 0.20;
    }

    if (!std::isfinite(out.max_recovery_duration_s) || out.max_recovery_duration_s < 0.1) {
      out.max_recovery_duration_s = 8.0;
    }

    out.output_limits = out.output_limits.sanitized();

    return out;
  }
};

struct RecoveryManagerState
{
  RecoveryState state{RecoveryState::IDLE};
  RecoveryTrigger trigger{RecoveryTrigger::NONE};
  RecoveryAction action{RecoveryAction::NONE};

  double start_s{0.0};
  double phase_start_s{0.0};
  double last_update_s{0.0};

  int phase_index{0};

  bool requested{false};
  bool active{false};
  bool completed{false};
  bool failed{false};
  bool blocked{false};
};

struct RecoveryManagerResult
{
  RecoveryState state{RecoveryState::IDLE};
  RecoveryTrigger trigger{RecoveryTrigger::NONE};
  RecoveryAction action{RecoveryAction::NONE};

  TwistCommand command{};

  bool active{false};
  bool finished{false};
  bool recovery_request_active{false};
  bool command_valid{true};

  double elapsed_s{0.0};
  double phase_elapsed_s{0.0};

  std::string reason{"idle"};
};

class RecoveryManager
{
public:
  RecoveryManager()
  : config_(RecoveryManagerConfig{}.sanitized()),
    limiter_(config_.output_limits)
  {
  }

  explicit RecoveryManager(const RecoveryManagerConfig & config)
  : config_(config.sanitized()),
    limiter_(config_.output_limits)
  {
  }

  void set_config(const RecoveryManagerConfig & config)
  {
    config_ = config.sanitized();
    limiter_.set_limits(config_.output_limits);
  }

  const RecoveryManagerConfig & config() const
  {
    return config_;
  }

  const RecoveryManagerState & state() const
  {
    return state_;
  }

  void reset()
  {
    state_ = RecoveryManagerState{};
  }

  bool request(
    const double now_s,
    const RecoveryTrigger trigger = RecoveryTrigger::MANUAL_REQUEST,
    const RecoveryAction action = RecoveryAction::NONE)
  {
    if (!config_.enabled) {
      state_.state = RecoveryState::FAILED;
      state_.trigger = trigger;
      state_.action = action;
      state_.failed = true;
      return false;
    }

    RecoveryAction selected_action = action;
    if (selected_action == RecoveryAction::NONE) {
      selected_action = config_.default_action;
    }

    selected_action = sanitize_action(selected_action);

    state_ = RecoveryManagerState{};
    state_.state =
      config_.stop_before_motion ? RecoveryState::REQUESTED : state_for_action(selected_action);
    state_.trigger = trigger;
    state_.action = selected_action;
    state_.start_s = ControlMath::finite_or_zero(now_s);
    state_.phase_start_s = state_.start_s;
    state_.last_update_s = state_.start_s;
    state_.phase_index = 0;
    state_.requested = true;
    state_.active = true;

    return true;
  }

  bool request_from_string(
    const double now_s,
    const std::string & trigger_text,
    const std::string & action_text)
  {
    const RecoveryTrigger trigger = parse_recovery_trigger(
      trigger_text,
      RecoveryTrigger::MANUAL_REQUEST);
    const RecoveryAction action = parse_recovery_action(
      action_text,
      RecoveryAction::NONE);

    return request(now_s, trigger, action);
  }

  void cancel()
  {
    state_.state = RecoveryState::CANCELLED;
    state_.active = false;
    state_.completed = false;
    state_.failed = false;
    state_.blocked = false;
    state_.requested = false;
  }

  void block()
  {
    state_.state = RecoveryState::BLOCKED;
    state_.active = false;
    state_.completed = false;
    state_.failed = false;
    state_.blocked = true;
    state_.requested = false;
  }

  RecoveryManagerResult update(const double now_s)
  {
    const double now = ControlMath::finite_or_zero(now_s);
    state_.last_update_s = now;

    RecoveryManagerResult result;
    result.state = state_.state;
    result.trigger = state_.trigger;
    result.action = state_.action;
    result.elapsed_s = elapsed(now);
    result.phase_elapsed_s = phase_elapsed(now);

    if (!config_.enabled) {
      finish_as(RecoveryState::FAILED);
      result.state = state_.state;
      result.finished = true;
      result.reason = "disabled";
      result.command = TwistCommand{};
      return result;
    }

    if (!state_.active) {
      result.state = state_.state;
      result.command = TwistCommand{};
      result.active = false;
      result.finished = is_recovery_terminal(state_.state);
      result.reason = result.finished ? "terminal" : "idle";
      return result;
    }

    if (result.elapsed_s > config_.max_recovery_duration_s) {
      finish_as(RecoveryState::TIMEOUT);
      result.state = state_.state;
      result.finished = true;
      result.active = false;
      result.reason = "timeout";
      result.command = TwistCommand{};
      return result;
    }

    if (state_.state == RecoveryState::REQUESTED) {
      if (config_.stop_before_motion && result.phase_elapsed_s < config_.stop_hold_s) {
        result.state = RecoveryState::REQUESTED;
        result.active = true;
        result.recovery_request_active = true;
        result.command = TwistCommand{};
        result.reason = "pre_stop";
        return result;
      }

      start_motion_phase(now);
    }

    result = run_motion_phase(now);
    return result;
  }

  RecoveryManagerResult update_blocked(const double now_s, const bool blocked)
  {
    if (blocked && state_.active) {
      block();

      RecoveryManagerResult result;
      result.state = state_.state;
      result.trigger = state_.trigger;
      result.action = state_.action;
      result.command = TwistCommand{};
      result.finished = true;
      result.active = false;
      result.reason = "blocked";
      result.elapsed_s = elapsed(ControlMath::finite_or_zero(now_s));
      result.phase_elapsed_s = phase_elapsed(ControlMath::finite_or_zero(now_s));
      return result;
    }

    return update(now_s);
  }

  bool active() const
  {
    return state_.active;
  }

  bool requested() const
  {
    return state_.requested;
  }

  bool finished() const
  {
    return is_recovery_terminal(state_.state);
  }

  RecoveryState current_state() const
  {
    return state_.state;
  }

  RecoveryAction current_action() const
  {
    return state_.action;
  }

  std::string status_string() const
  {
    std::ostringstream ss;
    ss << "state=" << to_string(state_.state)
       << "; trigger=" << to_string(state_.trigger)
       << "; action=" << to_string(state_.action)
       << "; active=" << bool_text(state_.active)
       << "; requested=" << bool_text(state_.requested)
       << "; completed=" << bool_text(state_.completed)
       << "; failed=" << bool_text(state_.failed)
       << "; blocked=" << bool_text(state_.blocked)
       << "; phase=" << state_.phase_index;

    return ss.str();
  }

private:
  RecoveryAction sanitize_action(const RecoveryAction action) const
  {
    if (action == RecoveryAction::ROTATE_LEFT && !config_.allow_rotate_left) {
      return config_.allow_rotate_right ? RecoveryAction::ROTATE_RIGHT : RecoveryAction::BACKUP;
    }

    if (action == RecoveryAction::ROTATE_RIGHT && !config_.allow_rotate_right) {
      return config_.allow_rotate_left ? RecoveryAction::ROTATE_LEFT : RecoveryAction::BACKUP;
    }

    if (action == RecoveryAction::BACKUP_THEN_LEFT && !config_.allow_rotate_left) {
      return config_.allow_rotate_right ? RecoveryAction::BACKUP_THEN_RIGHT :
             RecoveryAction::BACKUP;
    }

    if (action == RecoveryAction::BACKUP_THEN_RIGHT && !config_.allow_rotate_right) {
      return config_.allow_rotate_left ? RecoveryAction::BACKUP_THEN_LEFT : RecoveryAction::BACKUP;
    }

    return action;
  }

  RecoveryState state_for_action(const RecoveryAction action) const
  {
    switch (action) {
      case RecoveryAction::BACKUP:
      case RecoveryAction::BACKUP_THEN_LEFT:
      case RecoveryAction::BACKUP_THEN_RIGHT:
      case RecoveryAction::ESCAPE_SEQUENCE:
        return RecoveryState::BACKING_UP;

      case RecoveryAction::ROTATE_LEFT:
      case RecoveryAction::ROTATE_RIGHT:
        return RecoveryState::ROTATING;

      case RecoveryAction::STOP:
      case RecoveryAction::NONE:
        return RecoveryState::ACTIVE;
    }

    return RecoveryState::ACTIVE;
  }

  void start_motion_phase(const double now_s)
  {
    state_.phase_start_s = now_s;
    state_.phase_index = 0;
    state_.state = state_for_action(state_.action);
  }

  RecoveryManagerResult run_motion_phase(const double now_s)
  {
    RecoveryManagerResult result;
    result.state = state_.state;
    result.trigger = state_.trigger;
    result.action = state_.action;
    result.active = true;
    result.recovery_request_active = true;
    result.elapsed_s = elapsed(now_s);
    result.phase_elapsed_s = phase_elapsed(now_s);

    if (state_.phase_index >= 100) {
      complete_or_stop_after(now_s, result, "post_stop_done");
      return result;
    }

    switch (state_.action) {
      case RecoveryAction::NONE:
      case RecoveryAction::STOP:
        complete_or_stop_after(now_s, result, "stop");
        return result;

      case RecoveryAction::BACKUP:
        run_backup_only(now_s, result);
        return result;

      case RecoveryAction::ROTATE_LEFT:
        run_rotate_only(now_s, result, true);
        return result;

      case RecoveryAction::ROTATE_RIGHT:
        run_rotate_only(now_s, result, false);
        return result;

      case RecoveryAction::BACKUP_THEN_LEFT:
        run_backup_then_rotate(now_s, result, true);
        return result;

      case RecoveryAction::BACKUP_THEN_RIGHT:
        run_backup_then_rotate(now_s, result, false);
        return result;

      case RecoveryAction::ESCAPE_SEQUENCE:
        run_escape_sequence(now_s, result);
        return result;
    }

    complete(result, "unknown_action");
    return result;
  }

  void run_backup_only(const double now_s, RecoveryManagerResult & result)
  {
    if (phase_elapsed(now_s) >= config_.backup_duration_s) {
      complete_or_stop_after(now_s, result, "backup_done");
      return;
    }

    state_.state = RecoveryState::BACKING_UP;
    result.state = state_.state;
    result.command = limited_command(config_.backup_vx, 0.0, 0.0);
    result.reason = "backup";
  }

  void run_rotate_only(
    const double now_s,
    RecoveryManagerResult & result,
    const bool left)
  {
    if (phase_elapsed(now_s) >= config_.rotate_duration_s) {
      complete_or_stop_after(now_s, result, "rotate_done");
      return;
    }

    state_.state = RecoveryState::ROTATING;
    result.state = state_.state;
    result.command = limited_command(0.0, 0.0, left ? config_.rotate_wz : -config_.rotate_wz);
    result.reason = left ? "rotate_left" : "rotate_right";
  }

  void run_backup_then_rotate(
    const double now_s,
    RecoveryManagerResult & result,
    const bool left)
  {
    if (state_.phase_index == 0) {
      if (phase_elapsed(now_s) < config_.backup_duration_s) {
        state_.state = RecoveryState::BACKING_UP;
        result.state = state_.state;
        result.command = limited_command(config_.backup_vx, 0.0, 0.0);
        result.reason = "backup";
        return;
      }

      state_.phase_index = 1;
      state_.phase_start_s = now_s;
      state_.state = RecoveryState::ROTATING;
    }

    if (phase_elapsed(now_s) < config_.rotate_duration_s) {
      state_.state = RecoveryState::ROTATING;
      result.state = state_.state;
      result.command = limited_command(0.0, 0.0, left ? config_.rotate_wz : -config_.rotate_wz);
      result.reason = left ? "rotate_left" : "rotate_right";
      return;
    }

    complete_or_stop_after(now_s, result, "backup_then_rotate_done");
  }

  void run_escape_sequence(const double now_s, RecoveryManagerResult & result)
  {
    if (state_.phase_index == 0) {
      if (phase_elapsed(now_s) < config_.backup_duration_s) {
        state_.state = RecoveryState::BACKING_UP;
        result.state = state_.state;
        result.command = limited_command(config_.backup_vx, 0.0, 0.0);
        result.reason = "escape_backup";
        return;
      }

      state_.phase_index = 1;
      state_.phase_start_s = now_s;
      state_.state = RecoveryState::ROTATING;
    }

    if (state_.phase_index == 1) {
      if (phase_elapsed(now_s) < config_.rotate_duration_s) {
        state_.state = RecoveryState::ROTATING;
        result.state = state_.state;
        result.command = limited_command(0.0, 0.0, config_.rotate_wz);
        result.reason = "escape_rotate_left";
        return;
      }

      state_.phase_index = 2;
      state_.phase_start_s = now_s;
      state_.state = RecoveryState::ESCAPING;
    }

    if (phase_elapsed(now_s) < config_.backup_duration_s) {
      state_.state = RecoveryState::ESCAPING;
      result.state = state_.state;
      result.command = limited_command(config_.backup_vx, 0.0, -config_.rotate_wz);
      result.reason = "escape_arc";
      return;
    }

    complete_or_stop_after(now_s, result, "escape_done");
  }

  void complete_or_stop_after(
    const double now_s,
    RecoveryManagerResult & result,
    const std::string & reason)
  {
    if (config_.stop_after_motion && state_.state != RecoveryState::COMPLETED) {
      if (state_.phase_index < 100) {
        state_.phase_index = 100;
        state_.phase_start_s = now_s;
        state_.state = RecoveryState::ACTIVE;
      }

      if (phase_elapsed(now_s) < config_.stop_hold_s) {
        result.state = state_.state;
        result.active = true;
        result.recovery_request_active = true;
        result.command = TwistCommand{};
        result.reason = "post_stop";
        return;
      }
    }

    complete(result, reason);
  }

  void complete(RecoveryManagerResult & result, const std::string & reason)
  {
    finish_as(RecoveryState::COMPLETED);

    result.state = state_.state;
    result.command = TwistCommand{};
    result.finished = true;
    result.active = false;
    result.recovery_request_active = false;
    result.reason = reason;
  }

  void finish_as(const RecoveryState state)
  {
    state_.state = state;
    state_.active = false;
    state_.requested = false;
    state_.completed = state == RecoveryState::COMPLETED;
    state_.failed = state == RecoveryState::FAILED || state == RecoveryState::TIMEOUT;
    state_.blocked = state == RecoveryState::BLOCKED;
  }

  TwistCommand limited_command(
    const double vx,
    const double vy,
    const double wz) const
  {
    return limiter_.limit(make_twist_command(vx, vy, wz));
  }

  double elapsed(const double now_s) const
  {
    return std::max(0.0, ControlMath::finite_or_zero(now_s) - state_.start_s);
  }

  double phase_elapsed(const double now_s) const
  {
    return std::max(0.0, ControlMath::finite_or_zero(now_s) - state_.phase_start_s);
  }

  static const char * bool_text(const bool value)
  {
    return value ? "true" : "false";
  }

  RecoveryManagerConfig config_{};
  RecoveryManagerState state_{};
  CommandLimiter limiter_{};
};

}  // namespace savo_control
