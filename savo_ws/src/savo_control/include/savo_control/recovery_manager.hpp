#pragma once

// =============================================================================
// Robot SAVO — savo_control / recovery_manager.hpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Reusable local recovery decision/state helper for Robot Savo.
//
// This utility is ROS-independent and intended for use in:
//   - recovery_manager_node.cpp
//   - backup_escape_node.cpp (as a policy/state helper)
//   - future auto-test / fail-safe orchestration
//
// What it does
// ------------
// - Tracks a small recovery state machine
// - Decides when to trigger a local recovery attempt after stuck detection
// - Enforces cooldowns and per-event retry limits
// - Provides simple "backup then settle" timing windows
// - Exposes debug/status snapshots for ROS nodes to publish
//
// Design note
// -----------
// This class does NOT directly publish Twist or read ROS topics.
// Nodes should:
//   - feed inputs (stuck flags, safety stop, command activity, progress hints)
//   - call update(now_sec, ...)
//   - use the returned status/phase to command recovery actions
//
// Professional intent
// -------------------
// Keep recovery policy deterministic and easy to tune. Nav2-level recovery can
// come later; this is a local pre-Nav2 / support recovery layer.
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace savo_control
{

// -----------------------------------------------------------------------------
// Recovery state machine phases
// -----------------------------------------------------------------------------
enum class RecoveryPhase : std::uint8_t
{
  kIdle = 0,          // no recovery active
  kArmed,             // trigger conditions seen; waiting confirm/debounce
  kBackingUp,         // issue reverse command
  kSettling,          // short pause after backup
  kTurning,           // optional small yaw turn
  kComplete,          // recovery finished successfully (one-cycle status)
  kAborted,           // aborted due to safety/invalid conditions (one-cycle status)
  kCooldown           // waiting before next attempt
};

// Optional reason codes for status/debugging
enum class RecoveryReason : std::uint8_t
{
  kNone = 0,
  kStuckDetected,
  kNoProgress,
  kSafetyStopActive,
  kTooManyAttempts,
  kCooldownActive,
  kInvalidTime,
  kExternalCancel,
  kManualMode,
  kTimeout
};

// -----------------------------------------------------------------------------
// Inputs supplied by node on each update
// -----------------------------------------------------------------------------
struct RecoveryInputs
{
  // Current triggers / conditions
  bool stuck_detected {false};        // from stuck detector / logic
  bool no_progress {false};           // optional progress watchdog result
  bool safety_stop_active {false};    // from /safety/stop
  bool manual_override {false};       // operator active / manual mode
  bool external_cancel {false};       // explicit cancel/reset request

  // Whether a recovery action is allowed to command motion right now
  bool motion_allowed {true};

  // Optional hints for finishing/aborting current action
  bool backup_completed {false};      // node signals backup target distance/time reached
  bool turn_completed {false};        // node signals turn target reached
};

// -----------------------------------------------------------------------------
// Tunable recovery policy config
// -----------------------------------------------------------------------------
struct RecoveryManagerConfig
{
  // Trigger debounce before starting recovery (seconds)
  double trigger_confirm_sec {0.20};

  // Active action windows (seconds)
  double backup_duration_sec {0.60};
  double settle_duration_sec {0.20};
  double turn_duration_sec {0.40};   // used only if enable_turn_phase=true

  // If true, recovery includes a turn phase after backup+settle
  bool enable_turn_phase {true};

  // Max total active recovery duration safety timeout (seconds)
  double max_recovery_duration_sec {3.0};

  // Cooldown before next recovery can begin (seconds)
  double cooldown_sec {1.0};

  // Max attempts before lockout (within current streak)
  std::uint32_t max_attempts_per_streak {3};

  // If true, clear attempt streak after successful non-recovery progress interval
  // (node should call note_progress()).
  bool reset_attempts_on_progress {true};

  // dt / time sanity bounds
  double min_time_sec {0.0};
  double max_time_jump_sec {5.0};  // if exceeded, fail-safe abort + cooldown
};

// -----------------------------------------------------------------------------
// Recovery action hints for node command generation
// -----------------------------------------------------------------------------
struct RecoveryAction
{
  bool active {false};

  // Phase flags (node can map these to actual Twist commands)
  bool command_backup {false};
  bool command_turn {false};
  bool command_stop {false};

  // Recommended command signs (node may ignore/override with params)
  // backup: usually negative vx ; turn_sign: +1 CCW / -1 CW
  int turn_sign {+1};

  RecoveryPhase phase {RecoveryPhase::kIdle};
  RecoveryReason reason {RecoveryReason::kNone};
};

// -----------------------------------------------------------------------------
// Status snapshot for diagnostics / ROS status publishing
// -----------------------------------------------------------------------------
struct RecoveryManagerStatus
{
  RecoveryPhase phase {RecoveryPhase::kIdle};
  RecoveryReason reason {RecoveryReason::kNone};

  bool active {false};
  bool in_cooldown {false};
  bool trigger_pending {false};

  std::uint32_t attempts_in_streak {0};

  // Timing
  double now_sec {0.0};
  double phase_elapsed_sec {0.0};
  double active_elapsed_sec {0.0};
  double cooldown_remaining_sec {0.0};

  // Edge events (true for the update() call where transition happens)
  bool started {false};
  bool completed {false};
  bool aborted {false};
  bool phase_changed {false};

  // Convenience action recommendation
  RecoveryAction action {};
};

// -----------------------------------------------------------------------------
// Recovery manager
// -----------------------------------------------------------------------------
class RecoveryManager
{
public:
  RecoveryManager() = default;

  explicit RecoveryManager(const RecoveryManagerConfig & cfg)
  : config_(cfg)
  {
    normalize_config_();
  }

  // -------------------------
  // Config
  // -------------------------
  void set_config(const RecoveryManagerConfig & cfg)
  {
    config_ = cfg;
    normalize_config_();
  }

  const RecoveryManagerConfig & config() const
  {
    return config_;
  }

  // -------------------------
  // State lifecycle
  // -------------------------
  void reset()
  {
    phase_ = RecoveryPhase::kIdle;
    reason_ = RecoveryReason::kNone;

    initialized_ = false;
    last_now_sec_ = 0.0;

    trigger_start_sec_ = 0.0;
    phase_start_sec_ = 0.0;
    active_start_sec_ = 0.0;
    cooldown_until_sec_ = 0.0;

    attempts_in_streak_ = 0;
    turn_sign_ = +1;

    has_trigger_pending_ = false;
  }

  void initialize(double now_sec)
  {
    if (!valid_time_(now_sec)) {
      return;
    }
    initialized_ = true;
    last_now_sec_ = now_sec;
    if (phase_start_sec_ == 0.0) {
      phase_start_sec_ = now_sec;
    }
  }

  // External progress hint (e.g., robot moved normally again)
  // Useful to clear attempt streak after successful movement.
  void note_progress()
  {
    if (config_.reset_attempts_on_progress) {
      attempts_in_streak_ = 0;
    }
  }

  // Force cancel current recovery and go idle/cooldown
  void cancel(double now_sec, bool enter_cooldown = true)
  {
    if (!valid_time_(now_sec)) {
      return;
    }
    set_phase_(RecoveryPhase::kAborted, RecoveryReason::kExternalCancel, now_sec);
    if (enter_cooldown) {
      set_phase_(RecoveryPhase::kCooldown, RecoveryReason::kExternalCancel, now_sec);
      cooldown_until_sec_ = now_sec + config_.cooldown_sec;
    } else {
      set_phase_(RecoveryPhase::kIdle, RecoveryReason::kNone, now_sec);
    }
    has_trigger_pending_ = false;
  }

  // -------------------------
  // Accessors
  // -------------------------
  RecoveryPhase phase() const { return phase_; }
  RecoveryReason reason() const { return reason_; }

  bool is_active() const
  {
    return phase_ == RecoveryPhase::kArmed ||
           phase_ == RecoveryPhase::kBackingUp ||
           phase_ == RecoveryPhase::kSettling ||
           phase_ == RecoveryPhase::kTurning;
  }

  bool is_in_cooldown() const
  {
    return phase_ == RecoveryPhase::kCooldown;
  }

  std::uint32_t attempts_in_streak() const
  {
    return attempts_in_streak_;
  }

  // -------------------------
  // Main update
  // -------------------------
  RecoveryManagerStatus update(double now_sec, const RecoveryInputs & in)
  {
    RecoveryManagerStatus s{};
    s.now_sec = now_sec;

    if (!valid_time_(now_sec)) {
      // Invalid time -> fail-safe status (do not mutate deeply)
      s.phase = phase_;
      s.reason = RecoveryReason::kInvalidTime;
      s.active = is_active();
      s.in_cooldown = is_in_cooldown();
      s.trigger_pending = has_trigger_pending_;
      s.attempts_in_streak = attempts_in_streak_;
      s.action = make_action_(s.phase, s.reason, false);
      return s;
    }

    if (!initialized_) {
      initialize(now_sec);
      phase_start_sec_ = now_sec;
    }

    // Time jump sanity guard
    if (config_.max_time_jump_sec > 0.0 && std::isfinite(last_now_sec_)) {
      const double jump = now_sec - last_now_sec_;
      if (jump < 0.0 || jump > config_.max_time_jump_sec) {
        set_phase_(RecoveryPhase::kAborted, RecoveryReason::kInvalidTime, now_sec);
        set_phase_(RecoveryPhase::kCooldown, RecoveryReason::kInvalidTime, now_sec);
        cooldown_until_sec_ = now_sec + config_.cooldown_sec;
        has_trigger_pending_ = false;
      }
    }
    last_now_sec_ = now_sec;

    const RecoveryPhase prev_phase = phase_;
    const bool was_active = is_active();

    // Handle explicit cancel / manual override first
    if (in.external_cancel) {
      set_phase_(RecoveryPhase::kAborted, RecoveryReason::kExternalCancel, now_sec);
      set_phase_(RecoveryPhase::kCooldown, RecoveryReason::kExternalCancel, now_sec);
      cooldown_until_sec_ = now_sec + config_.cooldown_sec;
      has_trigger_pending_ = false;
    } else if (in.manual_override) {
      // Manual operator takes control: abort current recovery and cooldown
      if (is_active()) {
        set_phase_(RecoveryPhase::kAborted, RecoveryReason::kManualMode, now_sec);
        set_phase_(RecoveryPhase::kCooldown, RecoveryReason::kManualMode, now_sec);
        cooldown_until_sec_ = now_sec + config_.cooldown_sec;
      } else if (phase_ == RecoveryPhase::kArmed) {
        set_phase_(RecoveryPhase::kIdle, RecoveryReason::kNone, now_sec);
      }
      has_trigger_pending_ = false;
    }

    // Cooldown handling
    if (phase_ == RecoveryPhase::kCooldown) {
      if (now_sec >= cooldown_until_sec_) {
        set_phase_(RecoveryPhase::kIdle, RecoveryReason::kNone, now_sec);
      }
    }

    // Trigger detection when idle
    const bool trigger = in.stuck_detected || in.no_progress;
    const RecoveryReason trigger_reason =
      in.stuck_detected ? RecoveryReason::kStuckDetected :
      (in.no_progress ? RecoveryReason::kNoProgress : RecoveryReason::kNone);

    if (phase_ == RecoveryPhase::kIdle) {
      if (trigger) {
        if (!has_trigger_pending_) {
          has_trigger_pending_ = true;
          trigger_start_sec_ = now_sec;
          pending_trigger_reason_ = trigger_reason;
        }

        const double pending_elapsed = now_sec - trigger_start_sec_;
        if (pending_elapsed >= config_.trigger_confirm_sec) {
          if (attempts_in_streak_ >= config_.max_attempts_per_streak) {
            set_phase_(RecoveryPhase::kCooldown, RecoveryReason::kTooManyAttempts, now_sec);
            cooldown_until_sec_ = now_sec + config_.cooldown_sec;
            has_trigger_pending_ = false;
          } else if (!in.motion_allowed || in.safety_stop_active) {
            // Cannot start motion recovery while safety stop is active
            set_phase_(RecoveryPhase::kCooldown, RecoveryReason::kSafetyStopActive, now_sec);
            cooldown_until_sec_ = now_sec + config_.cooldown_sec;
            has_trigger_pending_ = false;
          } else {
            // Start active recovery
            attempts_in_streak_++;
            active_start_sec_ = now_sec;
            choose_next_turn_sign_();
            set_phase_(RecoveryPhase::kBackingUp, pending_trigger_reason_, now_sec);
            has_trigger_pending_ = false;
          }
        } else {
          // Optional armed state for observability
          if (phase_ != RecoveryPhase::kArmed) {
            set_phase_(RecoveryPhase::kArmed, pending_trigger_reason_, now_sec);
          }
        }
      } else {
        has_trigger_pending_ = false;
        if (phase_ == RecoveryPhase::kArmed) {
          set_phase_(RecoveryPhase::kIdle, RecoveryReason::kNone, now_sec);
        }
      }
    } else if (phase_ == RecoveryPhase::kArmed) {
      // Trigger dropped before confirm
      if (!trigger) {
        has_trigger_pending_ = false;
        set_phase_(RecoveryPhase::kIdle, RecoveryReason::kNone, now_sec);
      } else {
        const double pending_elapsed = now_sec - trigger_start_sec_;
        if (pending_elapsed >= config_.trigger_confirm_sec) {
          if (attempts_in_streak_ >= config_.max_attempts_per_streak) {
            set_phase_(RecoveryPhase::kCooldown, RecoveryReason::kTooManyAttempts, now_sec);
            cooldown_until_sec_ = now_sec + config_.cooldown_sec;
            has_trigger_pending_ = false;
          } else if (!in.motion_allowed || in.safety_stop_active) {
            set_phase_(RecoveryPhase::kCooldown, RecoveryReason::kSafetyStopActive, now_sec);
            cooldown_until_sec_ = now_sec + config_.cooldown_sec;
            has_trigger_pending_ = false;
          } else {
            attempts_in_streak_++;
            active_start_sec_ = now_sec;
            choose_next_turn_sign_();
            set_phase_(RecoveryPhase::kBackingUp, pending_trigger_reason_, now_sec);
            has_trigger_pending_ = false;
          }
        }
      }
    }

    // Active recovery timeout guard
    if (is_active() && config_.max_recovery_duration_sec > 0.0) {
      const double active_elapsed = now_sec - active_start_sec_;
      if (active_elapsed > config_.max_recovery_duration_sec) {
        set_phase_(RecoveryPhase::kAborted, RecoveryReason::kTimeout, now_sec);
        set_phase_(RecoveryPhase::kCooldown, RecoveryReason::kTimeout, now_sec);
        cooldown_until_sec_ = now_sec + config_.cooldown_sec;
      }
    }

    // Safety stop during active recovery -> abort to cooldown
    if (is_active() && (in.safety_stop_active || !in.motion_allowed)) {
      set_phase_(RecoveryPhase::kAborted, RecoveryReason::kSafetyStopActive, now_sec);
      set_phase_(RecoveryPhase::kCooldown, RecoveryReason::kSafetyStopActive, now_sec);
      cooldown_until_sec_ = now_sec + config_.cooldown_sec;
    }

    // Phase progression
    if (phase_ == RecoveryPhase::kBackingUp) {
      const double phase_elapsed = now_sec - phase_start_sec_;
      if (in.backup_completed || phase_elapsed >= config_.backup_duration_sec) {
        set_phase_(RecoveryPhase::kSettling, reason_, now_sec);
      }
    } else if (phase_ == RecoveryPhase::kSettling) {
      const double phase_elapsed = now_sec - phase_start_sec_;
      if (phase_elapsed >= config_.settle_duration_sec) {
        if (config_.enable_turn_phase) {
          set_phase_(RecoveryPhase::kTurning, reason_, now_sec);
        } else {
          set_phase_(RecoveryPhase::kComplete, reason_, now_sec);
        }
      }
    } else if (phase_ == RecoveryPhase::kTurning) {
      const double phase_elapsed = now_sec - phase_start_sec_;
      if (in.turn_completed || phase_elapsed >= config_.turn_duration_sec) {
        set_phase_(RecoveryPhase::kComplete, reason_, now_sec);
      }
    }

    // Complete -> cooldown (one update edge for observability)
    if (phase_ == RecoveryPhase::kComplete) {
      set_phase_(RecoveryPhase::kCooldown, reason_, now_sec);
      cooldown_until_sec_ = now_sec + config_.cooldown_sec;
    }

    // Build status snapshot
    s.phase = phase_;
    s.reason = reason_;
    s.active = is_active();
    s.in_cooldown = (phase_ == RecoveryPhase::kCooldown);
    s.trigger_pending = has_trigger_pending_;
    s.attempts_in_streak = attempts_in_streak_;

    s.phase_elapsed_sec = (now_sec - phase_start_sec_);
    s.active_elapsed_sec = (active_start_sec_ > 0.0) ? (now_sec - active_start_sec_) : 0.0;
    s.cooldown_remaining_sec = (phase_ == RecoveryPhase::kCooldown)
      ? std::max(0.0, cooldown_until_sec_ - now_sec) : 0.0;

    s.phase_changed = (prev_phase != phase_);
    s.started = (!was_active && s.active);
    s.completed = (prev_phase != RecoveryPhase::kCooldown &&
                   phase_ == RecoveryPhase::kCooldown &&
                   reason_ != RecoveryReason::kSafetyStopActive &&
                   reason_ != RecoveryReason::kExternalCancel &&
                   reason_ != RecoveryReason::kManualMode &&
                   reason_ != RecoveryReason::kTimeout &&
                   reason_ != RecoveryReason::kInvalidTime);
    s.aborted = (phase_ == RecoveryPhase::kCooldown) &&
                (reason_ == RecoveryReason::kSafetyStopActive ||
                 reason_ == RecoveryReason::kExternalCancel ||
                 reason_ == RecoveryReason::kManualMode ||
                 reason_ == RecoveryReason::kTimeout ||
                 reason_ == RecoveryReason::kInvalidTime);

    s.action = make_action_(phase_, reason_, s.active);
    return s;
  }

private:
  // -------------------------
  // Internal helpers
  // -------------------------
  void set_phase_(RecoveryPhase new_phase, RecoveryReason new_reason, double now_sec)
  {
    phase_ = new_phase;
    reason_ = new_reason;
    phase_start_sec_ = now_sec;
  }

  bool valid_time_(double now_sec) const
  {
    return std::isfinite(now_sec) && (now_sec >= config_.min_time_sec);
  }

  void choose_next_turn_sign_()
  {
    // Alternate turn direction each attempt to avoid repeating the same trap.
    turn_sign_ = (turn_sign_ >= 0) ? -1 : +1;
  }

  RecoveryAction make_action_(RecoveryPhase phase, RecoveryReason reason, bool active) const
  {
    RecoveryAction a{};
    a.active = active;
    a.phase = phase;
    a.reason = reason;
    a.turn_sign = turn_sign_;

    switch (phase) {
      case RecoveryPhase::kBackingUp:
        a.command_backup = true;
        a.command_stop = false;
        break;
      case RecoveryPhase::kSettling:
        a.command_stop = true;
        break;
      case RecoveryPhase::kTurning:
        a.command_turn = true;
        a.command_stop = false;
        break;
      case RecoveryPhase::kArmed:
      case RecoveryPhase::kCooldown:
      case RecoveryPhase::kComplete:
      case RecoveryPhase::kAborted:
      case RecoveryPhase::kIdle:
      default:
        a.command_stop = false;
        break;
    }
    return a;
  }

  void normalize_config_()
  {
    auto sane_nonneg = [](double v, double fallback) {
      return (std::isfinite(v) && v >= 0.0) ? v : fallback;
    };

    config_.trigger_confirm_sec = sane_nonneg(config_.trigger_confirm_sec, 0.20);
    config_.backup_duration_sec = sane_nonneg(config_.backup_duration_sec, 0.60);
    config_.settle_duration_sec = sane_nonneg(config_.settle_duration_sec, 0.20);
    config_.turn_duration_sec = sane_nonneg(config_.turn_duration_sec, 0.40);
    config_.max_recovery_duration_sec = sane_nonneg(config_.max_recovery_duration_sec, 3.0);
    config_.cooldown_sec = sane_nonneg(config_.cooldown_sec, 1.0);
    config_.min_time_sec = sane_nonneg(config_.min_time_sec, 0.0);
    config_.max_time_jump_sec = sane_nonneg(config_.max_time_jump_sec, 5.0);

    if (config_.max_attempts_per_streak == 0) {
      config_.max_attempts_per_streak = 1;
    }

    // Ensure max duration is at least enough for configured phases (if enabled)
    double min_active =
      config_.backup_duration_sec + config_.settle_duration_sec +
      (config_.enable_turn_phase ? config_.turn_duration_sec : 0.0);
    if (config_.max_recovery_duration_sec > 0.0) {
      config_.max_recovery_duration_sec =
        std::max(config_.max_recovery_duration_sec, min_active + 0.05);
    }
  }

private:
  RecoveryManagerConfig config_ {};

  RecoveryPhase phase_ {RecoveryPhase::kIdle};
  RecoveryReason reason_ {RecoveryReason::kNone};

  bool initialized_ {false};
  double last_now_sec_ {0.0};

  // Trigger/debounce
  bool has_trigger_pending_ {false};
  double trigger_start_sec_ {0.0};
  RecoveryReason pending_trigger_reason_ {RecoveryReason::kNone};

  // Timing
  double phase_start_sec_ {0.0};
  double active_start_sec_ {0.0};
  double cooldown_until_sec_ {0.0};

  // Policy state
  std::uint32_t attempts_in_streak_ {0};
  int turn_sign_ {+1};

  // (Optional future) edge memory can be added here if node wants richer events
};

}  // namespace savo_control