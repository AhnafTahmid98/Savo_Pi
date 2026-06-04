#pragma once

// ROS-independent recovery state machine. Feed inputs into update(),
// read RecoveryAction to drive /cmd_vel_recovery publishing.

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
  kIdle = 0,
  kArmed,
  kBackingUp,
  kSettling,
  kTurning,
  kComplete,
  kAborted,
  kCooldown
};

// -----------------------------------------------------------------------------
// Reason codes for status/debugging
// -----------------------------------------------------------------------------
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
// Inputs supplied by ROS node on each update
// -----------------------------------------------------------------------------
struct RecoveryInputs
{
  bool stuck_detected {false};
  bool no_progress {false};
  bool safety_stop_active {false};
  bool manual_override {false};
  bool external_cancel {false};

  // Whether the recovery node is allowed to command motion.
  // Example: false if safety layer says motion is blocked.
  bool motion_allowed {true};

  // Optional external completion hints.
  bool backup_completed {false};
  bool turn_completed {false};
};

// -----------------------------------------------------------------------------
// Tunable recovery policy config
// -----------------------------------------------------------------------------
struct RecoveryManagerConfig
{
  // Trigger debounce time before recovery starts.
  double trigger_confirm_sec {0.20};

  // Active action durations.
  double backup_duration_sec {0.60};
  double settle_duration_sec {0.20};
  double turn_duration_sec {0.40};

  // Enable optional turn after backup and settle.
  bool enable_turn_phase {true};

  // Max duration for active recovery sequence only:
  // BackingUp + Settling + Turning.
  // kArmed is intentionally NOT counted as active recovery.
  double max_recovery_duration_sec {3.0};

  // Cooldown before next recovery attempt.
  double cooldown_sec {1.0};

  // Max attempts in one stuck streak.
  std::uint32_t max_attempts_per_streak {3};

  // If true, note_progress() clears attempt streak.
  bool reset_attempts_on_progress {true};

  // Time sanity.
  double min_time_sec {0.0};
  double max_time_jump_sec {5.0};
};

// -----------------------------------------------------------------------------
// Recovery action hints for ROS node command generation
// -----------------------------------------------------------------------------
struct RecoveryAction
{
  bool active {false};

  bool command_backup {false};
  bool command_turn {false};
  bool command_stop {false};

  // +1 = CCW, -1 = CW.
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

  // active = actual recovery sequence is active.
  // This excludes kArmed.
  bool active {false};

  bool in_cooldown {false};
  bool trigger_pending {false};

  std::uint32_t attempts_in_streak {0};

  double now_sec {0.0};
  double phase_elapsed_sec {0.0};
  double active_elapsed_sec {0.0};
  double cooldown_remaining_sec {0.0};

  bool started {false};
  bool completed {false};
  bool aborted {false};
  bool phase_changed {false};

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

  // ---------------------------------------------------------------------------
  // Config
  // ---------------------------------------------------------------------------
  void set_config(const RecoveryManagerConfig & cfg)
  {
    config_ = cfg;
    normalize_config_();
  }

  const RecoveryManagerConfig & config() const
  {
    return config_;
  }

  // ---------------------------------------------------------------------------
  // Lifecycle
  // ---------------------------------------------------------------------------
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
    pending_trigger_reason_ = RecoveryReason::kNone;
  }

  void initialize(double now_sec)
  {
    if (!valid_time_(now_sec)) {
      return;
    }

    initialized_ = true;
    last_now_sec_ = now_sec;
    phase_start_sec_ = now_sec;
  }

  void note_progress()
  {
    if (config_.reset_attempts_on_progress) {
      attempts_in_streak_ = 0;
    }
  }

  void cancel(double now_sec, bool enter_cooldown = true)
  {
    if (!valid_time_(now_sec)) {
      return;
    }

    has_trigger_pending_ = false;

    if (enter_cooldown) {
      enter_cooldown_(RecoveryReason::kExternalCancel, now_sec);
    } else {
      set_phase_(RecoveryPhase::kIdle, RecoveryReason::kNone, now_sec);
      active_start_sec_ = 0.0;
    }
  }

  // ---------------------------------------------------------------------------
  // Accessors
  // ---------------------------------------------------------------------------
  RecoveryPhase phase() const
  {
    return phase_;
  }

  RecoveryReason reason() const
  {
    return reason_;
  }

  // True only for actual recovery sequence phases.
  // kArmed is not active motion recovery.
  bool is_active() const
  {
    return phase_ == RecoveryPhase::kBackingUp ||
           phase_ == RecoveryPhase::kSettling ||
           phase_ == RecoveryPhase::kTurning;
  }

  bool is_trigger_pending() const
  {
    return phase_ == RecoveryPhase::kArmed || has_trigger_pending_;
  }

  bool is_in_cooldown() const
  {
    return phase_ == RecoveryPhase::kCooldown;
  }

  std::uint32_t attempts_in_streak() const
  {
    return attempts_in_streak_;
  }

  // ---------------------------------------------------------------------------
  // Main update
  // ---------------------------------------------------------------------------
  RecoveryManagerStatus update(double now_sec, const RecoveryInputs & in)
  {
    RecoveryManagerStatus s{};
    s.now_sec = now_sec;

    if (!valid_time_(now_sec)) {
      s.phase = phase_;
      s.reason = RecoveryReason::kInvalidTime;
      s.active = is_active();
      s.in_cooldown = is_in_cooldown();
      s.trigger_pending = is_trigger_pending();
      s.attempts_in_streak = attempts_in_streak_;
      s.action = make_action_(s.phase, s.reason, s.active);
      return s;
    }

    if (!initialized_) {
      initialize(now_sec);
    }

    const RecoveryPhase prev_phase = phase_;
    const RecoveryReason prev_reason = reason_;
    const bool was_active = is_active();

    // -----------------------------------------------------------------------
    // Time jump guard
    // -----------------------------------------------------------------------
    if (config_.max_time_jump_sec > 0.0 && std::isfinite(last_now_sec_)) {
      const double jump = now_sec - last_now_sec_;
      if (jump < 0.0 || jump > config_.max_time_jump_sec) {
        enter_cooldown_(RecoveryReason::kInvalidTime, now_sec);
      }
    }
    last_now_sec_ = now_sec;

    // -----------------------------------------------------------------------
    // Highest-priority cancellation conditions
    // -----------------------------------------------------------------------
    if (in.external_cancel) {
      enter_cooldown_(RecoveryReason::kExternalCancel, now_sec);
    } else if (in.manual_override) {
      if (is_active() || phase_ == RecoveryPhase::kArmed) {
        enter_cooldown_(RecoveryReason::kManualMode, now_sec);
      }
    }

    // -----------------------------------------------------------------------
    // Cooldown handling
    // -----------------------------------------------------------------------
    if (phase_ == RecoveryPhase::kCooldown) {
      if (now_sec >= cooldown_until_sec_) {
        set_phase_(RecoveryPhase::kIdle, RecoveryReason::kNone, now_sec);
        active_start_sec_ = 0.0;
        has_trigger_pending_ = false;
      }
    }

    // -----------------------------------------------------------------------
    // Trigger handling
    // -----------------------------------------------------------------------
    const bool trigger = in.stuck_detected || in.no_progress;
    const RecoveryReason trigger_reason =
      in.stuck_detected ? RecoveryReason::kStuckDetected :
      (in.no_progress ? RecoveryReason::kNoProgress : RecoveryReason::kNone);

    if (phase_ == RecoveryPhase::kIdle) {
      if (trigger) {
        start_or_continue_trigger_(now_sec, trigger_reason);
        maybe_start_recovery_(now_sec, in);
      } else {
        clear_trigger_();
      }
    } else if (phase_ == RecoveryPhase::kArmed) {
      if (!trigger) {
        clear_trigger_();
        set_phase_(RecoveryPhase::kIdle, RecoveryReason::kNone, now_sec);
      } else {
        maybe_start_recovery_(now_sec, in);
      }
    }

    // -----------------------------------------------------------------------
    // Safety stop during active recovery
    // -----------------------------------------------------------------------
    if (is_active() && (in.safety_stop_active || !in.motion_allowed)) {
      enter_cooldown_(RecoveryReason::kSafetyStopActive, now_sec);
    }

    // -----------------------------------------------------------------------
    // Active recovery timeout
    //
    // Important:
    //   This applies only to kBackingUp/kSettling/kTurning.
    //   It does NOT apply to kArmed.
    // -----------------------------------------------------------------------
    if (is_active() && config_.max_recovery_duration_sec > 0.0) {
      const double active_elapsed =
        active_start_sec_ > 0.0 ? (now_sec - active_start_sec_) : 0.0;

      if (active_elapsed > config_.max_recovery_duration_sec) {
        enter_cooldown_(RecoveryReason::kTimeout, now_sec);
      }
    }

    // -----------------------------------------------------------------------
    // Active phase progression
    // -----------------------------------------------------------------------
    if (phase_ == RecoveryPhase::kBackingUp) {
      const double elapsed = now_sec - phase_start_sec_;
      if (in.backup_completed || elapsed >= config_.backup_duration_sec) {
        set_phase_(RecoveryPhase::kSettling, reason_, now_sec);
      }
    } else if (phase_ == RecoveryPhase::kSettling) {
      const double elapsed = now_sec - phase_start_sec_;
      if (elapsed >= config_.settle_duration_sec) {
        if (config_.enable_turn_phase) {
          set_phase_(RecoveryPhase::kTurning, reason_, now_sec);
        } else {
          set_phase_(RecoveryPhase::kComplete, reason_, now_sec);
        }
      }
    } else if (phase_ == RecoveryPhase::kTurning) {
      const double elapsed = now_sec - phase_start_sec_;
      if (in.turn_completed || elapsed >= config_.turn_duration_sec) {
        set_phase_(RecoveryPhase::kComplete, reason_, now_sec);
      }
    }

    // -----------------------------------------------------------------------
    // Complete is a one-update transition marker.
    // Immediately enter cooldown, but completed=true below preserves the edge.
    // -----------------------------------------------------------------------
    const bool reached_complete = (phase_ == RecoveryPhase::kComplete);
    if (reached_complete) {
      enter_cooldown_(reason_, now_sec);
    }

    // -----------------------------------------------------------------------
    // Build status
    // -----------------------------------------------------------------------
    s.phase = phase_;
    s.reason = reason_;
    s.active = is_active();
    s.in_cooldown = is_in_cooldown();
    s.trigger_pending = is_trigger_pending();
    s.attempts_in_streak = attempts_in_streak_;

    s.phase_elapsed_sec = now_sec - phase_start_sec_;
    s.active_elapsed_sec =
      active_start_sec_ > 0.0 ? std::max(0.0, now_sec - active_start_sec_) : 0.0;

    s.cooldown_remaining_sec =
      phase_ == RecoveryPhase::kCooldown ?
      std::max(0.0, cooldown_until_sec_ - now_sec) : 0.0;

    s.phase_changed = (prev_phase != phase_);
    s.started = (!was_active && s.active);

    s.completed =
      reached_complete ||
      (prev_phase == RecoveryPhase::kComplete &&
       phase_ == RecoveryPhase::kCooldown);

    s.aborted =
      phase_ == RecoveryPhase::kCooldown &&
      (reason_ == RecoveryReason::kSafetyStopActive ||
       reason_ == RecoveryReason::kExternalCancel ||
       reason_ == RecoveryReason::kManualMode ||
       reason_ == RecoveryReason::kTimeout ||
       reason_ == RecoveryReason::kInvalidTime);

    // If we changed from an active phase to cooldown because of an abort reason,
    // mark it aborted.
    if (was_active && phase_ == RecoveryPhase::kCooldown &&
        reason_ != RecoveryReason::kStuckDetected &&
        reason_ != RecoveryReason::kNoProgress &&
        prev_reason != RecoveryReason::kStuckDetected &&
        prev_reason != RecoveryReason::kNoProgress) {
      s.aborted = true;
    }

    s.action = make_action_(phase_, reason_, s.active);
    return s;
  }

private:
  // ---------------------------------------------------------------------------
  // Internal transition helpers
  // ---------------------------------------------------------------------------
  void set_phase_(RecoveryPhase new_phase, RecoveryReason new_reason, double now_sec)
  {
    phase_ = new_phase;
    reason_ = new_reason;
    phase_start_sec_ = now_sec;
  }

  void enter_cooldown_(RecoveryReason reason, double now_sec)
  {
    set_phase_(RecoveryPhase::kCooldown, reason, now_sec);
    cooldown_until_sec_ = now_sec + config_.cooldown_sec;
    active_start_sec_ = 0.0;
    clear_trigger_();
  }

  void start_or_continue_trigger_(double now_sec, RecoveryReason reason)
  {
    if (!has_trigger_pending_) {
      has_trigger_pending_ = true;
      trigger_start_sec_ = now_sec;
      pending_trigger_reason_ = reason;
    }

    if (phase_ != RecoveryPhase::kArmed) {
      set_phase_(RecoveryPhase::kArmed, reason, now_sec);
    }
  }

  void clear_trigger_()
  {
    has_trigger_pending_ = false;
    trigger_start_sec_ = 0.0;
    pending_trigger_reason_ = RecoveryReason::kNone;
  }

  void maybe_start_recovery_(double now_sec, const RecoveryInputs & in)
  {
    if (!has_trigger_pending_) {
      return;
    }

    const double pending_elapsed = now_sec - trigger_start_sec_;
    if (pending_elapsed < config_.trigger_confirm_sec) {
      return;
    }

    if (attempts_in_streak_ >= config_.max_attempts_per_streak) {
      enter_cooldown_(RecoveryReason::kTooManyAttempts, now_sec);
      return;
    }

    if (!in.motion_allowed || in.safety_stop_active) {
      enter_cooldown_(RecoveryReason::kSafetyStopActive, now_sec);
      return;
    }

    attempts_in_streak_++;
    choose_next_turn_sign_();
    active_start_sec_ = now_sec;
    clear_trigger_();

    set_phase_(RecoveryPhase::kBackingUp, pending_trigger_reason_, now_sec);
  }

  bool valid_time_(double now_sec) const
  {
    return std::isfinite(now_sec) && now_sec >= config_.min_time_sec;
  }

  void choose_next_turn_sign_()
  {
    // Alternate turn direction between attempts.
    turn_sign_ = (turn_sign_ >= 0) ? -1 : +1;
  }

  RecoveryAction make_action_(
    RecoveryPhase phase,
    RecoveryReason reason,
    bool active) const
  {
    RecoveryAction a{};
    a.active = active;
    a.phase = phase;
    a.reason = reason;
    a.turn_sign = turn_sign_;

    switch (phase) {
      case RecoveryPhase::kBackingUp:
        a.command_backup = true;
        a.command_turn = false;
        a.command_stop = false;
        break;

      case RecoveryPhase::kTurning:
        a.command_backup = false;
        a.command_turn = true;
        a.command_stop = false;
        break;

      case RecoveryPhase::kArmed:
      case RecoveryPhase::kSettling:
      case RecoveryPhase::kComplete:
      case RecoveryPhase::kAborted:
      case RecoveryPhase::kCooldown:
        a.command_backup = false;
        a.command_turn = false;
        a.command_stop = true;
        break;

      case RecoveryPhase::kIdle:
      default:
        a.command_backup = false;
        a.command_turn = false;
        a.command_stop = false;
        break;
    }

    return a;
  }

  void normalize_config_()
  {
    auto sane_nonneg = [](double value, double fallback) {
      return std::isfinite(value) && value >= 0.0 ? value : fallback;
    };

    config_.trigger_confirm_sec =
      sane_nonneg(config_.trigger_confirm_sec, 0.20);
    config_.backup_duration_sec =
      sane_nonneg(config_.backup_duration_sec, 0.60);
    config_.settle_duration_sec =
      sane_nonneg(config_.settle_duration_sec, 0.20);
    config_.turn_duration_sec =
      sane_nonneg(config_.turn_duration_sec, 0.40);
    config_.max_recovery_duration_sec =
      sane_nonneg(config_.max_recovery_duration_sec, 3.0);
    config_.cooldown_sec =
      sane_nonneg(config_.cooldown_sec, 1.0);
    config_.min_time_sec =
      sane_nonneg(config_.min_time_sec, 0.0);
    config_.max_time_jump_sec =
      sane_nonneg(config_.max_time_jump_sec, 5.0);

    if (config_.max_attempts_per_streak == 0) {
      config_.max_attempts_per_streak = 1;
    }

    const double min_active_duration =
      config_.backup_duration_sec +
      config_.settle_duration_sec +
      (config_.enable_turn_phase ? config_.turn_duration_sec : 0.0);

    if (config_.max_recovery_duration_sec > 0.0) {
      config_.max_recovery_duration_sec =
        std::max(config_.max_recovery_duration_sec, min_active_duration + 0.05);
    }
  }

private:
  RecoveryManagerConfig config_ {};

  RecoveryPhase phase_ {RecoveryPhase::kIdle};
  RecoveryReason reason_ {RecoveryReason::kNone};

  bool initialized_ {false};
  double last_now_sec_ {0.0};

  bool has_trigger_pending_ {false};
  double trigger_start_sec_ {0.0};
  RecoveryReason pending_trigger_reason_ {RecoveryReason::kNone};

  double phase_start_sec_ {0.0};
  double active_start_sec_ {0.0};
  double cooldown_until_sec_ {0.0};

  std::uint32_t attempts_in_streak_ {0};
  int turn_sign_ {+1};
};

}  // namespace savo_control