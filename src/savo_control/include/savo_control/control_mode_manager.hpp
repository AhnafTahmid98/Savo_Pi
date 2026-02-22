#pragma once

// =============================================================================
// Robot SAVO — savo_control / control_mode_manager.hpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Reusable control-mode arbitration policy helper for Robot Savo.
//
// This utility provides a deterministic, ROS-independent state/policy layer for
// deciding which command source should be active at a given moment, e.g.:
//
//   - MANUAL   (teleop / operator)
//   - AUTO     (test controllers / local autonomous behaviors)
//   - NAV      (future Nav2 / high-level navigation)
//   - RECOVERY (local recovery behaviors such as backup_escape)
//   - STOP     (forced zero-command mode)
//
// It is intended for use in:
//   - control_mode_manager_node.cpp
//   - future mode-aware mux / supervisors
//
// What it does
// ------------
// - Tracks requested and active control modes
// - Applies deterministic priority rules (e.g. recovery > auto/nav)
// - Handles manual override, safety stop, and external stop conditions
// - Supports optional "latched" recovery mode until explicitly released
// - Provides status/action snapshots for ROS nodes to publish
//
// What it does NOT do
// -------------------
// - Does not publish/subscribe ROS topics directly
// - Does not mux Twist messages itself
// - Does not generate motor commands
//
// Design note
// -----------
// Keep this class as pure policy logic. ROS nodes should feed inputs into
// update(), then use the returned state/action to:
//   - select active cmd_vel source
//   - publish mode state/debug topics
//   - command downstream mux/shaper nodes
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace savo_control
{

// -----------------------------------------------------------------------------
// Public mode enum (stable control-mode contract)
// -----------------------------------------------------------------------------
enum class ControlMode : std::uint8_t
{
  kStop = 0,       // forced zero output / idle-safe mode
  kManual,         // operator teleop control
  kAuto,           // local automatic tests/controllers
  kNav,            // future navigation source (Nav2)
  kRecovery        // local recovery behaviors (backup/escape, etc.)
};

// Optional reason/debug code for transitions/decisions
enum class ControlModeReason : std::uint8_t
{
  kNone = 0,
  kStartup,
  kRequested,
  kManualOverride,
  kRecoveryActive,
  kRecoveryLatched,
  kSafetyStopActive,
  kExternalStop,
  kInvalidTime,
  kTimeout,
  kNoSourceAllowed
};

// -----------------------------------------------------------------------------
// Inputs supplied by node on each update
// -----------------------------------------------------------------------------
struct ControlModeInputs
{
  // External mode requests (typically from UI, topic command, or test scripts)
  bool request_stop {false};
  bool request_manual {false};
  bool request_auto {false};
  bool request_nav {false};

  // Recovery status/command integration
  bool recovery_active {false};          // recovery behavior currently producing commands
  bool recovery_triggered {false};       // edge hint (optional)
  bool recovery_completed {false};       // edge hint (optional)
  bool recovery_aborted {false};         // edge hint (optional)

  // Safety / operator conditions
  bool safety_stop_active {false};       // /safety/stop
  bool manual_override_active {false};   // operator moving joystick etc.
  bool external_stop_active {false};     // E-stop / system hold / user stop

  // Source availability (helps avoid selecting dead sources)
  bool manual_source_available {true};
  bool auto_source_available {true};
  bool nav_source_available {true};
  bool recovery_source_available {true};
};

// -----------------------------------------------------------------------------
// Tunable policy config
// -----------------------------------------------------------------------------
struct ControlModeManagerConfig
{
  // Startup mode after reset/initialize
  ControlMode startup_mode {ControlMode::kStop};

  // If true, manual_override_active immediately forces MANUAL (unless safety/stop)
  bool manual_override_preempts_all {true};

  // If true, recovery_active overrides AUTO/NAV and takes control
  bool recovery_preempts_auto_nav {true};

  // If true, recovery can also preempt MANUAL (usually false for safety/operator UX)
  bool recovery_preempts_manual {false};

  // If true, when recovery becomes active, latch mode=RECOVERY until completed/aborted
  bool latch_recovery_mode {true};

  // If true, safety stop forces STOP mode immediately
  bool safety_stop_forces_stop_mode {true};

  // If true, external_stop_active forces STOP mode immediately
  bool external_stop_forces_stop_mode {true};

  // Preferred automatic mode if both AUTO and NAV are requested
  // false -> AUTO wins, true -> NAV wins
  bool prefer_nav_over_auto {false};

  // If requested mode source is unavailable, fallback to STOP (else keep previous if possible)
  bool fallback_to_stop_when_source_unavailable {true};

  // Time sanity (optional, for node-supplied monotonic seconds)
  double min_time_sec {0.0};
  double max_time_jump_sec {5.0};
};

// -----------------------------------------------------------------------------
// Action hint for node-side command routing
// -----------------------------------------------------------------------------
struct ControlModeAction
{
  // Final selected mode
  ControlMode active_mode {ControlMode::kStop};
  ControlModeReason reason {ControlModeReason::kNone};

  // Convenience routing flags
  bool select_stop {true};
  bool select_manual {false};
  bool select_auto {false};
  bool select_nav {false};
  bool select_recovery {false};

  // Safety behavior hint (node may publish zero Twist or inhibit mux output)
  bool force_zero_output {true};
};

// -----------------------------------------------------------------------------
// Status snapshot for diagnostics / ROS status publishing
// -----------------------------------------------------------------------------
struct ControlModeManagerStatus
{
  // Current state
  ControlMode active_mode {ControlMode::kStop};
  ControlMode requested_mode {ControlMode::kStop};
  ControlModeReason reason {ControlModeReason::kNone};

  // Recovery latch state
  bool recovery_latched {false};

  // Timing
  double now_sec {0.0};
  double mode_elapsed_sec {0.0};

  // Edge events for the current update() call
  bool mode_changed {false};
  bool request_changed {false};

  // Validity
  bool valid_time {false};

  // Action recommendation for node-side routing
  ControlModeAction action {};
};

// -----------------------------------------------------------------------------
// Control mode manager (policy/state helper)
// -----------------------------------------------------------------------------
class ControlModeManager
{
public:
  ControlModeManager() = default;

  explicit ControlModeManager(const ControlModeManagerConfig & cfg)
  : config_(cfg)
  {
    normalize_config_();
    reset();
  }

  // -------------------------
  // Config
  // -------------------------
  void set_config(const ControlModeManagerConfig & cfg)
  {
    config_ = cfg;
    normalize_config_();
  }

  const ControlModeManagerConfig & config() const
  {
    return config_;
  }

  // -------------------------
  // State lifecycle
  // -------------------------
  void reset()
  {
    active_mode_ = config_.startup_mode;
    requested_mode_ = config_.startup_mode;
    reason_ = ControlModeReason::kStartup;

    initialized_ = false;
    last_now_sec_ = 0.0;
    mode_start_sec_ = 0.0;

    recovery_latched_ = false;
  }

  void initialize(double now_sec)
  {
    if (!valid_time_(now_sec)) {
      return;
    }

    initialized_ = true;
    last_now_sec_ = now_sec;
    mode_start_sec_ = now_sec;
    reason_ = ControlModeReason::kStartup;
  }

  // -------------------------
  // External request helpers
  // -------------------------
  void set_requested_mode(ControlMode mode)
  {
    requested_mode_ = mode;
  }

  ControlMode requested_mode() const
  {
    return requested_mode_;
  }

  // -------------------------
  // Accessors
  // -------------------------
  ControlMode active_mode() const
  {
    return active_mode_;
  }

  ControlModeReason reason() const
  {
    return reason_;
  }

  bool recovery_latched() const
  {
    return recovery_latched_;
  }

  // -------------------------
  // Main update
  // -------------------------
  ControlModeManagerStatus update(double now_sec, const ControlModeInputs & in)
  {
    ControlModeManagerStatus s{};
    s.now_sec = now_sec;
    s.valid_time = valid_time_(now_sec);

    const ControlMode prev_active = active_mode_;
    const ControlMode prev_request = requested_mode_;

    if (!s.valid_time) {
      s.active_mode = active_mode_;
      s.requested_mode = requested_mode_;
      s.reason = ControlModeReason::kInvalidTime;
      s.mode_elapsed_sec = 0.0;
      s.action = make_action_(active_mode_, ControlModeReason::kInvalidTime, in);
      return s;
    }

    if (!initialized_) {
      initialize(now_sec);
    }

    // Time jump guard (fail-safe to STOP)
    if (config_.max_time_jump_sec > 0.0 && std::isfinite(last_now_sec_)) {
      const double jump = now_sec - last_now_sec_;
      if (jump < 0.0 || jump > config_.max_time_jump_sec) {
        transition_mode_(ControlMode::kStop, ControlModeReason::kInvalidTime, now_sec);
        recovery_latched_ = false;
      }
    }
    last_now_sec_ = now_sec;

    // -----------------------------------------------------------------------
    // 1) Determine requested_mode_ from input requests (deterministic)
    // Priority among requests:
    //   STOP > MANUAL > (NAV or AUTO by config preference)
    // -----------------------------------------------------------------------
    ControlMode new_request = requested_mode_;

    if (in.request_stop) {
      new_request = ControlMode::kStop;
    } else if (in.request_manual) {
      new_request = ControlMode::kManual;
    } else {
      const bool auto_req = in.request_auto;
      const bool nav_req = in.request_nav;

      if (auto_req && nav_req) {
        new_request = config_.prefer_nav_over_auto ? ControlMode::kNav : ControlMode::kAuto;
      } else if (nav_req) {
        new_request = ControlMode::kNav;
      } else if (auto_req) {
        new_request = ControlMode::kAuto;
      }
      // If no request flags are set, keep previous requested_mode_ (latched request behavior)
    }

    requested_mode_ = new_request;

    // -----------------------------------------------------------------------
    // 2) Recovery latch state update
    // -----------------------------------------------------------------------
    if (in.recovery_completed || in.recovery_aborted) {
      recovery_latched_ = false;
    }

    if (config_.latch_recovery_mode && (in.recovery_active || in.recovery_triggered)) {
      recovery_latched_ = true;
    }

    // -----------------------------------------------------------------------
    // 3) Select final active mode (policy priority)
    // -----------------------------------------------------------------------
    ControlMode selected = active_mode_;
    ControlModeReason selected_reason = reason_;

    // Hard stop conditions first
    if (config_.external_stop_forces_stop_mode && in.external_stop_active) {
      selected = ControlMode::kStop;
      selected_reason = ControlModeReason::kExternalStop;
    } else if (config_.safety_stop_forces_stop_mode && in.safety_stop_active) {
      selected = ControlMode::kStop;
      selected_reason = ControlModeReason::kSafetyStopActive;
    }
    // Manual override preemption
    else if (config_.manual_override_preempts_all && in.manual_override_active) {
      if (in.manual_source_available) {
        selected = ControlMode::kManual;
        selected_reason = ControlModeReason::kManualOverride;
      } else {
        selected = ControlMode::kStop;
        selected_reason = ControlModeReason::kNoSourceAllowed;
      }
    }
    // Recovery preemption / latch
    else {
      const bool recovery_should_control =
        (in.recovery_active && in.recovery_source_available) ||
        (recovery_latched_ && config_.latch_recovery_mode && in.recovery_source_available);

      if (recovery_should_control) {
        const bool can_preempt_manual =
          config_.recovery_preempts_manual && (requested_mode_ == ControlMode::kManual);
        const bool can_preempt_auto_nav =
          config_.recovery_preempts_auto_nav &&
          (requested_mode_ == ControlMode::kAuto || requested_mode_ == ControlMode::kNav);

        // If current/requested mode is manual and recovery is NOT allowed to preempt manual,
        // keep manual (if available)
        if ((requested_mode_ == ControlMode::kManual) && !can_preempt_manual) {
          if (in.manual_source_available) {
            selected = ControlMode::kManual;
            selected_reason = ControlModeReason::kRequested;
          } else {
            selected = ControlMode::kStop;
            selected_reason = ControlModeReason::kNoSourceAllowed;
          }
        } else {
          // Recovery wins for auto/nav or by explicit config
          if ((requested_mode_ == ControlMode::kAuto || requested_mode_ == ControlMode::kNav) &&
              !can_preempt_auto_nav && in.recovery_active)
          {
            // recovery not allowed to preempt auto/nav; fall through to requested selection
            select_requested_mode_(in, selected, selected_reason);
          } else {
            selected = ControlMode::kRecovery;
            selected_reason = recovery_latched_
              ? ControlModeReason::kRecoveryLatched
              : ControlModeReason::kRecoveryActive;
          }
        }
      } else {
        // Normal requested-mode selection
        select_requested_mode_(in, selected, selected_reason);
      }
    }

    // Commit transition if changed
    if (selected != active_mode_ || selected_reason != reason_) {
      transition_mode_(selected, selected_reason, now_sec);
    }

    // Build status
    s.active_mode = active_mode_;
    s.requested_mode = requested_mode_;
    s.reason = reason_;
    s.recovery_latched = recovery_latched_;
    s.mode_elapsed_sec = now_sec - mode_start_sec_;
    s.mode_changed = (prev_active != active_mode_);
    s.request_changed = (prev_request != requested_mode_);
    s.action = make_action_(active_mode_, reason_, in);

    return s;
  }

private:
  // -------------------------
  // Internal helpers
  // -------------------------
  void transition_mode_(ControlMode mode, ControlModeReason reason, double now_sec)
  {
    active_mode_ = mode;
    reason_ = reason;
    mode_start_sec_ = now_sec;
  }

  bool valid_time_(double now_sec) const
  {
    return std::isfinite(now_sec) && (now_sec >= config_.min_time_sec);
  }

  void select_requested_mode_(
    const ControlModeInputs & in,
    ControlMode & selected_mode,
    ControlModeReason & selected_reason) const
  {
    switch (requested_mode_) {
      case ControlMode::kManual:
        if (in.manual_source_available) {
          selected_mode = ControlMode::kManual;
          selected_reason = ControlModeReason::kRequested;
        } else {
          select_unavailable_fallback_(selected_mode, selected_reason);
        }
        break;

      case ControlMode::kAuto:
        if (in.auto_source_available) {
          selected_mode = ControlMode::kAuto;
          selected_reason = ControlModeReason::kRequested;
        } else {
          select_unavailable_fallback_(selected_mode, selected_reason);
        }
        break;

      case ControlMode::kNav:
        if (in.nav_source_available) {
          selected_mode = ControlMode::kNav;
          selected_reason = ControlModeReason::kRequested;
        } else {
          select_unavailable_fallback_(selected_mode, selected_reason);
        }
        break;

      case ControlMode::kRecovery:
        if (in.recovery_source_available && in.recovery_active) {
          selected_mode = ControlMode::kRecovery;
          selected_reason = ControlModeReason::kRecoveryActive;
        } else {
          select_unavailable_fallback_(selected_mode, selected_reason);
        }
        break;

      case ControlMode::kStop:
      default:
        selected_mode = ControlMode::kStop;
        selected_reason = ControlModeReason::kRequested;
        break;
    }
  }

  void select_unavailable_fallback_(
    ControlMode & selected_mode,
    ControlModeReason & selected_reason) const
  {
    if (config_.fallback_to_stop_when_source_unavailable) {
      selected_mode = ControlMode::kStop;
      selected_reason = ControlModeReason::kNoSourceAllowed;
    } else {
      // Keep current active mode by caller policy (caller usually passes existing selected)
      selected_reason = ControlModeReason::kNoSourceAllowed;
    }
  }

  ControlModeAction make_action_(
    ControlMode mode,
    ControlModeReason reason,
    const ControlModeInputs & in) const
  {
    ControlModeAction a{};
    a.active_mode = mode;
    a.reason = reason;

    // Default safe behavior
    a.select_stop = true;
    a.force_zero_output = true;

    // If hard stop condition currently active, force zero no matter what mode says
    const bool hard_stop_now =
      (config_.external_stop_forces_stop_mode && in.external_stop_active) ||
      (config_.safety_stop_forces_stop_mode && in.safety_stop_active);

    if (hard_stop_now) {
      a.active_mode = ControlMode::kStop;
      a.reason = in.external_stop_active
        ? ControlModeReason::kExternalStop
        : ControlModeReason::kSafetyStopActive;
      return a;
    }

    switch (mode) {
      case ControlMode::kManual:
        a.select_stop = false;
        a.select_manual = true;
        a.force_zero_output = false;
        break;

      case ControlMode::kAuto:
        a.select_stop = false;
        a.select_auto = true;
        a.force_zero_output = false;
        break;

      case ControlMode::kNav:
        a.select_stop = false;
        a.select_nav = true;
        a.force_zero_output = false;
        break;

      case ControlMode::kRecovery:
        a.select_stop = false;
        a.select_recovery = true;
        a.force_zero_output = false;
        break;

      case ControlMode::kStop:
      default:
        // keep safe defaults
        break;
    }

    return a;
  }

  void normalize_config_()
  {
    if (!std::isfinite(config_.min_time_sec) || config_.min_time_sec < 0.0) {
      config_.min_time_sec = 0.0;
    }
    if (!std::isfinite(config_.max_time_jump_sec) || config_.max_time_jump_sec < 0.0) {
      config_.max_time_jump_sec = 5.0;
    }

    // startup_mode must be a valid enum value (practically always true in C++)
    // no extra sanitization required here.
  }

private:
  ControlModeManagerConfig config_ {};

  // State
  bool initialized_ {false};

  ControlMode active_mode_ {ControlMode::kStop};
  ControlMode requested_mode_ {ControlMode::kStop};
  ControlModeReason reason_ {ControlModeReason::kStartup};

  bool recovery_latched_ {false};

  // Timing
  double last_now_sec_ {0.0};
  double mode_start_sec_ {0.0};
};

}  // namespace savo_control