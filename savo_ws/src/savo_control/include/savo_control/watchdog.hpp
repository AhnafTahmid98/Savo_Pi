#pragma once

// =============================================================================
// Robot SAVO — savo_control / watchdog.hpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Lightweight watchdog / freshness helper for command and sensor streams.
//
// This utility is ROS-independent and intended for use in nodes such as:
//   - twist_mux_node.cpp         (source command freshness / timeout)
//   - cmd_vel_shaper_node.cpp    (input timeout -> zero command / reset limiter)
//   - recovery_manager_node.cpp  (stale status / heartbeat checks)
//   - status/debug nodes         (fresh/stale reporting)
//
// What it provides
// ----------------
// - Marking "last seen" timestamps
// - Fresh/stale timeout checks
// - Optional startup grace behavior
// - Timeout edge detection (fresh -> stale, stale -> fresh)
// - Simple state reset
//
// Design note
// -----------
// This class intentionally uses plain time values (seconds as double).
// Node code should convert ROS clocks/timestamps to seconds and pass them in.
// This keeps the helper reusable and easy to unit test.
// =============================================================================

#include <cmath>
#include <cstdint>

namespace savo_control
{

// -----------------------------------------------------------------------------
// Watchdog configuration
// -----------------------------------------------------------------------------
struct WatchdogConfig
{
  // Timeout threshold in seconds.
  // If <= 0, timeout checks are effectively disabled (always fresh after first update).
  double timeout_sec {0.5};

  // Optional startup grace period in seconds, measured from initialize()/reset(now).
  // During grace, "never updated" state may be treated as not stale.
  double startup_grace_sec {0.0};

  // If true, a watchdog with no updates yet is considered stale (after grace).
  // If false, "never seen" remains not stale until first update.
  bool stale_if_never_seen {true};

  // Minimum valid time delta accepted between updates/checks.
  // Helps guard against invalid/non-finite time values.
  double min_time_sec {0.0};

  // Maximum valid time jump (optional sanity guard).
  // If <= 0, disabled. If enabled and exceeded, callers may decide to reset.
  double max_time_jump_sec {0.0};
};

// -----------------------------------------------------------------------------
// Status/result snapshot
// -----------------------------------------------------------------------------
struct WatchdogStatus
{
  bool initialized {false};
  bool has_seen_update {false};

  bool fresh {false};
  bool stale {false};
  bool in_startup_grace {false};

  // Edge indicators compared to previous check() call
  bool became_fresh {false};
  bool became_stale {false};

  // Timing diagnostics
  double now_sec {0.0};
  double start_time_sec {0.0};
  double last_update_sec {0.0};

  // Ages
  double age_since_update_sec {0.0};   // valid if has_seen_update=true
  double age_since_start_sec {0.0};    // valid if initialized=true

  // Whether the check input looked valid
  bool time_valid {false};
};

// -----------------------------------------------------------------------------
// Generic watchdog
// -----------------------------------------------------------------------------
class Watchdog
{
public:
  Watchdog() = default;

  explicit Watchdog(const WatchdogConfig & cfg)
  : config_(cfg)
  {
    normalize_config_();
  }

  // -------------------------
  // Configuration
  // -------------------------
  void set_config(const WatchdogConfig & cfg)
  {
    config_ = cfg;
    normalize_config_();
  }

  const WatchdogConfig & config() const
  {
    return config_;
  }

  // -------------------------
  // Lifecycle / state
  // -------------------------
  void reset()
  {
    initialized_ = false;
    has_seen_update_ = false;
    start_time_sec_ = 0.0;
    last_update_sec_ = 0.0;

    prev_fresh_ = false;
    prev_stale_ = false;
    has_prev_check_state_ = false;
  }

  // Initialize watchdog start time (useful for startup grace logic)
  void initialize(double now_sec)
  {
    if (!is_valid_time_(now_sec)) {
      return;
    }
    initialized_ = true;
    start_time_sec_ = now_sec;

    // Reset edge tracking baseline on explicit init
    prev_fresh_ = false;
    prev_stale_ = false;
    has_prev_check_state_ = false;
  }

  // Reset and initialize in one call
  void reset(double now_sec)
  {
    reset();
    initialize(now_sec);
  }

  bool initialized() const
  {
    return initialized_;
  }

  bool has_seen_update() const
  {
    return has_seen_update_;
  }

  double start_time_sec() const
  {
    return start_time_sec_;
  }

  double last_update_sec() const
  {
    return last_update_sec_;
  }

  // -------------------------
  // Update ("kick"/heartbeat)
  // -------------------------
  // Marks the source as seen at now_sec.
  // Returns false only if time is invalid.
  bool update(double now_sec)
  {
    if (!is_valid_time_(now_sec)) {
      return false;
    }

    if (!initialized_) {
      initialize(now_sec);
    }

    // Optional sanity guard for large backwards/forwards jumps can be handled by caller.
    last_update_sec_ = now_sec;
    has_seen_update_ = true;
    return true;
  }

  // Alias often used in watchdog semantics
  bool kick(double now_sec)
  {
    return update(now_sec);
  }

  // -------------------------
  // Queries
  // -------------------------
  // Age since last update. Returns fallback if unavailable/invalid.
  double age_since_update(double now_sec, double fallback = 0.0) const
  {
    if (!has_seen_update_ || !is_valid_time_(now_sec)) {
      return fallback;
    }
    return now_sec - last_update_sec_;
  }

  // Fresh means "not stale" and source considered present.
  bool is_fresh(double now_sec) const
  {
    return compute_state_(now_sec).fresh;
  }

  bool is_stale(double now_sec) const
  {
    return compute_state_(now_sec).stale;
  }

  // Full status with edge detection (updates internal edge baseline)
  WatchdogStatus check(double now_sec)
  {
    WatchdogStatus s = compute_state_(now_sec);

    if (has_prev_check_state_) {
      s.became_fresh = (!prev_fresh_ && s.fresh);
      s.became_stale = (!prev_stale_ && s.stale);
    } else {
      s.became_fresh = false;
      s.became_stale = false;
    }

    prev_fresh_ = s.fresh;
    prev_stale_ = s.stale;
    has_prev_check_state_ = s.time_valid;  // only lock baseline on valid checks
    return s;
  }

private:
  // Internal, non-mutating state computation
  WatchdogStatus compute_state_(double now_sec) const
  {
    WatchdogStatus s{};
    s.initialized = initialized_;
    s.has_seen_update = has_seen_update_;
    s.now_sec = now_sec;
    s.start_time_sec = start_time_sec_;
    s.last_update_sec = last_update_sec_;
    s.time_valid = is_valid_time_(now_sec);

    if (!s.time_valid) {
      // Safe default: invalid time -> treat as stale only if strict never-seen policy + no grace not applicable.
      s.fresh = false;
      s.stale = true;
      return s;
    }

    if (initialized_) {
      s.age_since_start_sec = now_sec - start_time_sec_;
      s.in_startup_grace =
        (config_.startup_grace_sec > 0.0) &&
        (s.age_since_start_sec >= 0.0) &&
        (s.age_since_start_sec < config_.startup_grace_sec);
    }

    if (has_seen_update_) {
      s.age_since_update_sec = now_sec - last_update_sec_;

      // If time appears to go backwards, fail safe to stale.
      if (s.age_since_update_sec < 0.0) {
        s.fresh = false;
        s.stale = true;
        return s;
      }

      // timeout <= 0 means no timeout after first valid update
      if (config_.timeout_sec <= 0.0) {
        s.fresh = true;
        s.stale = false;
        return s;
      }

      s.stale = (s.age_since_update_sec > config_.timeout_sec);
      s.fresh = !s.stale;
      return s;
    }

    // Never seen update yet
    if (s.in_startup_grace) {
      s.fresh = false;
      s.stale = false;  // grace period suppresses stale state
      return s;
    }

    if (config_.stale_if_never_seen) {
      s.fresh = false;
      s.stale = true;
    } else {
      s.fresh = false;
      s.stale = false;
    }

    return s;
  }

  bool is_valid_time_(double t) const
  {
    if (!std::isfinite(t)) {
      return false;
    }
    if (t < config_.min_time_sec) {
      return false;
    }
    return true;
  }

  void normalize_config_()
  {
    if (!std::isfinite(config_.timeout_sec)) {
      config_.timeout_sec = 0.5;
    }
    if (!std::isfinite(config_.startup_grace_sec) || config_.startup_grace_sec < 0.0) {
      config_.startup_grace_sec = 0.0;
    }
    if (!std::isfinite(config_.min_time_sec)) {
      config_.min_time_sec = 0.0;
    }
    if (!std::isfinite(config_.max_time_jump_sec)) {
      config_.max_time_jump_sec = 0.0;
    }
    if (config_.max_time_jump_sec < 0.0) {
      config_.max_time_jump_sec = 0.0;
    }
  }

private:
  WatchdogConfig config_ {};

  // Core timestamps/state
  bool initialized_ {false};
  bool has_seen_update_ {false};
  double start_time_sec_ {0.0};
  double last_update_sec_ {0.0};

  // Edge detection memory (updated by check())
  bool prev_fresh_ {false};
  bool prev_stale_ {false};
  bool has_prev_check_state_ {false};
};

}  // namespace savo_control