#pragma once

// =============================================================================
// Robot SAVO — savo_control / stuck_detector.hpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Reusable ROS-independent stuck / no-progress detector for Robot Savo.
//
// Intended use in:
//   - recovery_manager_node.cpp
//   - auto_test_manager_node.py / C++ equivalents (through wrapped logic)
//   - future navigation-support control nodes
//
// What it does
// ------------
// Detects "robot is commanded to move but is not making expected progress"
// using simple, robust heuristics:
//
//   1) Command activity gating (ignore when robot is intentionally stopped)
//   2) Low-motion persistence check (odom speed too low for too long)
//   3) Optional progress-distance check (pose change too small over window)
//   4) Optional safety-stop context pass-through for richer diagnostics
//
// Design note
// -----------
// This class does NOT read ROS topics directly. Nodes should feed in:
//   - current time (seconds)
//   - commanded velocities (vx, vy, wz)
//   - measured odom velocities (vx, vy, wz) OR pose deltas
//   - optional safety stop state
//
// It returns a compact status/result that nodes can publish or pass to
// RecoveryManager.
// =============================================================================

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace savo_control
{

// -----------------------------------------------------------------------------
// Basic motion containers (ROS-independent)
// -----------------------------------------------------------------------------
struct Twist2D
{
  double vx {0.0};  // m/s
  double vy {0.0};  // m/s
  double wz {0.0};  // rad/s
};

struct Pose2D
{
  double x {0.0};     // m
  double y {0.0};     // m
  double yaw {0.0};   // rad
};

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
struct StuckDetectorConfig
{
  // ---------- Time validity ----------
  double min_time_sec {0.0};
  double max_time_jump_sec {2.0};  // fail-safe reset on bad time jump if > 0

  // ---------- Command activity gating ----------
  // Robot is considered "commanded to move" if ANY axis exceeds threshold.
  double cmd_lin_active_mps {0.03};      // |vx| or |vy|
  double cmd_ang_active_radps {0.15};    // |wz|

  // Optional weighted command magnitude gate (additional criterion)
  // cmd_mag = hypot(vx, vy) + wz_weight * |wz|
  double cmd_mag_active_threshold {0.04};
  double wz_weight_for_mag {0.20};

  // ---------- Measured motion thresholds (odom / fused state) ----------
  // Robot is considered "actually moving" if any measured axis exceeds these.
  double meas_lin_moving_mps {0.015};
  double meas_ang_moving_radps {0.08};

  // Optional weighted measured magnitude gate
  double meas_mag_moving_threshold {0.02};

  // ---------- Persistence / debounce ----------
  // Command must be active for at least this long before we evaluate stuck.
  double command_grace_sec {0.25};

  // If commanded active and measured motion remains below thresholds for this
  // duration, flag no-progress/stuck.
  double no_motion_confirm_sec {0.70};

  // Clear stuck only after measured motion is healthy for this duration
  // (or command becomes inactive).
  double clear_confirm_sec {0.25};

  // ---------- Pose-window progress check (optional but useful) ----------
  // If enabled, detector also checks whether pose moved enough while commanded.
  bool enable_pose_progress_check {true};

  // Window length over which progress is evaluated (seconds)
  double pose_window_sec {0.80};

  // Minimum required XY translation over the pose window while commanded active
  double min_progress_dist_m {0.015};

  // Minimum required yaw change over the pose window for rotate-only commands
  double min_progress_yaw_rad {0.05};

  // Consider command "rotation-dominant" if angular dominates and linear small.
  double rotate_only_lin_max_mps {0.03};
  double rotate_only_ang_min_radps {0.18};

  // ---------- Safety context ----------
  // If true, when safety stop is active, detector will not assert "stuck"
  // (because the robot is intentionally prevented from moving), but it still
  // reports no_progress_context for diagnostics.
  bool suppress_stuck_while_safety_stop {true};

  // ---------- Startup behavior ----------
  double startup_grace_sec {0.30};
};

// -----------------------------------------------------------------------------
// Inputs per update
// -----------------------------------------------------------------------------
struct StuckDetectorInputs
{
  double now_sec {0.0};

  // Commanded motion (e.g., cmd_vel_safe preferred, or cmd_vel if you want to
  // detect pre-safety behavior depending on node role)
  Twist2D cmd {};

  // Measured motion from odom / EKF (recommended: /odometry/filtered twist)
  Twist2D meas {};

  // Optional pose for window-based progress check
  bool has_pose {false};
  Pose2D pose {};

  // Optional context
  bool safety_stop_active {false};
};

// -----------------------------------------------------------------------------
// Status / result snapshot
// -----------------------------------------------------------------------------
struct StuckDetectorStatus
{
  // Core outputs
  bool command_active {false};      // command says robot should be moving
  bool measured_moving {false};     // odom says robot is moving
  bool no_progress {false};         // command active but no sufficient movement
  bool stuck {false};               // debounced decision for recovery manager

  // Context
  bool safety_stop_active {false};
  bool suppressed_by_safety {false};

  // State / validity
  bool initialized {false};
  bool time_valid {false};
  bool in_startup_grace {false};

  // Edges (true only on transition update)
  bool became_stuck {false};
  bool cleared_stuck {false};

  // Timers / ages
  double now_sec {0.0};
  double command_active_elapsed_sec {0.0};
  double no_motion_elapsed_sec {0.0};
  double moving_elapsed_sec {0.0};
  double since_start_sec {0.0};

  // Progress window diagnostics
  bool pose_progress_check_used {false};
  double pose_window_elapsed_sec {0.0};
  double pose_progress_dist_m {0.0};
  double pose_progress_yaw_rad {0.0};
  bool rotate_only_command {false};

  // Magnitudes for debugging/tuning
  double cmd_lin_mag_mps {0.0};
  double cmd_ang_abs_radps {0.0};
  double cmd_weighted_mag {0.0};

  double meas_lin_mag_mps {0.0};
  double meas_ang_abs_radps {0.0};
  double meas_weighted_mag {0.0};
};

// -----------------------------------------------------------------------------
// Detector
// -----------------------------------------------------------------------------
class StuckDetector
{
public:
  StuckDetector() = default;

  explicit StuckDetector(const StuckDetectorConfig & cfg)
  : config_(cfg)
  {
    normalize_config_();
  }

  // -------------------------
  // Configuration
  // -------------------------
  void set_config(const StuckDetectorConfig & cfg)
  {
    config_ = cfg;
    normalize_config_();
  }

  const StuckDetectorConfig & config() const
  {
    return config_;
  }

  // -------------------------
  // Lifecycle
  // -------------------------
  void reset()
  {
    initialized_ = false;
    start_time_sec_ = 0.0;
    last_now_sec_ = 0.0;

    command_active_ = false;
    command_active_start_sec_ = 0.0;

    no_motion_active_ = false;
    no_motion_start_sec_ = 0.0;

    moving_active_ = false;
    moving_start_sec_ = 0.0;

    stuck_ = false;

    pose_window_has_start_ = false;
    pose_window_start_sec_ = 0.0;
    pose_window_start_pose_ = Pose2D{};
  }

  void initialize(double now_sec)
  {
    if (!valid_time_(now_sec)) {
      return;
    }

    initialized_ = true;
    start_time_sec_ = now_sec;
    last_now_sec_ = now_sec;

    // Baseline timers
    command_active_ = false;
    no_motion_active_ = false;
    moving_active_ = false;
    stuck_ = false;

    pose_window_has_start_ = false;
  }

  // -------------------------
  // Main update
  // -------------------------
  StuckDetectorStatus update(const StuckDetectorInputs & in)
  {
    StuckDetectorStatus s{};
    s.now_sec = in.now_sec;
    s.safety_stop_active = in.safety_stop_active;
    s.time_valid = valid_time_(in.now_sec);

    if (!s.time_valid) {
      // Invalid time -> fail-safe status snapshot (do not mutate)
      s.initialized = initialized_;
      s.stuck = stuck_;
      return s;
    }

    if (!initialized_) {
      initialize(in.now_sec);
    }

    // Time jump guard
    if (config_.max_time_jump_sec > 0.0) {
      const double jump = in.now_sec - last_now_sec_;
      if (jump < 0.0 || jump > config_.max_time_jump_sec) {
        // Reset timing state but preserve initialization at current time
        reset();
        initialize(in.now_sec);
      }
    }
    last_now_sec_ = in.now_sec;

    s.initialized = initialized_;
    s.since_start_sec = in.now_sec - start_time_sec_;
    s.in_startup_grace = (config_.startup_grace_sec > 0.0) &&
                         (s.since_start_sec >= 0.0) &&
                         (s.since_start_sec < config_.startup_grace_sec);

    // Compute magnitudes
    s.cmd_lin_mag_mps = hypot2_(in.cmd.vx, in.cmd.vy);
    s.cmd_ang_abs_radps = std::abs(in.cmd.wz);
    s.cmd_weighted_mag =
      s.cmd_lin_mag_mps + std::abs(config_.wz_weight_for_mag) * s.cmd_ang_abs_radps;

    s.meas_lin_mag_mps = hypot2_(in.meas.vx, in.meas.vy);
    s.meas_ang_abs_radps = std::abs(in.meas.wz);
    s.meas_weighted_mag =
      s.meas_lin_mag_mps + std::abs(config_.wz_weight_for_mag) * s.meas_ang_abs_radps;

    // Command active gate (axis thresholds OR weighted magnitude)
    const bool cmd_axis_active =
      (s.cmd_lin_mag_mps >= config_.cmd_lin_active_mps) ||
      (s.cmd_ang_abs_radps >= config_.cmd_ang_active_radps);

    const bool cmd_mag_active =
      (s.cmd_weighted_mag >= config_.cmd_mag_active_threshold);

    s.command_active = (cmd_axis_active || cmd_mag_active);

    // Measured moving gate (axis thresholds OR weighted magnitude)
    const bool meas_axis_moving =
      (s.meas_lin_mag_mps >= config_.meas_lin_moving_mps) ||
      (s.meas_ang_abs_radps >= config_.meas_ang_moving_radps);

    const bool meas_mag_moving =
      (s.meas_weighted_mag >= config_.meas_mag_moving_threshold);

    s.measured_moving = (meas_axis_moving || meas_mag_moving);

    // Rotation-only command classification (for pose progress check)
    s.rotate_only_command =
      (s.cmd_lin_mag_mps <= config_.rotate_only_lin_max_mps) &&
      (s.cmd_ang_abs_radps >= config_.rotate_only_ang_min_radps);

    // Update command-active timer state
    update_boolean_timer_(s.command_active, in.now_sec, command_active_, command_active_start_sec_);
    s.command_active_elapsed_sec = command_active_ ? (in.now_sec - command_active_start_sec_) : 0.0;

    // Update moving timer state
    update_boolean_timer_(s.measured_moving, in.now_sec, moving_active_, moving_start_sec_);
    s.moving_elapsed_sec = moving_active_ ? (in.now_sec - moving_start_sec_) : 0.0;

    // Pose progress window (optional)
    bool pose_progress_ok = true;  // neutral if disabled/not available
    if (config_.enable_pose_progress_check && in.has_pose) {
      s.pose_progress_check_used = true;
      update_pose_window_(in.now_sec, in.pose, s.command_active);

      if (pose_window_has_start_) {
        s.pose_window_elapsed_sec = in.now_sec - pose_window_start_sec_;
        s.pose_progress_dist_m = hypot2_(
          in.pose.x - pose_window_start_pose_.x,
          in.pose.y - pose_window_start_pose_.y);
        s.pose_progress_yaw_rad = std::abs(wrap_angle_rad_(in.pose.yaw - pose_window_start_pose_.yaw));

        // Evaluate progress only after window matured
        if (s.pose_window_elapsed_sec >= config_.pose_window_sec) {
          if (s.rotate_only_command) {
            pose_progress_ok = (s.pose_progress_yaw_rad >= config_.min_progress_yaw_rad);
          } else {
            pose_progress_ok = (s.pose_progress_dist_m >= config_.min_progress_dist_m);
          }
        } else {
          // Window not mature yet -> do not force failure
          pose_progress_ok = true;
        }
      }
    } else {
      // If pose progress check is enabled but no pose available, degrade gracefully:
      // rely on measured twist only.
      s.pose_progress_check_used = false;
      pose_progress_ok = true;
      if (!s.command_active) {
        pose_window_has_start_ = false;
      }
    }

    // "No progress" condition (raw, before safety suppression)
    bool no_progress_raw = false;

    const bool command_grace_done =
      s.command_active && (s.command_active_elapsed_sec >= config_.command_grace_sec);

    if (!s.in_startup_grace && command_grace_done) {
      // If measured is not moving OR pose progress window says insufficient progress,
      // accumulate no-motion persistence.
      no_progress_raw = (!s.measured_moving) || (!pose_progress_ok);
    }

    // Update no-motion timer state
    update_boolean_timer_(no_progress_raw, in.now_sec, no_motion_active_, no_motion_start_sec_);
    s.no_motion_elapsed_sec = no_motion_active_ ? (in.now_sec - no_motion_start_sec_) : 0.0;

    // Debounced no_progress
    s.no_progress = no_progress_raw &&
                    (s.no_motion_elapsed_sec >= config_.no_motion_confirm_sec);

    // Safety suppression behavior
    s.suppressed_by_safety = false;
    bool candidate_stuck = s.no_progress;

    if (config_.suppress_stuck_while_safety_stop && in.safety_stop_active) {
      if (candidate_stuck) {
        s.suppressed_by_safety = true;
      }
      candidate_stuck = false;
    }

    // Stuck latch/debounce with clear hysteresis
    const bool prev_stuck = stuck_;

    if (!s.command_active) {
      // If no command, clear immediately (robot is not expected to move)
      stuck_ = false;
    } else if (candidate_stuck) {
      stuck_ = true;
    } else {
      // Clear only after healthy movement persists OR command becomes inactive
      const bool healthy_motion = s.measured_moving && (s.moving_elapsed_sec >= config_.clear_confirm_sec);
      if (healthy_motion) {
        stuck_ = false;
      }
    }

    s.stuck = stuck_;
    s.became_stuck = (!prev_stuck && s.stuck);
    s.cleared_stuck = (prev_stuck && !s.stuck);

    return s;
  }

private:
  // -------------------------
  // Helpers
  // -------------------------
  void normalize_config_()
  {
    auto sane_nonneg = [](double v, double fallback) -> double {
      return (std::isfinite(v) && v >= 0.0) ? v : fallback;
    };

    config_.min_time_sec = sane_nonneg(config_.min_time_sec, 0.0);
    config_.max_time_jump_sec = sane_nonneg(config_.max_time_jump_sec, 2.0);

    config_.cmd_lin_active_mps = sane_nonneg(config_.cmd_lin_active_mps, 0.03);
    config_.cmd_ang_active_radps = sane_nonneg(config_.cmd_ang_active_radps, 0.15);
    config_.cmd_mag_active_threshold = sane_nonneg(config_.cmd_mag_active_threshold, 0.04);

    if (!std::isfinite(config_.wz_weight_for_mag)) {
      config_.wz_weight_for_mag = 0.20;
    } else {
      config_.wz_weight_for_mag = std::abs(config_.wz_weight_for_mag);
    }

    config_.meas_lin_moving_mps = sane_nonneg(config_.meas_lin_moving_mps, 0.015);
    config_.meas_ang_moving_radps = sane_nonneg(config_.meas_ang_moving_radps, 0.08);
    config_.meas_mag_moving_threshold = sane_nonneg(config_.meas_mag_moving_threshold, 0.02);

    config_.command_grace_sec = sane_nonneg(config_.command_grace_sec, 0.25);
    config_.no_motion_confirm_sec = sane_nonneg(config_.no_motion_confirm_sec, 0.70);
    config_.clear_confirm_sec = sane_nonneg(config_.clear_confirm_sec, 0.25);

    config_.pose_window_sec = sane_nonneg(config_.pose_window_sec, 0.80);
    config_.min_progress_dist_m = sane_nonneg(config_.min_progress_dist_m, 0.015);
    config_.min_progress_yaw_rad = sane_nonneg(config_.min_progress_yaw_rad, 0.05);

    config_.rotate_only_lin_max_mps = sane_nonneg(config_.rotate_only_lin_max_mps, 0.03);
    config_.rotate_only_ang_min_radps = sane_nonneg(config_.rotate_only_ang_min_radps, 0.18);

    config_.startup_grace_sec = sane_nonneg(config_.startup_grace_sec, 0.30);

    if (config_.pose_window_sec <= 0.0) {
      config_.enable_pose_progress_check = false;
    }

    // Keep clear debounce <= no_motion confirm in typical tuning
    if (config_.clear_confirm_sec > config_.no_motion_confirm_sec && config_.no_motion_confirm_sec > 0.0) {
      config_.clear_confirm_sec = config_.no_motion_confirm_sec;
    }
  }

  bool valid_time_(double t) const
  {
    return std::isfinite(t) && (t >= config_.min_time_sec);
  }

  static double hypot2_(double a, double b)
  {
    if (!std::isfinite(a) || !std::isfinite(b)) {
      return 0.0;
    }
    return std::hypot(a, b);
  }

  static double wrap_angle_rad_(double a)
  {
    if (!std::isfinite(a)) {
      return 0.0;
    }
    constexpr double kPi = 3.1415926535897932384626433832795;
    constexpr double kTwoPi = 2.0 * kPi;
    a = std::fmod(a + kPi, kTwoPi);
    if (a < 0.0) {
      a += kTwoPi;
    }
    return a - kPi;
  }

  static void update_boolean_timer_(
    bool state_now,
    double now_sec,
    bool & state_latched,
    double & state_start_sec)
  {
    if (state_now) {
      if (!state_latched) {
        state_latched = true;
        state_start_sec = now_sec;
      }
    } else {
      state_latched = false;
      state_start_sec = 0.0;
    }
  }

  void update_pose_window_(double now_sec, const Pose2D & pose, bool command_active_now)
  {
    if (!command_active_now) {
      pose_window_has_start_ = false;
      return;
    }

    if (!pose_window_has_start_) {
      pose_window_has_start_ = true;
      pose_window_start_sec_ = now_sec;
      pose_window_start_pose_ = pose;
      return;
    }

    // Slide window anchor when window exceeds configured duration.
    // This keeps the window bounded and responsive.
    const double elapsed = now_sec - pose_window_start_sec_;
    if (elapsed > config_.pose_window_sec) {
      pose_window_start_sec_ = now_sec;
      pose_window_start_pose_ = pose;
    }
  }

private:
  StuckDetectorConfig config_ {};

  // Lifecycle / time
  bool initialized_ {false};
  double start_time_sec_ {0.0};
  double last_now_sec_ {0.0};

  // Boolean timers / latches
  bool command_active_ {false};
  double command_active_start_sec_ {0.0};

  bool no_motion_active_ {false};
  double no_motion_start_sec_ {0.0};

  bool moving_active_ {false};
  double moving_start_sec_ {0.0};

  // Stuck latched state
  bool stuck_ {false};

  // Pose progress window
  bool pose_window_has_start_ {false};
  double pose_window_start_sec_ {0.0};
  Pose2D pose_window_start_pose_ {};
};

}  // namespace savo_control