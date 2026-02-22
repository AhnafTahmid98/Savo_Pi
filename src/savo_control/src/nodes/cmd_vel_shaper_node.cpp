// =============================================================================
// Robot SAVO — savo_control / src/nodes/cmd_vel_shaper_node.cpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Shape / sanitize multiplexed velocity commands before they enter the safety
// gate. This node typically sits between:
//
//   twist_mux_node -> /cmd_vel_mux -> cmd_vel_shaper_node -> /cmd_vel
//                                                -> savo_perception safety gate
//                                                -> /cmd_vel_safe
//
// What this node does
// -------------------
// - Subscribes to /cmd_vel_mux (selected command source)
// - Applies command limiting (clamp + deadband + slew-rate) using CommandLimiter
// - Handles stale-input timeout and publishes zero on timeout
// - Supports optional hard stop from /safety/stop (disabled by default;
//   main authoritative safety gate remains in `savo_perception`)
// - Publishes shaped command to /cmd_vel
//
// Notes
// -----
// - This node is ROS-facing glue. The math/limiting logic stays in
//   `command_limiter.hpp`.
// - Current CommandLimiter is header-only, which is fine for this stage.
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "savo_control/command_limiter.hpp"
#include "savo_control/topic_names.hpp"

using namespace std::chrono_literals;

namespace savo_control
{

class CmdVelShaperNode : public rclcpp::Node
{
public:
  CmdVelShaperNode()
  : Node("cmd_vel_shaper_node")
  {
    // -------------------------------------------------------------------------
    // Parameters: loop + timeouts
    // -------------------------------------------------------------------------
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 30.0);
    input_timeout_sec_ = this->declare_parameter<double>("input_timeout_sec", 0.30);
    zero_on_stale_input_ = this->declare_parameter<bool>("zero_on_stale_input", true);

    // Optional local hard gate (main safety is still /cmd_vel_safe downstream)
    use_safety_stop_gate_ = this->declare_parameter<bool>("use_safety_stop_gate", false);
    safety_stop_hold_sec_ = this->declare_parameter<double>("safety_stop_hold_sec", 0.50);

    // -------------------------------------------------------------------------
    // Parameters: command limits (vx, vy, wz)
    // -------------------------------------------------------------------------
    // Absolute bounds
    vx_max_abs_ = this->declare_parameter<double>("limits.vx.max_abs", 0.30);
    vy_max_abs_ = this->declare_parameter<double>("limits.vy.max_abs", 0.20);
    wz_max_abs_ = this->declare_parameter<double>("limits.wz.max_abs", 1.20);

    // Deadbands
    vx_deadband_ = this->declare_parameter<double>("limits.vx.deadband", 0.00);
    vy_deadband_ = this->declare_parameter<double>("limits.vy.deadband", 0.00);
    wz_deadband_ = this->declare_parameter<double>("limits.wz.deadband", 0.00);

    // Slew rates (units/s)
    vx_rise_rate_ = this->declare_parameter<double>("limits.vx.max_rise_rate", 0.60);
    vx_fall_rate_ = this->declare_parameter<double>("limits.vx.max_fall_rate", 0.80);
    vy_rise_rate_ = this->declare_parameter<double>("limits.vy.max_rise_rate", 0.60);
    vy_fall_rate_ = this->declare_parameter<double>("limits.vy.max_fall_rate", 0.80);
    wz_rise_rate_ = this->declare_parameter<double>("limits.wz.max_rise_rate", 2.50);
    wz_fall_rate_ = this->declare_parameter<double>("limits.wz.max_fall_rate", 3.00);

    // Internal dt guards for limiter calls
    limiter_min_dt_sec_ = this->declare_parameter<double>("limiter.min_dt_sec", 1e-6);
    limiter_max_dt_sec_ = this->declare_parameter<double>("limiter.max_dt_sec", 0.50);

    // Logging behavior
    log_stale_warn_throttle_ms_ =
      this->declare_parameter<int>("log.stale_warn_throttle_ms", 2000);
    log_debug_throttle_ms_ =
      this->declare_parameter<int>("log.debug_throttle_ms", 1000);

    sanitize_params_();
    build_limiter_config_();

    // -------------------------------------------------------------------------
    // Publishers / Subscribers
    // -------------------------------------------------------------------------
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
      topic_names::kCmdVelShaped, rclcpp::QoS(10));

    sub_cmd_vel_mux_ = this->create_subscription<geometry_msgs::msg::Twist>(
      topic_names::kCmdVelMuxSelected, rclcpp::QoS(10),
      std::bind(&CmdVelShaperNode::on_cmd_vel_mux_, this, std::placeholders::_1));

    sub_safety_stop_ = this->create_subscription<std_msgs::msg::Bool>(
      topic_names::kSafetyStop, rclcpp::QoS(10),
      std::bind(&CmdVelShaperNode::on_safety_stop_, this, std::placeholders::_1));

    // -------------------------------------------------------------------------
    // Timer loop
    // -------------------------------------------------------------------------
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&CmdVelShaperNode::on_timer_, this));

    RCLCPP_INFO(
      this->get_logger(),
      "cmd_vel_shaper_node started | pub=%.1f Hz | timeout=%.2fs | safety_gate=%s | "
      "vx<=%.2f vy<=%.2f wz<=%.2f",
      publish_rate_hz_,
      input_timeout_sec_,
      use_safety_stop_gate_ ? "true" : "false",
      vx_max_abs_, vy_max_abs_, wz_max_abs_);
  }

private:
  // ---------------------------------------------------------------------------
  // Input channel state
  // ---------------------------------------------------------------------------
  struct TwistInputState
  {
    geometry_msgs::msg::Twist msg{};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool has_msg{false};
  };

  // ---------------------------------------------------------------------------
  // Subscribers callbacks
  // ---------------------------------------------------------------------------
  void on_cmd_vel_mux_(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    in_cmd_mux_.msg = sanitize_twist_(*msg);
    in_cmd_mux_.stamp = this->now();
    in_cmd_mux_.has_msg = true;
  }

  void on_safety_stop_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    safety_stop_active_ = msg->data;
    safety_stop_stamp_ = this->now();
  }

  // ---------------------------------------------------------------------------
  // Main timer loop
  // ---------------------------------------------------------------------------
  void on_timer_()
  {
    const rclcpp::Time now = this->now();

    // Compute dt from loop timing
    double dt_sec = 0.0;
    bool dt_valid = false;
    if (has_prev_loop_time_) {
      dt_sec = (now - prev_loop_time_).seconds();
      dt_valid = std::isfinite(dt_sec) &&
                 dt_sec >= limiter_min_dt_sec_ &&
                 dt_sec <= limiter_max_dt_sec_;
    }
    prev_loop_time_ = now;
    has_prev_loop_time_ = true;

    // Decide target command
    geometry_msgs::msg::Twist target_twist{};
    bool input_fresh = is_input_fresh_(in_cmd_mux_, now, input_timeout_sec_);

    if (input_fresh) {
      target_twist = in_cmd_mux_.msg;
    } else {
      if (zero_on_stale_input_) {
        target_twist = zero_twist_();
      } else if (in_cmd_mux_.has_msg) {
        // hold-last (not generally recommended for mobile robot motion)
        target_twist = in_cmd_mux_.msg;
      } else {
        target_twist = zero_twist_();
      }

      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(),
        std::max(0, log_stale_warn_throttle_ms_),
        "cmd_vel_shaper: input %s (timeout=%.2fs), publishing %s",
        in_cmd_mux_.has_msg ? "stale" : "missing",
        input_timeout_sec_,
        zero_on_stale_input_ ? "zero" : "hold-last/zero");
    }

    // Optional local hard stop gate (defensive; authoritative safety remains downstream)
    if (use_safety_stop_gate_ && is_recent_safety_stop_(now) && safety_stop_active_) {
      target_twist = zero_twist_();

      // Important safety behavior:
      // reset limiter state so release does not resume from stale nonzero history.
      limiter_.reset();
    }

    // Shape target command
    const MotionCommand target_cmd = twist_to_motion_(target_twist);

    MotionCommand shaped_cmd{};
    if (dt_valid) {
      shaped_cmd = limiter_.limit(target_cmd, dt_sec);
    } else {
      // First cycle or timing glitch:
      // publish clamp-only for safe deterministic behavior, then sync limiter state.
      shaped_cmd = CommandLimiter::clamp_only(target_cmd, limiter_cfg_);
      limiter_.reset_to(shaped_cmd);
    }

    // Publish
    const geometry_msgs::msg::Twist out = motion_to_twist_(shaped_cmd);
    pub_cmd_vel_->publish(out);

    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(),
      std::max(0, log_debug_throttle_ms_),
      "cmd_vel_shaper: in[vx=%.3f vy=%.3f wz=%.3f] -> out[vx=%.3f vy=%.3f wz=%.3f], dt=%.4f valid=%s fresh=%s",
      target_twist.linear.x, target_twist.linear.y, target_twist.angular.z,
      out.linear.x, out.linear.y, out.angular.z,
      dt_sec,
      dt_valid ? "true" : "false",
      input_fresh ? "true" : "false");
  }

  // ---------------------------------------------------------------------------
  // Limiter config
  // ---------------------------------------------------------------------------
  void build_limiter_config_()
  {
    CommandLimiterConfig cfg{};

    // vx
    cfg.vx.set_symmetric_bounds(vx_max_abs_);
    cfg.vx.deadband = std::abs(vx_deadband_);
    cfg.vx.max_rise_rate = std::abs(vx_rise_rate_);
    cfg.vx.max_fall_rate = std::abs(vx_fall_rate_);

    // vy
    cfg.vy.set_symmetric_bounds(vy_max_abs_);
    cfg.vy.deadband = std::abs(vy_deadband_);
    cfg.vy.max_rise_rate = std::abs(vy_rise_rate_);
    cfg.vy.max_fall_rate = std::abs(vy_fall_rate_);

    // wz
    cfg.wz.set_symmetric_bounds(wz_max_abs_);
    cfg.wz.deadband = std::abs(wz_deadband_);
    cfg.wz.max_rise_rate = std::abs(wz_rise_rate_);
    cfg.wz.max_fall_rate = std::abs(wz_fall_rate_);

    limiter_cfg_ = cfg;
    limiter_.set_config(limiter_cfg_);
    limiter_.reset();
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------
  void sanitize_params_()
  {
    if (!(publish_rate_hz_ > 0.0) || !std::isfinite(publish_rate_hz_)) {
      publish_rate_hz_ = 30.0;
    }
    if (!(input_timeout_sec_ > 0.0) || !std::isfinite(input_timeout_sec_)) {
      input_timeout_sec_ = 0.30;
    }
    if (!(safety_stop_hold_sec_ > 0.0) || !std::isfinite(safety_stop_hold_sec_)) {
      safety_stop_hold_sec_ = 0.50;
    }

    // limits / deadbands / rates
    vx_max_abs_ = finite_nonneg_or_(vx_max_abs_, 0.30);
    vy_max_abs_ = finite_nonneg_or_(vy_max_abs_, 0.20);
    wz_max_abs_ = finite_nonneg_or_(wz_max_abs_, 1.20);

    vx_deadband_ = finite_nonneg_or_(vx_deadband_, 0.0);
    vy_deadband_ = finite_nonneg_or_(vy_deadband_, 0.0);
    wz_deadband_ = finite_nonneg_or_(wz_deadband_, 0.0);

    vx_rise_rate_ = finite_nonneg_or_(vx_rise_rate_, 0.60);
    vx_fall_rate_ = finite_nonneg_or_(vx_fall_rate_, 0.80);
    vy_rise_rate_ = finite_nonneg_or_(vy_rise_rate_, 0.60);
    vy_fall_rate_ = finite_nonneg_or_(vy_fall_rate_, 0.80);
    wz_rise_rate_ = finite_nonneg_or_(wz_rise_rate_, 2.50);
    wz_fall_rate_ = finite_nonneg_or_(wz_fall_rate_, 3.00);

    limiter_min_dt_sec_ = finite_positive_or_(limiter_min_dt_sec_, 1e-6);
    limiter_max_dt_sec_ = finite_positive_or_(limiter_max_dt_sec_, 0.50);
    if (limiter_max_dt_sec_ < limiter_min_dt_sec_) {
      limiter_max_dt_sec_ = std::max(0.50, limiter_min_dt_sec_);
    }

    if (log_stale_warn_throttle_ms_ < 0) {
      log_stale_warn_throttle_ms_ = 2000;
    }
    if (log_debug_throttle_ms_ < 0) {
      log_debug_throttle_ms_ = 1000;
    }
  }

  static double finite_nonneg_or_(double v, double fallback)
  {
    return (std::isfinite(v) && v >= 0.0) ? v : fallback;
  }

  static double finite_positive_or_(double v, double fallback)
  {
    return (std::isfinite(v) && v > 0.0) ? v : fallback;
  }

  static bool finite_(double v)
  {
    return std::isfinite(v);
  }

  static geometry_msgs::msg::Twist zero_twist_()
  {
    return geometry_msgs::msg::Twist{};
  }

  geometry_msgs::msg::Twist sanitize_twist_(const geometry_msgs::msg::Twist & in) const
  {
    geometry_msgs::msg::Twist out{};

    // Keep only the fields used in Robot SAVO cmd_vel path.
    out.linear.x  = finite_(in.linear.x)  ? in.linear.x  : 0.0;
    out.linear.y  = finite_(in.linear.y)  ? in.linear.y  : 0.0;
    out.angular.z = finite_(in.angular.z) ? in.angular.z : 0.0;

    // Zero unused components
    out.linear.z  = 0.0;
    out.angular.x = 0.0;
    out.angular.y = 0.0;

    return out;
  }

  static MotionCommand twist_to_motion_(const geometry_msgs::msg::Twist & t)
  {
    MotionCommand m;
    m.vx = t.linear.x;
    m.vy = t.linear.y;
    m.wz = t.angular.z;
    return m;
  }

  static geometry_msgs::msg::Twist motion_to_twist_(const MotionCommand & m)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = m.vx;
    t.linear.y = m.vy;
    t.linear.z = 0.0;
    t.angular.x = 0.0;
    t.angular.y = 0.0;
    t.angular.z = m.wz;
    return t;
  }

  bool is_input_fresh_(
    const TwistInputState & st,
    const rclcpp::Time & now,
    double timeout_sec) const
  {
    if (!st.has_msg) {
      return false;
    }
    if (!(timeout_sec > 0.0) || !std::isfinite(timeout_sec)) {
      return true;
    }
    const double age = (now - st.stamp).seconds();
    return std::isfinite(age) && age >= 0.0 && age <= timeout_sec;
  }

  bool is_recent_safety_stop_(const rclcpp::Time & now) const
  {
    if (safety_stop_stamp_.nanoseconds() == 0) {
      return false;
    }
    const double age = (now - safety_stop_stamp_).seconds();
    return std::isfinite(age) && age >= 0.0 && age <= safety_stop_hold_sec_;
  }

private:
  // Publishers/Subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_mux_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_safety_stop_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Input state
  TwistInputState in_cmd_mux_;
  bool safety_stop_active_{false};
  rclcpp::Time safety_stop_stamp_{0, 0, RCL_ROS_TIME};

  // Loop timing
  rclcpp::Time prev_loop_time_{0, 0, RCL_ROS_TIME};
  bool has_prev_loop_time_{false};

  // Limiter
  CommandLimiter limiter_{};
  CommandLimiterConfig limiter_cfg_{};

  // Params: general
  double publish_rate_hz_{30.0};
  double input_timeout_sec_{0.30};
  bool zero_on_stale_input_{true};

  // Params: optional local safety gate
  bool use_safety_stop_gate_{false};
  double safety_stop_hold_sec_{0.50};

  // Params: limits
  double vx_max_abs_{0.30};
  double vy_max_abs_{0.20};
  double wz_max_abs_{1.20};

  double vx_deadband_{0.00};
  double vy_deadband_{0.00};
  double wz_deadband_{0.00};

  double vx_rise_rate_{0.60};
  double vx_fall_rate_{0.80};
  double vy_rise_rate_{0.60};
  double vy_fall_rate_{0.80};
  double wz_rise_rate_{2.50};
  double wz_fall_rate_{3.00};

  // Params: dt guard for limiter
  double limiter_min_dt_sec_{1e-6};
  double limiter_max_dt_sec_{0.50};

  // Params: logging
  int log_stale_warn_throttle_ms_{2000};
  int log_debug_throttle_ms_{1000};
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<savo_control::CmdVelShaperNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}