// =============================================================================
// Robot SAVO — savo_control / src/nodes/heading_pid_node.cpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Closed-loop heading (yaw) controller node for Robot SAVO using the reusable
// `HeadingController` + `Pid` utilities.
//
// Typical use cases
// -----------------
// - Heading hold while driving (keep robot facing same direction)
// - Rotate-to-heading tests (publish target yaw and let node generate angular.z)
// - PID tuning on real robot before deeper Nav2 integration
//
// Typical pipeline example
// ------------------------
//   base cmd source (/cmd_vel_raw or test pattern linear commands)
//          + /odometry/filtered (yaw feedback)
//          + /savo_control/heading_target (optional)
//          + /savo_control/heading_hold_enable
//      --> heading_pid_node --> /cmd_vel_auto
//      --> twist_mux_node (AUTO mode) --> cmd_vel_shaper_node --> /cmd_vel
//      --> safety gate (savo_perception) --> /cmd_vel_safe
//
// Notes
// -----
// - This node controls angular.z only (heading / yaw).
// - linear.x and linear.y can be passed through from a base command topic.
// - Mecanum-friendly: vy pass-through is supported.
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "savo_control/heading_controller.hpp"
#include "savo_control/topic_names.hpp"

using namespace std::chrono_literals;

namespace savo_control
{

class HeadingPidNode : public rclcpp::Node
{
public:
  HeadingPidNode()
  : Node("heading_pid_node")
  {
    // -------------------------------------------------------------------------
    // Parameters: topics
    // -------------------------------------------------------------------------
    odom_topic_ = this->declare_parameter<std::string>(
      "odom_topic", topic_names::kOdomFiltered);

    base_cmd_topic_ = this->declare_parameter<std::string>(
      "base_cmd_topic", topic_names::kCmdVelRaw);

    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", topic_names::kCmdVelAuto);

    heading_target_topic_ = this->declare_parameter<std::string>(
      "heading_target_topic", topic_names::kHeadingTarget);

    heading_hold_enable_topic_ = this->declare_parameter<std::string>(
      "heading_hold_enable_topic", topic_names::kHeadingHoldEnable);

    state_topic_ = this->declare_parameter<std::string>(
      "state_topic", topic_names::kRotateState);

    // -------------------------------------------------------------------------
    // Parameters: loop + freshness
    // -------------------------------------------------------------------------
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 30.0);
    odom_timeout_sec_ = this->declare_parameter<double>("odom_timeout_sec", 0.30);
    base_cmd_timeout_sec_ = this->declare_parameter<double>("base_cmd_timeout_sec", 0.30);
    zero_on_stale_odom_ = this->declare_parameter<bool>("zero_on_stale_odom", true);
    zero_linear_on_stale_base_cmd_ = this->declare_parameter<bool>("zero_linear_on_stale_base_cmd", true);

    // -------------------------------------------------------------------------
    // Parameters: behavior
    // -------------------------------------------------------------------------
    enable_linear_passthrough_ = this->declare_parameter<bool>("enable_linear_passthrough", true);
    passthrough_wz_when_disabled_ = this->declare_parameter<bool>("passthrough_wz_when_disabled", false);
    capture_hold_target_on_enable_ = this->declare_parameter<bool>("capture_hold_target_on_enable", true);
    require_target_or_hold_ = this->declare_parameter<bool>("require_target_or_hold", true);
    zero_wz_when_within_tolerance_ = this->declare_parameter<bool>("zero_wz_when_within_tolerance", true);

    // Optional fixed linear command mode (for simple heading tests)
    use_fixed_linear_cmd_ = this->declare_parameter<bool>("use_fixed_linear_cmd", false);
    fixed_vx_m_s_ = this->declare_parameter<double>("fixed_vx_m_s", 0.0);
    fixed_vy_m_s_ = this->declare_parameter<double>("fixed_vy_m_s", 0.0);

    // Output clamps (extra node-level safety clamps)
    out_vx_max_abs_ = this->declare_parameter<double>("output.vx_max_abs", 0.30);
    out_vy_max_abs_ = this->declare_parameter<double>("output.vy_max_abs", 0.20);
    out_wz_max_abs_ = this->declare_parameter<double>("output.wz_max_abs", 1.20);

    // -------------------------------------------------------------------------
    // Parameters: Heading controller config (PID + shaping)
    // -------------------------------------------------------------------------
    HeadingControllerConfig cfg{};

    cfg.pid.kp = this->declare_parameter<double>("pid.kp", 2.2);
    cfg.pid.ki = this->declare_parameter<double>("pid.ki", 0.0);
    cfg.pid.kd = this->declare_parameter<double>("pid.kd", 0.05);

    cfg.pid.output_min = this->declare_parameter<double>("pid.output_min", -1.0);
    cfg.pid.output_max = this->declare_parameter<double>("pid.output_max",  1.0);
    cfg.pid.integral_clamp = this->declare_parameter<double>("pid.integral_clamp", 0.5);
    cfg.pid.d_filter_alpha = this->declare_parameter<double>("pid.d_filter_alpha", 0.15);
    cfg.pid.min_dt_sec = this->declare_parameter<double>("pid.min_dt_sec", 1e-6);
    cfg.pid.max_dt_sec = this->declare_parameter<double>("pid.max_dt_sec", 0.5);
    cfg.pid.freeze_integral_on_invalid_dt =
      this->declare_parameter<bool>("pid.freeze_integral_on_invalid_dt", true);

    cfg.heading_tolerance_rad =
      this->declare_parameter<double>("heading_tolerance_rad", 0.03);
    cfg.output_deadband_rad_s =
      this->declare_parameter<double>("output_deadband_rad_s", 0.00);
    cfg.min_effective_wz_rad_s =
      this->declare_parameter<double>("min_effective_wz_rad_s", 0.00);
    cfg.max_wz_rad_s =
      this->declare_parameter<double>("max_wz_rad_s", 1.00);

    cfg.reset_pid_on_hold_capture =
      this->declare_parameter<bool>("reset_pid_on_hold_capture", true);
    cfg.reset_pid_on_target_change =
      this->declare_parameter<bool>("reset_pid_on_target_change", true);
    cfg.target_change_reset_threshold_rad =
      this->declare_parameter<double>("target_change_reset_threshold_rad", 0.02);

    cfg.min_dt_sec = this->declare_parameter<double>("ctrl.min_dt_sec", 1e-6);
    cfg.max_dt_sec = this->declare_parameter<double>("ctrl.max_dt_sec", 0.5);

    sanitize_params_();
    heading_ctrl_.set_config(cfg);

    // -------------------------------------------------------------------------
    // Publishers
    // -------------------------------------------------------------------------
    pub_cmd_out_ = this->create_publisher<geometry_msgs::msg::Twist>(
      output_topic_, rclcpp::QoS(10));

    pub_state_ = this->create_publisher<std_msgs::msg::String>(
      state_topic_, rclcpp::QoS(10).transient_local());

    // -------------------------------------------------------------------------
    // Subscribers
    // -------------------------------------------------------------------------
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&HeadingPidNode::on_odom_, this, std::placeholders::_1));

    sub_base_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
      base_cmd_topic_, rclcpp::QoS(10),
      std::bind(&HeadingPidNode::on_base_cmd_, this, std::placeholders::_1));

    sub_heading_target_ = this->create_subscription<std_msgs::msg::Float64>(
      heading_target_topic_, rclcpp::QoS(10),
      std::bind(&HeadingPidNode::on_heading_target_, this, std::placeholders::_1));

    sub_heading_hold_enable_ = this->create_subscription<std_msgs::msg::Bool>(
      heading_hold_enable_topic_, rclcpp::QoS(10),
      std::bind(&HeadingPidNode::on_heading_hold_enable_, this, std::placeholders::_1));

    // -------------------------------------------------------------------------
    // Timer loop
    // -------------------------------------------------------------------------
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&HeadingPidNode::on_timer_, this));

    publish_state_(true);

    RCLCPP_INFO(
      this->get_logger(),
      "heading_pid_node started | odom=%s | base_cmd=%s | out=%s | rate=%.1fHz",
      odom_topic_.c_str(), base_cmd_topic_.c_str(), output_topic_.c_str(), publish_rate_hz_);
  }

private:
  struct OdomState
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool has_msg{false};
    double yaw_rad{0.0};
  };

  struct TwistState
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool has_msg{false};
    geometry_msgs::msg::Twist msg{};
  };

  // ---------------------------------------------------------------------------
  // Callbacks
  // ---------------------------------------------------------------------------
  void on_odom_(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    double yaw = 0.0;
    if (!quat_to_yaw_(msg->pose.pose.orientation, yaw)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Invalid odometry orientation quaternion");
      return;
    }

    odom_.yaw_rad = yaw;
    odom_.stamp = this->now();
    odom_.has_msg = true;
  }

  void on_base_cmd_(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    base_cmd_.msg = sanitize_twist_(*msg);
    base_cmd_.stamp = this->now();
    base_cmd_.has_msg = true;
  }

  void on_heading_target_(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!msg || !std::isfinite(msg->data)) {
      return;
    }

    last_heading_target_rad_ = msg->data;
    has_heading_target_cmd_ = true;
    heading_target_stamp_ = this->now();

    heading_ctrl_.set_target_yaw(last_heading_target_rad_);

    RCLCPP_INFO(
      this->get_logger(),
      "Heading target set: %.3f rad (%.1f deg)",
      heading_ctrl_.target_yaw_rad(),
      heading_ctrl_.target_yaw_rad() * (180.0 / 3.14159265358979323846));
  }

  void on_heading_hold_enable_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    const bool prev = heading_hold_enabled_;
    heading_hold_enabled_ = msg->data;

    if (!prev && heading_hold_enabled_) {
      // Rising edge: capture current heading (if requested and odom is fresh)
      if (capture_hold_target_on_enable_ && is_odom_fresh_(this->now())) {
        heading_ctrl_.capture_hold_target(odom_.yaw_rad);
        has_heading_target_cmd_ = true;  // controller now has a valid target
        heading_target_stamp_ = this->now();
      }
      RCLCPP_INFO(this->get_logger(), "Heading hold ENABLED");
    } else if (prev && !heading_hold_enabled_) {
      RCLCPP_INFO(this->get_logger(), "Heading hold DISABLED");
      heading_ctrl_.reset();
    }

    publish_state_(true);
  }

  // ---------------------------------------------------------------------------
  // Main loop
  // ---------------------------------------------------------------------------
  void on_timer_()
  {
    const rclcpp::Time now = this->now();

    double dt_sec = 0.0;
    bool dt_valid = false;
    if (has_prev_loop_time_) {
      dt_sec = (now - prev_loop_time_).seconds();
      dt_valid = std::isfinite(dt_sec) &&
                 dt_sec >= heading_ctrl_.config().min_dt_sec &&
                 dt_sec <= heading_ctrl_.config().max_dt_sec;
    }
    prev_loop_time_ = now;
    has_prev_loop_time_ = true;

    geometry_msgs::msg::Twist out = zero_twist_();

    // Linear passthrough / fixed test command
    if (use_fixed_linear_cmd_) {
      out.linear.x = clamp_abs_(fixed_vx_m_s_, out_vx_max_abs_);
      out.linear.y = clamp_abs_(fixed_vy_m_s_, out_vy_max_abs_);
    } else if (enable_linear_passthrough_) {
      if (is_base_cmd_fresh_(now)) {
        out.linear.x = clamp_abs_(base_cmd_.msg.linear.x, out_vx_max_abs_);
        out.linear.y = clamp_abs_(base_cmd_.msg.linear.y, out_vy_max_abs_);
      } else if (!zero_linear_on_stale_base_cmd_ && base_cmd_.has_msg) {
        out.linear.x = clamp_abs_(base_cmd_.msg.linear.x, out_vx_max_abs_);
        out.linear.y = clamp_abs_(base_cmd_.msg.linear.y, out_vy_max_abs_);
      } else {
        out.linear.x = 0.0;
        out.linear.y = 0.0;
      }
    }

    // If no fresh odom, fail-safe behavior
    if (!is_odom_fresh_(now)) {
      if (zero_on_stale_odom_) {
        out.angular.z = 0.0;
        out = sanitize_twist_(out);
        pub_cmd_out_->publish(out);

        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1500,
          "Stale/missing odom -> zero angular.z (and continuing linear policy)");
        publish_state_(false);
        return;
      }
    }

    const bool can_control = odom_.has_msg && std::isfinite(odom_.yaw_rad);
    const bool has_target = heading_ctrl_.has_target();
    const bool control_active = heading_hold_enabled_ || has_target;

    if (!can_control || (require_target_or_hold_ && !control_active)) {
      if (passthrough_wz_when_disabled_ && is_base_cmd_fresh_(now)) {
        out.angular.z = clamp_abs_(base_cmd_.msg.angular.z, out_wz_max_abs_);
      } else {
        out.angular.z = 0.0;
      }

      out = sanitize_twist_(out);
      pub_cmd_out_->publish(out);
      publish_state_(false);
      return;
    }

    // If hold is enabled and there is no target yet, capture current heading
    if (heading_hold_enabled_ && !heading_ctrl_.has_target() && can_control) {
      heading_ctrl_.capture_hold_target(odom_.yaw_rad);
    }

    // Run heading controller
    HeadingControllerResult res{};
    if (dt_valid) {
      res = heading_ctrl_.update(odom_.yaw_rad, dt_sec);
    } else {
      // First loop / timing glitch -> safe zero angular command
      res.wz_cmd_rad_s = 0.0;
      res.valid = false;
    }

    double wz = (res.valid ? res.wz_cmd_rad_s : 0.0);

    if (!zero_wz_when_within_tolerance_ && res.valid && res.within_tolerance) {
      // If user wants no forced zero inside tolerance, use controller output (already computed)
      wz = res.wz_cmd_rad_s;
    }

    out.angular.z = clamp_abs_(wz, out_wz_max_abs_);
    out = sanitize_twist_(out);
    pub_cmd_out_->publish(out);

    // Debug logs
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "heading_pid: yaw=%.3f target=%s%.3f err=%.3f wz=%.3f dt=%.4f hold=%s",
      odom_.yaw_rad,
      heading_ctrl_.has_target() ? "" : "(none) ",
      heading_ctrl_.has_target() ? heading_ctrl_.target_yaw_rad() : 0.0,
      res.error_yaw_rad,
      out.angular.z,
      dt_sec,
      heading_hold_enabled_ ? "true" : "false");

    publish_state_(false);
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------
  void sanitize_params_()
  {
    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 30.0;
    }
    if (!std::isfinite(odom_timeout_sec_) || odom_timeout_sec_ <= 0.0) {
      odom_timeout_sec_ = 0.30;
    }
    if (!std::isfinite(base_cmd_timeout_sec_) || base_cmd_timeout_sec_ <= 0.0) {
      base_cmd_timeout_sec_ = 0.30;
    }

    out_vx_max_abs_ = std::isfinite(out_vx_max_abs_) ? std::abs(out_vx_max_abs_) : 0.30;
    out_vy_max_abs_ = std::isfinite(out_vy_max_abs_) ? std::abs(out_vy_max_abs_) : 0.20;
    out_wz_max_abs_ = std::isfinite(out_wz_max_abs_) ? std::abs(out_wz_max_abs_) : 1.20;

    fixed_vx_m_s_ = std::isfinite(fixed_vx_m_s_) ? fixed_vx_m_s_ : 0.0;
    fixed_vy_m_s_ = std::isfinite(fixed_vy_m_s_) ? fixed_vy_m_s_ : 0.0;

    if (odom_topic_.empty()) odom_topic_ = topic_names::kOdomFiltered;
    if (base_cmd_topic_.empty()) base_cmd_topic_ = topic_names::kCmdVelRaw;
    if (output_topic_.empty()) output_topic_ = topic_names::kCmdVelAuto;
    if (heading_target_topic_.empty()) heading_target_topic_ = topic_names::kHeadingTarget;
    if (heading_hold_enable_topic_.empty()) heading_hold_enable_topic_ = topic_names::kHeadingHoldEnable;
    if (state_topic_.empty()) state_topic_ = topic_names::kRotateState;
  }

  bool is_odom_fresh_(const rclcpp::Time & now) const
  {
    if (!odom_.has_msg) {
      return false;
    }
    const double age = (now - odom_.stamp).seconds();
    return std::isfinite(age) && age >= 0.0 && age <= odom_timeout_sec_;
  }

  bool is_base_cmd_fresh_(const rclcpp::Time & now) const
  {
    if (!base_cmd_.has_msg) {
      return false;
    }
    const double age = (now - base_cmd_.stamp).seconds();
    return std::isfinite(age) && age >= 0.0 && age <= base_cmd_timeout_sec_;
  }

  static double clamp_abs_(double v, double abs_max)
  {
    const double m = std::abs(abs_max);
    if (!(m > 0.0) || !std::isfinite(m) || !std::isfinite(v)) {
      return 0.0;
    }
    return std::max(-m, std::min(v, m));
  }

  static bool quat_to_yaw_(const geometry_msgs::msg::Quaternion & q_msg, double & yaw_rad)
  {
    if (!std::isfinite(q_msg.x) || !std::isfinite(q_msg.y) ||
        !std::isfinite(q_msg.z) || !std::isfinite(q_msg.w))
    {
      return false;
    }

    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    if (q.length2() <= 0.0 || !std::isfinite(q.length2())) {
      return false;
    }
    q.normalize();

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_rad = yaw;
    return std::isfinite(yaw_rad);
  }

  static geometry_msgs::msg::Twist sanitize_twist_(const geometry_msgs::msg::Twist & in)
  {
    geometry_msgs::msg::Twist out{};
    out.linear.x  = std::isfinite(in.linear.x)  ? in.linear.x  : 0.0;
    out.linear.y  = std::isfinite(in.linear.y)  ? in.linear.y  : 0.0;
    out.linear.z  = 0.0;
    out.angular.x = 0.0;
    out.angular.y = 0.0;
    out.angular.z = std::isfinite(in.angular.z) ? in.angular.z : 0.0;
    return out;
  }

  static geometry_msgs::msg::Twist zero_twist_()
  {
    return geometry_msgs::msg::Twist{};
  }

  void publish_state_(bool force)
  {
    std::string s = "hold=" + std::string(heading_hold_enabled_ ? "true" : "false");
    s += " odom=" + std::string(odom_.has_msg ? "yes" : "no");
    s += " target=" + std::string(heading_ctrl_.has_target() ? "yes" : "no");
    if (heading_ctrl_.has_target()) {
      s += " target_rad=" + std::to_string(heading_ctrl_.target_yaw_rad());
    }
    s += " out_topic=" + output_topic_;

    if (!force && s == last_state_str_) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = s;
    pub_state_->publish(msg);
    last_state_str_ = std::move(s);
  }

private:
  // Publishers/subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_out_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_base_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_heading_target_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_heading_hold_enable_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Topics
  std::string odom_topic_;
  std::string base_cmd_topic_;
  std::string output_topic_;
  std::string heading_target_topic_;
  std::string heading_hold_enable_topic_;
  std::string state_topic_;

  // Params
  double publish_rate_hz_{30.0};
  double odom_timeout_sec_{0.30};
  double base_cmd_timeout_sec_{0.30};
  bool zero_on_stale_odom_{true};
  bool zero_linear_on_stale_base_cmd_{true};

  bool enable_linear_passthrough_{true};
  bool passthrough_wz_when_disabled_{false};
  bool capture_hold_target_on_enable_{true};
  bool require_target_or_hold_{true};
  bool zero_wz_when_within_tolerance_{true};

  bool use_fixed_linear_cmd_{false};
  double fixed_vx_m_s_{0.0};
  double fixed_vy_m_s_{0.0};

  double out_vx_max_abs_{0.30};
  double out_vy_max_abs_{0.20};
  double out_wz_max_abs_{1.20};

  // Runtime state
  OdomState odom_{};
  TwistState base_cmd_{};

  HeadingController heading_ctrl_{};
  bool heading_hold_enabled_{false};

  bool has_heading_target_cmd_{false};
  double last_heading_target_rad_{0.0};
  rclcpp::Time heading_target_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Time prev_loop_time_{0, 0, RCL_ROS_TIME};
  bool has_prev_loop_time_{false};

  std::string last_state_str_;
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<savo_control::HeadingPidNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}