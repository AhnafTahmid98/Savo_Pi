// =============================================================================
// Robot SAVO — savo_control / src/nodes/rotate_to_heading_node.cpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Professional rotate-in-place primitive for Robot SAVO using the reusable
// HeadingController (PID-based yaw control).
//
// This node is ideal for:
// - PID tuning of yaw control on real robot
// - rotate-to-angle validation before Nav2 behaviors
// - recovery primitives (future integration with recovery_manager)
// - heading alignment tests using EKF / odometry feedback
//
// Behavior summary
// ----------------
// - Subscribes to odometry yaw (default: /odometry/filtered)
// - Accepts target heading in radians on /savo_control/rotate_target (Float64)
// - Publishes angular.z-only Twist to /cmd_vel_recovery by default
// - Supports enable/disable and cancel
// - Uses "settle" logic (must remain within tolerance for N cycles)
// - Handles stale odometry safely (zero command)
// - Publishes simple state/status strings for debugging
//
// Notes
// -----
// - This node outputs wz only (linear x/y = 0) for true rotate-in-place behavior.
// - Keep cmd_vel_shaper + safety gate in the pipeline for real-robot tests.
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "savo_control/control_math.hpp"
#include "savo_control/heading_controller.hpp"
#include "savo_control/topic_names.hpp"

using namespace std::chrono_literals;

namespace savo_control
{

class RotateToHeadingNode : public rclcpp::Node
{
public:
  RotateToHeadingNode()
  : Node("rotate_to_heading_node")
  {
    // -------------------------------------------------------------------------
    // Topics
    // -------------------------------------------------------------------------
    odom_topic_ = this->declare_parameter<std::string>(
      "odom_topic", topic_names::kOdomFiltered);

    rotate_target_topic_ = this->declare_parameter<std::string>(
      "rotate_target_topic", topic_names::kRotateTarget);

    enable_topic_ = this->declare_parameter<std::string>(
      "enable_topic", topic_names::kHeadingHoldEnable);  // reusable Bool topic (can override)

    cancel_topic_ = this->declare_parameter<std::string>(
      "cancel_topic", topic_names::kRecoveryTrigger);    // reusable trigger topic (Bool true=cxl)

    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", topic_names::kCmdVelRecovery);

    state_topic_ = this->declare_parameter<std::string>(
      "state_topic", topic_names::kRotateState);

    status_topic_ = this->declare_parameter<std::string>(
      "status_topic", topic_names::kRecoveryStatus);

    // -------------------------------------------------------------------------
    // Loop / freshness
    // -------------------------------------------------------------------------
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 30.0);
    odom_timeout_sec_ = this->declare_parameter<double>("odom_timeout_sec", 0.30);
    zero_on_stale_odom_ = this->declare_parameter<bool>("zero_on_stale_odom", true);

    // -------------------------------------------------------------------------
    // Rotate behavior
    // -------------------------------------------------------------------------
    auto_enable_on_target_ = this->declare_parameter<bool>("auto_enable_on_target", true);
    stop_and_hold_zero_after_done_ = this->declare_parameter<bool>("stop_and_hold_zero_after_done", true);
    reset_pid_on_new_target_ = this->declare_parameter<bool>("reset_pid_on_new_target", true);

    // Completion / settle logic
    settle_cycles_required_ = this->declare_parameter<int>("settle_cycles_required", 5);
    max_rotate_time_sec_ = this->declare_parameter<double>("max_rotate_time_sec", 15.0);  // 0 = disabled
    min_error_to_command_rad_ = this->declare_parameter<double>("min_error_to_command_rad", 0.0);

    // Extra node-level clamps (final safety clamp after controller)
    out_wz_max_abs_ = this->declare_parameter<double>("output.wz_max_abs", 0.8);

    // -------------------------------------------------------------------------
    // Heading controller config (PID + shaping)
    // -------------------------------------------------------------------------
    HeadingControllerConfig cfg{};

    cfg.pid.kp = this->declare_parameter<double>("pid.kp", 2.2);
    cfg.pid.ki = this->declare_parameter<double>("pid.ki", 0.0);
    cfg.pid.kd = this->declare_parameter<double>("pid.kd", 0.05);

    cfg.pid.output_min = this->declare_parameter<double>("pid.output_min", -0.8);
    cfg.pid.output_max = this->declare_parameter<double>("pid.output_max",  0.8);
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
      this->declare_parameter<double>("max_wz_rad_s", 0.60);

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

    pub_status_ = this->create_publisher<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(10));

    // -------------------------------------------------------------------------
    // Subscribers
    // -------------------------------------------------------------------------
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&RotateToHeadingNode::on_odom_, this, std::placeholders::_1));

    sub_rotate_target_ = this->create_subscription<std_msgs::msg::Float64>(
      rotate_target_topic_, rclcpp::QoS(10),
      std::bind(&RotateToHeadingNode::on_rotate_target_, this, std::placeholders::_1));

    sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
      enable_topic_, rclcpp::QoS(10),
      std::bind(&RotateToHeadingNode::on_enable_, this, std::placeholders::_1));

    sub_cancel_ = this->create_subscription<std_msgs::msg::Bool>(
      cancel_topic_, rclcpp::QoS(10),
      std::bind(&RotateToHeadingNode::on_cancel_, this, std::placeholders::_1));

    // -------------------------------------------------------------------------
    // Timer loop
    // -------------------------------------------------------------------------
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&RotateToHeadingNode::on_timer_, this));

    publish_state_(true);
    publish_status_("idle");

    RCLCPP_INFO(
      this->get_logger(),
      "rotate_to_heading_node started | odom=%s | target=%s | out=%s | rate=%.1fHz",
      odom_topic_.c_str(), rotate_target_topic_.c_str(), output_topic_.c_str(), publish_rate_hz_);
  }

private:
  enum class RotateState
  {
    IDLE = 0,
    WAITING_FOR_ODOM,
    READY,
    ROTATING,
    DONE,
    CANCELED,
    TIMEOUT,
    ERROR
  };

  struct OdomState
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool has_msg{false};
    double yaw_rad{0.0};
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
        "rotate_to_heading_node: invalid odometry quaternion");
      return;
    }

    odom_.yaw_rad = yaw;
    odom_.stamp = this->now();
    odom_.has_msg = true;
  }

  void on_rotate_target_(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!msg || !std::isfinite(msg->data)) {
      RCLCPP_WARN(this->get_logger(), "rotate target ignored (invalid Float64)");
      return;
    }

    const double target_rad = msg->data;
    target_received_ = true;
    target_cmd_stamp_ = this->now();

    if (reset_pid_on_new_target_) {
      heading_ctrl_.reset();
    }
    heading_ctrl_.set_target_yaw(target_rad);

    settle_count_ = 0;
    rotate_start_stamp_ = this->now();

    if (auto_enable_on_target_) {
      enabled_ = true;
    }

    // Move to READY/ROTATING depending on odom freshness
    if (is_odom_fresh_(this->now())) {
      set_state_(RotateState::READY);
    } else {
      set_state_(RotateState::WAITING_FOR_ODOM);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Rotate target set: %.3f rad (wrapped=%.3f rad, %.1f deg), enabled=%s",
      target_rad,
      heading_ctrl_.target_yaw_rad(),
      ControlMath::rad_to_deg(heading_ctrl_.target_yaw_rad()),
      enabled_ ? "true" : "false");

    publish_state_(true);
  }

  void on_enable_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    const bool prev = enabled_;
    enabled_ = msg->data;

    if (enabled_ && !prev) {
      // Fresh enable
      settle_count_ = 0;
      rotate_start_stamp_ = this->now();

      if (!heading_ctrl_.has_target()) {
        set_state_(RotateState::IDLE);
        RCLCPP_INFO(this->get_logger(), "Rotate enable=true but no target yet");
      } else if (!is_odom_fresh_(this->now())) {
        set_state_(RotateState::WAITING_FOR_ODOM);
        RCLCPP_INFO(this->get_logger(), "Rotate enabled; waiting for fresh odom");
      } else {
        set_state_(RotateState::READY);
        RCLCPP_INFO(this->get_logger(), "Rotate enabled");
      }
    } else if (!enabled_ && prev) {
      heading_ctrl_.reset();
      settle_count_ = 0;
      set_state_(RotateState::IDLE);
      RCLCPP_INFO(this->get_logger(), "Rotate disabled");
      pub_cmd_out_->publish(zero_twist_());
    }

    publish_state_(true);
  }

  void on_cancel_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg || !msg->data) {
      return;
    }

    enabled_ = false;
    settle_count_ = 0;
    heading_ctrl_.reset();
    set_state_(RotateState::CANCELED);
    pub_cmd_out_->publish(zero_twist_());

    RCLCPP_WARN(this->get_logger(), "Rotate canceled");
    publish_state_(true);
    publish_status_("canceled");
  }

  // ---------------------------------------------------------------------------
  // Main loop
  // ---------------------------------------------------------------------------
  void on_timer_()
  {
    const rclcpp::Time now = this->now();

    // dt for PID/controller update
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

    // No target -> idle safe zero
    if (!heading_ctrl_.has_target()) {
      set_state_(RotateState::IDLE);
      pub_cmd_out_->publish(out);
      publish_state_(false);
      return;
    }

    // Disabled -> zero output (ready target can still remain stored)
    if (!enabled_) {
      if (state_ != RotateState::CANCELED && state_ != RotateState::DONE) {
        set_state_(RotateState::IDLE);
      }
      pub_cmd_out_->publish(out);
      publish_state_(false);
      return;
    }

    // Odom freshness
    if (!is_odom_fresh_(now)) {
      set_state_(RotateState::WAITING_FOR_ODOM);

      if (zero_on_stale_odom_) {
        pub_cmd_out_->publish(zero_twist_());
      } else {
        pub_cmd_out_->publish(out);
      }

      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1500,
        "Rotate waiting for fresh odom");
      publish_state_(false);
      return;
    }

    // Timeout guard
    if (max_rotate_time_sec_ > 0.0 && rotate_start_stamp_.nanoseconds() != 0) {
      const double elapsed = (now - rotate_start_stamp_).seconds();
      if (std::isfinite(elapsed) && elapsed > max_rotate_time_sec_) {
        enabled_ = false;
        heading_ctrl_.reset();
        settle_count_ = 0;
        set_state_(RotateState::TIMEOUT);
        pub_cmd_out_->publish(zero_twist_());

        RCLCPP_WARN(
          this->get_logger(),
          "Rotate timeout after %.2f s (limit %.2f s)",
          elapsed, max_rotate_time_sec_);
        publish_state_(true);
        publish_status_("timeout");
        return;
      }
    }

    // Ready -> rotating transition
    if (state_ == RotateState::READY || state_ == RotateState::WAITING_FOR_ODOM) {
      set_state_(RotateState::ROTATING);
    }

    // Run heading controller
    HeadingControllerResult res{};
    if (dt_valid) {
      res = heading_ctrl_.update(odom_.yaw_rad, dt_sec);
    } else {
      // First cycle / timing glitch: safe zero
      res.valid = false;
      res.wz_cmd_rad_s = 0.0;
      res.current_yaw_rad = odom_.yaw_rad;
      if (heading_ctrl_.has_target()) {
        res.target_yaw_rad = heading_ctrl_.target_yaw_rad();
        res.error_yaw_rad = ControlMath::shortest_angular_distance_rad(
          ControlMath::wrap_angle_rad(odom_.yaw_rad), res.target_yaw_rad);
        res.within_tolerance = std::abs(res.error_yaw_rad) <=
                               std::abs(heading_ctrl_.config().heading_tolerance_rad);
      }
    }

    // Optional no-command inner zone beyond tolerance logic
    if (std::isfinite(min_error_to_command_rad_) &&
        min_error_to_command_rad_ > 0.0 &&
        std::abs(res.error_yaw_rad) < min_error_to_command_rad_)
    {
      res.wz_cmd_rad_s = 0.0;
    }

    // Settle logic: require N consecutive cycles within tolerance
    if (res.within_tolerance) {
      settle_count_++;
    } else {
      settle_count_ = 0;
    }

    const bool settled = (settle_count_ >= settle_cycles_required_);

    if (settled) {
      out = zero_twist_();
      pub_cmd_out_->publish(out);

      set_state_(RotateState::DONE);
      publish_status_("done");

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Rotate complete: target=%.3f yaw=%.3f err=%.4f rad",
        res.target_yaw_rad, res.current_yaw_rad, res.error_yaw_rad);

      if (stop_and_hold_zero_after_done_) {
        enabled_ = false;  // remain done, stop commanding
      }
      publish_state_(false);
      return;
    }

    // Active rotating command (wz only)
    out.angular.z = clamp_abs_(res.valid ? res.wz_cmd_rad_s : 0.0, out_wz_max_abs_);
    out.linear.x = 0.0;
    out.linear.y = 0.0;
    out.linear.z = 0.0;
    out.angular.x = 0.0;
    out.angular.y = 0.0;

    pub_cmd_out_->publish(out);

    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "rotate: yaw=%.3f target=%.3f err=%.3f wz=%.3f tol=%s settle=%d/%d dt=%.4f",
      res.current_yaw_rad,
      res.target_yaw_rad,
      res.error_yaw_rad,
      out.angular.z,
      res.within_tolerance ? "true" : "false",
      settle_count_, settle_cycles_required_,
      dt_sec);

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
    if (!std::isfinite(max_rotate_time_sec_) || max_rotate_time_sec_ < 0.0) {
      max_rotate_time_sec_ = 15.0;
    }
    if (!std::isfinite(min_error_to_command_rad_) || min_error_to_command_rad_ < 0.0) {
      min_error_to_command_rad_ = 0.0;
    }

    out_wz_max_abs_ = (std::isfinite(out_wz_max_abs_) && out_wz_max_abs_ > 0.0)
      ? std::abs(out_wz_max_abs_) : 0.8;

    if (settle_cycles_required_ < 1) {
      settle_cycles_required_ = 1;
    }

    if (odom_topic_.empty()) odom_topic_ = topic_names::kOdomFiltered;
    if (rotate_target_topic_.empty()) rotate_target_topic_ = topic_names::kRotateTarget;
    if (enable_topic_.empty()) enable_topic_ = topic_names::kHeadingHoldEnable;
    if (cancel_topic_.empty()) cancel_topic_ = topic_names::kRecoveryTrigger;
    if (output_topic_.empty()) output_topic_ = topic_names::kCmdVelRecovery;
    if (state_topic_.empty()) state_topic_ = topic_names::kRotateState;
    if (status_topic_.empty()) status_topic_ = topic_names::kRecoveryStatus;
  }

  bool is_odom_fresh_(const rclcpp::Time & now) const
  {
    if (!odom_.has_msg) {
      return false;
    }
    const double age = (now - odom_.stamp).seconds();
    return std::isfinite(age) && age >= 0.0 && age <= odom_timeout_sec_;
  }

  void set_state_(RotateState s)
  {
    if (state_ == s) {
      return;
    }
    state_ = s;
    publish_status_(state_to_string_(state_));
  }

  static std::string state_to_string_(RotateState s)
  {
    switch (s) {
      case RotateState::IDLE: return "idle";
      case RotateState::WAITING_FOR_ODOM: return "waiting_for_odom";
      case RotateState::READY: return "ready";
      case RotateState::ROTATING: return "rotating";
      case RotateState::DONE: return "done";
      case RotateState::CANCELED: return "canceled";
      case RotateState::TIMEOUT: return "timeout";
      case RotateState::ERROR: return "error";
      default: return "unknown";
    }
  }

  void publish_state_(bool force)
  {
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(3);

    ss << "state=" << state_to_string_(state_)
       << " enabled=" << (enabled_ ? "true" : "false")
       << " odom=" << (odom_.has_msg ? "yes" : "no")
       << " target=" << (heading_ctrl_.has_target() ? "yes" : "no");

    if (heading_ctrl_.has_target()) {
      ss << " target_rad=" << heading_ctrl_.target_yaw_rad();
    }
    if (odom_.has_msg) {
      ss << " yaw_rad=" << odom_.yaw_rad;
      if (heading_ctrl_.has_target()) {
        const double err = ControlMath::shortest_angular_distance_rad(
          ControlMath::wrap_angle_rad(odom_.yaw_rad),
          heading_ctrl_.target_yaw_rad());
        ss << " err_rad=" << err;
      }
    }
    ss << " settle=" << settle_count_ << "/" << settle_cycles_required_;

    const std::string state_str = ss.str();
    if (!force && state_str == last_state_msg_) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = state_str;
    pub_state_->publish(msg);
    last_state_msg_ = state_str;
  }

  void publish_status_(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    pub_status_->publish(msg);
  }

  static geometry_msgs::msg::Twist zero_twist_()
  {
    return geometry_msgs::msg::Twist{};
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
    const double len2 = q.length2();
    if (!(len2 > 0.0) || !std::isfinite(len2)) {
      return false;
    }

    q.normalize();

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_rad = yaw;
    return std::isfinite(yaw_rad);
  }

private:
  // Publishers/subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_out_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_rotate_target_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cancel_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Topics
  std::string odom_topic_;
  std::string rotate_target_topic_;
  std::string enable_topic_;
  std::string cancel_topic_;
  std::string output_topic_;
  std::string state_topic_;
  std::string status_topic_;

  // Params
  double publish_rate_hz_{30.0};
  double odom_timeout_sec_{0.30};
  bool zero_on_stale_odom_{true};

  bool auto_enable_on_target_{true};
  bool stop_and_hold_zero_after_done_{true};
  bool reset_pid_on_new_target_{true};

  int settle_cycles_required_{5};
  double max_rotate_time_sec_{15.0};
  double min_error_to_command_rad_{0.0};

  double out_wz_max_abs_{0.8};

  // Runtime state
  OdomState odom_{};
  HeadingController heading_ctrl_{};

  bool enabled_{false};
  bool target_received_{false};

  RotateState state_{RotateState::IDLE};
  int settle_count_{0};

  rclcpp::Time target_cmd_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time rotate_start_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Time prev_loop_time_{0, 0, RCL_ROS_TIME};
  bool has_prev_loop_time_{false};

  std::string last_state_msg_;
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<savo_control::RotateToHeadingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}