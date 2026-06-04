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
#include "std_msgs/msg/string.hpp"

#include "savo_control/stuck_detector.hpp"
#include "savo_control/topic_names.hpp"

namespace savo_control
{
namespace
{

// Small safety helper.
// We never want NaN or Inf values to enter the stuck detector logic.
bool finite_twist(const Twist2D & t)
{
  return std::isfinite(t.vx) && std::isfinite(t.vy) && std::isfinite(t.wz);
}

std::string bool_text(bool v)
{
  return v ? "true" : "false";
}

}  // namespace

class StuckDetectorNode : public rclcpp::Node
{
public:
  StuckDetectorNode()
  : Node("stuck_detector_node")
  {
    load_parameters_();

    // The actual stuck detection math lives in the reusable core class.
    // The ROS node only handles parameters, topics, freshness checks, and publishing.
    detector_.set_config(detector_cfg_);

    if (!enabled_) {
      RCLCPP_WARN(get_logger(), "stuck_detector_node is disabled by parameter");
    }

    // Subscribe to the command after the safety gate.
    // This is important: we detect stuck only from motion that the robot was actually allowed to do.
    sub_cmd_safe_ = create_subscription<geometry_msgs::msg::Twist>(
      topic_cmd_vel_safe_, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_safe_.vx = msg->linear.x;
        last_cmd_safe_.vy = msg->linear.y;
        last_cmd_safe_.wz = msg->angular.z;

        last_cmd_safe_stamp_ = now();
        have_cmd_safe_ = true;
      });

    // Subscribe to fused odometry.
    // For Robot Savo this should normally come from robot_localization:
    // /odometry/filtered = wheel odom + IMU + later VO.
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      topic_odom_, rclcpp::QoS(20),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_meas_.vx = msg->twist.twist.linear.x;
        last_meas_.vy = msg->twist.twist.linear.y;
        last_meas_.wz = msg->twist.twist.angular.z;

        last_pose_.x = msg->pose.pose.position.x;
        last_pose_.y = msg->pose.pose.position.y;
        last_pose_.yaw = yaw_from_quat_(
          msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z,
          msg->pose.pose.orientation.w);

        last_odom_stamp_ = now();
        have_odom_ = true;
      });

    // Safety stop is used as a suppression condition.
    // If the safety system intentionally stopped the robot, that should not be treated as "stuck".
    sub_safety_stop_ = create_subscription<std_msgs::msg::Bool>(
      topic_safety_stop_, rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        safety_stop_active_ = msg->data;
        last_safety_stop_stamp_ = now();
        have_safety_stop_ = true;
      });

    pub_stuck_state_ = create_publisher<std_msgs::msg::Bool>(
      topic_stuck_state_, rclcpp::QoS(10));

    pub_status_ = create_publisher<std_msgs::msg::String>(
      topic_status_, rclcpp::QoS(10));

    pub_recovery_request_ = create_publisher<std_msgs::msg::Bool>(
      topic_recovery_request_, rclcpp::QoS(10));

    // Start in a safe known state.
    // This prevents old latched-style assumptions from previous runs.
    if (publish_clear_on_startup_) {
      publish_stuck_state_(false);
      publish_recovery_request_(false);
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, loop_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&StuckDetectorNode::on_timer_, this));

    RCLCPP_INFO(
      get_logger(),
      "stuck_detector_node ready | cmd_safe=%s odom=%s safety_stop=%s stuck_state=%s",
      topic_cmd_vel_safe_.c_str(),
      topic_odom_.c_str(),
      topic_safety_stop_.c_str(),
      topic_stuck_state_.c_str());
  }

private:
  void load_parameters_()
  {
    // ------------------------------------------------------------
    // General enable flags
    // ------------------------------------------------------------
    enabled_ = declare_parameter<bool>("enabled", true);
    active_detection_ = declare_parameter<bool>("active_detection", true);

    // ------------------------------------------------------------
    // Topic parameters
    // ------------------------------------------------------------
    topic_cmd_vel_safe_ = declare_parameter<std::string>(
      "topics.cmd_vel_safe", "/cmd_vel_safe");

    topic_odom_ = declare_parameter<std::string>(
      "topics.odom", "/odometry/filtered");

    topic_safety_stop_ = declare_parameter<std::string>(
      "topics.safety_stop", "/safety/stop");

    topic_stuck_state_ = declare_parameter<std::string>(
      "topics.stuck_state", "/savo_control/stuck_state");

    topic_status_ = declare_parameter<std::string>(
      "topics.status", "/savo_control/stuck_detector_status");

    topic_recovery_request_ = declare_parameter<std::string>(
      "topics.recovery_request", topic_names::kRecoveryRequest);

    // ------------------------------------------------------------
    // Timing parameters
    // ------------------------------------------------------------
    loop_hz_ = declare_parameter<double>("timing.loop_hz", 20.0);
    cmd_safe_timeout_s_ = declare_parameter<double>("timing.cmd_safe_timeout_s", 0.40);
    odom_timeout_s_ = declare_parameter<double>("timing.odom_timeout_s", 0.50);
    safety_timeout_s_ = declare_parameter<double>("timing.safety_timeout_s", 0.50);

    const double stuck_time_s = declare_parameter<double>("timing.stuck_time_s", 2.0);
    const double clear_time_s = declare_parameter<double>("timing.clear_time_s", 1.0);
    retrigger_cooldown_s_ = declare_parameter<double>("timing.retrigger_cooldown_s", 3.0);

    // ------------------------------------------------------------
    // Command thresholds
    // ------------------------------------------------------------
    // If command magnitude is smaller than these values, we do not consider the robot
    // to be actively trying to move.
    const double min_cmd_safe_vx = declare_parameter<double>(
      "command_thresholds.min_cmd_safe_vx", 0.05);

    const double min_cmd_safe_vy = declare_parameter<double>(
      "command_thresholds.min_cmd_safe_vy", 0.05);

    const double min_cmd_safe_wz = declare_parameter<double>(
      "command_thresholds.min_cmd_safe_wz", 0.12);

    // ------------------------------------------------------------
    // Observed movement thresholds
    // ------------------------------------------------------------
    // If odometry is below these values while command is active, the robot may be stuck.
    const double min_linear = declare_parameter<double>(
      "observed_motion_thresholds.min_linear_speed_m_s", 0.025);

    const double min_strafe = declare_parameter<double>(
      "observed_motion_thresholds.min_strafe_speed_m_s", 0.020);

    const double min_angular = declare_parameter<double>(
      "observed_motion_thresholds.min_angular_speed_rad_s", 0.06);

    // ------------------------------------------------------------
    // Detection safety rules
    // ------------------------------------------------------------
    ignore_when_safety_stop_true_ = declare_parameter<bool>(
      "detection.ignore_when_safety_stop_true", true);

    do_not_declare_stuck_on_odom_stale_ = declare_parameter<bool>(
      "detection.do_not_declare_stuck_on_odom_stale", true);

    do_not_declare_stuck_on_cmd_safe_stale_ = declare_parameter<bool>(
      "detection.do_not_declare_stuck_on_cmd_safe_stale", true);

    const double startup_grace_s = declare_parameter<double>(
      "detection.startup_grace_s", 2.0);

    // ------------------------------------------------------------
    // Recovery output
    // ------------------------------------------------------------
    // Keep publish_recovery_request false during first real robot tests.
    // We want to observe detection first, then enable automatic recovery later.
    publish_recovery_request_enabled_ = declare_parameter<bool>(
      "recovery.publish_recovery_request", false);

    min_stuck_time_before_recovery_s_ = declare_parameter<double>(
      "recovery.min_stuck_time_before_recovery_s", 2.5);

    max_recovery_requests_per_event_ = declare_parameter<int>(
      "recovery.max_recovery_requests_per_event", 1);

    // ------------------------------------------------------------
    // Output / diagnostics
    // ------------------------------------------------------------
    publish_status_ = declare_parameter<bool>("output.publish_status", true);
    status_hz_ = declare_parameter<double>("output.status_hz", 5.0);
    publish_clear_on_startup_ = declare_parameter<bool>("output.publish_clear_on_startup", true);
    publish_clear_when_unstuck_ = declare_parameter<bool>("output.publish_clear_when_unstuck", true);

    block_recovery_when_safety_stop_true_ = declare_parameter<bool>(
      "safety.block_recovery_when_safety_stop_true", true);

    block_recovery_when_odom_stale_ = declare_parameter<bool>(
      "safety.block_recovery_when_odom_stale", true);

    block_recovery_when_cmd_safe_stale_ = declare_parameter<bool>(
      "safety.block_recovery_when_cmd_safe_stale", true);

    log_stuck_detected_ = declare_parameter<bool>("diagnostics.log_stuck_detected", true);
    log_stuck_cleared_ = declare_parameter<bool>("diagnostics.log_stuck_cleared", true);
    log_throttle_s_ = declare_parameter<double>("diagnostics.log_throttle_s", 2.0);

    // ------------------------------------------------------------
    // Clamp values into safe ranges
    // ------------------------------------------------------------
    loop_hz_ = std::clamp(loop_hz_, 1.0, 100.0);
    status_hz_ = std::clamp(status_hz_, 0.2, loop_hz_);

    cmd_safe_timeout_s_ = std::max(0.05, cmd_safe_timeout_s_);
    odom_timeout_s_ = std::max(0.05, odom_timeout_s_);
    safety_timeout_s_ = std::max(0.05, safety_timeout_s_);

    // ------------------------------------------------------------
    // Convert ROS parameters into the core stuck detector config
    // ------------------------------------------------------------
    detector_cfg_.cmd_lin_active_mps = std::max(0.0, std::min(min_cmd_safe_vx, min_cmd_safe_vy));
    detector_cfg_.cmd_ang_active_radps = std::max(0.0, min_cmd_safe_wz);
    detector_cfg_.cmd_mag_active_threshold = detector_cfg_.cmd_lin_active_mps;

    detector_cfg_.meas_lin_moving_mps = std::max(0.0, std::min(min_linear, min_strafe));
    detector_cfg_.meas_ang_moving_radps = std::max(0.0, min_angular);
    detector_cfg_.meas_mag_moving_threshold = detector_cfg_.meas_lin_moving_mps;

    detector_cfg_.no_motion_confirm_sec = std::max(0.1, stuck_time_s);
    detector_cfg_.clear_confirm_sec = std::max(0.1, clear_time_s);
    detector_cfg_.startup_grace_sec = std::max(0.0, startup_grace_s);
    detector_cfg_.suppress_stuck_while_safety_stop = ignore_when_safety_stop_true_;
  }

  void on_timer_()
  {
    if (!enabled_) {
      detector_.reset();
      publish_stuck_state_(false);
      publish_recovery_request_(false);
      maybe_publish_status_("DISABLED", false, false, false, StuckDetectorStatus{});
      return;
    }

    const rclcpp::Time t = now();
    const double now_sec = t.seconds();

    const bool cmd_fresh =
      have_cmd_safe_ && age_sec_(last_cmd_safe_stamp_, t) <= cmd_safe_timeout_s_;

    const bool odom_fresh =
      have_odom_ && age_sec_(last_odom_stamp_, t) <= odom_timeout_s_;

    const bool safety_fresh =
      have_safety_stop_ && age_sec_(last_safety_stop_stamp_, t) <= safety_timeout_s_;

    // Detection can be intentionally disabled while keeping the node alive.
    if (!active_detection_) {
      detector_.reset();
      publish_stuck_state_(false);
      publish_recovery_request_(false);
      maybe_publish_status_("INACTIVE", cmd_fresh, odom_fresh, safety_fresh, StuckDetectorStatus{});
      return;
    }

    // Production safety rule:
    // Never declare stuck from stale data. Bad odom or stale command can cause false positives.
    const bool block_for_stale_cmd = do_not_declare_stuck_on_cmd_safe_stale_ && !cmd_fresh;
    const bool block_for_stale_odom = do_not_declare_stuck_on_odom_stale_ && !odom_fresh;

    if (block_for_stale_cmd || block_for_stale_odom) {
      detector_.reset();
      publish_stuck_state_(false);
      publish_recovery_request_(false);

      maybe_publish_status_(
        block_for_stale_cmd ? "CMD_SAFE_STALE" : "ODOM_STALE",
        cmd_fresh,
        odom_fresh,
        safety_fresh,
        StuckDetectorStatus{});

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        static_cast<int>(log_throttle_s_ * 1000.0),
        "Stuck detection held clear because %s",
        block_for_stale_cmd ? "cmd_vel_safe is stale" : "odometry is stale");

      return;
    }

    // Prepare detector input.
    StuckDetectorInputs input;
    input.now_sec = now_sec;
    input.cmd = finite_twist(last_cmd_safe_) ? last_cmd_safe_ : Twist2D{};
    input.meas = finite_twist(last_meas_) ? last_meas_ : Twist2D{};
    input.has_pose = odom_fresh;
    input.pose = last_pose_;

    // If safety stop data is stale, assume not stopped for detection.
    // The separate freshness flags are still reported in status.
    input.safety_stop_active = safety_fresh ? safety_stop_active_ : false;

    const StuckDetectorStatus status = detector_.update(input);

    publish_stuck_state_(status.stuck);
    handle_recovery_request_(status, cmd_fresh, odom_fresh, safety_fresh);
    maybe_publish_status_("OK", cmd_fresh, odom_fresh, safety_fresh, status);

    if (status.became_stuck && log_stuck_detected_) {
      RCLCPP_ERROR(
        get_logger(),
        "STUCK detected | cmd=(%.3f, %.3f, %.3f), odom=(%.3f, %.3f, %.3f), no_motion=%.2fs",
        input.cmd.vx,
        input.cmd.vy,
        input.cmd.wz,
        input.meas.vx,
        input.meas.vy,
        input.meas.wz,
        status.no_motion_elapsed_sec);
    }

    if (status.cleared_stuck && log_stuck_cleared_) {
      RCLCPP_INFO(get_logger(), "Stuck condition cleared");

      if (publish_clear_when_unstuck_) {
        publish_stuck_state_(false);
        publish_recovery_request_(false);
      }
    }
  }

  void handle_recovery_request_(
    const StuckDetectorStatus & status,
    bool cmd_fresh,
    bool odom_fresh,
    bool safety_fresh)
  {
    if (!status.stuck) {
      recovery_event_active_ = false;
      recovery_requests_this_event_ = 0;
      return;
    }

    if (!publish_recovery_request_enabled_) {
      return;
    }

    const rclcpp::Time t = now();

    const bool cooldown_ok =
      !have_last_recovery_request_ ||
      age_sec_(last_recovery_request_stamp_, t) >= retrigger_cooldown_s_;

    const bool stuck_long_enough =
      status.no_motion_elapsed_sec >= min_stuck_time_before_recovery_s_;

    // Recovery should never be requested when the safety layer is actively stopping
    // or when the data needed to make the decision is stale.
    const bool recovery_allowed =
      (!block_recovery_when_safety_stop_true_ || !status.safety_stop_active) &&
      (!block_recovery_when_odom_stale_ || odom_fresh) &&
      (!block_recovery_when_cmd_safe_stale_ || cmd_fresh) &&
      safety_fresh;

    if (!recovery_event_active_) {
      recovery_event_active_ = true;
      recovery_requests_this_event_ = 0;
    }

    if (
      stuck_long_enough &&
      cooldown_ok &&
      recovery_allowed &&
      recovery_requests_this_event_ < max_recovery_requests_per_event_)
    {
      publish_recovery_request_(true);

      last_recovery_request_stamp_ = t;
      have_last_recovery_request_ = true;
      ++recovery_requests_this_event_;

      RCLCPP_WARN(get_logger(), "Published recovery request from stuck detector");
    }
  }

  void publish_stuck_state_(bool stuck)
  {
    std_msgs::msg::Bool msg;
    msg.data = stuck;
    pub_stuck_state_->publish(msg);
  }

  void publish_recovery_request_(bool request)
  {
    std_msgs::msg::Bool msg;
    msg.data = request;
    pub_recovery_request_->publish(msg);
  }

  void maybe_publish_status_(
    const std::string & state,
    bool cmd_fresh,
    bool odom_fresh,
    bool safety_fresh,
    const StuckDetectorStatus & status)
  {
    if (!publish_status_) {
      return;
    }

    const rclcpp::Time t = now();

    if (
      have_last_status_publish_ &&
      age_sec_(last_status_publish_stamp_, t) < (1.0 / status_hz_))
    {
      return;
    }

    std_msgs::msg::String msg;
    std::ostringstream oss;

    // Simple key=value text is easy to read with ros2 topic echo,
    // easy to log, and easy to parse later if needed.
    oss << "state=" << state
        << "; stuck=" << bool_text(status.stuck)
        << "; command_active=" << bool_text(status.command_active)
        << "; measured_moving=" << bool_text(status.measured_moving)
        << "; safety_stop=" << bool_text(status.safety_stop_active)
        << "; cmd_fresh=" << bool_text(cmd_fresh)
        << "; odom_fresh=" << bool_text(odom_fresh)
        << "; safety_fresh=" << bool_text(safety_fresh)
        << "; cmd_mag=" << status.cmd_weighted_mag
        << "; meas_mag=" << status.meas_weighted_mag
        << "; no_motion_s=" << status.no_motion_elapsed_sec;

    msg.data = oss.str();
    pub_status_->publish(msg);

    last_status_publish_stamp_ = t;
    have_last_status_publish_ = true;
  }

  static double yaw_from_quat_(double x, double y, double z, double w)
  {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double age_sec_(const rclcpp::Time & stamp, const rclcpp::Time & t)
  {
    return std::max(0.0, (t - stamp).seconds());
  }

private:
  // General state
  bool enabled_{true};
  bool active_detection_{true};

  // Detection behavior
  bool ignore_when_safety_stop_true_{true};
  bool do_not_declare_stuck_on_odom_stale_{true};
  bool do_not_declare_stuck_on_cmd_safe_stale_{true};

  // Recovery behavior
  bool publish_recovery_request_enabled_{false};
  bool block_recovery_when_safety_stop_true_{true};
  bool block_recovery_when_odom_stale_{true};
  bool block_recovery_when_cmd_safe_stale_{true};

  // Output and diagnostics
  bool publish_status_{true};
  bool publish_clear_on_startup_{true};
  bool publish_clear_when_unstuck_{true};
  bool log_stuck_detected_{true};
  bool log_stuck_cleared_{true};

  // Timing
  double loop_hz_{20.0};
  double status_hz_{5.0};
  double cmd_safe_timeout_s_{0.40};
  double odom_timeout_s_{0.50};
  double safety_timeout_s_{0.50};
  double retrigger_cooldown_s_{3.0};
  double min_stuck_time_before_recovery_s_{2.5};
  double log_throttle_s_{2.0};

  int max_recovery_requests_per_event_{1};
  int recovery_requests_this_event_{0};

  // Topics
  std::string topic_cmd_vel_safe_{"/cmd_vel_safe"};
  std::string topic_odom_{"/odometry/filtered"};
  std::string topic_safety_stop_{"/safety/stop"};
  std::string topic_stuck_state_{"/savo_control/stuck_state"};
  std::string topic_status_{"/savo_control/stuck_detector_status"};
  std::string topic_recovery_request_{topic_names::kRecoveryRequest};

  // Core detector
  StuckDetectorConfig detector_cfg_{};
  StuckDetector detector_{};

  // Latest received data
  Twist2D last_cmd_safe_{};
  Twist2D last_meas_{};
  Pose2D last_pose_{};
  bool safety_stop_active_{false};

  bool have_cmd_safe_{false};
  bool have_odom_{false};
  bool have_safety_stop_{false};

  rclcpp::Time last_cmd_safe_stamp_{};
  rclcpp::Time last_odom_stamp_{};
  rclcpp::Time last_safety_stop_stamp_{};

  // Recovery event bookkeeping
  bool recovery_event_active_{false};
  bool have_last_recovery_request_{false};
  rclcpp::Time last_recovery_request_stamp_{};

  // Status publishing throttle
  bool have_last_status_publish_{false};
  rclcpp::Time last_status_publish_stamp_{};

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_safe_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_safety_stop_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stuck_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_recovery_request_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<savo_control::StuckDetectorNode>());
  rclcpp::shutdown();
  return 0;
}