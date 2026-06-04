// ROS2 node wrapper for StuckDetector. Subscribes to cmd_vel and odom, publishes stuck_detected.

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

class StuckDetectorNode : public rclcpp::Node
{
public:
  StuckDetectorNode()
  : Node("stuck_detector_node")
  {
    // -------------------------------------------------------------------------
    // Parameters: topics
    // -------------------------------------------------------------------------
    topic_cmd_vel_ = declare_parameter<std::string>("topics.cmd_vel", topic_names::kCmdVelSafe);
    topic_odom_ = declare_parameter<std::string>("topics.odom", topic_names::kOdomFiltered);
    topic_safety_stop_ = declare_parameter<std::string>(
      "topics.safety_stop", topic_names::kSafetyStop);
    topic_stuck_detected_ = declare_parameter<std::string>(
      "topics.stuck_detected", topic_names::kStuckDetected);
    // Optional: wire no_progress separately to recovery_manager's topics.no_progress
    topic_no_progress_ = declare_parameter<std::string>("topics.no_progress", "");
    topic_status_ = declare_parameter<std::string>(
      "topics.status", topic_names::kControlDebug);

    // -------------------------------------------------------------------------
    // Parameters: loop / staleness
    // -------------------------------------------------------------------------
    loop_rate_hz_ = declare_parameter<double>("loop_rate_hz", 20.0);
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 0.5);
    stale_cmd_as_zero_ = declare_parameter<bool>("stale.cmd_as_zero", true);
    stale_safety_stop_as_true_ = declare_parameter<bool>("stale.safety_stop_as_true", false);

    // -------------------------------------------------------------------------
    // Parameters: StuckDetectorConfig
    // -------------------------------------------------------------------------
    StuckDetectorConfig cfg{};
    cfg.max_time_jump_sec =
      declare_parameter<double>("detector.max_time_jump_sec", cfg.max_time_jump_sec);
    cfg.cmd_lin_active_mps =
      declare_parameter<double>("detector.cmd_lin_active_mps", cfg.cmd_lin_active_mps);
    cfg.cmd_ang_active_radps =
      declare_parameter<double>("detector.cmd_ang_active_radps", cfg.cmd_ang_active_radps);
    cfg.cmd_mag_active_threshold =
      declare_parameter<double>("detector.cmd_mag_active_threshold", cfg.cmd_mag_active_threshold);
    cfg.wz_weight_for_mag =
      declare_parameter<double>("detector.wz_weight_for_mag", cfg.wz_weight_for_mag);
    cfg.meas_lin_moving_mps =
      declare_parameter<double>("detector.meas_lin_moving_mps", cfg.meas_lin_moving_mps);
    cfg.meas_ang_moving_radps =
      declare_parameter<double>("detector.meas_ang_moving_radps", cfg.meas_ang_moving_radps);
    cfg.meas_mag_moving_threshold =
      declare_parameter<double>("detector.meas_mag_moving_threshold", cfg.meas_mag_moving_threshold);
    cfg.command_grace_sec =
      declare_parameter<double>("detector.command_grace_sec", cfg.command_grace_sec);
    cfg.no_motion_confirm_sec =
      declare_parameter<double>("detector.no_motion_confirm_sec", cfg.no_motion_confirm_sec);
    cfg.clear_confirm_sec =
      declare_parameter<double>("detector.clear_confirm_sec", cfg.clear_confirm_sec);
    cfg.enable_pose_progress_check =
      declare_parameter<bool>("detector.enable_pose_progress_check", cfg.enable_pose_progress_check);
    cfg.pose_window_sec =
      declare_parameter<double>("detector.pose_window_sec", cfg.pose_window_sec);
    cfg.min_progress_dist_m =
      declare_parameter<double>("detector.min_progress_dist_m", cfg.min_progress_dist_m);
    cfg.min_progress_yaw_rad =
      declare_parameter<double>("detector.min_progress_yaw_rad", cfg.min_progress_yaw_rad);
    cfg.rotate_only_lin_max_mps =
      declare_parameter<double>("detector.rotate_only_lin_max_mps", cfg.rotate_only_lin_max_mps);
    cfg.rotate_only_ang_min_radps =
      declare_parameter<double>("detector.rotate_only_ang_min_radps", cfg.rotate_only_ang_min_radps);
    cfg.suppress_stuck_while_safety_stop =
      declare_parameter<bool>(
        "detector.suppress_stuck_while_safety_stop", cfg.suppress_stuck_while_safety_stop);
    cfg.startup_grace_sec =
      declare_parameter<double>("detector.startup_grace_sec", cfg.startup_grace_sec);

    sanitize_params_();
    detector_.set_config(cfg);

    // -------------------------------------------------------------------------
    // Publishers
    // -------------------------------------------------------------------------
    pub_stuck_ = create_publisher<std_msgs::msg::Bool>(
      topic_stuck_detected_, rclcpp::QoS(10));

    if (!topic_no_progress_.empty()) {
      pub_no_progress_ = create_publisher<std_msgs::msg::Bool>(
        topic_no_progress_, rclcpp::QoS(10));
    }

    pub_status_ = create_publisher<std_msgs::msg::String>(
      topic_status_, rclcpp::QoS(10));

    // -------------------------------------------------------------------------
    // Subscribers
    // -------------------------------------------------------------------------
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      topic_cmd_vel_, rclcpp::QoS(10),
      std::bind(&StuckDetectorNode::on_cmd_vel_, this, std::placeholders::_1));

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      topic_odom_, rclcpp::QoS(10),
      std::bind(&StuckDetectorNode::on_odom_, this, std::placeholders::_1));

    sub_safety_stop_ = create_subscription<std_msgs::msg::Bool>(
      topic_safety_stop_, rclcpp::QoS(10),
      std::bind(&StuckDetectorNode::on_safety_stop_, this, std::placeholders::_1));

    // -------------------------------------------------------------------------
    // Timer
    // -------------------------------------------------------------------------
    const auto period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&StuckDetectorNode::on_timer_, this));

    RCLCPP_INFO(
      get_logger(),
      "stuck_detector_node started | cmd=%s | odom=%s | out=%s | rate=%.1f Hz",
      topic_cmd_vel_.c_str(), topic_odom_.c_str(),
      topic_stuck_detected_.c_str(), loop_rate_hz_);
  }

private:
  // ---------------------------------------------------------------------------
  // Input caches
  // ---------------------------------------------------------------------------
  struct TimedTwist
  {
    Twist2D twist{};
    bool has_msg{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  struct TimedOdom
  {
    Twist2D twist{};
    Pose2D pose{};
    bool has_msg{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  struct TimedBool
  {
    bool value{false};
    bool has_msg{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  // ---------------------------------------------------------------------------
  // Subscriber callbacks
  // ---------------------------------------------------------------------------
  void on_cmd_vel_(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!msg) {return;}
    cmd_.twist.vx = msg->linear.x;
    cmd_.twist.vy = msg->linear.y;
    cmd_.twist.wz = msg->angular.z;
    cmd_.has_msg = true;
    cmd_.stamp = now();
  }

  void on_odom_(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg) {return;}
    odom_.twist.vx = msg->twist.twist.linear.x;
    odom_.twist.vy = msg->twist.twist.linear.y;
    odom_.twist.wz = msg->twist.twist.angular.z;
    odom_.pose.x = msg->pose.pose.position.x;
    odom_.pose.y = msg->pose.pose.position.y;
    odom_.pose.yaw = yaw_from_quaternion_(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
    odom_.has_msg = true;
    odom_.stamp = now();
  }

  void on_safety_stop_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) {return;}
    safety_stop_.value = msg->data;
    safety_stop_.has_msg = true;
    safety_stop_.stamp = now();
  }

  // ---------------------------------------------------------------------------
  // Timer loop
  // ---------------------------------------------------------------------------
  void on_timer_()
  {
    const rclcpp::Time now_ros = now();
    const double now_sec = now_ros.seconds();

    StuckDetectorInputs in{};
    in.now_sec = now_sec;

    // Commanded velocity: treat stale as zero so detector sees no command
    if (cmd_.has_msg) {
      const double age = (now_ros - cmd_.stamp).seconds();
      if (std::isfinite(age) && age >= 0.0 && age <= input_timeout_sec_) {
        in.cmd = cmd_.twist;
      } else if (!stale_cmd_as_zero_) {
        in.cmd = cmd_.twist;
      }
    }

    // Odometry: measured velocity + pose
    if (odom_.has_msg) {
      const double age = (now_ros - odom_.stamp).seconds();
      if (std::isfinite(age) && age >= 0.0 && age <= input_timeout_sec_) {
        in.meas = odom_.twist;
        in.has_pose = true;
        in.pose = odom_.pose;
      }
    }

    // Safety stop
    if (safety_stop_.has_msg) {
      const double age = (now_ros - safety_stop_.stamp).seconds();
      const bool fresh = std::isfinite(age) && age >= 0.0 && age <= input_timeout_sec_;
      in.safety_stop_active = fresh ? safety_stop_.value : stale_safety_stop_as_true_;
    }

    const StuckDetectorStatus s = detector_.update(in);

    // Publish stuck_detected
    std_msgs::msg::Bool stuck_msg;
    stuck_msg.data = s.stuck;
    pub_stuck_->publish(stuck_msg);

    // Publish no_progress if topic is wired
    if (pub_no_progress_) {
      std_msgs::msg::Bool np_msg;
      np_msg.data = s.no_progress;
      pub_no_progress_->publish(np_msg);
    }

    // Edge logging
    if (s.became_stuck) {
      RCLCPP_WARN(
        get_logger(),
        "STUCK | cmd_lin=%.3f cmd_ang=%.3f meas_lin=%.3f meas_ang=%.3f no_motion_t=%.2fs",
        s.cmd_lin_mag_mps, s.cmd_ang_abs_radps,
        s.meas_lin_mag_mps, s.meas_ang_abs_radps,
        s.no_motion_elapsed_sec);
    }
    if (s.cleared_stuck) {
      RCLCPP_INFO(
        get_logger(),
        "Stuck cleared | meas_lin=%.3f meas_ang=%.3f moving_t=%.2fs",
        s.meas_lin_mag_mps, s.meas_ang_abs_radps, s.moving_elapsed_sec);
    }

    publish_status_(s);
  }

  // ---------------------------------------------------------------------------
  // Status string (published on change only)
  // ---------------------------------------------------------------------------
  void publish_status_(const StuckDetectorStatus & s)
  {
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(3);
    ss << "stuck=" << bool_str_(s.stuck)
       << " no_progress=" << bool_str_(s.no_progress)
       << " cmd_active=" << bool_str_(s.command_active)
       << " meas_moving=" << bool_str_(s.measured_moving)
       << " suppressed=" << bool_str_(s.suppressed_by_safety)
       << " startup_grace=" << bool_str_(s.in_startup_grace)
       << " cmd_lin=" << s.cmd_lin_mag_mps
       << " cmd_ang=" << s.cmd_ang_abs_radps
       << " meas_lin=" << s.meas_lin_mag_mps
       << " meas_ang=" << s.meas_ang_abs_radps
       << " no_motion_t=" << s.no_motion_elapsed_sec
       << " pose_dist=" << s.pose_progress_dist_m;

    const std::string text = ss.str();
    if (text == last_status_) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = text;
    pub_status_->publish(msg);
    last_status_ = text;
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------
  void sanitize_params_()
  {
    if (!std::isfinite(loop_rate_hz_) || loop_rate_hz_ <= 0.0) {
      loop_rate_hz_ = 20.0;
    }
    if (!std::isfinite(input_timeout_sec_) || input_timeout_sec_ <= 0.0) {
      input_timeout_sec_ = 0.5;
    }
    if (topic_cmd_vel_.empty()) {topic_cmd_vel_ = topic_names::kCmdVelSafe;}
    if (topic_odom_.empty()) {topic_odom_ = topic_names::kOdomFiltered;}
    if (topic_safety_stop_.empty()) {topic_safety_stop_ = topic_names::kSafetyStop;}
    if (topic_stuck_detected_.empty()) {topic_stuck_detected_ = topic_names::kStuckDetected;}
    if (topic_status_.empty()) {topic_status_ = topic_names::kControlDebug;}
  }

  static double yaw_from_quaternion_(double w, double x, double y, double z)
  {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static const char * bool_str_(bool v) {return v ? "true" : "false";}

private:
  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stuck_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_no_progress_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_safety_stop_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Topic names
  std::string topic_cmd_vel_;
  std::string topic_odom_;
  std::string topic_safety_stop_;
  std::string topic_stuck_detected_;
  std::string topic_no_progress_;
  std::string topic_status_;

  // Params
  double loop_rate_hz_{20.0};
  double input_timeout_sec_{0.5};
  bool stale_cmd_as_zero_{true};
  bool stale_safety_stop_as_true_{false};

  // Core logic
  StuckDetector detector_{};

  // Input caches
  TimedTwist cmd_{};
  TimedOdom odom_{};
  TimedBool safety_stop_{};

  std::string last_status_;
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<savo_control::StuckDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
