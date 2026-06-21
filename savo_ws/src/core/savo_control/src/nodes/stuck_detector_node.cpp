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

namespace
{

double now_seconds(const rclcpp::Node & node)
{
  return node.get_clock()->now().seconds();
}

std_msgs::msg::Bool bool_msg(const bool value)
{
  std_msgs::msg::Bool msg;
  msg.data = value;
  return msg;
}

std_msgs::msg::String string_msg(const std::string & value)
{
  std_msgs::msg::String msg;
  msg.data = value;
  return msg;
}

savo_control::TwistCommand twist_to_command(const geometry_msgs::msg::Twist & msg)
{
  return savo_control::make_twist_command(
    msg.linear.x,
    msg.linear.y,
    msg.angular.z);
}

savo_control::ObservedMotion odom_to_motion(const nav_msgs::msg::Odometry & msg)
{
  return savo_control::ObservedMotion{
    savo_control::ControlMath::finite_or_zero(msg.twist.twist.linear.x),
    savo_control::ControlMath::finite_or_zero(msg.twist.twist.linear.y),
    savo_control::ControlMath::finite_or_zero(msg.twist.twist.angular.z)};
}

const char * bool_text(const bool value)
{
  return value ? "true" : "false";
}

}  // namespace

namespace savo_control
{

class StuckDetectorNode : public rclcpp::Node
{
public:
  StuckDetectorNode()
  : Node("stuck_detector_node")
  {
    declare_parameters();
    load_parameters();

    detector_.set_config(make_detector_config());

    stuck_pub_ = create_publisher<std_msgs::msg::Bool>(stuck_detected_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(stuck_state_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(stuck_status_topic_, 10);
    recovery_request_pub_ =
      create_publisher<std_msgs::msg::Bool>(recovery_request_topic_, 10);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = twist_to_command(*msg);
        last_cmd_stamp_s_ = now_seconds(*this);
        have_cmd_ = true;
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_motion_ = odom_to_motion(*msg);
        last_odom_stamp_s_ = now_seconds(*this);
        have_odom_ = true;
      });

    safety_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      safety_stop_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        safety_stop_ = msg->data;
        safety_stop_seen_ = true;
        safety_stop_stamp_s_ = now_seconds(*this);
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_hz_),
      [this]() {
        on_timer();
      });

    RCLCPP_INFO(
      get_logger(),
      "stuck_detector_node started | cmd=%s | odom=%s | stuck=%s",
      cmd_topic_.c_str(),
      odom_topic_.c_str(),
      stuck_detected_topic_.c_str());
  }

private:
  void declare_parameters()
  {
    declare_parameter<double>("publish_hz", 20.0);

    declare_parameter<bool>("enabled", true);
    declare_parameter<bool>("detect_vx", true);
    declare_parameter<bool>("detect_vy", true);
    declare_parameter<bool>("detect_wz", true);
    declare_parameter<bool>("require_all_requested_axes_stopped", false);

    declare_parameter<double>("command_linear_threshold", 0.04);
    declare_parameter<double>("command_angular_threshold", 0.10);
    declare_parameter<double>("observed_linear_threshold", 0.015);
    declare_parameter<double>("observed_angular_threshold", 0.04);

    declare_parameter<double>("stuck_duration_s", 1.00);
    declare_parameter<double>("stale_timeout_s", 0.50);

    declare_parameter<bool>("suppress_while_safety_stop", true);
    declare_parameter<bool>("auto_request_recovery", true);
    declare_parameter<double>("recovery_request_cooldown_s", 2.0);
    declare_parameter<double>("min_stuck_time_before_recovery_s", 0.0);

    declare_parameter<std::string>("cmd_topic", topics::CMD_VEL_SAFE);
    declare_parameter<std::string>("odom_topic", topics::ODOM_FILTERED);
    declare_parameter<std::string>("safety_stop_topic", topics::SAFETY_STOP);

    declare_parameter<std::string>("stuck_detected_topic", topics::STUCK_DETECTED);
    declare_parameter<std::string>("stuck_state_topic", topics::STUCK_STATE);
    declare_parameter<std::string>(
      "stuck_status_topic",
      "/savo_control/stuck_detector/status");
    declare_parameter<std::string>("recovery_request_topic", topics::RECOVERY_REQUEST);
  }

  void load_parameters()
  {
    publish_hz_ = positive_param("publish_hz", 20.0);

    enabled_ = get_parameter("enabled").as_bool();
    detect_vx_ = get_parameter("detect_vx").as_bool();
    detect_vy_ = get_parameter("detect_vy").as_bool();
    detect_wz_ = get_parameter("detect_wz").as_bool();
    require_all_requested_axes_stopped_ =
      get_parameter("require_all_requested_axes_stopped").as_bool();

    command_linear_threshold_ = nonnegative_param("command_linear_threshold", 0.04);
    command_angular_threshold_ = nonnegative_param("command_angular_threshold", 0.10);
    observed_linear_threshold_ = nonnegative_param("observed_linear_threshold", 0.015);
    observed_angular_threshold_ = nonnegative_param("observed_angular_threshold", 0.04);

    stuck_duration_s_ = nonnegative_param("stuck_duration_s", 1.00);
    stale_timeout_s_ = nonnegative_param("stale_timeout_s", 0.50);

    suppress_while_safety_stop_ = get_parameter("suppress_while_safety_stop").as_bool();
    auto_request_recovery_ = get_parameter("auto_request_recovery").as_bool();
    recovery_request_cooldown_s_ =
      nonnegative_param("recovery_request_cooldown_s", 2.0);
    min_stuck_time_before_recovery_s_ =
      nonnegative_param("min_stuck_time_before_recovery_s", 0.0);

    cmd_topic_ = get_parameter("cmd_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    safety_stop_topic_ = get_parameter("safety_stop_topic").as_string();

    stuck_detected_topic_ = get_parameter("stuck_detected_topic").as_string();
    stuck_state_topic_ = get_parameter("stuck_state_topic").as_string();
    stuck_status_topic_ = get_parameter("stuck_status_topic").as_string();
    recovery_request_topic_ = get_parameter("recovery_request_topic").as_string();
  }

  StuckDetectorConfig make_detector_config() const
  {
    StuckDetectorConfig config;

    config.command_linear_threshold = command_linear_threshold_;
    config.command_angular_threshold = command_angular_threshold_;
    config.observed_linear_threshold = observed_linear_threshold_;
    config.observed_angular_threshold = observed_angular_threshold_;

    config.stuck_duration_s = stuck_duration_s_;
    config.stale_timeout_s = stale_timeout_s_;

    config.enabled = enabled_;
    config.detect_vx = detect_vx_;
    config.detect_vy = detect_vy_;
    config.detect_wz = detect_wz_;
    config.require_all_requested_axes_stopped = require_all_requested_axes_stopped_;

    return config.sanitized();
  }

  void on_timer()
  {
    const double now_s = now_seconds(*this);

    const bool suppress =
      suppress_while_safety_stop_ &&
      safety_stop_active(now_s);

    const double command_stamp = have_cmd_ ? last_cmd_stamp_s_ : old_stamp(now_s);
    const double odom_stamp = have_odom_ ? last_odom_stamp_s_ : old_stamp(now_s);

    const StuckDetectorResult result = detector_.update(
      have_cmd_ ? last_cmd_ : TwistCommand{},
      have_odom_ ? last_motion_ : ObservedMotion{},
      now_s,
      command_stamp,
      odom_stamp,
      suppress);

    maybe_request_recovery(result, now_s);
    publish_result(result, now_s);
  }

  void maybe_request_recovery(const StuckDetectorResult & result, const double now_s)
  {
    if (!auto_request_recovery_) {
      recovery_request_pub_->publish(bool_msg(false));
      return;
    }

    const bool cooldown_ok =
      !have_recovery_request_ ||
      recovery_request_cooldown_s_ <= 0.0 ||
      (now_s - last_recovery_request_s_) >= recovery_request_cooldown_s_;

    const bool duration_ok = result.duration_s >= min_stuck_time_before_recovery_s_;

    if (result.stuck && cooldown_ok && duration_ok) {
      detector_.request_recovery();

      recovery_request_pub_->publish(bool_msg(true));
      last_recovery_request_s_ = now_s;
      have_recovery_request_ = true;
      return;
    }

    recovery_request_pub_->publish(bool_msg(false));
  }

  void publish_result(const StuckDetectorResult & result, const double now_s)
  {
    stuck_pub_->publish(bool_msg(result.stuck));
    state_pub_->publish(string_msg(to_string(result.state)));

    std::ostringstream ss;
    ss << "state=" << to_string(result.state)
       << "; stuck=" << bool_text(result.stuck)
       << "; recovery_requested=" << bool_text(result.recovery_requested)
       << "; command_active=" << bool_text(result.command_active)
       << "; observed_moving=" << bool_text(result.observed_moving)
       << "; stale=" << bool_text(result.stale)
       << "; enabled=" << bool_text(result.enabled)
       << "; safety_stop=" << bool_text(safety_stop_active(now_s))
       << "; reason=" << result.reason
       << "; duration_s=" << result.duration_s
       << "; command_age_s=" << result.command_age_s
       << "; odom_age_s=" << result.odom_age_s
       << "; cmd_vx=" << last_cmd_.vx
       << "; cmd_vy=" << last_cmd_.vy
       << "; cmd_wz=" << last_cmd_.wz
       << "; obs_vx=" << last_motion_.vx
       << "; obs_vy=" << last_motion_.vy
       << "; obs_wz=" << last_motion_.wz;

    status_pub_->publish(string_msg(ss.str()));
  }

  bool safety_stop_active(const double now_s) const
  {
    if (!safety_stop_seen_) {
      return false;
    }

    if (stale_timeout_s_ > 0.0 && (now_s - safety_stop_stamp_s_) > stale_timeout_s_) {
      return false;
    }

    return safety_stop_;
  }

  double old_stamp(const double now_s) const
  {
    return now_s - (stale_timeout_s_ + 1.0);
  }

  double positive_param(const std::string & name, const double fallback) const
  {
    const double value = get_parameter(name).as_double();
    if (!std::isfinite(value) || value <= 0.0) {
      return fallback;
    }

    return value;
  }

  double nonnegative_param(const std::string & name, const double fallback) const
  {
    const double value = get_parameter(name).as_double();
    if (!std::isfinite(value) || value < 0.0) {
      return fallback;
    }

    return value;
  }

  double publish_hz_{20.0};

  bool enabled_{true};
  bool detect_vx_{true};
  bool detect_vy_{true};
  bool detect_wz_{true};
  bool require_all_requested_axes_stopped_{false};

  double command_linear_threshold_{0.04};
  double command_angular_threshold_{0.10};
  double observed_linear_threshold_{0.015};
  double observed_angular_threshold_{0.04};

  double stuck_duration_s_{1.00};
  double stale_timeout_s_{0.50};

  bool suppress_while_safety_stop_{true};
  bool auto_request_recovery_{true};
  double recovery_request_cooldown_s_{2.0};
  double min_stuck_time_before_recovery_s_{0.0};

  std::string cmd_topic_{topics::CMD_VEL_SAFE};
  std::string odom_topic_{topics::ODOM_FILTERED};
  std::string safety_stop_topic_{topics::SAFETY_STOP};

  std::string stuck_detected_topic_{topics::STUCK_DETECTED};
  std::string stuck_state_topic_{topics::STUCK_STATE};
  std::string stuck_status_topic_{"/savo_control/stuck_detector/status"};
  std::string recovery_request_topic_{topics::RECOVERY_REQUEST};

  TwistCommand last_cmd_{};
  ObservedMotion last_motion_{};

  bool have_cmd_{false};
  bool have_odom_{false};
  double last_cmd_stamp_s_{0.0};
  double last_odom_stamp_s_{0.0};

  bool safety_stop_{false};
  bool safety_stop_seen_{false};
  double safety_stop_stamp_s_{0.0};

  bool have_recovery_request_{false};
  double last_recovery_request_s_{0.0};

  StuckDetector detector_{};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stuck_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recovery_request_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_stop_sub_;

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
