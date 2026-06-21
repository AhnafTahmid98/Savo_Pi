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

#include "savo_control/heading_pid.hpp"
#include "savo_control/topic_names.hpp"

namespace
{

double now_seconds(const rclcpp::Node & node)
{
  return node.get_clock()->now().seconds();
}

double yaw_from_odom(const nav_msgs::msg::Odometry & msg)
{
  const auto & q = msg.pose.pose.orientation;

  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

  return std::atan2(siny_cosp, cosy_cosp);
}

std_msgs::msg::String string_msg(const std::string & value)
{
  std_msgs::msg::String msg;
  msg.data = value;
  return msg;
}

geometry_msgs::msg::Twist zero_twist()
{
  geometry_msgs::msg::Twist msg;
  return msg;
}

geometry_msgs::msg::Twist sanitized_twist(const geometry_msgs::msg::Twist & input)
{
  geometry_msgs::msg::Twist out = input;

  if (!std::isfinite(out.linear.x)) {
    out.linear.x = 0.0;
  }
  if (!std::isfinite(out.linear.y)) {
    out.linear.y = 0.0;
  }
  if (!std::isfinite(out.linear.z)) {
    out.linear.z = 0.0;
  }
  if (!std::isfinite(out.angular.x)) {
    out.angular.x = 0.0;
  }
  if (!std::isfinite(out.angular.y)) {
    out.angular.y = 0.0;
  }
  if (!std::isfinite(out.angular.z)) {
    out.angular.z = 0.0;
  }

  return out;
}

const char * bool_text(const bool value)
{
  return value ? "true" : "false";
}

}  // namespace

namespace savo_control
{

class HeadingPidNode : public rclcpp::Node
{
public:
  HeadingPidNode()
  : Node("heading_pid_node")
  {
    declare_parameters();
    load_parameters();

    controller_.set_config(make_config());

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_yaw_rad_ = yaw_from_odom(*msg);
        odom_stamp_s_ = now_seconds(*this);
        have_odom_ = true;
      });

    base_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      base_cmd_topic_,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        base_cmd_ = sanitized_twist(*msg);
        base_cmd_stamp_s_ = now_seconds(*this);
        have_base_cmd_ = true;
      });

    target_sub_ = create_subscription<std_msgs::msg::Float64>(
      heading_target_topic_,
      10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        set_target(msg->data, now_seconds(*this));
        heading_hold_enabled_ = true;
        controller_.reset();
      });

    hold_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      heading_hold_enable_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        const double now_s = now_seconds(*this);
        heading_hold_enabled_ = msg->data;

        if (heading_hold_enabled_ && !has_target_ && have_odom_) {
          set_target(current_yaw_rad_, now_s);
        }

        if (!heading_hold_enabled_) {
          controller_.reset();
        }
      });

    safety_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      safety_stop_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        safety_stop_ = msg->data;
        safety_stop_seen_ = true;
        safety_stop_stamp_s_ = now_seconds(*this);

        if (safety_stop_) {
          controller_.reset();
        }
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_hz_),
      [this]() {
        on_timer();
      });

    RCLCPP_INFO(
      get_logger(),
      "heading_pid_node started | odom=%s base_cmd=%s output=%s",
      odom_topic_.c_str(),
      base_cmd_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  void declare_parameters()
  {
    declare_parameter<double>("publish_hz", 20.0);
    declare_parameter<double>("input_timeout_s", 0.50);

    declare_parameter<bool>("enabled", true);
    declare_parameter<bool>("publish_zero_when_inactive", true);
    declare_parameter<bool>("safety_stop_blocks_motion", true);
    declare_parameter<bool>("capture_current_heading_on_start", true);
    declare_parameter<bool>("preserve_base_linear_velocity", true);

    declare_parameter<double>("kp", 1.20);
    declare_parameter<double>("ki", 0.0);
    declare_parameter<double>("kd", 0.05);

    declare_parameter<double>("tolerance_rad", 0.035);
    declare_parameter<double>("max_wz_rad_s", 0.45);
    declare_parameter<double>("min_wz_when_active", 0.08);
    declare_parameter<double>("disable_min_wz_below_error_rad", 0.08);
    declare_parameter<double>("output_deadband_rad_s", 0.0);

    declare_parameter<double>("min_dt_sec", 1.0e-4);
    declare_parameter<double>("max_dt_sec", 0.50);

    declare_parameter<std::string>("odom_topic", topics::ODOM_FILTERED);
    declare_parameter<std::string>("base_cmd_topic", "/cmd_vel_raw");
    declare_parameter<std::string>("output_topic", topics::CMD_VEL_AUTO);
    declare_parameter<std::string>("heading_target_topic", "/savo_control/heading_target_rad");
    declare_parameter<std::string>("heading_hold_enable_topic",
        "/savo_control/heading_hold_enable");
    declare_parameter<std::string>("safety_stop_topic", topics::SAFETY_STOP);
    declare_parameter<std::string>("state_topic", "/savo_control/heading_pid_state");
    declare_parameter<std::string>("status_topic", "/savo_control/heading_pid_status");
  }

  void load_parameters()
  {
    publish_hz_ = positive_param("publish_hz", 20.0);
    input_timeout_s_ = nonnegative_param("input_timeout_s", 0.50);

    enabled_ = get_parameter("enabled").as_bool();
    publish_zero_when_inactive_ = get_parameter("publish_zero_when_inactive").as_bool();
    safety_stop_blocks_motion_ = get_parameter("safety_stop_blocks_motion").as_bool();
    capture_current_heading_on_start_ =
      get_parameter("capture_current_heading_on_start").as_bool();
    preserve_base_linear_velocity_ =
      get_parameter("preserve_base_linear_velocity").as_bool();

    kp_ = finite_param("kp", 1.20);
    ki_ = finite_param("ki", 0.0);
    kd_ = finite_param("kd", 0.05);

    tolerance_rad_ = nonnegative_param("tolerance_rad", 0.035);
    max_wz_rad_s_ = nonnegative_param("max_wz_rad_s", 0.45);
    min_wz_when_active_ = nonnegative_param("min_wz_when_active", 0.08);
    disable_min_wz_below_error_rad_ =
      nonnegative_param("disable_min_wz_below_error_rad", 0.08);
    output_deadband_rad_s_ = nonnegative_param("output_deadband_rad_s", 0.0);

    min_dt_sec_ = positive_param("min_dt_sec", 1.0e-4);
    max_dt_sec_ = positive_param("max_dt_sec", 0.50);
    if (max_dt_sec_ < min_dt_sec_) {
      max_dt_sec_ = min_dt_sec_;
    }

    odom_topic_ = get_parameter("odom_topic").as_string();
    base_cmd_topic_ = get_parameter("base_cmd_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    heading_target_topic_ = get_parameter("heading_target_topic").as_string();
    heading_hold_enable_topic_ = get_parameter("heading_hold_enable_topic").as_string();
    safety_stop_topic_ = get_parameter("safety_stop_topic").as_string();
    state_topic_ = get_parameter("state_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
  }

  HeadingControllerConfig make_config() const
  {
    HeadingControllerConfig config;

    config.target_yaw_rad = target_yaw_rad_;
    config.tolerance_rad = tolerance_rad_;

    config.kp = kp_;
    config.ki = ki_;
    config.kd = kd_;

    config.max_wz_rad_s = max_wz_rad_s_;
    config.min_wz_when_active = min_wz_when_active_;
    config.disable_min_wz_below_error_rad = disable_min_wz_below_error_rad_;
    config.output_deadband_rad_s = output_deadband_rad_s_;

    config.min_dt_sec = min_dt_sec_;
    config.max_dt_sec = max_dt_sec_;

    return config.sanitized();
  }

  void set_target(const double yaw_rad, const double now_s)
  {
    target_yaw_rad_ = ControlMath::wrap_angle_rad(ControlMath::finite_or_zero(yaw_rad));
    target_stamp_s_ = now_s;
    has_target_ = true;

    controller_.set_target_yaw(target_yaw_rad_);
  }

  void on_timer()
  {
    const double now_s = now_seconds(*this);

    if (!enabled_) {
      publish_zero("disabled", now_s);
      return;
    }

    if (safety_stop_blocks_motion_ && safety_stop_active(now_s)) {
      publish_zero("safety_stop", now_s);
      return;
    }

    if (!heading_hold_enabled_) {
      publish_inactive(now_s);
      return;
    }

    if (!have_odom_ || !odom_fresh(now_s)) {
      controller_.reset();
      publish_zero("odom_stale", now_s);
      return;
    }

    if (!has_target_ && capture_current_heading_on_start_) {
      set_target(current_yaw_rad_, now_s);
    }

    if (!has_target_) {
      publish_zero("no_target", now_s);
      return;
    }

    double dt_s = now_s - last_update_s_;
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
      dt_s = min_dt_sec_;
    }
    dt_s = std::clamp(dt_s, min_dt_sec_, max_dt_sec_);
    last_update_s_ = now_s;

    controller_.set_config(make_config());

    const HeadingControllerResult result =
      controller_.update(current_yaw_rad_, target_yaw_rad_, dt_s);

    geometry_msgs::msg::Twist cmd = zero_twist();

    if (preserve_base_linear_velocity_ && base_cmd_fresh(now_s)) {
      cmd.linear.x = base_cmd_.linear.x;
      cmd.linear.y = base_cmd_.linear.y;
    }

    if (result.valid && !result.within_tolerance) {
      cmd.angular.z = result.wz_cmd_rad_s;
    }

    cmd_pub_->publish(cmd);
    publish_state_and_status(result, cmd, "tracking", now_s);
  }

  void publish_inactive(const double now_s)
  {
    if (publish_zero_when_inactive_) {
      publish_zero("inactive", now_s);
      return;
    }

    HeadingControllerResult result;
    result.valid = false;
    result.current_yaw_rad = current_yaw_rad_;
    result.target_yaw_rad = target_yaw_rad_;
    result.error_rad = has_target_ ?
      ControlMath::shortest_angular_distance_rad(current_yaw_rad_, target_yaw_rad_) :
      0.0;
    result.reason = "inactive";

    publish_state_and_status(result, zero_twist(), "inactive", now_s);
  }

  void publish_zero(const std::string & reason, const double now_s)
  {
    const geometry_msgs::msg::Twist cmd = zero_twist();
    cmd_pub_->publish(cmd);

    HeadingControllerResult result;
    result.valid = false;
    result.current_yaw_rad = current_yaw_rad_;
    result.target_yaw_rad = target_yaw_rad_;
    result.error_rad = has_target_ ?
      ControlMath::shortest_angular_distance_rad(current_yaw_rad_, target_yaw_rad_) :
      0.0;
    result.reason = reason;

    publish_state_and_status(result, cmd, reason, now_s);
  }

  void publish_state_and_status(
    const HeadingControllerResult & result,
    const geometry_msgs::msg::Twist & cmd,
    const std::string & reason,
    const double now_s)
  {
    std::ostringstream state;
    state << "enabled=" << bool_text(enabled_)
          << "; hold=" << bool_text(heading_hold_enabled_)
          << "; target=" << bool_text(has_target_)
          << "; reason=" << reason;

    state_pub_->publish(string_msg(state.str()));

    std::ostringstream status;
    status << "enabled=" << bool_text(enabled_)
           << "; hold=" << bool_text(heading_hold_enabled_)
           << "; has_target=" << bool_text(has_target_)
           << "; have_odom=" << bool_text(have_odom_)
           << "; odom_fresh=" << bool_text(odom_fresh(now_s))
           << "; base_cmd_fresh=" << bool_text(base_cmd_fresh(now_s))
           << "; safety_stop=" << bool_text(safety_stop_active(now_s))
           << "; valid=" << bool_text(result.valid)
           << "; within_tolerance=" << bool_text(result.within_tolerance)
           << "; reason=" << reason
           << "; current_yaw_rad=" << current_yaw_rad_
           << "; target_yaw_rad=" << target_yaw_rad_
           << "; error_rad=" << result.error_rad
           << "; vx=" << cmd.linear.x
           << "; vy=" << cmd.linear.y
           << "; wz=" << cmd.angular.z
           << "; now_s=" << now_s;

    status_pub_->publish(string_msg(status.str()));
  }

  bool odom_fresh(const double now_s) const
  {
    if (!have_odom_) {
      return false;
    }

    if (input_timeout_s_ <= 0.0) {
      return true;
    }

    return (now_s - odom_stamp_s_) <= input_timeout_s_;
  }

  bool base_cmd_fresh(const double now_s) const
  {
    if (!have_base_cmd_) {
      return false;
    }

    if (input_timeout_s_ <= 0.0) {
      return true;
    }

    return (now_s - base_cmd_stamp_s_) <= input_timeout_s_;
  }

  bool safety_stop_active(const double now_s) const
  {
    if (!safety_stop_seen_) {
      return false;
    }

    if (input_timeout_s_ > 0.0 && (now_s - safety_stop_stamp_s_) > input_timeout_s_) {
      return false;
    }

    return safety_stop_;
  }

  double finite_param(const std::string & name, const double fallback) const
  {
    const double value = get_parameter(name).as_double();
    return std::isfinite(value) ? value : fallback;
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
  double input_timeout_s_{0.50};

  bool enabled_{true};
  bool publish_zero_when_inactive_{true};
  bool safety_stop_blocks_motion_{true};
  bool capture_current_heading_on_start_{true};
  bool preserve_base_linear_velocity_{true};

  double kp_{1.20};
  double ki_{0.0};
  double kd_{0.05};

  double tolerance_rad_{0.035};
  double max_wz_rad_s_{0.45};
  double min_wz_when_active_{0.08};
  double disable_min_wz_below_error_rad_{0.08};
  double output_deadband_rad_s_{0.0};

  double min_dt_sec_{1.0e-4};
  double max_dt_sec_{0.50};

  std::string odom_topic_{topics::ODOM_FILTERED};
  std::string base_cmd_topic_{"/cmd_vel_raw"};
  std::string output_topic_{topics::CMD_VEL_AUTO};
  std::string heading_target_topic_{"/savo_control/heading_target_rad"};
  std::string heading_hold_enable_topic_{"/savo_control/heading_hold_enable"};
  std::string safety_stop_topic_{topics::SAFETY_STOP};
  std::string state_topic_{"/savo_control/heading_pid_state"};
  std::string status_topic_{"/savo_control/heading_pid_status"};

  double current_yaw_rad_{0.0};
  double target_yaw_rad_{0.0};

  bool heading_hold_enabled_{false};
  bool has_target_{false};
  bool have_odom_{false};
  bool have_base_cmd_{false};

  double odom_stamp_s_{0.0};
  double base_cmd_stamp_s_{0.0};
  double target_stamp_s_{0.0};
  double last_update_s_{0.0};

  geometry_msgs::msg::Twist base_cmd_{};

  bool safety_stop_{false};
  bool safety_stop_seen_{false};
  double safety_stop_stamp_s_{0.0};

  HeadingPid controller_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr base_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hold_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_stop_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<savo_control::HeadingPidNode>());
  rclcpp::shutdown();
  return 0;
}
