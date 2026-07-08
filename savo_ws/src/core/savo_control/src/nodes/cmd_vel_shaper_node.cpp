#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_control/control_mode.hpp"
#include "savo_control/topic_names.hpp"
#include "savo_control/velocity_smoother.hpp"

namespace
{

double now_seconds(const rclcpp::Node & node)
{
  return node.get_clock()->now().seconds();
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

geometry_msgs::msg::Twist command_to_twist(const savo_control::TwistCommand & cmd)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = cmd.vx;
  msg.linear.y = cmd.vy;
  msg.angular.z = cmd.wz;
  return msg;
}

const char * bool_text(const bool value)
{
  return value ? "true" : "false";
}

}  // namespace

namespace savo_control
{

class CmdVelShaperNode : public rclcpp::Node
{
public:
  CmdVelShaperNode()
  : Node("cmd_vel_shaper_node")
  {
    declare_parameters();
    load_parameters();

    smoother_.set_config(make_smoother_config());

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = *msg;
        last_cmd_stamp_s_ = now_seconds(*this);
        have_cmd_ = true;
      });

    safety_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      safety_stop_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        safety_stop_ = msg->data;
        safety_stop_seen_ = true;
        safety_stop_stamp_s_ = now_seconds(*this);

        if (safety_stop_) {
          smoother_.reset_to_zero(safety_stop_stamp_s_);
        }
      });

    slowdown_sub_ = create_subscription<std_msgs::msg::Float32>(
      slowdown_topic_,
      10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        slowdown_factor_ = sanitize_slowdown_factor(msg->data);
        slowdown_seen_ = true;
        slowdown_stamp_s_ = now_seconds(*this);
      });

    mode_sub_ = create_subscription<std_msgs::msg::String>(
      mode_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        current_mode_ = parse_control_mode(msg->data, ControlMode::STOP);
        last_mode_text_ = msg->data;
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_hz_),
      [this]() {
        on_timer();
      });

    RCLCPP_INFO(
      get_logger(),
      "cmd_vel_shaper_node started | input=%s | output=%s | safety=%s",
      input_topic_.c_str(),
      output_topic_.c_str(),
      safety_stop_topic_.c_str());
  }

private:
  void declare_parameters()
  {
    declare_parameter<double>("publish_hz", 30.0);
    declare_parameter<double>("command_timeout_s", 0.50);

    declare_parameter<double>("max_vx", 0.25);
    declare_parameter<double>("max_vy", 0.25);
    declare_parameter<double>("max_wz", 0.60);

    declare_parameter<double>("deadband_vx", 0.0);
    declare_parameter<double>("deadband_vy", 0.0);
    declare_parameter<double>("deadband_wz", 0.0);

    declare_parameter<double>("max_accel_vx", 0.40);
    declare_parameter<double>("max_accel_vy", 0.40);
    declare_parameter<double>("max_accel_wz", 1.20);

    declare_parameter<double>("max_decel_vx", 0.60);
    declare_parameter<double>("max_decel_vy", 0.60);
    declare_parameter<double>("max_decel_wz", 1.80);

    declare_parameter<double>("min_dt_s", 1.0e-4);
    declare_parameter<double>("max_dt_s", 0.20);

    declare_parameter<bool>("reset_on_timeout", true);
    declare_parameter<bool>("use_decel_when_slowing", true);
    declare_parameter<bool>("safety_stop_forces_zero", true);

    declare_parameter<bool>("use_slowdown_factor", true);
    declare_parameter<bool>("ignore_slowdown_in_recovery", true);
    declare_parameter<double>("slowdown_timeout_s", 0.50);
    declare_parameter<double>("default_slowdown_factor", 1.0);
    declare_parameter<double>("min_slowdown_factor", 0.0);
    declare_parameter<double>("max_slowdown_factor", 1.0);

    declare_parameter<std::string>("input_topic", topics::CMD_VEL_MUX);
    declare_parameter<std::string>("output_topic", topics::CMD_VEL);
    declare_parameter<std::string>("mode_topic", topics::CONTROL_MODE_STATE);
    declare_parameter<std::string>("safety_stop_topic", topics::SAFETY_STOP);
    declare_parameter<std::string>("slowdown_topic", topics::SAFETY_SLOWDOWN_FACTOR);
    declare_parameter<std::string>("status_topic", "/savo_control/cmd_vel_shaper/status");
  }

  void load_parameters()
  {
    publish_hz_ = get_parameter("publish_hz").as_double();
    if (!std::isfinite(publish_hz_) || publish_hz_ <= 0.0) {
      publish_hz_ = 30.0;
    }

    command_timeout_s_ = get_parameter("command_timeout_s").as_double();
    if (!std::isfinite(command_timeout_s_) || command_timeout_s_ < 0.0) {
      command_timeout_s_ = 0.50;
    }

    max_vx_ = nonnegative_param("max_vx", 0.25);
    max_vy_ = nonnegative_param("max_vy", 0.25);
    max_wz_ = nonnegative_param("max_wz", 0.60);

    deadband_vx_ = nonnegative_param("deadband_vx", 0.0);
    deadband_vy_ = nonnegative_param("deadband_vy", 0.0);
    deadband_wz_ = nonnegative_param("deadband_wz", 0.0);

    max_accel_vx_ = nonnegative_param("max_accel_vx", 0.40);
    max_accel_vy_ = nonnegative_param("max_accel_vy", 0.40);
    max_accel_wz_ = nonnegative_param("max_accel_wz", 1.20);

    max_decel_vx_ = nonnegative_param("max_decel_vx", 0.60);
    max_decel_vy_ = nonnegative_param("max_decel_vy", 0.60);
    max_decel_wz_ = nonnegative_param("max_decel_wz", 1.80);

    min_dt_s_ = positive_param("min_dt_s", 1.0e-4);
    max_dt_s_ = positive_param("max_dt_s", 0.20);
    if (max_dt_s_ < min_dt_s_) {
      max_dt_s_ = min_dt_s_;
    }

    reset_on_timeout_ = get_parameter("reset_on_timeout").as_bool();
    use_decel_when_slowing_ = get_parameter("use_decel_when_slowing").as_bool();
    safety_stop_forces_zero_ = get_parameter("safety_stop_forces_zero").as_bool();

    use_slowdown_factor_ = get_parameter("use_slowdown_factor").as_bool();
    ignore_slowdown_in_recovery_ =
      get_parameter("ignore_slowdown_in_recovery").as_bool();
    slowdown_timeout_s_ = nonnegative_param("slowdown_timeout_s", 0.50);

    default_slowdown_factor_ = sanitize_slowdown_factor(
      get_parameter("default_slowdown_factor").as_double());
    min_slowdown_factor_ = sanitize_slowdown_factor(
      get_parameter("min_slowdown_factor").as_double());
    max_slowdown_factor_ = sanitize_slowdown_factor(
      get_parameter("max_slowdown_factor").as_double());

    if (min_slowdown_factor_ > max_slowdown_factor_) {
      std::swap(min_slowdown_factor_, max_slowdown_factor_);
    }

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    mode_topic_ = get_parameter("mode_topic").as_string();
    safety_stop_topic_ = get_parameter("safety_stop_topic").as_string();
    slowdown_topic_ = get_parameter("slowdown_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
  }

  VelocitySmootherConfig make_smoother_config() const
  {
    VelocitySmootherConfig config;

    config.max_accel_vx = max_accel_vx_;
    config.max_accel_vy = max_accel_vy_;
    config.max_accel_wz = max_accel_wz_;

    config.max_decel_vx = max_decel_vx_;
    config.max_decel_vy = max_decel_vy_;
    config.max_decel_wz = max_decel_wz_;

    config.command_timeout_s = command_timeout_s_;
    config.min_dt_s = min_dt_s_;
    config.max_dt_s = max_dt_s_;

    config.reset_on_timeout = reset_on_timeout_;
    config.use_decel_when_slowing = use_decel_when_slowing_;

    config.output_limits.max_vx = max_vx_;
    config.output_limits.max_vy = max_vy_;
    config.output_limits.max_wz = max_wz_;

    config.output_limits.deadband_vx = deadband_vx_;
    config.output_limits.deadband_vy = deadband_vy_;
    config.output_limits.deadband_wz = deadband_wz_;

    config.output_limits.use_symmetric_limits = true;

    return config.sanitized();
  }

  void on_timer()
  {
    const double now_s = now_seconds(*this);

    TwistCommand target{};
    std::string reason = "no_command";
    bool stale = true;

    const bool safety_active = safety_stop_active(now_s);
    if (safety_stop_forces_zero_ && safety_active) {
      target = TwistCommand{};
      smoother_.reset_to_zero(now_s);
      reason = "safety_stop";
      stale = false;
    } else if (command_fresh(now_s)) {
      target = twist_to_command(last_cmd_);
      if (!(ignore_slowdown_in_recovery_ && current_mode_ == ControlMode::RECOVERY)) {
        target = apply_slowdown(target, slowdown_factor_for_now(now_s));
      }
      reason = "tracking";
      stale = false;
    } else {
      target = TwistCommand{};
      reason = "command_stale";
      stale = true;
    }

    const TwistCommand shaped = smoother_.update(target, now_s);
    cmd_pub_->publish(command_to_twist(shaped));

    publish_status(now_s, target, shaped, reason, stale, safety_active);
  }

  bool command_fresh(const double now_s) const
  {
    if (!have_cmd_) {
      return false;
    }

    if (command_timeout_s_ <= 0.0) {
      return true;
    }

    return (now_s - last_cmd_stamp_s_) <= command_timeout_s_;
  }

  bool safety_stop_active(const double now_s) const
  {
    if (!safety_stop_seen_) {
      return false;
    }

    if (command_timeout_s_ > 0.0 && (now_s - safety_stop_stamp_s_) > command_timeout_s_) {
      return false;
    }

    return safety_stop_;
  }

  double slowdown_factor_for_now(const double now_s) const
  {
    if (!use_slowdown_factor_) {
      return 1.0;
    }

    if (!slowdown_seen_) {
      return default_slowdown_factor_;
    }

    if (slowdown_timeout_s_ > 0.0 && (now_s - slowdown_stamp_s_) > slowdown_timeout_s_) {
      return default_slowdown_factor_;
    }

    return sanitize_slowdown_factor(slowdown_factor_);
  }

  TwistCommand apply_slowdown(const TwistCommand & input, const double factor) const
  {
    const double scale = std::clamp(
      sanitize_slowdown_factor(factor),
      min_slowdown_factor_,
      max_slowdown_factor_);

    return make_twist_command(
      input.vx * scale,
      input.vy * scale,
      input.wz * scale);
  }

  double sanitize_slowdown_factor(const double value) const
  {
    if (!std::isfinite(value)) {
      return 1.0;
    }

    return std::clamp(value, 0.0, 1.0);
  }

  double nonnegative_param(const std::string & name, const double fallback) const
  {
    const double value = get_parameter(name).as_double();
    if (!std::isfinite(value) || value < 0.0) {
      return fallback;
    }
    return value;
  }

  double positive_param(const std::string & name, const double fallback) const
  {
    const double value = get_parameter(name).as_double();
    if (!std::isfinite(value) || value <= 0.0) {
      return fallback;
    }
    return value;
  }

  void publish_status(
    const double now_s,
    const TwistCommand & target,
    const TwistCommand & shaped,
    const std::string & reason,
    const bool stale,
    const bool safety_active)
  {
    std::ostringstream ss;
    ss << "reason=" << reason
       << "; stale=" << bool_text(stale)
       << "; safety_stop=" << bool_text(safety_active)
       << "; mode=" << last_mode_text_
       << "; slowdown=" << slowdown_factor_for_now(now_s)
       << "; ignore_slowdown_in_recovery=" << bool_text(ignore_slowdown_in_recovery_)
       << "; target_vx=" << target.vx
       << "; target_vy=" << target.vy
       << "; target_wz=" << target.wz
       << "; shaped_vx=" << shaped.vx
       << "; shaped_vy=" << shaped.vy
       << "; shaped_wz=" << shaped.wz
       << "; timed_out=" << bool_text(smoother_.timed_out())
       << "; now_s=" << now_s;

    status_pub_->publish(string_msg(ss.str()));
  }

  double publish_hz_{30.0};
  double command_timeout_s_{0.50};

  double max_vx_{0.25};
  double max_vy_{0.25};
  double max_wz_{0.60};

  double deadband_vx_{0.0};
  double deadband_vy_{0.0};
  double deadband_wz_{0.0};

  double max_accel_vx_{0.40};
  double max_accel_vy_{0.40};
  double max_accel_wz_{1.20};

  double max_decel_vx_{0.60};
  double max_decel_vy_{0.60};
  double max_decel_wz_{1.80};

  double min_dt_s_{1.0e-4};
  double max_dt_s_{0.20};

  bool reset_on_timeout_{true};
  bool use_decel_when_slowing_{true};
  bool safety_stop_forces_zero_{true};

  bool use_slowdown_factor_{true};
  bool ignore_slowdown_in_recovery_{true};
  double slowdown_timeout_s_{0.50};
  double default_slowdown_factor_{1.0};
  double min_slowdown_factor_{0.0};
  double max_slowdown_factor_{1.0};

  std::string input_topic_{topics::CMD_VEL_MUX};
  std::string output_topic_{topics::CMD_VEL};
  std::string mode_topic_{topics::CONTROL_MODE_STATE};
  std::string safety_stop_topic_{topics::SAFETY_STOP};
  std::string slowdown_topic_{topics::SAFETY_SLOWDOWN_FACTOR};
  std::string status_topic_{"/savo_control/cmd_vel_shaper/status"};

  geometry_msgs::msg::Twist last_cmd_{};
  bool have_cmd_{false};
  double last_cmd_stamp_s_{0.0};

  bool safety_stop_{false};
  bool safety_stop_seen_{false};
  double safety_stop_stamp_s_{0.0};

  double slowdown_factor_{1.0};
  bool slowdown_seen_{false};
  double slowdown_stamp_s_{0.0};

  ControlMode current_mode_{ControlMode::STOP};
  std::string last_mode_text_{"STOP"};

  VelocitySmoother smoother_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr slowdown_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<savo_control::CmdVelShaperNode>());
  rclcpp::shutdown();
  return 0;
}
