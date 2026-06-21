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
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_control/distance_pid.hpp"
#include "savo_control/topic_names.hpp"

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

geometry_msgs::msg::Twist zero_twist()
{
  geometry_msgs::msg::Twist msg;
  return msg;
}

const char * bool_text(const bool value)
{
  return value ? "true" : "false";
}

}  // namespace

namespace savo_control
{

class DistanceApproachNode : public rclcpp::Node
{
public:
  DistanceApproachNode()
  : Node("distance_approach_node")
  {
    declare_parameters();
    load_parameters();

    controller_.set_config(make_config());

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic_, 10);
    mode_cmd_pub_ = create_publisher<std_msgs::msg::String>(mode_cmd_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    distance_sub_ = create_subscription<std_msgs::msg::Float32>(
      distance_topic_,
      10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        latest_distance_m_ = static_cast<double>(msg->data);
        distance_stamp_s_ = now_seconds(*this);
        have_distance_ = true;
      });

    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        const bool was_enabled = active_;
        active_ = msg->data;

        if (active_ && !was_enabled) {
          active_start_s_ = now_seconds(*this);
          last_update_s_ = active_start_s_;
          controller_.reset();

          if (request_auto_mode_on_enable_) {
            mode_cmd_pub_->publish(string_msg("AUTO"));
          }
        }

        if (!active_) {
          controller_.reset();
          publish_zero("disabled_by_command", now_seconds(*this));
        }
      });

    target_sub_ = create_subscription<std_msgs::msg::Float64>(
      target_topic_,
      10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        target_distance_m_ = std::max(0.0, ControlMath::finite_or_zero(msg->data));
        controller_.set_target_distance(target_distance_m_);

        if (auto_enable_on_target_) {
          active_ = true;
          active_start_s_ = now_seconds(*this);
          last_update_s_ = active_start_s_;

          if (request_auto_mode_on_enable_) {
            mode_cmd_pub_->publish(string_msg("AUTO"));
          }
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
      "distance_approach_node started | distance=%s cmd_out=%s target=%.3f",
      distance_topic_.c_str(),
      cmd_vel_out_topic_.c_str(),
      target_distance_m_);
  }

private:
  void declare_parameters()
  {
    declare_parameter<double>("publish_hz", 20.0);
    declare_parameter<double>("input_timeout_s", 0.50);

    declare_parameter<bool>("enabled", true);
    declare_parameter<bool>("start_active", false);
    declare_parameter<bool>("auto_enable_on_target", false);
    declare_parameter<bool>("auto_disable_when_goal_reached", false);
    declare_parameter<bool>("safety_stop_blocks_motion", true);
    declare_parameter<bool>("publish_zero_when_inactive", true);
    declare_parameter<bool>("request_auto_mode_on_enable", true);

    declare_parameter<double>("target_distance_m", 0.60);
    declare_parameter<double>("tolerance_m", 0.04);
    declare_parameter<double>("hard_min_distance_m", 0.35);
    declare_parameter<double>("min_valid_distance_m", 0.05);
    declare_parameter<double>("max_valid_distance_m", 3.00);

    declare_parameter<double>("kp", 0.45);
    declare_parameter<double>("ki", 0.0);
    declare_parameter<double>("kd", 0.03);

    declare_parameter<double>("max_forward_vx", 0.10);
    declare_parameter<bool>("allow_reverse", false);
    declare_parameter<double>("max_reverse_vx", 0.05);

    declare_parameter<double>("min_vx_when_active", 0.04);
    declare_parameter<double>("disable_min_vx_below_error_m", 0.08);
    declare_parameter<double>("output_deadband_m_s", 0.0);

    declare_parameter<double>("min_dt_sec", 1.0e-4);
    declare_parameter<double>("max_dt_sec", 0.50);
    declare_parameter<double>("max_duration_s", 10.0);

    declare_parameter<std::string>("distance_topic", topics::DEPTH_MIN_FRONT);
    declare_parameter<std::string>("enable_topic", topics::DISTANCE_TEST_ENABLE);
    declare_parameter<std::string>("target_topic", topics::DISTANCE_TEST_TARGET);
    declare_parameter<std::string>("cmd_vel_out_topic", topics::CMD_VEL_AUTO);
    declare_parameter<std::string>("mode_cmd_topic", topics::CONTROL_MODE_CMD);
    declare_parameter<std::string>("safety_stop_topic", topics::SAFETY_STOP);
    declare_parameter<std::string>("state_topic", topics::DISTANCE_TEST_STATE);
    declare_parameter<std::string>("status_topic", "/savo_control/distance_approach_status");
  }

  void load_parameters()
  {
    publish_hz_ = positive_param("publish_hz", 20.0);
    input_timeout_s_ = nonnegative_param("input_timeout_s", 0.50);

    enabled_ = get_parameter("enabled").as_bool();
    active_ = get_parameter("start_active").as_bool();
    auto_enable_on_target_ = get_parameter("auto_enable_on_target").as_bool();
    auto_disable_when_goal_reached_ =
      get_parameter("auto_disable_when_goal_reached").as_bool();
    safety_stop_blocks_motion_ = get_parameter("safety_stop_blocks_motion").as_bool();
    publish_zero_when_inactive_ = get_parameter("publish_zero_when_inactive").as_bool();
    request_auto_mode_on_enable_ = get_parameter("request_auto_mode_on_enable").as_bool();

    target_distance_m_ = nonnegative_param("target_distance_m", 0.60);
    tolerance_m_ = nonnegative_param("tolerance_m", 0.04);
    hard_min_distance_m_ = nonnegative_param("hard_min_distance_m", 0.35);
    min_valid_distance_m_ = nonnegative_param("min_valid_distance_m", 0.05);
    max_valid_distance_m_ = nonnegative_param("max_valid_distance_m", 3.00);
    if (max_valid_distance_m_ < min_valid_distance_m_) {
      max_valid_distance_m_ = min_valid_distance_m_;
    }

    kp_ = finite_param("kp", 0.45);
    ki_ = finite_param("ki", 0.0);
    kd_ = finite_param("kd", 0.03);

    max_forward_vx_ = nonnegative_param("max_forward_vx", 0.10);
    allow_reverse_ = get_parameter("allow_reverse").as_bool();
    max_reverse_vx_ = nonnegative_param("max_reverse_vx", 0.05);

    min_vx_when_active_ = nonnegative_param("min_vx_when_active", 0.04);
    disable_min_vx_below_error_m_ =
      nonnegative_param("disable_min_vx_below_error_m", 0.08);
    output_deadband_m_s_ = nonnegative_param("output_deadband_m_s", 0.0);

    min_dt_sec_ = positive_param("min_dt_sec", 1.0e-4);
    max_dt_sec_ = positive_param("max_dt_sec", 0.50);
    if (max_dt_sec_ < min_dt_sec_) {
      max_dt_sec_ = min_dt_sec_;
    }

    max_duration_s_ = positive_param("max_duration_s", 10.0);

    distance_topic_ = get_parameter("distance_topic").as_string();
    enable_topic_ = get_parameter("enable_topic").as_string();
    target_topic_ = get_parameter("target_topic").as_string();
    cmd_vel_out_topic_ = get_parameter("cmd_vel_out_topic").as_string();
    mode_cmd_topic_ = get_parameter("mode_cmd_topic").as_string();
    safety_stop_topic_ = get_parameter("safety_stop_topic").as_string();
    state_topic_ = get_parameter("state_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();

    active_start_s_ = 0.0;
    last_update_s_ = 0.0;
  }

  DistanceControllerConfig make_config() const
  {
    DistanceControllerConfig config;

    config.target_distance_m = target_distance_m_;
    config.tolerance_m = tolerance_m_;
    config.hard_min_distance_m = hard_min_distance_m_;

    config.min_valid_distance_m = min_valid_distance_m_;
    config.max_valid_distance_m = max_valid_distance_m_;
    config.distance_timeout_s = input_timeout_s_;

    config.kp = kp_;
    config.ki = ki_;
    config.kd = kd_;

    config.max_forward_vx = max_forward_vx_;
    config.allow_reverse = allow_reverse_;
    config.max_reverse_vx = max_reverse_vx_;

    config.min_vx_when_active = min_vx_when_active_;
    config.disable_min_vx_below_error_m = disable_min_vx_below_error_m_;
    config.output_deadband_m_s = output_deadband_m_s_;

    config.min_dt_sec = min_dt_sec_;
    config.max_dt_sec = max_dt_sec_;

    return config.sanitized();
  }

  void on_timer()
  {
    const double now_s = now_seconds(*this);

    if (!enabled_) {
      active_ = false;
      publish_zero("disabled", now_s);
      return;
    }

    if (!active_) {
      if (publish_zero_when_inactive_) {
        publish_zero("inactive", now_s);
      }
      return;
    }

    if (active_start_s_ <= 0.0) {
      active_start_s_ = now_s;
      last_update_s_ = now_s;
    }

    if ((now_s - active_start_s_) > max_duration_s_) {
      active_ = false;
      controller_.reset();
      publish_zero("timeout", now_s);
      return;
    }

    if (safety_stop_blocks_motion_ && safety_stop_active(now_s)) {
      controller_.reset();
      publish_zero("safety_stop", now_s);
      return;
    }

    if (!distance_fresh(now_s)) {
      controller_.reset();
      publish_zero("distance_stale", now_s);
      return;
    }

    double dt_s = now_s - last_update_s_;
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
      dt_s = min_dt_sec_;
    }
    dt_s = std::clamp(dt_s, min_dt_sec_, max_dt_sec_);
    last_update_s_ = now_s;

    controller_.set_config(make_config());

    const DistanceControllerResult result =
      controller_.update(latest_distance_m_, dt_s);

    geometry_msgs::msg::Twist cmd = zero_twist();

    if (result.valid && !result.within_tolerance && !result.hard_min_stop) {
      cmd.linear.x = result.vx_cmd_m_s;
    }

    if (result.within_tolerance && auto_disable_when_goal_reached_) {
      active_ = false;
      controller_.reset();
    }

    cmd_pub_->publish(cmd);
    publish_state_and_status(result, cmd, "tracking", now_s);
  }

  void publish_zero(const std::string & reason, const double now_s)
  {
    const geometry_msgs::msg::Twist cmd = zero_twist();
    cmd_pub_->publish(cmd);

    DistanceControllerResult result;
    result.valid = false;
    result.distance_valid = distance_fresh(now_s);
    result.distance_m = latest_distance_m_;
    result.target_distance_m = target_distance_m_;
    result.error_m = latest_distance_m_ - target_distance_m_;
    result.reason = reason;

    publish_state_and_status(result, cmd, reason, now_s);
  }

  void publish_state_and_status(
    const DistanceControllerResult & result,
    const geometry_msgs::msg::Twist & cmd,
    const std::string & reason,
    const double now_s)
  {
    std::ostringstream state;
    state << "active=" << bool_text(active_)
          << "; reason=" << reason
          << "; target_m=" << target_distance_m_;

    state_pub_->publish(string_msg(state.str()));

    if (!publish_status_enabled_) {
      return;
    }

    std::ostringstream status;
    status << "active=" << bool_text(active_)
           << "; enabled=" << bool_text(enabled_)
           << "; have_distance=" << bool_text(have_distance_)
           << "; distance_fresh=" << bool_text(distance_fresh(now_s))
           << "; safety_stop=" << bool_text(safety_stop_active(now_s))
           << "; valid=" << bool_text(result.valid)
           << "; distance_valid=" << bool_text(result.distance_valid)
           << "; within_tolerance=" << bool_text(result.within_tolerance)
           << "; hard_min_stop=" << bool_text(result.hard_min_stop)
           << "; reverse_blocked=" << bool_text(result.reverse_blocked)
           << "; reason=" << reason
           << "; distance_m=" << latest_distance_m_
           << "; target_m=" << target_distance_m_
           << "; error_m=" << result.error_m
           << "; vx_cmd=" << cmd.linear.x
           << "; now_s=" << now_s;

    status_pub_->publish(string_msg(status.str()));
  }

  bool distance_fresh(const double now_s) const
  {
    if (!have_distance_) {
      return false;
    }

    if (input_timeout_s_ <= 0.0) {
      return true;
    }

    return (now_s - distance_stamp_s_) <= input_timeout_s_;
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
  bool active_{false};
  bool auto_enable_on_target_{false};
  bool auto_disable_when_goal_reached_{false};
  bool safety_stop_blocks_motion_{true};
  bool publish_zero_when_inactive_{true};
  bool request_auto_mode_on_enable_{true};
  bool publish_status_enabled_{true};

  double target_distance_m_{0.60};
  double tolerance_m_{0.04};
  double hard_min_distance_m_{0.35};
  double min_valid_distance_m_{0.05};
  double max_valid_distance_m_{3.00};

  double kp_{0.45};
  double ki_{0.0};
  double kd_{0.03};

  double max_forward_vx_{0.10};
  bool allow_reverse_{false};
  double max_reverse_vx_{0.05};

  double min_vx_when_active_{0.04};
  double disable_min_vx_below_error_m_{0.08};
  double output_deadband_m_s_{0.0};

  double min_dt_sec_{1.0e-4};
  double max_dt_sec_{0.50};
  double max_duration_s_{10.0};

  std::string distance_topic_{topics::DEPTH_MIN_FRONT};
  std::string enable_topic_{topics::DISTANCE_TEST_ENABLE};
  std::string target_topic_{topics::DISTANCE_TEST_TARGET};
  std::string cmd_vel_out_topic_{topics::CMD_VEL_AUTO};
  std::string mode_cmd_topic_{topics::CONTROL_MODE_CMD};
  std::string safety_stop_topic_{topics::SAFETY_STOP};
  std::string state_topic_{topics::DISTANCE_TEST_STATE};
  std::string status_topic_{"/savo_control/distance_approach_status"};

  double latest_distance_m_{0.0};
  bool have_distance_{false};
  double distance_stamp_s_{0.0};

  bool safety_stop_{false};
  bool safety_stop_seen_{false};
  double safety_stop_stamp_s_{0.0};

  double active_start_s_{0.0};
  double last_update_s_{0.0};

  DistancePid controller_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_stop_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<savo_control::DistanceApproachNode>());
  rclcpp::shutdown();
  return 0;
}
