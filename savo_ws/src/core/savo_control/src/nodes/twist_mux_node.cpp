#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_control/control_mode.hpp"
#include "savo_control/topic_names.hpp"

namespace
{

double now_seconds(const rclcpp::Node & node)
{
  return node.get_clock()->now().seconds();
}

geometry_msgs::msg::Twist zero_twist()
{
  geometry_msgs::msg::Twist msg;
  return msg;
}

std_msgs::msg::String make_string_msg(const std::string & value)
{
  std_msgs::msg::String msg;
  msg.data = value;
  return msg;
}

bool finite_twist(const geometry_msgs::msg::Twist & msg)
{
  return
    std::isfinite(msg.linear.x) &&
    std::isfinite(msg.linear.y) &&
    std::isfinite(msg.linear.z) &&
    std::isfinite(msg.angular.x) &&
    std::isfinite(msg.angular.y) &&
    std::isfinite(msg.angular.z);
}

geometry_msgs::msg::Twist sanitize_twist(const geometry_msgs::msg::Twist & msg)
{
  geometry_msgs::msg::Twist out = msg;

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

class TwistMuxNode : public rclcpp::Node
{
public:
  TwistMuxNode()
  : Node("twist_mux_node")
  {
    declare_parameters();
    load_parameters();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_out_topic_, 10);
    selected_source_pub_ = create_publisher<std_msgs::msg::String>(selected_source_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    manual_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      manual_topic_,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        update_sample(manual_, *msg);
      });

    auto_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      auto_topic_,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        update_sample(auto_, *msg);
      });

    nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      nav_topic_,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        update_sample(nav_, *msg);
      });

    recovery_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      recovery_topic_,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        update_sample(recovery_, *msg);
      });

    mode_sub_ = create_subscription<std_msgs::msg::String>(
      mode_topic_,
      rclcpp::QoS(10).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        current_mode_ = parse_control_mode(msg->data, ControlMode::STOP);
        last_mode_text_ = msg->data;
      });

    recovery_active_sub_ = create_subscription<std_msgs::msg::Bool>(
      recovery_active_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        recovery_active_ = msg->data;
        recovery_active_seen_ = true;
        recovery_active_stamp_s_ = now_seconds(*this);
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
      "twist_mux_node started | manual=%s auto=%s nav=%s recovery=%s out=%s",
      manual_topic_.c_str(),
      auto_topic_.c_str(),
      nav_topic_.c_str(),
      recovery_topic_.c_str(),
      cmd_out_topic_.c_str());
  }

private:
  struct CommandSample
  {
    geometry_msgs::msg::Twist msg{};
    double stamp_s{0.0};
    bool seen{false};
  };

  void declare_parameters()
  {
    declare_parameter<double>("publish_hz", 30.0);
    declare_parameter<double>("command_timeout_s", 0.35);
    declare_parameter<bool>("zero_on_stale", true);
    declare_parameter<bool>("safety_stop_forces_zero", true);
    declare_parameter<bool>("require_recovery_active_for_recovery_cmd", true);

    declare_parameter<std::string>("manual_topic", topics::CMD_VEL_MANUAL);
    declare_parameter<std::string>("auto_topic", topics::CMD_VEL_AUTO);
    declare_parameter<std::string>("nav_topic", topics::CMD_VEL_NAV);
    declare_parameter<std::string>("recovery_topic", topics::CMD_VEL_RECOVERY);
    declare_parameter<std::string>("cmd_out_topic", topics::CMD_VEL_MUX);

    declare_parameter<std::string>("mode_topic", topics::CONTROL_MODE_STATE);
    declare_parameter<std::string>("recovery_active_topic", topics::RECOVERY_ACTIVE);
    declare_parameter<std::string>("safety_stop_topic", topics::SAFETY_STOP);

    declare_parameter<std::string>("selected_source_topic", "/savo_control/twist_mux/source");
    declare_parameter<std::string>("status_topic", "/savo_control/twist_mux/status");
  }

  void load_parameters()
  {
    publish_hz_ = get_parameter("publish_hz").as_double();
    if (!std::isfinite(publish_hz_) || publish_hz_ <= 0.0) {
      publish_hz_ = 30.0;
    }

    command_timeout_s_ = get_parameter("command_timeout_s").as_double();
    if (!std::isfinite(command_timeout_s_) || command_timeout_s_ < 0.0) {
      command_timeout_s_ = 0.35;
    }

    zero_on_stale_ = get_parameter("zero_on_stale").as_bool();
    safety_stop_forces_zero_ = get_parameter("safety_stop_forces_zero").as_bool();
    require_recovery_active_for_recovery_cmd_ =
      get_parameter("require_recovery_active_for_recovery_cmd").as_bool();

    manual_topic_ = get_parameter("manual_topic").as_string();
    auto_topic_ = get_parameter("auto_topic").as_string();
    nav_topic_ = get_parameter("nav_topic").as_string();
    recovery_topic_ = get_parameter("recovery_topic").as_string();
    cmd_out_topic_ = get_parameter("cmd_out_topic").as_string();

    mode_topic_ = get_parameter("mode_topic").as_string();
    recovery_active_topic_ = get_parameter("recovery_active_topic").as_string();
    safety_stop_topic_ = get_parameter("safety_stop_topic").as_string();

    selected_source_topic_ = get_parameter("selected_source_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
  }

  void update_sample(CommandSample & sample, const geometry_msgs::msg::Twist & msg)
  {
    sample.msg = sanitize_twist(msg);
    sample.stamp_s = now_seconds(*this);
    sample.seen = true;
  }

  bool sample_fresh(const CommandSample & sample, const double now_s) const
  {
    if (!sample.seen) {
      return false;
    }

    if (command_timeout_s_ <= 0.0) {
      return true;
    }

    return (now_s - sample.stamp_s) <= command_timeout_s_;
  }

  bool bool_sample_fresh(const bool seen, const double stamp_s, const double now_s) const
  {
    if (!seen) {
      return false;
    }

    if (command_timeout_s_ <= 0.0) {
      return true;
    }

    return (now_s - stamp_s) <= command_timeout_s_;
  }

  void on_timer()
  {
    const double now_s = now_seconds(*this);

    geometry_msgs::msg::Twist selected = zero_twist();
    std::string source = "STOP";
    std::string reason = "stop_mode";
    bool stale = false;

    const bool safety_fresh = bool_sample_fresh(safety_stop_seen_, safety_stop_stamp_s_, now_s);
    const bool safety_active = safety_fresh && safety_stop_;

    if (safety_stop_forces_zero_ && safety_active) {
      selected = zero_twist();
      source = "SAFETY_STOP";
      reason = "safety_stop";
      publish(selected, source, reason, stale, now_s);
      return;
    }

    switch (current_mode_) {
      case ControlMode::MANUAL:
        select_sample(manual_, "MANUAL", now_s, selected, source, reason, stale);
        break;

      case ControlMode::AUTO:
        select_sample(auto_, "AUTO", now_s, selected, source, reason, stale);
        break;

      case ControlMode::NAV:
        select_sample(nav_, "NAV", now_s, selected, source, reason, stale);
        break;

      case ControlMode::RECOVERY:
        select_recovery(now_s, selected, source, reason, stale);
        break;

      case ControlMode::STOP:
      default:
        selected = zero_twist();
        source = "STOP";
        reason = "stop_mode";
        stale = false;
        break;
    }

    publish(selected, source, reason, stale, now_s);
  }

  void select_sample(
    const CommandSample & sample,
    const std::string & requested_source,
    const double now_s,
    geometry_msgs::msg::Twist & selected,
    std::string & source,
    std::string & reason,
    bool & stale) const
  {
    const bool fresh = sample_fresh(sample, now_s);

    if (fresh && finite_twist(sample.msg)) {
      selected = sample.msg;
      source = requested_source;
      reason = "selected";
      stale = false;
      return;
    }

    selected = zero_twist();
    source = requested_source;
    stale = true;
    reason = zero_on_stale_ ? "stale_zero" : "stale";
  }

  void select_recovery(
    const double now_s,
    geometry_msgs::msg::Twist & selected,
    std::string & source,
    std::string & reason,
    bool & stale) const
  {
    const bool recovery_active_fresh = bool_sample_fresh(
      recovery_active_seen_,
      recovery_active_stamp_s_,
      now_s);

    const bool recovery_allowed =
      !require_recovery_active_for_recovery_cmd_ ||
      (recovery_active_fresh && recovery_active_);

    if (!recovery_allowed) {
      selected = zero_twist();
      source = "RECOVERY";
      reason = "recovery_not_active";
      stale = true;
      return;
    }

    select_sample(recovery_, "RECOVERY", now_s, selected, source, reason, stale);
  }

  void publish(
    const geometry_msgs::msg::Twist & selected,
    const std::string & source,
    const std::string & reason,
    const bool stale,
    const double now_s)
  {
    cmd_pub_->publish(selected);
    selected_source_pub_->publish(make_string_msg(source));

    std::ostringstream ss;
    ss << "mode=" << to_string(current_mode_)
       << "; source=" << source
       << "; reason=" << reason
       << "; stale=" << bool_text(stale)
       << "; safety_stop=" << bool_text(safety_stop_)
       << "; recovery_active=" << bool_text(recovery_active_)
       << "; last_mode_text=" << last_mode_text_
       << "; now_s=" << now_s
       << "; vx=" << selected.linear.x
       << "; vy=" << selected.linear.y
       << "; wz=" << selected.angular.z;

    status_pub_->publish(make_string_msg(ss.str()));
  }

  double publish_hz_{30.0};
  double command_timeout_s_{0.35};

  bool zero_on_stale_{true};
  bool safety_stop_forces_zero_{true};
  bool require_recovery_active_for_recovery_cmd_{true};

  std::string manual_topic_{topics::CMD_VEL_MANUAL};
  std::string auto_topic_{topics::CMD_VEL_AUTO};
  std::string nav_topic_{topics::CMD_VEL_NAV};
  std::string recovery_topic_{topics::CMD_VEL_RECOVERY};
  std::string cmd_out_topic_{topics::CMD_VEL_MUX};

  std::string mode_topic_{topics::CONTROL_MODE_STATE};
  std::string recovery_active_topic_{topics::RECOVERY_ACTIVE};
  std::string safety_stop_topic_{topics::SAFETY_STOP};

  std::string selected_source_topic_{"/savo_control/twist_mux/source"};
  std::string status_topic_{"/savo_control/twist_mux/status"};

  ControlMode current_mode_{ControlMode::STOP};
  std::string last_mode_text_{"STOP"};

  CommandSample manual_{};
  CommandSample auto_{};
  CommandSample nav_{};
  CommandSample recovery_{};

  bool recovery_active_{false};
  bool recovery_active_seen_{false};
  double recovery_active_stamp_s_{0.0};

  bool safety_stop_{false};
  bool safety_stop_seen_{false};
  double safety_stop_stamp_s_{0.0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selected_source_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr auto_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr recovery_sub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recovery_active_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_stop_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<savo_control::TwistMuxNode>());
  rclcpp::shutdown();
  return 0;
}
