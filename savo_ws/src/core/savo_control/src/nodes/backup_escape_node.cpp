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

#include "savo_control/recovery_manager.hpp"
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

std_msgs::msg::Bool bool_msg(const bool value)
{
  std_msgs::msg::Bool msg;
  msg.data = value;
  return msg;
}

geometry_msgs::msg::Twist command_to_twist(const savo_control::TwistCommand & command)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = command.vx;
  msg.linear.y = command.vy;
  msg.angular.z = command.wz;
  return msg;
}

const char * bool_text(const bool value)
{
  return value ? "true" : "false";
}

}  // namespace

namespace savo_control
{

class BackupEscapeNode : public rclcpp::Node
{
public:
  BackupEscapeNode()
  : Node("backup_escape_node")
  {
    declare_parameters();
    load_parameters();

    manager_.set_config(make_config());

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_out_topic_, 10);
    active_pub_ = create_publisher<std_msgs::msg::Bool>(active_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
      trigger_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        const double now_s = now_seconds(*this);

        previous_trigger_ = trigger_;
        trigger_ = msg->data;
        trigger_seen_ = true;
        trigger_stamp_s_ = now_s;

        if (trigger_ && !previous_trigger_) {
          request_escape(now_s, RecoveryTrigger::MANUAL_REQUEST);
        }
      });

    safety_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      safety_stop_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        safety_stop_ = msg->data;
        safety_stop_seen_ = true;
        safety_stop_stamp_s_ = now_seconds(*this);
      });

    slowdown_sub_ = create_subscription<std_msgs::msg::Float32>(
      slowdown_topic_,
      10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        slowdown_factor_ = sanitize_slowdown_factor(msg->data);
        slowdown_seen_ = true;
        slowdown_stamp_s_ = now_seconds(*this);
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_hz_),
      [this]() {
        on_timer();
      });

    RCLCPP_INFO(
      get_logger(),
      "backup_escape_node started | trigger=%s | cmd_out=%s | safety=%s",
      trigger_topic_.c_str(),
      cmd_out_topic_.c_str(),
      safety_stop_topic_.c_str());
  }

private:
  void declare_parameters()
  {
    declare_parameter<double>("publish_hz", 20.0);
    declare_parameter<double>("input_timeout_s", 0.50);

    declare_parameter<bool>("enabled", true);
    declare_parameter<bool>("safety_stop_blocks_escape", true);
    declare_parameter<bool>("publish_zero_when_inactive", true);
    declare_parameter<bool>("auto_start_on_trigger_level", false);

    declare_parameter<std::string>("action", "BACKUP");
    declare_parameter<double>("backup_vx", -0.06);
    declare_parameter<double>("rotate_wz", 0.25);
    declare_parameter<double>("backup_duration_s", 1.0);
    declare_parameter<double>("rotate_duration_s", 0.8);
    declare_parameter<double>("stop_hold_s", 0.20);
    declare_parameter<double>("max_recovery_duration_s", 6.0);

    declare_parameter<double>("max_vx", 0.10);
    declare_parameter<double>("max_vy", 0.10);
    declare_parameter<double>("max_wz", 0.35);

    declare_parameter<bool>("stop_before_motion", true);
    declare_parameter<bool>("stop_after_motion", true);
    declare_parameter<bool>("allow_rotate_left", true);
    declare_parameter<bool>("allow_rotate_right", true);

    declare_parameter<bool>("use_slowdown_factor", true);
    declare_parameter<double>("slowdown_timeout_s", 0.50);
    declare_parameter<double>("default_slowdown_factor", 1.0);

    declare_parameter<std::string>("trigger_topic", topics::RECOVERY_REQUEST);
    declare_parameter<std::string>("safety_stop_topic", topics::SAFETY_STOP);
    declare_parameter<std::string>("slowdown_topic", topics::SAFETY_SLOWDOWN_FACTOR);
    declare_parameter<std::string>("cmd_out_topic", topics::CMD_VEL_RECOVERY);
    declare_parameter<std::string>("active_topic", "/savo_control/backup_escape_active");
    declare_parameter<std::string>("state_topic", "/savo_control/backup_escape_state");
    declare_parameter<std::string>("status_topic", topics::BACKUP_ESCAPE_STATUS);
  }

  void load_parameters()
  {
    publish_hz_ = positive_param("publish_hz", 20.0);
    input_timeout_s_ = nonnegative_param("input_timeout_s", 0.50);

    enabled_ = get_parameter("enabled").as_bool();
    safety_stop_blocks_escape_ = get_parameter("safety_stop_blocks_escape").as_bool();
    publish_zero_when_inactive_ = get_parameter("publish_zero_when_inactive").as_bool();
    auto_start_on_trigger_level_ = get_parameter("auto_start_on_trigger_level").as_bool();

    action_text_ = get_parameter("action").as_string();

    backup_vx_ = get_parameter("backup_vx").as_double();
    rotate_wz_ = get_parameter("rotate_wz").as_double();

    backup_duration_s_ = nonnegative_param("backup_duration_s", 1.0);
    rotate_duration_s_ = nonnegative_param("rotate_duration_s", 0.8);
    stop_hold_s_ = nonnegative_param("stop_hold_s", 0.20);
    max_recovery_duration_s_ = positive_param("max_recovery_duration_s", 6.0);

    max_vx_ = nonnegative_param("max_vx", 0.10);
    max_vy_ = nonnegative_param("max_vy", 0.10);
    max_wz_ = nonnegative_param("max_wz", 0.35);

    stop_before_motion_ = get_parameter("stop_before_motion").as_bool();
    stop_after_motion_ = get_parameter("stop_after_motion").as_bool();
    allow_rotate_left_ = get_parameter("allow_rotate_left").as_bool();
    allow_rotate_right_ = get_parameter("allow_rotate_right").as_bool();

    use_slowdown_factor_ = get_parameter("use_slowdown_factor").as_bool();
    slowdown_timeout_s_ = nonnegative_param("slowdown_timeout_s", 0.50);
    default_slowdown_factor_ =
      sanitize_slowdown_factor(get_parameter("default_slowdown_factor").as_double());

    trigger_topic_ = get_parameter("trigger_topic").as_string();
    safety_stop_topic_ = get_parameter("safety_stop_topic").as_string();
    slowdown_topic_ = get_parameter("slowdown_topic").as_string();
    cmd_out_topic_ = get_parameter("cmd_out_topic").as_string();
    active_topic_ = get_parameter("active_topic").as_string();
    state_topic_ = get_parameter("state_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
  }

  RecoveryManagerConfig make_config() const
  {
    RecoveryManagerConfig config;

    config.backup_vx = backup_vx_;
    config.rotate_wz = rotate_wz_;

    config.backup_duration_s = backup_duration_s_;
    config.rotate_duration_s = rotate_duration_s_;
    config.stop_hold_s = stop_hold_s_;
    config.max_recovery_duration_s = max_recovery_duration_s_;

    config.default_action = parse_recovery_action(action_text_, RecoveryAction::BACKUP);

    config.output_limits.max_vx = max_vx_;
    config.output_limits.max_vy = max_vy_;
    config.output_limits.max_wz = max_wz_;
    config.output_limits.use_symmetric_limits = true;

    config.enabled = enabled_;
    config.stop_before_motion = stop_before_motion_;
    config.stop_after_motion = stop_after_motion_;
    config.allow_rotate_left = allow_rotate_left_;
    config.allow_rotate_right = allow_rotate_right_;

    return config.sanitized();
  }

  void request_escape(const double now_s, const RecoveryTrigger trigger)
  {
    if (!enabled_) {
      return;
    }

    if (safety_stop_blocks_escape_ && safety_stop_active(now_s)) {
      return;
    }

    manager_.request(
      now_s,
      trigger,
      parse_recovery_action(action_text_, RecoveryAction::BACKUP));
  }

  void on_timer()
  {
    const double now_s = now_seconds(*this);

    if (
      auto_start_on_trigger_level_ &&
      trigger_active(now_s) &&
      !manager_.active() &&
      !(safety_stop_blocks_escape_ && safety_stop_active(now_s)))
    {
      request_escape(now_s, RecoveryTrigger::MANUAL_REQUEST);
    }

    const bool blocked = safety_stop_blocks_escape_ && safety_stop_active(now_s);
    const RecoveryManagerResult result = manager_.update_blocked(now_s, blocked);

    publish_result(result, now_s);
  }

  bool trigger_active(const double now_s) const
  {
    if (!trigger_seen_) {
      return false;
    }

    if (input_timeout_s_ > 0.0 && (now_s - trigger_stamp_s_) > input_timeout_s_) {
      return false;
    }

    return trigger_;
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

  TwistCommand apply_slowdown(const TwistCommand & command, const double factor) const
  {
    const double scale = sanitize_slowdown_factor(factor);

    return make_twist_command(
      command.vx * scale,
      command.vy * scale,
      command.wz * scale);
  }

  void publish_result(const RecoveryManagerResult & result, const double now_s)
  {
    TwistCommand command = result.command;

    if (result.active && !safety_stop_active(now_s)) {
      command = apply_slowdown(command, slowdown_factor_for_now(now_s));
    } else {
      command = TwistCommand{};
    }

    if (result.active || publish_zero_when_inactive_) {
      cmd_pub_->publish(command_to_twist(command));
    }

    active_pub_->publish(bool_msg(result.active));
    state_pub_->publish(string_msg(to_string(result.state)));

    std::ostringstream ss;
    ss << "state=" << to_string(result.state)
       << "; trigger=" << to_string(result.trigger)
       << "; action=" << to_string(result.action)
       << "; active=" << bool_text(result.active)
       << "; finished=" << bool_text(result.finished)
       << "; safety_stop=" << bool_text(safety_stop_active(now_s))
       << "; trigger_active=" << bool_text(trigger_active(now_s))
       << "; reason=" << result.reason
       << "; elapsed_s=" << result.elapsed_s
       << "; phase_elapsed_s=" << result.phase_elapsed_s
       << "; slowdown=" << slowdown_factor_for_now(now_s)
       << "; vx=" << command.vx
       << "; vy=" << command.vy
       << "; wz=" << command.wz
       << "; manager=[" << manager_.status_string() << "]";

    status_pub_->publish(string_msg(ss.str()));
  }

  double sanitize_slowdown_factor(const double value) const
  {
    if (!std::isfinite(value)) {
      return 1.0;
    }

    return std::clamp(value, 0.0, 1.0);
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
  bool safety_stop_blocks_escape_{true};
  bool publish_zero_when_inactive_{true};
  bool auto_start_on_trigger_level_{false};

  std::string action_text_{"BACKUP"};

  double backup_vx_{-0.06};
  double rotate_wz_{0.25};
  double backup_duration_s_{1.0};
  double rotate_duration_s_{0.8};
  double stop_hold_s_{0.20};
  double max_recovery_duration_s_{6.0};

  double max_vx_{0.10};
  double max_vy_{0.10};
  double max_wz_{0.35};

  bool stop_before_motion_{true};
  bool stop_after_motion_{true};
  bool allow_rotate_left_{true};
  bool allow_rotate_right_{true};

  bool use_slowdown_factor_{true};
  double slowdown_timeout_s_{0.50};
  double default_slowdown_factor_{1.0};

  std::string trigger_topic_{topics::RECOVERY_REQUEST};
  std::string safety_stop_topic_{topics::SAFETY_STOP};
  std::string slowdown_topic_{topics::SAFETY_SLOWDOWN_FACTOR};
  std::string cmd_out_topic_{topics::CMD_VEL_RECOVERY};
  std::string active_topic_{"/savo_control/backup_escape_active"};
  std::string state_topic_{"/savo_control/backup_escape_state"};
  std::string status_topic_{topics::BACKUP_ESCAPE_STATUS};

  bool trigger_{false};
  bool previous_trigger_{false};
  bool trigger_seen_{false};
  double trigger_stamp_s_{0.0};

  bool safety_stop_{false};
  bool safety_stop_seen_{false};
  double safety_stop_stamp_s_{0.0};

  double slowdown_factor_{1.0};
  bool slowdown_seen_{false};
  double slowdown_stamp_s_{0.0};

  RecoveryManager manager_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr slowdown_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<savo_control::BackupEscapeNode>());
  rclcpp::shutdown();
  return 0;
}
