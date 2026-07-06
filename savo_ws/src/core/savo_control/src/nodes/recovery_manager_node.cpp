#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/qos.hpp"
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

geometry_msgs::msg::Twist command_to_twist(const savo_control::TwistCommand & command)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = command.vx;
  msg.linear.y = command.vy;
  msg.angular.z = command.wz;
  return msg;
}

geometry_msgs::msg::Twist zero_twist()
{
  geometry_msgs::msg::Twist msg;
  return msg;
}

bool twist_has_motion(
  const geometry_msgs::msg::Twist & msg,
  const double vx_threshold,
  const double vy_threshold,
  const double wz_threshold)
{
  return
    std::isfinite(msg.linear.x) &&
    std::isfinite(msg.linear.y) &&
    std::isfinite(msg.angular.z) &&
    (
      std::abs(msg.linear.x) >= vx_threshold ||
      std::abs(msg.linear.y) >= vy_threshold ||
      std::abs(msg.angular.z) >= wz_threshold
    );
}

const char * bool_text(const bool value)
{
  return value ? "true" : "false";
}

}  // namespace

namespace savo_control
{

class RecoveryManagerNode : public rclcpp::Node
{
public:
  RecoveryManagerNode()
  : Node("recovery_manager_node")
  {
    declare_parameters();
    load_parameters();

    manager_.set_config(make_config());

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_recovery_topic_, 10);
    active_pub_ = create_publisher<std_msgs::msg::Bool>(recovery_active_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(recovery_state_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(recovery_status_topic_, 10);

    stuck_sub_ = create_subscription<std_msgs::msg::Bool>(
      stuck_detected_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        stuck_detected_ = msg->data;
        stuck_seen_ = true;
        stuck_stamp_s_ = now_seconds(*this);
      });

    recovery_request_sub_ = create_subscription<std_msgs::msg::Bool>(
      recovery_request_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        recovery_request_ = msg->data;
        recovery_request_seen_ = true;
        recovery_request_stamp_s_ = now_seconds(*this);
      });

    safety_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      safety_stop_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        safety_stop_ = msg->data;
        safety_stop_seen_ = true;
        safety_stop_stamp_s_ = now_seconds(*this);
      });

    left_range_sub_ = create_subscription<std_msgs::msg::Float32>(
      left_range_topic_,
      rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        if (std::isfinite(static_cast<double>(msg->data))) {
          left_range_m_ = static_cast<double>(msg->data);
          left_range_seen_ = true;
          left_range_stamp_s_ = now_seconds(*this);
        }
      });

    right_range_sub_ = create_subscription<std_msgs::msg::Float32>(
      right_range_topic_,
      rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        if (std::isfinite(static_cast<double>(msg->data))) {
          right_range_m_ = static_cast<double>(msg->data);
          right_range_seen_ = true;
          right_range_stamp_s_ = now_seconds(*this);
        }
      });

    manual_motion_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      manual_motion_topic_,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        observe_motion_command(*msg);
      });

    auto_motion_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      auto_motion_topic_,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        observe_motion_command(*msg);
      });

    nav_motion_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      nav_motion_topic_,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        observe_motion_command(*msg);
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_hz_),
      [this]() {
        on_timer();
      });

    RCLCPP_INFO(
      get_logger(),
      "recovery_manager_node started | stuck=%s request=%s cmd=%s active=%s",
      stuck_detected_topic_.c_str(),
      recovery_request_topic_.c_str(),
      cmd_vel_recovery_topic_.c_str(),
      recovery_active_topic_.c_str());
  }

private:
  void declare_parameters()
  {
    declare_parameter<double>("publish_hz", 20.0);
    declare_parameter<double>("input_timeout_s", 0.50);

    declare_parameter<bool>("enabled", true);
    declare_parameter<bool>("auto_start_on_stuck", true);
    declare_parameter<bool>("auto_start_on_safety_stop", true);
    declare_parameter<bool>("manual_request_starts_recovery", true);
    declare_parameter<bool>("safety_stop_blocks_recovery", false);
    declare_parameter<bool>("publish_zero_when_inactive", true);

    declare_parameter<bool>("dynamic_action_enabled", true);
    declare_parameter<bool>("prefer_larger_free_side", true);
    declare_parameter<double>("side_block_m", 0.12);
    declare_parameter<double>("side_clear_m", 0.20);
    declare_parameter<double>("range_timeout_s", 0.60);

    declare_parameter<bool>("safety_stop_requires_recent_motion", true);
    declare_parameter<double>("recent_motion_timeout_s", 1.25);
    declare_parameter<double>("motion_vx_threshold", 0.03);
    declare_parameter<double>("motion_vy_threshold", 0.03);
    declare_parameter<double>("motion_wz_threshold", 0.10);

    declare_parameter<std::string>("default_action", "BACKUP_THEN_LEFT");

    declare_parameter<double>("backup_vx", -0.06);
    declare_parameter<double>("rotate_wz", 0.25);
    declare_parameter<double>("backup_duration_s", 1.0);
    declare_parameter<double>("rotate_duration_s", 1.0);
    declare_parameter<double>("stop_hold_s", 0.20);
    declare_parameter<double>("max_recovery_duration_s", 8.0);

    declare_parameter<double>("max_vx", 0.10);
    declare_parameter<double>("max_vy", 0.10);
    declare_parameter<double>("max_wz", 0.35);

    declare_parameter<bool>("stop_before_motion", true);
    declare_parameter<bool>("stop_after_motion", true);
    declare_parameter<bool>("allow_rotate_left", true);
    declare_parameter<bool>("allow_rotate_right", true);

    declare_parameter<std::string>("stuck_detected_topic", topics::STUCK_DETECTED);
    declare_parameter<std::string>("recovery_request_topic", topics::RECOVERY_REQUEST);
    declare_parameter<std::string>("recovery_active_topic", topics::RECOVERY_ACTIVE);
    declare_parameter<std::string>("safety_stop_topic", topics::SAFETY_STOP);
    declare_parameter<std::string>("left_range_topic", topics::RANGE_LEFT);
    declare_parameter<std::string>("right_range_topic", topics::RANGE_RIGHT);
    declare_parameter<std::string>("manual_motion_topic", topics::CMD_VEL_MANUAL);
    declare_parameter<std::string>("auto_motion_topic", topics::CMD_VEL_AUTO);
    declare_parameter<std::string>("nav_motion_topic", topics::CMD_VEL_NAV);
    declare_parameter<std::string>("cmd_vel_recovery_topic", topics::CMD_VEL_RECOVERY);
    declare_parameter<std::string>("recovery_state_topic", "/savo_control/recovery_state");
    declare_parameter<std::string>("recovery_status_topic", topics::RECOVERY_STATUS);
  }

  void load_parameters()
  {
    publish_hz_ = positive_param("publish_hz", 20.0);
    input_timeout_s_ = nonnegative_param("input_timeout_s", 0.50);

    enabled_ = get_parameter("enabled").as_bool();
    auto_start_on_stuck_ = get_parameter("auto_start_on_stuck").as_bool();
    auto_start_on_safety_stop_ = get_parameter("auto_start_on_safety_stop").as_bool();
    manual_request_starts_recovery_ =
      get_parameter("manual_request_starts_recovery").as_bool();
    safety_stop_blocks_recovery_ = get_parameter("safety_stop_blocks_recovery").as_bool();
    publish_zero_when_inactive_ = get_parameter("publish_zero_when_inactive").as_bool();

    dynamic_action_enabled_ = get_parameter("dynamic_action_enabled").as_bool();
    prefer_larger_free_side_ = get_parameter("prefer_larger_free_side").as_bool();
    side_block_m_ = nonnegative_param("side_block_m", 0.12);
    side_clear_m_ = nonnegative_param("side_clear_m", 0.20);
    range_timeout_s_ = nonnegative_param("range_timeout_s", 0.60);

    safety_stop_requires_recent_motion_ =
      get_parameter("safety_stop_requires_recent_motion").as_bool();
    recent_motion_timeout_s_ = nonnegative_param("recent_motion_timeout_s", 1.25);
    motion_vx_threshold_ = nonnegative_param("motion_vx_threshold", 0.03);
    motion_vy_threshold_ = nonnegative_param("motion_vy_threshold", 0.03);
    motion_wz_threshold_ = nonnegative_param("motion_wz_threshold", 0.10);

    if (side_clear_m_ < side_block_m_) {
      std::swap(side_clear_m_, side_block_m_);
    }

    default_action_text_ = get_parameter("default_action").as_string();

    backup_vx_ = get_parameter("backup_vx").as_double();
    rotate_wz_ = get_parameter("rotate_wz").as_double();

    backup_duration_s_ = nonnegative_param("backup_duration_s", 1.0);
    rotate_duration_s_ = nonnegative_param("rotate_duration_s", 1.0);
    stop_hold_s_ = nonnegative_param("stop_hold_s", 0.20);
    max_recovery_duration_s_ = positive_param("max_recovery_duration_s", 8.0);

    max_vx_ = nonnegative_param("max_vx", 0.10);
    max_vy_ = nonnegative_param("max_vy", 0.10);
    max_wz_ = nonnegative_param("max_wz", 0.35);

    stop_before_motion_ = get_parameter("stop_before_motion").as_bool();
    stop_after_motion_ = get_parameter("stop_after_motion").as_bool();
    allow_rotate_left_ = get_parameter("allow_rotate_left").as_bool();
    allow_rotate_right_ = get_parameter("allow_rotate_right").as_bool();

    stuck_detected_topic_ = get_parameter("stuck_detected_topic").as_string();
    recovery_request_topic_ = get_parameter("recovery_request_topic").as_string();
    recovery_active_topic_ = get_parameter("recovery_active_topic").as_string();
    safety_stop_topic_ = get_parameter("safety_stop_topic").as_string();
    left_range_topic_ = get_parameter("left_range_topic").as_string();
    right_range_topic_ = get_parameter("right_range_topic").as_string();
    manual_motion_topic_ = get_parameter("manual_motion_topic").as_string();
    auto_motion_topic_ = get_parameter("auto_motion_topic").as_string();
    nav_motion_topic_ = get_parameter("nav_motion_topic").as_string();
    cmd_vel_recovery_topic_ = get_parameter("cmd_vel_recovery_topic").as_string();
    recovery_state_topic_ = get_parameter("recovery_state_topic").as_string();
    recovery_status_topic_ = get_parameter("recovery_status_topic").as_string();
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

    config.default_action = parse_recovery_action(
      default_action_text_,
      RecoveryAction::BACKUP_THEN_LEFT);

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

  void on_timer()
  {
    const double now_s = now_seconds(*this);

    maybe_start_recovery(now_s);

    const bool blocked = safety_stop_blocks_recovery_ && safety_stop_active(now_s);
    const RecoveryManagerResult result = manager_.update_blocked(now_s, blocked);

    publish_result(result, now_s);
  }

  void maybe_start_recovery(const double now_s)
  {
    if (manager_.active()) {
      return;
    }

    if (
      auto_start_on_safety_stop_ &&
      safety_stop_active(now_s) &&
      safety_stop_recovery_allowed(now_s))
    {
      manager_.request(
        now_s,
        RecoveryTrigger::SAFETY_STOP,
        choose_recovery_action(now_s));
      return;
    }

    if (auto_start_on_stuck_ && stuck_active(now_s)) {
      manager_.request(
        now_s,
        RecoveryTrigger::STUCK_DETECTED,
        choose_recovery_action(now_s));
      return;
    }

    if (manual_request_starts_recovery_ && recovery_request_active(now_s)) {
      manager_.request(
        now_s,
        RecoveryTrigger::MANUAL_REQUEST,
        choose_recovery_action(now_s));
    }
  }

  bool stuck_active(const double now_s) const
  {
    if (!stuck_seen_) {
      return false;
    }

    if (input_timeout_s_ > 0.0 && (now_s - stuck_stamp_s_) > input_timeout_s_) {
      return false;
    }

    return stuck_detected_;
  }

  void observe_motion_command(const geometry_msgs::msg::Twist & msg)
  {
    if (
      twist_has_motion(
        msg,
        motion_vx_threshold_,
        motion_vy_threshold_,
        motion_wz_threshold_))
    {
      command_motion_seen_ = true;
      command_motion_stamp_s_ = now_seconds(*this);
    }
  }

  bool recent_command_motion_active(const double now_s) const
  {
    if (!command_motion_seen_) {
      return false;
    }

    if (recent_motion_timeout_s_ <= 0.0) {
      return true;
    }

    return (now_s - command_motion_stamp_s_) <= recent_motion_timeout_s_;
  }

  bool safety_stop_recovery_allowed(const double now_s) const
  {
    if (!safety_stop_requires_recent_motion_) {
      return true;
    }

    return recent_command_motion_active(now_s);
  }

  bool recovery_request_active(const double now_s) const
  {
    if (!recovery_request_seen_) {
      return false;
    }

    if (input_timeout_s_ > 0.0 && (now_s - recovery_request_stamp_s_) > input_timeout_s_) {
      return false;
    }

    return recovery_request_;
  }

  bool range_fresh(
    const bool seen,
    const double stamp_s,
    const double now_s) const
  {
    if (!seen) {
      return false;
    }

    if (range_timeout_s_ <= 0.0) {
      return true;
    }

    return (now_s - stamp_s) <= range_timeout_s_;
  }

  RecoveryAction choose_recovery_action(const double now_s) const
  {
    if (!dynamic_action_enabled_) {
      return parse_recovery_action(default_action_text_, RecoveryAction::BACKUP_THEN_LEFT);
    }

    const bool left_fresh = range_fresh(left_range_seen_, left_range_stamp_s_, now_s);
    const bool right_fresh = range_fresh(right_range_seen_, right_range_stamp_s_, now_s);

    const bool left_blocked = left_fresh && left_range_m_ <= side_block_m_;
    const bool right_blocked = right_fresh && right_range_m_ <= side_block_m_;

    const bool left_free = left_fresh && left_range_m_ >= side_clear_m_;
    const bool right_free = right_fresh && right_range_m_ >= side_clear_m_;

    if (left_blocked && right_blocked) {
      return RecoveryAction::BACKUP;
    }

    if (left_blocked && !right_blocked) {
      return RecoveryAction::BACKUP_THEN_RIGHT;
    }

    if (right_blocked && !left_blocked) {
      return RecoveryAction::BACKUP_THEN_LEFT;
    }

    if (left_free && !right_free) {
      return RecoveryAction::BACKUP_THEN_LEFT;
    }

    if (right_free && !left_free) {
      return RecoveryAction::BACKUP_THEN_RIGHT;
    }

    if (left_free && right_free) {
      if (prefer_larger_free_side_ && right_range_m_ > left_range_m_) {
        return RecoveryAction::BACKUP_THEN_RIGHT;
      }

      return RecoveryAction::BACKUP_THEN_LEFT;
    }

    return RecoveryAction::BACKUP;
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

  void publish_result(const RecoveryManagerResult & result, const double now_s)
  {
    geometry_msgs::msg::Twist cmd = zero_twist();

    if (result.active) {
      cmd = command_to_twist(result.command);
    } else if (!publish_zero_when_inactive_ && !result.finished) {
      cmd = command_to_twist(result.command);
    }

    cmd_pub_->publish(cmd);
    active_pub_->publish(bool_msg(result.active));
    state_pub_->publish(string_msg(to_string(result.state)));

    std::ostringstream ss;
    ss << "state=" << to_string(result.state)
       << "; trigger=" << to_string(result.trigger)
       << "; action=" << to_string(result.action)
       << "; active=" << bool_text(result.active)
       << "; finished=" << bool_text(result.finished)
       << "; request_active=" << bool_text(result.recovery_request_active)
       << "; safety_stop=" << bool_text(safety_stop_active(now_s))
       << "; recent_command_motion=" << bool_text(recent_command_motion_active(now_s))
       << "; stuck=" << bool_text(stuck_active(now_s))
       << "; reason=" << result.reason
       << "; elapsed_s=" << result.elapsed_s
       << "; phase_elapsed_s=" << result.phase_elapsed_s
       << "; vx=" << cmd.linear.x
       << "; vy=" << cmd.linear.y
       << "; wz=" << cmd.angular.z
       << "; manager=[" << manager_.status_string() << "]";

    status_pub_->publish(string_msg(ss.str()));
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
  bool auto_start_on_stuck_{true};
  bool auto_start_on_safety_stop_{true};
  bool manual_request_starts_recovery_{true};
  bool safety_stop_blocks_recovery_{false};
  bool publish_zero_when_inactive_{true};

  bool dynamic_action_enabled_{true};
  bool prefer_larger_free_side_{true};
  double side_block_m_{0.12};
  double side_clear_m_{0.20};
  double range_timeout_s_{0.60};

  bool safety_stop_requires_recent_motion_{true};
  double recent_motion_timeout_s_{1.25};
  double motion_vx_threshold_{0.03};
  double motion_vy_threshold_{0.03};
  double motion_wz_threshold_{0.10};

  std::string default_action_text_{"BACKUP_THEN_LEFT"};

  double backup_vx_{-0.06};
  double rotate_wz_{0.25};

  double backup_duration_s_{1.0};
  double rotate_duration_s_{1.0};
  double stop_hold_s_{0.20};
  double max_recovery_duration_s_{8.0};

  double max_vx_{0.10};
  double max_vy_{0.10};
  double max_wz_{0.35};

  bool stop_before_motion_{true};
  bool stop_after_motion_{true};
  bool allow_rotate_left_{true};
  bool allow_rotate_right_{true};

  std::string stuck_detected_topic_{topics::STUCK_DETECTED};
  std::string recovery_request_topic_{topics::RECOVERY_REQUEST};
  std::string recovery_active_topic_{topics::RECOVERY_ACTIVE};
  std::string safety_stop_topic_{topics::SAFETY_STOP};
  std::string left_range_topic_{topics::RANGE_LEFT};
  std::string right_range_topic_{topics::RANGE_RIGHT};
  std::string manual_motion_topic_{topics::CMD_VEL_MANUAL};
  std::string auto_motion_topic_{topics::CMD_VEL_AUTO};
  std::string nav_motion_topic_{topics::CMD_VEL_NAV};
  std::string cmd_vel_recovery_topic_{topics::CMD_VEL_RECOVERY};
  std::string recovery_state_topic_{"/savo_control/recovery_state"};
  std::string recovery_status_topic_{topics::RECOVERY_STATUS};

  bool stuck_detected_{false};
  bool stuck_seen_{false};
  double stuck_stamp_s_{0.0};

  bool recovery_request_{false};
  bool recovery_request_seen_{false};
  double recovery_request_stamp_s_{0.0};

  bool safety_stop_{false};
  bool safety_stop_seen_{false};
  double safety_stop_stamp_s_{0.0};

  double left_range_m_{0.0};
  bool left_range_seen_{false};
  double left_range_stamp_s_{0.0};

  double right_range_m_{0.0};
  bool right_range_seen_{false};
  double right_range_stamp_s_{0.0};

  bool command_motion_seen_{false};
  double command_motion_stamp_s_{0.0};

  RecoveryManager manager_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stuck_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recovery_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_range_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_range_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_motion_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr auto_motion_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_motion_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<savo_control::RecoveryManagerNode>());
  rclcpp::shutdown();
  return 0;
}
