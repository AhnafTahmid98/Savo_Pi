#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_control/control_mode_manager.hpp"
#include "savo_control/topic_names.hpp"

namespace
{

using namespace std::chrono_literals;

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

}  // namespace

namespace savo_control
{

class ControlModeManagerNode : public rclcpp::Node
{
public:
  ControlModeManagerNode()
  : Node("control_mode_manager_node")
  {
    declare_parameters();
    load_parameters();

    mode_cmd_sub_ = create_subscription<std_msgs::msg::String>(
      mode_cmd_topic_,
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        handle_mode_command(msg->data);
      });

    safety_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      safety_stop_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        manager_.set_safety_stop(msg->data);
        publish_now();
      });

    external_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      external_stop_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        manager_.set_external_stop(msg->data);
        publish_now();
      });

    recovery_active_sub_ = create_subscription<std_msgs::msg::Bool>(
      recovery_active_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        manager_.set_recovery_active(msg->data, now_seconds(*this));
        publish_now();
      });

    manual_override_sub_ = create_subscription<std_msgs::msg::Bool>(
      manual_override_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        manager_.set_manual_override(msg->data, now_seconds(*this));
        publish_now();
      });

    mode_state_pub_ = create_publisher<std_msgs::msg::String>(
      mode_state_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    mode_reason_pub_ = create_publisher<std_msgs::msg::String>(mode_reason_topic_, 10);
    control_status_pub_ = create_publisher<std_msgs::msg::String>(control_status_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_hz_),
      [this]() {
        on_timer();
      });

    manager_.reset(now_seconds(*this));
    publish_now();

    RCLCPP_INFO(
      get_logger(),
      "control_mode_manager_node started | cmd=%s | state=%s | status=%s",
      mode_cmd_topic_.c_str(),
      mode_state_topic_.c_str(),
      control_status_topic_.c_str());
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("startup_mode", "STOP");
    declare_parameter<std::string>("fallback_mode", "STOP");

    declare_parameter<double>("request_timeout_s", 1.0);
    declare_parameter<double>("recovery_latch_timeout_s", 2.0);
    declare_parameter<double>("manual_override_timeout_s", 1.5);

    declare_parameter<bool>("allow_manual", true);
    declare_parameter<bool>("allow_auto", true);
    declare_parameter<bool>("allow_nav", true);
    declare_parameter<bool>("allow_recovery", true);

    declare_parameter<bool>("safety_stop_forces_stop", true);
    declare_parameter<bool>("external_stop_forces_stop", true);
    declare_parameter<bool>("recovery_active_forces_recovery", true);

    declare_parameter<double>("publish_hz", 10.0);

    declare_parameter<std::string>("mode_cmd_topic", topics::CONTROL_MODE_CMD);
    declare_parameter<std::string>("mode_state_topic", topics::CONTROL_MODE_STATE);
    declare_parameter<std::string>("mode_reason_topic", topics::CONTROL_MODE_REASON);
    declare_parameter<std::string>("control_status_topic", topics::CONTROL_STATUS);

    declare_parameter<std::string>("safety_stop_topic", topics::SAFETY_STOP);
    declare_parameter<std::string>("external_stop_topic", "/savo_control/external_stop");
    declare_parameter<std::string>("recovery_active_topic", topics::RECOVERY_ACTIVE);
    declare_parameter<std::string>("manual_override_topic", "/savo_control/manual_override");
  }

  void load_parameters()
  {
    ControlModeManagerConfig config;

    config.startup_mode = parse_control_mode(
      get_parameter("startup_mode").as_string(),
      ControlMode::STOP);
    config.fallback_mode = parse_control_mode(
      get_parameter("fallback_mode").as_string(),
      ControlMode::STOP);

    config.request_timeout_s = get_parameter("request_timeout_s").as_double();
    config.recovery_latch_timeout_s = get_parameter("recovery_latch_timeout_s").as_double();
    config.manual_override_timeout_s = get_parameter("manual_override_timeout_s").as_double();

    config.allow_manual = get_parameter("allow_manual").as_bool();
    config.allow_auto = get_parameter("allow_auto").as_bool();
    config.allow_nav = get_parameter("allow_nav").as_bool();
    config.allow_recovery = get_parameter("allow_recovery").as_bool();

    config.safety_stop_forces_stop = get_parameter("safety_stop_forces_stop").as_bool();
    config.external_stop_forces_stop = get_parameter("external_stop_forces_stop").as_bool();
    config.recovery_active_forces_recovery =
      get_parameter("recovery_active_forces_recovery").as_bool();

    manager_.set_config(config);

    publish_hz_ = get_parameter("publish_hz").as_double();
    if (!std::isfinite(publish_hz_) || publish_hz_ <= 0.0) {
      publish_hz_ = 10.0;
    }

    mode_cmd_topic_ = get_parameter("mode_cmd_topic").as_string();
    mode_state_topic_ = get_parameter("mode_state_topic").as_string();
    mode_reason_topic_ = get_parameter("mode_reason_topic").as_string();
    control_status_topic_ = get_parameter("control_status_topic").as_string();

    safety_stop_topic_ = get_parameter("safety_stop_topic").as_string();
    external_stop_topic_ = get_parameter("external_stop_topic").as_string();
    recovery_active_topic_ = get_parameter("recovery_active_topic").as_string();
    manual_override_topic_ = get_parameter("manual_override_topic").as_string();
  }

  void handle_mode_command(const std::string & mode_text)
  {
    const double now_s = now_seconds(*this);
    const bool accepted = manager_.request_mode(mode_text, now_s, "mode_cmd");

    if (!accepted) {
      RCLCPP_WARN(
        get_logger(),
        "Rejected control mode command: '%s'",
        mode_text.c_str());
    }

    publish_now();
  }

  void on_timer()
  {
    manager_.update(now_seconds(*this));
    publish_now();
  }

  void publish_now()
  {
    const auto & state = manager_.state();

    mode_state_pub_->publish(string_msg(to_string(state.mode)));
    mode_reason_pub_->publish(string_msg(to_string(state.reason)));
    control_status_pub_->publish(string_msg(manager_.status_string()));
  }

  ControlModeManager manager_{};

  double publish_hz_{10.0};

  std::string mode_cmd_topic_{topics::CONTROL_MODE_CMD};
  std::string mode_state_topic_{topics::CONTROL_MODE_STATE};
  std::string mode_reason_topic_{topics::CONTROL_MODE_REASON};
  std::string control_status_topic_{topics::CONTROL_STATUS};

  std::string safety_stop_topic_{topics::SAFETY_STOP};
  std::string external_stop_topic_{"/savo_control/external_stop"};
  std::string recovery_active_topic_{topics::RECOVERY_ACTIVE};
  std::string manual_override_topic_{"/savo_control/manual_override"};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr external_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recovery_active_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_override_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_reason_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_status_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<savo_control::ControlModeManagerNode>());
  rclcpp::shutdown();
  return 0;
}
