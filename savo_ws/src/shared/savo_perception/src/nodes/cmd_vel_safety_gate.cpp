#include "savo_perception/cmd_vel_safety_gate.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>

namespace savo_perception
{
namespace
{

double safe_rate_hz(const double value, const double fallback)
{
  if (!std::isfinite(value) || value <= 0.0) {
    return fallback;
  }

  return value;
}

std::string json_escape(const std::string & value)
{
  std::ostringstream out;

  for (const auto ch : value) {
    switch (ch) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }

  return out.str();
}

}  // namespace

CmdVelSafetyGateNode::CmdVelSafetyGateNode(const rclcpp::NodeOptions & options)
: rclcpp::Node(constants::kCmdVelSafetyGateNodeName, options)
{
  declare_parameters();
  load_parameters();
  setup_interfaces();
}

void CmdVelSafetyGateNode::declare_parameters()
{
  declare_parameter<std::string>("cmd_vel_topic", topics::kCmdVel);
  declare_parameter<std::string>("cmd_vel_safe_topic", topics::kCmdVelSafe);

  declare_parameter<std::string>("safety_stop_topic", topics::kSafetyStop);
  declare_parameter<std::string>("slowdown_topic", topics::kSafetySlowdownFactor);
  declare_parameter<std::string>("safety_state_topic", topics::kSafetyState);

  declare_parameter<double>("loop_hz", constants::kCmdVelGateLoopHzDefault);
  declare_parameter<double>("stale_timeout_s", constants::kSensorStaleTimeoutSDefault);
  declare_parameter<double>("cmd_timeout_s", constants::kCmdTimeoutSDefault);

  declare_parameter<bool>("fail_safe_on_stale", true);
  declare_parameter<bool>("publish_zero_on_stop", true);
  declare_parameter<bool>("publish_zero_on_stale", true);

  declare_parameter<bool>("apply_slowdown_to_linear", true);
  declare_parameter<bool>("apply_slowdown_to_angular", true);

  declare_parameter<double>("slowdown_min", 0.0);
  declare_parameter<double>("slowdown_max", 1.0);

  declare_parameter<double>("max_linear_x_mps", constants::kMaxLinearXMpsDefault);
  declare_parameter<double>("max_linear_y_mps", constants::kMaxLinearYMpsDefault);
  declare_parameter<double>("max_angular_z_radps", constants::kMaxAngularZRadpsDefault);

  declare_parameter<bool>("allow_recovery_when_stopped", false);
  declare_parameter<std::string>("recovery_command_topic", topics::kRecoveryCmdVel);
  declare_parameter<std::string>("recovery_cmd_vel_safe_topic", topics::kCmdVelSafe);

  declare_parameter<bool>("publish_gate_state", true);
  declare_parameter<std::string>("gate_state_topic", topics::kCmdVelGateState);

  declare_parameter<bool>("startup_fail_is_fatal", constants::kStartupFailIsFatalDefault);
}

void CmdVelSafetyGateNode::load_parameters()
{
  cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
  cmd_vel_safe_topic_ = get_parameter("cmd_vel_safe_topic").as_string();

  safety_stop_topic_ = get_parameter("safety_stop_topic").as_string();
  slowdown_topic_ = get_parameter("slowdown_topic").as_string();
  safety_state_topic_ = get_parameter("safety_state_topic").as_string();

  loop_hz_ = safe_rate_hz(
    get_parameter("loop_hz").as_double(),
    constants::kCmdVelGateLoopHzDefault);

  gate_config_.stale_timeout_s = std::max(
    0.01,
    get_parameter("stale_timeout_s").as_double());

  gate_config_.cmd_timeout_s = std::max(
    0.01,
    get_parameter("cmd_timeout_s").as_double());

  gate_config_.fail_safe_on_stale = get_parameter("fail_safe_on_stale").as_bool();
  gate_config_.publish_zero_on_stop = get_parameter("publish_zero_on_stop").as_bool();
  gate_config_.publish_zero_on_stale = get_parameter("publish_zero_on_stale").as_bool();

  gate_config_.apply_slowdown_to_linear =
    get_parameter("apply_slowdown_to_linear").as_bool();

  gate_config_.apply_slowdown_to_angular =
    get_parameter("apply_slowdown_to_angular").as_bool();

  gate_config_.slowdown_min = get_parameter("slowdown_min").as_double();
  gate_config_.slowdown_max = get_parameter("slowdown_max").as_double();

  if (gate_config_.slowdown_max < gate_config_.slowdown_min) {
    std::swap(gate_config_.slowdown_max, gate_config_.slowdown_min);
  }

  gate_config_.max_linear_x_mps = std::max(
    0.0,
    get_parameter("max_linear_x_mps").as_double());

  gate_config_.max_linear_y_mps = std::max(
    0.0,
    get_parameter("max_linear_y_mps").as_double());

  gate_config_.max_angular_z_radps = std::max(
    0.0,
    get_parameter("max_angular_z_radps").as_double());

  gate_config_.allow_recovery_when_stopped =
    get_parameter("allow_recovery_when_stopped").as_bool();

  recovery_command_topic_ = get_parameter("recovery_command_topic").as_string();
  recovery_cmd_vel_safe_topic_ = get_parameter("recovery_cmd_vel_safe_topic").as_string();

  publish_gate_state_ = get_parameter("publish_gate_state").as_bool();
  gate_state_topic_ = get_parameter("gate_state_topic").as_string();

  latest_safety_stop_ = true;
  latest_slowdown_factor_ = 0.0;
}

void CmdVelSafetyGateNode::setup_interfaces()
{
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_,
    rclcpp::QoS(10),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      on_cmd_vel(msg);
    });

  recovery_cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    recovery_command_topic_,
    rclcpp::QoS(10),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      on_recovery_cmd_vel(msg);
    });

  safety_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    safety_stop_topic_,
    rclcpp::QoS(10).reliable(),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      on_safety_stop(msg);
    });

  slowdown_sub_ = create_subscription<std_msgs::msg::Float32>(
    slowdown_topic_,
    rclcpp::QoS(10).reliable(),
    [this](const std_msgs::msg::Float32::SharedPtr msg) {
      on_slowdown(msg);
    });

  safety_state_sub_ = create_subscription<std_msgs::msg::String>(
    safety_state_topic_,
    rclcpp::QoS(10).reliable(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      on_safety_state(msg);
    });

  cmd_vel_safe_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_safe_topic_,
    rclcpp::QoS(10));

  gate_state_pub_ = create_publisher<std_msgs::msg::String>(
    gate_state_topic_,
    rclcpp::QoS(10).reliable());

  const auto period = std::chrono::duration<double>(1.0 / loop_hz_);

  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {
      on_timer();
    });

  RCLCPP_INFO(
    get_logger(),
    "CmdVel safety gate started: %s -> %s loop=%.2fHz",
    cmd_vel_topic_.c_str(),
    cmd_vel_safe_topic_.c_str(),
    loop_hz_);
}

void CmdVelSafetyGateNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_command_ = twist_to_command(*msg, "cmd_vel");
  has_command_ = true;
}

void CmdVelSafetyGateNode::on_safety_stop(const std_msgs::msg::Bool::SharedPtr msg)
{
  latest_safety_stop_ = msg->data;
  safety_stop_stamp_ = std::chrono::steady_clock::now();
}

void CmdVelSafetyGateNode::on_slowdown(const std_msgs::msg::Float32::SharedPtr msg)
{
  const auto value = static_cast<double>(msg->data);

  if (std::isfinite(value)) {
    latest_slowdown_factor_ = clamp_slowdown(
      value,
      gate_config_.slowdown_min,
      gate_config_.slowdown_max);
  } else {
    latest_slowdown_factor_ = 0.0;
  }

  slowdown_stamp_ = std::chrono::steady_clock::now();
}

void CmdVelSafetyGateNode::on_safety_state(const std_msgs::msg::String::SharedPtr msg)
{
  latest_safety_state_json_ = msg->data;
  safety_state_stamp_ = std::chrono::steady_clock::now();
}

void CmdVelSafetyGateNode::on_recovery_cmd_vel(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_recovery_command_ = twist_to_command(*msg, "recovery_cmd_vel");
  has_recovery_command_ = true;
}

void CmdVelSafetyGateNode::on_timer()
{
  const auto output = evaluate_gate();

  publish_output(output);

  if (publish_gate_state_) {
    publish_gate_state(output);
  }
}

SafetyGateInput CmdVelSafetyGateNode::make_gate_input() const
{
  SafetyGateInput input;

  input.command = latest_command_;
  input.has_command = has_command_;

  input.safety_stop = latest_safety_stop_;
  input.safety_stop_stale = stamp_is_stale(
    safety_stop_stamp_,
    gate_config_.stale_timeout_s);

  input.slowdown_stale = stamp_is_stale(
    slowdown_stamp_,
    gate_config_.stale_timeout_s);

  if (!input.slowdown_stale) {
    input.slowdown_factor = latest_slowdown_factor_;
  }

  input.has_safety_decision = false;

  input.recovery_requested = gate_config_.allow_recovery_when_stopped &&
    has_recovery_command_ &&
    !latest_recovery_command_.stale(gate_config_.cmd_timeout_s);

  if (input.recovery_requested) {
    input.recovery_command = latest_recovery_command_;
  }

  return input;
}

SafetyGateOutput CmdVelSafetyGateNode::evaluate_gate() const
{
  return gate_velocity_command(make_gate_input(), gate_config_);
}

VelocityCommand CmdVelSafetyGateNode::twist_to_command(
  const geometry_msgs::msg::Twist & msg,
  const std::string & source) const
{
  VelocityCommand command;
  command.linear_x_mps = msg.linear.x;
  command.linear_y_mps = msg.linear.y;
  command.angular_z_radps = msg.angular.z;
  command.stamp = std::chrono::steady_clock::now();
  command.valid = finite_velocity_command(command);
  command.source = source;
  return command;
}

geometry_msgs::msg::Twist CmdVelSafetyGateNode::command_to_twist(
  const VelocityCommand & command) const
{
  geometry_msgs::msg::Twist msg;

  msg.linear.x = command.linear_x_mps;
  msg.linear.y = command.linear_y_mps;
  msg.linear.z = 0.0;

  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = command.angular_z_radps;

  return msg;
}

void CmdVelSafetyGateNode::publish_output(const SafetyGateOutput & output)
{
  if (!cmd_vel_safe_pub_) {
    return;
  }

  if (output.stopped && !gate_config_.publish_zero_on_stop) {
    return;
  }

  if (output.stale && !gate_config_.publish_zero_on_stale) {
    return;
  }

  cmd_vel_safe_pub_->publish(command_to_twist(output.output_command));
}

void CmdVelSafetyGateNode::publish_gate_state(const SafetyGateOutput & output)
{
  if (!gate_state_pub_) {
    return;
  }

  std_msgs::msg::String msg;

  std::ostringstream out;
  out << "{";
  out << "\"allowed\":" << (output.allowed ? "true" : "false") << ",";
  out << "\"stopped\":" << (output.stopped ? "true" : "false") << ",";
  out << "\"stale\":" << (output.stale ? "true" : "false") << ",";
  out << "\"applied_slowdown\":" << output.applied_slowdown << ",";
  out << "\"reason\":\"" << json_escape(output.reason) << "\",";
  out << "\"cmd_vel_topic\":\"" << json_escape(cmd_vel_topic_) << "\",";
  out << "\"cmd_vel_safe_topic\":\"" << json_escape(cmd_vel_safe_topic_) << "\",";
  out << "\"latest_safety_stop\":" << (latest_safety_stop_ ? "true" : "false") << ",";
  out << "\"has_command\":" << (has_command_ ? "true" : "false") << ",";
  out << "\"has_recovery_command\":" << (has_recovery_command_ ? "true" : "false");
  out << "}";

  msg.data = out.str();
  gate_state_pub_->publish(msg);
}

bool CmdVelSafetyGateNode::stamp_is_stale(
  const std::optional<std::chrono::steady_clock::time_point> & stamp,
  const double timeout_s) const
{
  if (!stamp.has_value()) {
    return true;
  }

  const auto age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - *stamp).count();

  return age > timeout_s;
}

}  // namespace savo_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<savo_perception::CmdVelSafetyGateNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}