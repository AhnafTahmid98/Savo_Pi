#include "savo_base/base_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

namespace savo_base
{

BaseDriverNode::BaseDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("base_driver_node", options)
{
  declare_parameters();
  load_parameters();
  configure_runtime();

  RCLCPP_INFO(
    get_logger(),
    "base_driver_node C++ started | cmd_topic=%s loop_hz=%.1f watchdog_timeout=%.2fs "
    "max_duty=%d backend=%s dryrun=%s",
    config_.cmd_topic.c_str(),
    config_.loop_hz,
    config_.watchdog_timeout_s,
    config_.motor.max_duty,
    config_.board.backend.c_str(),
    bool_text(config_.board.dryrun).c_str());
}

BaseDriverNode::~BaseDriverNode()
{
  if (board_) {
    try {
      board_->stop();
      board_->close();
    } catch (...) {
    }
  }
}

void BaseDriverNode::declare_parameters()
{
  declare_parameter<std::string>("robot_name", config_.robot_name);
  declare_parameter<std::string>("cmd_topic", config_.cmd_topic);
  declare_parameter<std::string>("safety_stop_topic", config_.safety_stop_topic);
  declare_parameter<std::string>("slowdown_topic", config_.slowdown_topic);
  declare_parameter<std::string>("base_state_topic", config_.base_state_topic);

  declare_parameter<double>("loop_hz", config_.loop_hz);
  declare_parameter<double>("watchdog_timeout_s", config_.watchdog_timeout_s);
  declare_parameter<double>("turn_gain", config_.turn_gain);

  declare_parameter<double>("vx_limit", config_.velocity.vx);
  declare_parameter<double>("vy_limit", config_.velocity.vy);
  declare_parameter<double>("wz_limit", config_.velocity.wz);

  declare_parameter<int>("forward_sign", config_.signs.forward);
  declare_parameter<int>("strafe_sign", config_.signs.strafe);
  declare_parameter<int>("rotate_sign", config_.signs.rotate);

  declare_parameter<int>("max_duty", config_.motor.max_duty);
  declare_parameter<bool>(
    "enable_breakaway_compensation",
    config_.motor.enable_breakaway_compensation);
  declare_parameter<int>("min_motion_duty", config_.motor.min_motion_duty);
  declare_parameter<int>("breakaway_trigger_duty", config_.motor.breakaway_trigger_duty);

  declare_parameter<std::string>("board_backend", config_.board.backend);
  declare_parameter<std::string>("board_name", config_.board.name);
  declare_parameter<bool>("dryrun", config_.board.dryrun);
  declare_parameter<bool>("debug", config_.board.debug);
  declare_parameter<int>("i2c_bus", config_.board.i2c_bus);
  declare_parameter<int>("pca9685_addr", config_.board.address);
  declare_parameter<double>("pwm_freq_hz", config_.board.pwm_freq_hz);
  declare_parameter<int>("quench_ms", config_.board.quench_ms);

  declare_parameter<bool>("invert_fl", config_.inverts.fl);
  declare_parameter<bool>("invert_rl", config_.inverts.rl);
  declare_parameter<bool>("invert_fr", config_.inverts.fr);
  declare_parameter<bool>("invert_rr", config_.inverts.rr);

  declare_parameter<int>("fl_channel_a", config_.channels.fl[0]);
  declare_parameter<int>("fl_channel_b", config_.channels.fl[1]);
  declare_parameter<int>("rl_channel_a", config_.channels.rl[0]);
  declare_parameter<int>("rl_channel_b", config_.channels.rl[1]);
  declare_parameter<int>("fr_channel_a", config_.channels.fr[0]);
  declare_parameter<int>("fr_channel_b", config_.channels.fr[1]);
  declare_parameter<int>("rr_channel_a", config_.channels.rr[0]);
  declare_parameter<int>("rr_channel_b", config_.channels.rr[1]);
}

void BaseDriverNode::load_parameters()
{
  config_.robot_name = get_parameter("robot_name").as_string();
  config_.cmd_topic = get_parameter("cmd_topic").as_string();
  config_.safety_stop_topic = get_parameter("safety_stop_topic").as_string();
  config_.slowdown_topic = get_parameter("slowdown_topic").as_string();
  config_.base_state_topic = get_parameter("base_state_topic").as_string();

  config_.loop_hz = clamp_double(get_parameter("loop_hz").as_double(), 1.0, 200.0);
  config_.watchdog_timeout_s =
    clamp_double(get_parameter("watchdog_timeout_s").as_double(), 0.05, 5.0);
  config_.turn_gain = clamp_double(get_parameter("turn_gain").as_double(), 0.0, 5.0);

  config_.velocity.vx = clamp_double(get_parameter("vx_limit").as_double(), 0.01, 5.0);
  config_.velocity.vy = clamp_double(get_parameter("vy_limit").as_double(), 0.01, 5.0);
  config_.velocity.wz = clamp_double(get_parameter("wz_limit").as_double(), 0.01, 10.0);

  config_.signs.forward = clamp_int(get_parameter("forward_sign").as_int(), -1, 1);
  config_.signs.strafe = clamp_int(get_parameter("strafe_sign").as_int(), -1, 1);
  config_.signs.rotate = clamp_int(get_parameter("rotate_sign").as_int(), -1, 1);

  config_.motor.max_duty = clamp_int(get_parameter("max_duty").as_int(), 0, 4095);
  config_.motor.enable_breakaway_compensation =
    get_parameter("enable_breakaway_compensation").as_bool();
  config_.motor.min_motion_duty =
    clamp_int(get_parameter("min_motion_duty").as_int(), 0, config_.motor.max_duty);
  config_.motor.breakaway_trigger_duty =
    clamp_int(
    get_parameter("breakaway_trigger_duty").as_int(),
    0,
    config_.motor.min_motion_duty);

  config_.board.backend = get_parameter("board_backend").as_string();
  config_.board.name = get_parameter("board_name").as_string();
  config_.board.dryrun = get_parameter("dryrun").as_bool() || config_.board.backend == "dryrun";
  config_.board.debug = get_parameter("debug").as_bool();
  config_.board.i2c_bus = get_parameter("i2c_bus").as_int();
  config_.board.address = get_parameter("pca9685_addr").as_int();
  config_.board.pwm_freq_hz =
    clamp_double(get_parameter("pwm_freq_hz").as_double(), 24.0, 1526.0);
  config_.board.quench_ms = clamp_int(get_parameter("quench_ms").as_int(), 0, 1000);

  config_.inverts.fl = get_parameter("invert_fl").as_bool();
  config_.inverts.rl = get_parameter("invert_rl").as_bool();
  config_.inverts.fr = get_parameter("invert_fr").as_bool();
  config_.inverts.rr = get_parameter("invert_rr").as_bool();

  config_.channels.fl = {
    clamp_int(get_parameter("fl_channel_a").as_int(), 0, 15),
    clamp_int(get_parameter("fl_channel_b").as_int(), 0, 15)};
  config_.channels.rl = {
    clamp_int(get_parameter("rl_channel_a").as_int(), 0, 15),
    clamp_int(get_parameter("rl_channel_b").as_int(), 0, 15)};
  config_.channels.fr = {
    clamp_int(get_parameter("fr_channel_a").as_int(), 0, 15),
    clamp_int(get_parameter("fr_channel_b").as_int(), 0, 15)};
  config_.channels.rr = {
    clamp_int(get_parameter("rr_channel_a").as_int(), 0, 15),
    clamp_int(get_parameter("rr_channel_b").as_int(), 0, 15)};
}

void BaseDriverNode::configure_runtime()
{
  command_guard_ = std::make_unique<CommandGuard>(config_.velocity);
  watchdog_ = std::make_unique<TimeoutWatchdog>(config_.watchdog_timeout_s);
  safety_ = std::make_unique<BaseSafety>(config_.watchdog_timeout_s);
  mixer_ = std::make_unique<MecanumMixer>(config_);
  board_ = std::make_unique<FreenoveMotorBoard>(
    config_.board,
    config_.channels,
    config_.inverts);

  if (!board_->ping()) {
    throw std::runtime_error("Freenove motor board ping failed");
  }

  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    config_.cmd_topic,
    rclcpp::QoS(10),
    std::bind(&BaseDriverNode::cmd_callback, this, std::placeholders::_1));

  safety_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    config_.safety_stop_topic,
    rclcpp::QoS(10),
    std::bind(&BaseDriverNode::safety_stop_callback, this, std::placeholders::_1));

  slowdown_sub_ = create_subscription<std_msgs::msg::Float32>(
    config_.slowdown_topic,
    rclcpp::QoS(10),
    std::bind(&BaseDriverNode::slowdown_callback, this, std::placeholders::_1));

  state_pub_ = create_publisher<std_msgs::msg::String>(
    config_.base_state_topic,
    rclcpp::QoS(10));

  const auto period = std::chrono::duration<double>(1.0 / config_.loop_hz);
  loop_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&BaseDriverNode::loop_callback, this));
}

void BaseDriverNode::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cmd_ = *msg;
  latest_cmd_time_ = now();
  has_command_ = true;
  ++counters_.command_seq;
}

void BaseDriverNode::safety_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  safety_stop_ = msg->data;
}

void BaseDriverNode::slowdown_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  slowdown_factor_ = BaseSafety::clamp_slowdown(static_cast<double>(msg->data));
}

void BaseDriverNode::loop_callback()
{
  ++counters_.loop_count;

  const double age_s = command_age_s();
  const WatchdogStatus watchdog = watchdog_->evaluate(has_command_, age_s);

  SafetySnapshot snapshot{};
  snapshot.has_command = has_command_;
  snapshot.command_age_s = age_s;
  snapshot.command_stale = watchdog.stale;
  snapshot.safety_stop = safety_stop_;
  snapshot.slowdown_factor = slowdown_factor_;

  SafetyDecision decision = safety_->evaluate(snapshot);
  WheelDuty duty{0, 0, 0, 0};

  try {
    if (decision.force_zero) {
      board_->stop();
      ++counters_.zero_count;

      if (decision.blocked || watchdog.tripped) {
        ++counters_.trip_count;
      }
    } else {
      const GuardedCommand command = command_guard_->apply(
        latest_cmd_.linear.x * decision.slowdown_factor,
        latest_cmd_.linear.y * decision.slowdown_factor,
        latest_cmd_.angular.z * decision.slowdown_factor);

      if (!command.valid) {
        decision.force_zero = true;
        decision.blocked = true;
        decision.reason = "invalid_command";
        board_->stop();
        ++counters_.zero_count;
        ++counters_.trip_count;
      } else {
        duty = mixer_->mix(command.vx, command.vy, command.wz);
        board_->write(duty);
        ++counters_.board_write_count;
      }
    }

    last_board_error_.clear();
  } catch (const std::exception & exc) {
    last_board_error_ = exc.what();
    board_->stop();
    ++counters_.zero_count;
    ++counters_.trip_count;

    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "base_driver_node board error: %s",
      exc.what());
  }

  publish_state(decision, watchdog, duty);
}

void BaseDriverNode::publish_state(
  const SafetyDecision & decision,
  const WatchdogStatus & watchdog,
  const WheelDuty & duty)
{
  BaseRuntimeState state{};
  state.node_name = "base_driver_node";
  state.last_board_error = last_board_error_;
  state.connected = board_ && board_->connected();
  state.safety_stop = safety_stop_;

  state.command.vx = latest_cmd_.linear.x;
  state.command.vy = latest_cmd_.linear.y;
  state.command.wz = latest_cmd_.angular.z;
  state.command.age_s = command_age_s();
  state.command.stale = watchdog.stale;

  state.safety_decision = decision;
  state.watchdog = watchdog;
  state.last_duty = duty;
  state.counters = counters_;

  std_msgs::msg::String msg;
  msg.data = BaseStateJson::make(config_, state);
  state_pub_->publish(msg);
}

double BaseDriverNode::command_age_s() const
{
  if (!has_command_) {
    return 1.0e9;
  }

  return (now() - latest_cmd_time_).seconds();
}

double BaseDriverNode::clamp_double(
  const double value,
  const double min_value,
  const double max_value)
{
  if (!std::isfinite(value)) {
    return min_value;
  }

  return std::clamp(value, min_value, max_value);
}

int BaseDriverNode::clamp_int(
  const int value,
  const int min_value,
  const int max_value)
{
  return std::clamp(value, min_value, max_value);
}

std::string BaseDriverNode::bool_text(const bool value)
{
  return value ? "true" : "false";
}

}  // namespace savo_base

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<savo_base::BaseDriverNode>());
  } catch (const std::exception & exc) {
    RCLCPP_FATAL(rclcpp::get_logger("base_driver_node"), "%s", exc.what());
  }

  rclcpp::shutdown();
  return 0;
}
