#include "savo_perception/safety_stop_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

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

std::string optional_double_to_json(const std::optional<double> & value)
{
  if (!value.has_value() || !std::isfinite(*value)) {
    return "null";
  }

  std::ostringstream out;
  out << *value;
  return out.str();
}

}  // namespace

SafetyStopNode::SafetyStopNode(const rclcpp::NodeOptions & options)
: rclcpp::Node(constants::kSafetyStopNodeName, options)
{
  declare_parameters();
  load_parameters();
  setup_interfaces();
}

void SafetyStopNode::declare_parameters()
{
  declare_parameter<std::string>("depth_front_topic", topics::kDepthFrontM);
  declare_parameter<std::string>("tof_left_topic", topics::kTofLeftM);
  declare_parameter<std::string>("tof_right_topic", topics::kTofRightM);
  declare_parameter<std::string>("ultrasonic_front_topic", topics::kUltrasonicFrontM);

  declare_parameter<std::string>("safety_stop_topic", topics::kSafetyStop);
  declare_parameter<std::string>("slowdown_topic", topics::kSafetySlowdownFactor);
  declare_parameter<std::string>("safety_state_topic", topics::kSafetyState);

  declare_parameter<double>("loop_hz", constants::kSafetyLoopHzDefault);
  declare_parameter<double>("stale_timeout_s", constants::kSensorStaleTimeoutSDefault);

  declare_parameter<double>("front_stop_m", constants::kFrontStopMDefault);
  declare_parameter<double>("front_slow_m", constants::kFrontSlowMDefault);
  declare_parameter<double>("side_stop_m", constants::kSideStopMDefault);
  declare_parameter<double>("side_slow_m", constants::kSideSlowMDefault);

  declare_parameter<double>(
    "front_clear_hysteresis_m",
    constants::kFrontClearHysteresisMDefault);
  declare_parameter<double>(
    "side_clear_hysteresis_m",
    constants::kSideClearHysteresisMDefault);

  declare_parameter<int>("stop_debounce_count", constants::kStopDebounceCountDefault);
  declare_parameter<int>("clear_debounce_count", constants::kClearDebounceCountDefault);

  declare_parameter<double>("slowdown_min", constants::kSlowdownMinDefault);
  declare_parameter<double>("slowdown_max", constants::kSlowdownMaxDefault);
  declare_parameter<double>("slowdown_ema_alpha", constants::kSlowdownEmaAlphaDefault);

  declare_parameter<bool>("fail_safe_on_stale", constants::kFailSafeOnStaleDefault);

  declare_parameter<std::vector<std::string>>(
    "required_sensors",
    std::vector<std::string>{"tof_left", "tof_right"});

  declare_parameter<std::vector<std::string>>(
    "optional_sensors",
    std::vector<std::string>{"depth_front"});

  declare_parameter<bool>("use_depth_front", true);
  declare_parameter<bool>("depth_front_required", false);

  declare_parameter<bool>("publish_state_json", true);
  declare_parameter<bool>("startup_fail_is_fatal", constants::kStartupFailIsFatalDefault);
}

void SafetyStopNode::load_parameters()
{
  depth_front_topic_ = get_parameter("depth_front_topic").as_string();
  tof_left_topic_ = get_parameter("tof_left_topic").as_string();
  tof_right_topic_ = get_parameter("tof_right_topic").as_string();
  ultrasonic_front_topic_ = get_parameter("ultrasonic_front_topic").as_string();

  safety_stop_topic_ = get_parameter("safety_stop_topic").as_string();
  slowdown_topic_ = get_parameter("slowdown_topic").as_string();
  safety_state_topic_ = get_parameter("safety_state_topic").as_string();

  loop_hz_ = safe_rate_hz(
    get_parameter("loop_hz").as_double(),
    constants::kSafetyLoopHzDefault);

  policy_config_.fusion.stale_timeout_s = std::max(
    0.01,
    get_parameter("stale_timeout_s").as_double());

  policy_config_.fusion.front_stop_m = get_parameter("front_stop_m").as_double();
  policy_config_.fusion.front_slow_m = get_parameter("front_slow_m").as_double();
  policy_config_.fusion.side_stop_m = get_parameter("side_stop_m").as_double();
  policy_config_.fusion.side_slow_m = get_parameter("side_slow_m").as_double();

  if (policy_config_.fusion.front_slow_m < policy_config_.fusion.front_stop_m) {
    std::swap(policy_config_.fusion.front_slow_m, policy_config_.fusion.front_stop_m);
  }

  if (policy_config_.fusion.side_slow_m < policy_config_.fusion.side_stop_m) {
    std::swap(policy_config_.fusion.side_slow_m, policy_config_.fusion.side_stop_m);
  }

  policy_config_.front_clear_hysteresis_m = std::max(
    0.0,
    get_parameter("front_clear_hysteresis_m").as_double());

  policy_config_.side_clear_hysteresis_m = std::max(
    0.0,
    get_parameter("side_clear_hysteresis_m").as_double());

  policy_config_.stop_debounce_count = std::max(
    1,
    static_cast<int>(get_parameter("stop_debounce_count").as_int()));

  policy_config_.clear_debounce_count = std::max(
    1,
    static_cast<int>(get_parameter("clear_debounce_count").as_int()));

  policy_config_.fusion.slowdown_min = get_parameter("slowdown_min").as_double();
  policy_config_.fusion.slowdown_max = get_parameter("slowdown_max").as_double();

  if (policy_config_.fusion.slowdown_max < policy_config_.fusion.slowdown_min) {
    std::swap(policy_config_.fusion.slowdown_max, policy_config_.fusion.slowdown_min);
  }

  policy_config_.slowdown_ema_alpha = clamp_value(
    get_parameter("slowdown_ema_alpha").as_double(),
    0.0,
    1.0);

  policy_config_.fusion.fail_safe_on_stale = get_parameter("fail_safe_on_stale").as_bool();

  policy_config_.fusion.required_sensors =
    get_parameter("required_sensors").as_string_array();

  policy_config_.fusion.use_depth_front = get_parameter("use_depth_front").as_bool();
  policy_config_.fusion.depth_front_required = get_parameter("depth_front_required").as_bool();

  publish_state_json_ = get_parameter("publish_state_json").as_bool();

  policy_.set_config(policy_config_);
  policy_.reset();

  depth_front_ = missing_sample("depth_front", "not_received", true);
  tof_left_ = missing_sample("tof_left", "not_received", true);
  tof_right_ = missing_sample("tof_right", "not_received", true);
  ultrasonic_front_ = missing_sample("ultrasonic_front", "not_received", true);
}

void SafetyStopNode::setup_interfaces()
{
  depth_front_sub_ = create_subscription<std_msgs::msg::Float32>(
    depth_front_topic_,
    rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Float32::SharedPtr msg) {
      on_depth_front(msg);
    });

  tof_left_sub_ = create_subscription<std_msgs::msg::Float32>(
    tof_left_topic_,
    rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Float32::SharedPtr msg) {
      on_tof_left(msg);
    });

  tof_right_sub_ = create_subscription<std_msgs::msg::Float32>(
    tof_right_topic_,
    rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Float32::SharedPtr msg) {
      on_tof_right(msg);
    });

  ultrasonic_front_sub_ = create_subscription<std_msgs::msg::Float32>(
    ultrasonic_front_topic_,
    rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Float32::SharedPtr msg) {
      on_ultrasonic_front(msg);
    });

  const auto safety_qos = rclcpp::QoS(10).reliable();

  safety_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
    safety_stop_topic_,
    safety_qos);

  slowdown_pub_ = create_publisher<std_msgs::msg::Float32>(
    slowdown_topic_,
    safety_qos);

  safety_state_pub_ = create_publisher<std_msgs::msg::String>(
    safety_state_topic_,
    safety_qos);

  const auto period = std::chrono::duration<double>(1.0 / loop_hz_);

  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() {
      on_timer();
    });

  RCLCPP_INFO(
    get_logger(),
    "Safety stop node started: front_stop=%.3fm front_slow=%.3fm side_stop=%.3fm side_slow=%.3fm",
    policy_config_.fusion.front_stop_m,
    policy_config_.fusion.front_slow_m,
    policy_config_.fusion.side_stop_m,
    policy_config_.fusion.side_slow_m);
}

void SafetyStopNode::on_depth_front(const std_msgs::msg::Float32::SharedPtr msg)
{
  depth_front_ = sample_from_value("depth_front", msg->data, "depth_front");
}

void SafetyStopNode::on_tof_left(const std_msgs::msg::Float32::SharedPtr msg)
{
  tof_left_ = sample_from_value("tof_left", msg->data, "tof_left");
}

void SafetyStopNode::on_tof_right(const std_msgs::msg::Float32::SharedPtr msg)
{
  tof_right_ = sample_from_value("tof_right", msg->data, "tof_right");
}

void SafetyStopNode::on_ultrasonic_front(const std_msgs::msg::Float32::SharedPtr msg)
{
  ultrasonic_front_ = sample_from_value("ultrasonic_front", msg->data, "ultrasonic_front");
}

void SafetyStopNode::on_timer()
{
  const auto update = policy_.update(current_snapshot());
  publish_decision(update);
}

RangeSample SafetyStopNode::sample_from_value(
  const std::string & sensor_name,
  const float value,
  const std::string & source) const
{
  const auto distance_m = static_cast<double>(value);

  if (std::isnan(distance_m) || distance_m <= 0.0) {
    return make_invalid_range_sample(sensor_name, "invalid_distance", source);
  }

  return make_valid_range_sample(sensor_name, distance_m, source);
}

RangeSample SafetyStopNode::missing_sample(
  const std::string & sensor_name,
  const std::string & reason,
  const bool stale_now) const
{
  auto sample = make_invalid_range_sample(sensor_name, reason, "safety_stop_node");

  if (stale_now) {
    sample.stamp = std::chrono::steady_clock::now() - std::chrono::seconds(3600);
  }

  return sample;
}

RangeSnapshot SafetyStopNode::current_snapshot() const
{
  RangeSnapshot snapshot;
  snapshot.depth_front = depth_front_;
  snapshot.tof_left = tof_left_;
  snapshot.tof_right = tof_right_;
  snapshot.ultrasonic_front = ultrasonic_front_;

  return snapshot;
}

void SafetyStopNode::publish_decision(const SafetyPolicyUpdate & update)
{
  std_msgs::msg::Bool stop_msg;
  stop_msg.data = update.published_decision.stop_required;
  safety_stop_pub_->publish(stop_msg);

  std_msgs::msg::Float32 slowdown_msg;
  slowdown_msg.data = static_cast<float>(update.published_decision.slowdown_factor);
  slowdown_pub_->publish(slowdown_msg);

  std_msgs::msg::String state_msg;

  if (publish_state_json_) {
    state_msg.data = state_to_json(update.state);
  } else {
    state_msg.data = to_string(update.published_decision.status);
  }

  safety_state_pub_->publish(state_msg);
}

std::string SafetyStopNode::decision_to_json(const SafetyDecision & decision) const
{
  std::ostringstream out;

  out << "{";
  out << "\"status\":\"" << to_string(decision.status) << "\",";
  out << "\"stop_required\":" << (decision.stop_required ? "true" : "false") << ",";
  out << "\"slowdown_factor\":" << decision.slowdown_factor << ",";
  out << "\"reason\":\"" << json_escape(decision.reason) << "\",";
  out << "\"front_distance_m\":" << optional_double_to_json(decision.front_distance_m) << ",";
  out << "\"side_distance_m\":" << optional_double_to_json(decision.side_distance_m) << ",";
  out << "\"stale_sensors\":" << string_list_to_json(decision.stale_sensors) << ",";
  out << "\"invalid_sensors\":" << string_list_to_json(decision.invalid_sensors) << ",";
  out << "\"source\":\"" << json_escape(decision.source) << "\"";
  out << "}";

  return out.str();
}

std::string SafetyStopNode::state_to_json(const SafetyState & state) const
{
  std::ostringstream out;

  out << "{";
  out << "\"update_count\":" << state.update_count << ",";
  out << "\"stop_count\":" << state.stop_count << ",";
  out << "\"clear_count\":" << state.clear_count << ",";
  out << "\"active_decision\":" << decision_to_json(state.active_decision);
  out << "}";

  return out.str();
}

std::string SafetyStopNode::string_list_to_json(const std::vector<std::string> & values) const
{
  std::ostringstream out;

  out << "[";

  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      out << ",";
    }

    out << "\"" << json_escape(values[i]) << "\"";
  }

  out << "]";
  return out.str();
}

}  // namespace savo_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<savo_perception::SafetyStopNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
