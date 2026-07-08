#include "savo_perception/range_health_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
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

RangeHealthNode::RangeHealthNode(const rclcpp::NodeOptions & options)
: rclcpp::Node(constants::kRangeHealthNodeName, options)
{
  declare_parameters();
  load_parameters();
  setup_interfaces();
}

void RangeHealthNode::declare_parameters()
{
  declare_parameter<std::string>("depth_front_topic", topics::kDepthFrontM);
  declare_parameter<std::string>("tof_left_topic", topics::kTofLeftM);
  declare_parameter<std::string>("tof_right_topic", topics::kTofRightM);
  declare_parameter<std::string>("ultrasonic_front_topic", topics::kUltrasonicFrontM);

  declare_parameter<std::string>("range_health_topic", topics::kRangeHealth);
  declare_parameter<std::string>("sensor_status_topic", topics::kSensorStatus);
  declare_parameter<std::string>("heartbeat_topic", topics::kHeartbeat);

  declare_parameter<double>("publish_hz", constants::kRangeHealthPublishHzDefault);
  declare_parameter<double>("stale_timeout_s", constants::kSensorStaleTimeoutSDefault);
  declare_parameter<double>("heartbeat_hz", 1.0);

  declare_parameter<bool>("include_depth_in_overall_ok", false);
  declare_parameter<bool>("depth_front_required", false);

  declare_parameter<std::vector<std::string>>(
    "required_sensors",
    std::vector<std::string>{"tof_left", "tof_right"});

  declare_parameter<std::vector<std::string>>(
    "optional_sensors",
    std::vector<std::string>{"depth_front"});

  declare_parameter<bool>("publish_json", true);
  declare_parameter<bool>("publish_compact_status", true);
  declare_parameter<bool>("startup_fail_is_fatal", constants::kStartupFailIsFatalDefault);
}

void RangeHealthNode::load_parameters()
{
  depth_front_topic_ = get_parameter("depth_front_topic").as_string();
  tof_left_topic_ = get_parameter("tof_left_topic").as_string();
  tof_right_topic_ = get_parameter("tof_right_topic").as_string();
  ultrasonic_front_topic_ = get_parameter("ultrasonic_front_topic").as_string();

  range_health_topic_ = get_parameter("range_health_topic").as_string();
  sensor_status_topic_ = get_parameter("sensor_status_topic").as_string();
  heartbeat_topic_ = get_parameter("heartbeat_topic").as_string();

  publish_hz_ = safe_rate_hz(
    get_parameter("publish_hz").as_double(),
    constants::kRangeHealthPublishHzDefault);

  stale_timeout_s_ = std::max(
    0.01,
    get_parameter("stale_timeout_s").as_double());

  heartbeat_hz_ = safe_rate_hz(
    get_parameter("heartbeat_hz").as_double(),
    1.0);

  include_depth_in_overall_ok_ = get_parameter("include_depth_in_overall_ok").as_bool();
  depth_front_required_ = get_parameter("depth_front_required").as_bool();

  required_sensors_ = get_parameter("required_sensors").as_string_array();
  optional_sensors_ = get_parameter("optional_sensors").as_string_array();

  if (depth_front_required_ && !is_required_sensor("depth_front")) {
    required_sensors_.push_back("depth_front");
  }

  publish_json_ = get_parameter("publish_json").as_bool();
  publish_compact_status_ = get_parameter("publish_compact_status").as_bool();

  depth_front_ = missing_sample("depth_front", "not_received", true);
  tof_left_ = missing_sample("tof_left", "not_received", true);
  tof_right_ = missing_sample("tof_right", "not_received", true);
  ultrasonic_front_ = missing_sample("ultrasonic_front", "not_received", true);
}

void RangeHealthNode::setup_interfaces()
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

  const auto status_qos = rclcpp::QoS(10).reliable();

  range_health_pub_ = create_publisher<std_msgs::msg::String>(
    range_health_topic_,
    status_qos);

  sensor_status_pub_ = create_publisher<std_msgs::msg::String>(
    sensor_status_topic_,
    status_qos);

  heartbeat_pub_ = create_publisher<std_msgs::msg::String>(
    heartbeat_topic_,
    status_qos);

  const auto publish_period = std::chrono::duration<double>(1.0 / publish_hz_);

  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
    [this]() {
      on_timer();
    });

  const auto heartbeat_period = std::chrono::duration<double>(1.0 / heartbeat_hz_);

  heartbeat_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(heartbeat_period),
    [this]() {
      on_heartbeat_timer();
    });

  RCLCPP_INFO(
    get_logger(),
    "Range health node started: publish=%.2fHz heartbeat=%.2fHz stale_timeout=%.2fs",
    publish_hz_,
    heartbeat_hz_,
    stale_timeout_s_);
}

void RangeHealthNode::on_depth_front(const std_msgs::msg::Float32::SharedPtr msg)
{
  depth_front_ = sample_from_value("depth_front", msg->data, "depth_front");
}

void RangeHealthNode::on_tof_left(const std_msgs::msg::Float32::SharedPtr msg)
{
  tof_left_ = sample_from_value("tof_left", msg->data, "tof_left");
}

void RangeHealthNode::on_tof_right(const std_msgs::msg::Float32::SharedPtr msg)
{
  tof_right_ = sample_from_value("tof_right", msg->data, "tof_right");
}

void RangeHealthNode::on_ultrasonic_front(const std_msgs::msg::Float32::SharedPtr msg)
{
  ultrasonic_front_ = sample_from_value("ultrasonic_front", msg->data, "ultrasonic_front");
}

void RangeHealthNode::on_timer()
{
  const auto health = current_health();

  publish_health(health);
  publish_sensor_status(health);
}

void RangeHealthNode::on_heartbeat_timer()
{
  publish_heartbeat();
}

RangeSample RangeHealthNode::sample_from_value(
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

RangeSample RangeHealthNode::missing_sample(
  const std::string & sensor_name,
  const std::string & reason,
  const bool stale_now) const
{
  auto sample = make_invalid_range_sample(sensor_name, reason, "range_health_node");

  if (stale_now) {
    sample.stamp = std::chrono::steady_clock::now() - std::chrono::seconds(3600);
  }

  return sample;
}

std::vector<SensorHealth> RangeHealthNode::current_health() const
{
  const auto now = std::chrono::steady_clock::now();

  return {
    make_sensor_health(depth_front_, stale_timeout_s_, now),
    make_sensor_health(tof_left_, stale_timeout_s_, now),
    make_sensor_health(tof_right_, stale_timeout_s_, now),
    make_sensor_health(ultrasonic_front_, stale_timeout_s_, now),
  };
}

bool RangeHealthNode::overall_ok(const std::vector<SensorHealth> & health) const
{
  for (const auto & item : health) {
    if (is_required_sensor(item.sensor_name) && !item.ok) {
      return false;
    }

    if (
      include_depth_in_overall_ok_ &&
      item.sensor_name == "depth_front" &&
      !item.ok)
    {
      return false;
    }
  }

  return true;
}

std::string RangeHealthNode::overall_status(const std::vector<SensorHealth> & health) const
{
  const auto stale_required = stale_required_sensors(health);
  const auto error_required = error_required_sensors(health);

  if (!error_required.empty()) {
    return "ERROR";
  }

  if (!stale_required.empty()) {
    return "STALE";
  }

  if (!overall_ok(health)) {
    return "ERROR";
  }

  return "OK";
}

bool RangeHealthNode::is_required_sensor(const std::string & sensor_name) const
{
  return std::find(
    required_sensors_.begin(),
    required_sensors_.end(),
    sensor_name) != required_sensors_.end();
}

bool RangeHealthNode::is_optional_sensor(const std::string & sensor_name) const
{
  return std::find(
    optional_sensors_.begin(),
    optional_sensors_.end(),
    sensor_name) != optional_sensors_.end();
}

std::vector<std::string> RangeHealthNode::stale_required_sensors(
  const std::vector<SensorHealth> & health) const
{
  std::vector<std::string> out;

  for (const auto & item : health) {
    if (is_required_sensor(item.sensor_name) && item.stale) {
      out.push_back(item.sensor_name);
    }
  }

  return out;
}

std::vector<std::string> RangeHealthNode::error_required_sensors(
  const std::vector<SensorHealth> & health) const
{
  std::vector<std::string> out;

  for (const auto & item : health) {
    if (
      is_required_sensor(item.sensor_name) &&
      item.status == SensorStatus::kError)
    {
      out.push_back(item.sensor_name);
    }
  }

  return out;
}

void RangeHealthNode::publish_health(const std::vector<SensorHealth> & health)
{
  if (!range_health_pub_) {
    return;
  }

  std_msgs::msg::String msg;

  if (publish_json_) {
    msg.data = health_to_json(health);
  } else {
    msg.data = overall_status(health);
  }

  range_health_pub_->publish(msg);
}

void RangeHealthNode::publish_sensor_status(const std::vector<SensorHealth> & health)
{
  if (!sensor_status_pub_) {
    return;
  }

  std_msgs::msg::String msg;

  if (publish_compact_status_) {
    std::ostringstream out;
    out << overall_status(health);

    for (const auto & item : health) {
      out << " " << item.sensor_name << "=" << to_string(item.status);
    }

    msg.data = out.str();
  } else {
    msg.data = health_to_json(health);
  }

  sensor_status_pub_->publish(msg);
}

void RangeHealthNode::publish_heartbeat()
{
  if (!heartbeat_pub_) {
    return;
  }

  heartbeat_count_ += 1;

  std_msgs::msg::String msg;
  std::ostringstream out;

  out << "{";
  out << "\"node\":\"" << constants::kRangeHealthNodeName << "\",";
  out << "\"count\":" << heartbeat_count_ << ",";
  out << "\"ok\":true";
  out << "}";

  msg.data = out.str();
  heartbeat_pub_->publish(msg);
}

std::string RangeHealthNode::health_to_json(const std::vector<SensorHealth> & health) const
{
  std::ostringstream out;

  out << "{";
  out << "\"overall_ok\":" << (overall_ok(health) ? "true" : "false") << ",";
  out << "\"overall_status\":\"" << overall_status(health) << "\",";
  out << "\"stale_required_sensors\":" << string_list_to_json(stale_required_sensors(health)) << ",";
  out << "\"error_required_sensors\":" << string_list_to_json(error_required_sensors(health)) << ",";
  out << "\"required_sensors\":" << string_list_to_json(required_sensors_) << ",";
  out << "\"optional_sensors\":" << string_list_to_json(optional_sensors_) << ",";
  out << "\"sensors\":[";

  for (std::size_t i = 0; i < health.size(); ++i) {
    if (i > 0) {
      out << ",";
    }

    out << sensor_health_to_json(health[i]);
  }

  out << "]";
  out << "}";

  return out.str();
}

std::string RangeHealthNode::sensor_health_to_json(const SensorHealth & health) const
{
  std::ostringstream out;

  out << "{";
  out << "\"sensor_name\":\"" << json_escape(health.sensor_name) << "\",";
  out << "\"status\":\"" << to_string(health.status) << "\",";
  out << "\"ok\":" << (health.ok ? "true" : "false") << ",";
  out << "\"stale\":" << (health.stale ? "true" : "false") << ",";
  out << "\"valid\":" << (health.valid ? "true" : "false") << ",";
  out << "\"last_distance_m\":" << optional_double_to_json(health.last_distance_m) << ",";
  out << "\"age_s\":" << health.age_s << ",";
  out << "\"required\":" << (is_required_sensor(health.sensor_name) ? "true" : "false") << ",";
  out << "\"optional\":" << (is_optional_sensor(health.sensor_name) ? "true" : "false") << ",";
  out << "\"error\":\"" << json_escape(health.error) << "\",";
  out << "\"source\":\"" << json_escape(health.source) << "\"";
  out << "}";

  return out.str();
}

std::string RangeHealthNode::string_list_to_json(const std::vector<std::string> & values) const
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

  const auto node = std::make_shared<savo_perception::RangeHealthNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
