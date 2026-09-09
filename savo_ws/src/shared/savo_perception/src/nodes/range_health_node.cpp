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
  declare_parameter<double>("rate_window_s", 2.0);
  declare_parameter<std::int64_t>("rate_min_samples", 5);
  declare_parameter<double>("tof_minimum_rate_hz", 5.0);
  declare_parameter<double>("tof_good_rate_hz", 8.0);
  declare_parameter<double>("tof_excellent_rate_hz", 9.0);
  declare_parameter<double>("depth_minimum_rate_hz", 5.0);
  declare_parameter<double>("depth_good_rate_hz", 10.0);
  declare_parameter<double>("depth_excellent_rate_hz", 12.0);

  declare_parameter<bool>("include_depth_in_overall_ok", false);
  declare_parameter<bool>("depth_front_required", false);
  declare_parameter<bool>("use_ultrasonic", true);

  declare_parameter<std::vector<std::string>>(
    "required_sensors",
    std::vector<std::string>{"tof_left", "tof_right"});

  declare_parameter<std::vector<std::string>>(
    "optional_sensors",
    std::vector<std::string>{"depth_front", "ultrasonic_front"});

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

  rate_window_s_ = safe_rate_hz(
    get_parameter("rate_window_s").as_double(),
    2.0);
  const auto rate_min_samples = get_parameter("rate_min_samples").as_int();
  rate_min_samples_ = rate_min_samples >= 2 ?
    static_cast<std::size_t>(rate_min_samples) : 5U;
  tof_minimum_rate_hz_ = safe_rate_hz(
    get_parameter("tof_minimum_rate_hz").as_double(), 5.0);
  tof_good_rate_hz_ = std::max(
    tof_minimum_rate_hz_,
    get_parameter("tof_good_rate_hz").as_double());
  tof_excellent_rate_hz_ = std::max(
    tof_good_rate_hz_,
    get_parameter("tof_excellent_rate_hz").as_double());
  depth_minimum_rate_hz_ = safe_rate_hz(
    get_parameter("depth_minimum_rate_hz").as_double(), 5.0);
  depth_good_rate_hz_ = std::max(
    depth_minimum_rate_hz_,
    get_parameter("depth_good_rate_hz").as_double());
  depth_excellent_rate_hz_ = std::max(
    depth_good_rate_hz_,
    get_parameter("depth_excellent_rate_hz").as_double());

  include_depth_in_overall_ok_ = get_parameter("include_depth_in_overall_ok").as_bool();
  depth_front_required_ = get_parameter("depth_front_required").as_bool();
  use_ultrasonic_ = get_parameter("use_ultrasonic").as_bool();

  required_sensors_ = get_parameter("required_sensors").as_string_array();
  optional_sensors_ = get_parameter("optional_sensors").as_string_array();

  if (!use_ultrasonic_) {
    required_sensors_.erase(
      std::remove(required_sensors_.begin(), required_sensors_.end(), "ultrasonic_front"),
      required_sensors_.end());
  }

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

  if (use_ultrasonic_) {
    ultrasonic_front_sub_ = create_subscription<std_msgs::msg::Float32>(
      ultrasonic_front_topic_,
      rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        on_ultrasonic_front(msg);
      });
  }

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
  record_receipt("depth_front");
  depth_front_ = sample_from_value("depth_front", msg->data, "depth_front");
}

void RangeHealthNode::on_tof_left(const std_msgs::msg::Float32::SharedPtr msg)
{
  record_receipt("tof_left");
  tof_left_ = sample_from_value("tof_left", msg->data, "tof_left");
}

void RangeHealthNode::on_tof_right(const std_msgs::msg::Float32::SharedPtr msg)
{
  record_receipt("tof_right");
  tof_right_ = sample_from_value("tof_right", msg->data, "tof_right");
}

void RangeHealthNode::on_ultrasonic_front(const std_msgs::msg::Float32::SharedPtr msg)
{
  record_receipt("ultrasonic_front");
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

  if (!std::isfinite(distance_m) || distance_m <= 0.0) {
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

  std::vector<SensorHealth> health{
    make_sensor_health(depth_front_, stale_timeout_s_, now),
    make_sensor_health(tof_left_, stale_timeout_s_, now),
    make_sensor_health(tof_right_, stale_timeout_s_, now),
  };

  if (use_ultrasonic_) {
    health.push_back(make_sensor_health(ultrasonic_front_, stale_timeout_s_, now));
  }

  return health;
}

bool RangeHealthNode::overall_ok(const std::vector<SensorHealth> & health) const
{
  for (const auto & item : health) {
    if (
      is_required_sensor(item.sensor_name) &&
      (!item.ok || !rate_ok(item.sensor_name)))
    {
      return false;
    }

    if (
      include_depth_in_overall_ok_ &&
      item.sensor_name == "depth_front" &&
      (!item.ok || !rate_ok(item.sensor_name)))
    {
      return false;
    }
  }

  return true;
}

void RangeHealthNode::record_receipt(const std::string & sensor_name)
{
  auto & times = receipt_times_[sensor_name];
  const auto receipt_time = std::chrono::steady_clock::now();
  times.push_back(receipt_time);
  while (
    times.size() > 2U &&
    std::chrono::duration<double>(receipt_time - times.front()).count() >
    rate_window_s_)
  {
    times.pop_front();
  }
}

bool RangeHealthNode::rate_valid(const std::string & sensor_name) const
{
  const auto iterator = receipt_times_.find(sensor_name);
  return iterator != receipt_times_.end() &&
         iterator->second.size() >= rate_min_samples_;
}

double RangeHealthNode::receive_rate_hz(const std::string & sensor_name) const
{
  const auto iterator = receipt_times_.find(sensor_name);
  if (iterator == receipt_times_.end() || iterator->second.size() < 2U) {
    return 0.0;
  }

  const auto & times = iterator->second;
  const double interval_s =
    std::chrono::duration<double>(times.back() - times.front()).count();
  if (!std::isfinite(interval_s) || interval_s <= 0.0) {
    return 0.0;
  }
  return static_cast<double>(times.size() - 1U) / interval_s;
}

std::string RangeHealthNode::rate_quality(const std::string & sensor_name) const
{
  if (sensor_name == "ultrasonic_front") {
    return "NOT_APPLICABLE";
  }
  if (!rate_valid(sensor_name)) {
    return "ESTABLISHING";
  }

  const bool is_depth = sensor_name == "depth_front";
  const double rate_hz = receive_rate_hz(sensor_name);
  const double minimum_hz = is_depth ?
    depth_minimum_rate_hz_ : tof_minimum_rate_hz_;
  const double good_hz = is_depth ? depth_good_rate_hz_ : tof_good_rate_hz_;
  const double excellent_hz = is_depth ?
    depth_excellent_rate_hz_ : tof_excellent_rate_hz_;

  if (rate_hz < minimum_hz) {
    return "BELOW_MINIMUM";
  }
  if (rate_hz < good_hz) {
    return "MINIMUM";
  }
  if (rate_hz < excellent_hz) {
    return "GOOD";
  }
  return "EXCELLENT";
}

bool RangeHealthNode::rate_ok(const std::string & sensor_name) const
{
  return rate_quality(sensor_name) != "BELOW_MINIMUM";
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
      (item.status == SensorStatus::kError ||
      (!item.stale && !rate_ok(item.sensor_name))))
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
      if (item.sensor_name != "ultrasonic_front") {
        out << "/" << rate_quality(item.sensor_name);
      }
    }

    if (!use_ultrasonic_) {
      out << " ultrasonic_front=DISABLED";
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
  std::vector<std::string> disabled_sensors;
  if (!use_ultrasonic_) {
    disabled_sensors.push_back("ultrasonic_front");
  }

  std::ostringstream out;

  out << "{";
  out << "\"overall_ok\":" << (overall_ok(health) ? "true" : "false") << ",";
  out << "\"overall_status\":\"" << overall_status(health) << "\",";
  out << "\"quality\":\"" << (overall_ok(health) ? "MINIMUM" : "BELOW_MINIMUM") << "\",";
  out << "\"quality_reason\":\"required_range_health_only\",";
  out << "\"stale_required_sensors\":" <<
    string_list_to_json(stale_required_sensors(health)) << ",";
  out << "\"error_required_sensors\":" <<
    string_list_to_json(error_required_sensors(health)) << ",";
  out << "\"required_sensors\":" << string_list_to_json(required_sensors_) << ",";
  out << "\"optional_sensors\":" << string_list_to_json(optional_sensors_) << ",";
  out << "\"disabled_sensors\":" << string_list_to_json(disabled_sensors) << ",";
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
  out << "\"receive_rate_hz\":" << receive_rate_hz(health.sensor_name) << ",";
  out << "\"rate_valid\":" << (rate_valid(health.sensor_name) ? "true" : "false") << ",";
  out << "\"rate_quality\":\"" << rate_quality(health.sensor_name) << "\",";
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
