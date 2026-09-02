// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_perception/obstacle_cloud_filter_node.hpp"

#include "savo_perception/constants.hpp"
#include "savo_perception/topic_names.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace savo_perception
{

namespace
{

std::string json_escape(const std::string & value)
{
  std::string escaped;
  escaped.reserve(value.size());

  for (const char character : value) {
    switch (character) {
      case '"':
        escaped += "\\\"";
        break;
      case '\\':
        escaped += "\\\\";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped += character;
        break;
    }
  }

  return escaped;
}

bool finite_positive(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

const sensor_msgs::msg::PointField * find_field(
  const sensor_msgs::msg::PointCloud2 & message,
  const std::string & name)
{
  const auto iterator =
    std::find_if(
    message.fields.begin(),
    message.fields.end(),
    [&name](
      const sensor_msgs::msg::PointField & field)
    {
      return field.name == name;
    });

  if (iterator == message.fields.end()) {
    return nullptr;
  }

  return &(*iterator);
}

}  // namespace

ObstacleCloudFilterNode::ObstacleCloudFilterNode()
: Node(constants::kObstacleCloudFilterNodeName)
{
  declare_and_read_parameters();

  const auto status_qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  cloud_publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic_,
    rclcpp::SensorDataQoS());

  health_publisher_ =
    create_publisher<std_msgs::msg::Bool>(
    health_topic_,
    status_qos);

  status_publisher_ =
    create_publisher<std_msgs::msg::String>(
    status_topic_,
    status_qos);

  heartbeat_publisher_ =
    create_publisher<std_msgs::msg::String>(
    heartbeat_topic_,
    status_qos);

  const auto parameter_error =
    validate_parameters();

  configuration_valid_ = parameter_error.empty();

  if (!configuration_valid_) {
    state_ = "invalid_configuration";
    reason_ = parameter_error;
    healthy_ = false;

    if (startup_fail_is_fatal_) {
      throw std::invalid_argument(parameter_error);
    }
  }

  if (configuration_valid_) {
    processing_gate_ =
      std::make_unique<ObstacleCloudProcessingGate>(
      max_processing_hz_);

    filter_accumulator_ =
      std::make_unique<ObstacleCloudFilterAccumulator>(
      filter_config_);
  }

  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(get_clock());

  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer_);

  if (configuration_valid_) {
    auto input_qos = rclcpp::SensorDataQoS();
    input_qos.keep_last(1).best_effort().durability_volatile();

    cloud_subscription_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      input_qos,
      std::bind(
        &ObstacleCloudFilterNode::handle_cloud,
        this,
        std::placeholders::_1));
  }

  status_timer_ =
    create_wall_timer(
    std::chrono::duration<double>(
      1.0 / status_publish_hz_),
    std::bind(
      &ObstacleCloudFilterNode::
      publish_health_and_status,
      this));

  heartbeat_timer_ =
    create_wall_timer(
    std::chrono::duration<double>(
      1.0 / heartbeat_hz_),
    std::bind(
      &ObstacleCloudFilterNode::
      publish_heartbeat,
      this));

  publish_health_and_status();
}

void ObstacleCloudFilterNode::declare_and_read_parameters()
{
  input_topic_ =
    declare_parameter<std::string>(
    "input_topic",
    topics::kRawRealSensePointCloud);

  output_topic_ =
    declare_parameter<std::string>(
    "output_topic",
    topics::kFilteredObstaclePointCloud);

  output_frame_ =
    declare_parameter<std::string>(
    "output_frame",
    "base_link");

  health_topic_ =
    declare_parameter<std::string>(
    "health_topic",
    topics::kObstacleCloudHealth);

  status_topic_ =
    declare_parameter<std::string>(
    "status_topic",
    topics::kObstacleCloudStatus);

  heartbeat_topic_ =
    declare_parameter<std::string>(
    "heartbeat_topic",
    topics::kObstacleCloudHeartbeat);

  filter_config_.min_range_m =
    declare_parameter<double>(
    "min_range_m",
    0.20);

  filter_config_.max_range_m =
    declare_parameter<double>(
    "max_range_m",
    3.00);

  filter_config_.min_height_m =
    declare_parameter<double>(
    "min_height_m",
    0.05);

  filter_config_.max_height_m =
    declare_parameter<double>(
    "max_height_m",
    1.60);

  filter_config_.voxel_size_m =
    declare_parameter<double>(
    "voxel_size_m",
    0.05);

  filter_config_.self_filter_enabled =
    declare_parameter<bool>(
    "self_filter_enabled",
    true);

  filter_config_.self_min_x_m =
    declare_parameter<double>(
    "self_min_x_m",
    -0.30);

  filter_config_.self_max_x_m =
    declare_parameter<double>(
    "self_max_x_m",
    0.30);

  filter_config_.self_min_y_m =
    declare_parameter<double>(
    "self_min_y_m",
    -0.26);

  filter_config_.self_max_y_m =
    declare_parameter<double>(
    "self_max_y_m",
    0.26);

  filter_config_.self_min_z_m =
    declare_parameter<double>(
    "self_min_z_m",
    -0.10);

  filter_config_.self_max_z_m =
    declare_parameter<double>(
    "self_max_z_m",
    0.70);

  const auto max_output_points =
    declare_parameter<std::int64_t>(
    "max_output_points",
    100000);

  if (max_output_points > 0) {
    filter_config_.max_output_points =
      static_cast<std::size_t>(
      max_output_points);
  } else {
    filter_config_.max_output_points = 0U;
  }

  transform_timeout_s_ =
    declare_parameter<double>(
    "transform_timeout_s",
    0.10);

  max_processing_hz_ =
    declare_parameter<double>(
    "max_processing_hz",
    10.0);

  stale_timeout_s_ =
    declare_parameter<double>(
    "stale_timeout_s",
    0.75);

  status_publish_hz_ =
    declare_parameter<double>(
    "status_publish_hz",
    2.0);

  heartbeat_hz_ =
    declare_parameter<double>(
    "heartbeat_hz",
    1.0);

  startup_fail_is_fatal_ =
    declare_parameter<bool>(
    "startup_fail_is_fatal",
    false);
}

std::string ObstacleCloudFilterNode::validate_parameters() const
{
  if (
    input_topic_.empty() ||
    output_topic_.empty() ||
    output_frame_.empty() ||
    health_topic_.empty() ||
    status_topic_.empty() ||
    heartbeat_topic_.empty())
  {
    return "required_string_parameter_empty";
  }

  const auto filter_error =
    validate_obstacle_cloud_filter_config(
    filter_config_);

  if (!filter_error.empty()) {
    return filter_error;
  }

  if (!finite_positive(transform_timeout_s_)) {
    return "invalid_transform_timeout";
  }

  if (!finite_positive(max_processing_hz_)) {
    return "invalid_max_processing_rate";
  }

  if (!finite_positive(stale_timeout_s_)) {
    return "invalid_stale_timeout";
  }

  if (!finite_positive(status_publish_hz_)) {
    return "invalid_status_publish_rate";
  }

  if (!finite_positive(heartbeat_hz_)) {
    return "invalid_heartbeat_rate";
  }

  return {};
}

bool ObstacleCloudFilterNode::validate_cloud_layout(
  const sensor_msgs::msg::PointCloud2 & message,
  std::string & reason) const
{
  if (message.header.frame_id.empty()) {
    reason = "empty_input_frame";
    return false;
  }

  if (message.is_bigendian) {
    reason = "big_endian_cloud_not_supported";
    return false;
  }

  const auto storage_error =
    validate_point_cloud_storage_layout(
    PointCloudStorageLayout{
      static_cast<std::size_t>(message.width),
      static_cast<std::size_t>(message.height),
      static_cast<std::size_t>(message.point_step),
      static_cast<std::size_t>(message.row_step),
      message.data.size()});

  if (!storage_error.empty()) {
    reason = storage_error;
    return false;
  }

  const auto * x_field = find_field(message, "x");
  const auto * y_field = find_field(message, "y");
  const auto * z_field = find_field(message, "z");

  if (
    x_field == nullptr ||
    y_field == nullptr ||
    z_field == nullptr)
  {
    reason = "missing_xyz_fields";
    return false;
  }

  for (const auto * field :
    {x_field, y_field, z_field})
  {
    if (
      field->datatype !=
      sensor_msgs::msg::PointField::FLOAT32 ||
      field->count != 1U)
    {
      reason = "unsupported_xyz_field_type";
      return false;
    }

    const auto field_offset =
      static_cast<std::size_t>(field->offset);

    const auto point_step =
      static_cast<std::size_t>(message.point_step);

    if (
      field_offset > point_step ||
      sizeof(float) > point_step - field_offset)
    {
      reason = "xyz_field_outside_point_step";
      return false;
    }
  }

  return true;
}

void ObstacleCloudFilterNode::handle_cloud(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  ++clouds_received_;

  const auto monotonic_time_s =
    std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();

  if (!processing_gate_->should_process(monotonic_time_s)) {
    ++clouds_rate_limited_;
    return;
  }

  if (!message) {
    reject_cloud(
      "malformed_cloud",
      "null_cloud_message",
      false);
    return;
  }

  std::string layout_error;

  if (!validate_cloud_layout(*message, layout_error)) {
    reject_cloud(
      "malformed_cloud",
      layout_error,
      false);
    return;
  }

  geometry_msgs::msg::TransformStamped transform_message;

  try {
    const rclcpp::Time cloud_time(
      message->header.stamp,
      get_clock()->get_clock_type());

    transform_message =
      tf_buffer_->lookupTransform(
      output_frame_,
      message->header.frame_id,
      cloud_time,
      rclcpp::Duration::from_seconds(
        transform_timeout_s_));
  } catch (const tf2::TransformException & error) {
    reject_cloud(
      "transform_unavailable",
      error.what(),
      true);
    return;
  } catch (const std::exception & error) {
    reject_cloud(
      "transform_unavailable",
      error.what(),
      true);
    return;
  }

  const auto & translation =
    transform_message.transform.translation;

  const auto & rotation =
    transform_message.transform.rotation;

  const tf2::Quaternion quaternion(
    rotation.x,
    rotation.y,
    rotation.z,
    rotation.w);

  const tf2::Matrix3x3 rotation_matrix(quaternion);

  const double r00 = rotation_matrix[0][0];
  const double r01 = rotation_matrix[0][1];
  const double r02 = rotation_matrix[0][2];
  const double r10 = rotation_matrix[1][0];
  const double r11 = rotation_matrix[1][1];
  const double r12 = rotation_matrix[1][2];
  const double r20 = rotation_matrix[2][0];
  const double r21 = rotation_matrix[2][1];
  const double r22 = rotation_matrix[2][2];
  const double tx = translation.x;
  const double ty = translation.y;
  const double tz = translation.z;

  const std::size_t point_count =
    static_cast<std::size_t>(message->width) *
    static_cast<std::size_t>(message->height);

  filter_accumulator_->reset();

  try {
    sensor_msgs::PointCloud2ConstIterator<float>
    x_iterator(*message, "x");

    sensor_msgs::PointCloud2ConstIterator<float>
    y_iterator(*message, "y");

    sensor_msgs::PointCloud2ConstIterator<float>
    z_iterator(*message, "z");

    for (std::size_t index = 0U;
      index < point_count;
      ++index, ++x_iterator, ++y_iterator, ++z_iterator)
    {
      const double x = static_cast<double>(*x_iterator);
      const double y = static_cast<double>(*y_iterator);
      const double z = static_cast<double>(*z_iterator);

      if (
        !std::isfinite(x) ||
        !std::isfinite(y) ||
        !std::isfinite(z))
      {
        filter_accumulator_->reject_non_finite_input();
        continue;
      }

      filter_accumulator_->consume(
        PointXYZ{
          r00 * x + r01 * y + r02 * z + tx,
          r10 * x + r11 * y + r12 * z + ty,
          r20 * x + r21 * y + r22 * z + tz});
    }
  } catch (const std::exception & error) {
    reject_cloud(
      "malformed_cloud",
      error.what(),
      false);
    return;
  }

  const auto & result = filter_accumulator_->result();

  sensor_msgs::msg::PointCloud2 output;
  output.header.stamp = message->header.stamp;
  output.header.frame_id = output_frame_;
  output.height = 1U;
  output.is_bigendian = false;
  output.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(output);

  modifier.setPointCloud2Fields(
    3,
    "x",
    1,
    sensor_msgs::msg::PointField::FLOAT32,
    "y",
    1,
    sensor_msgs::msg::PointField::FLOAT32,
    "z",
    1,
    sensor_msgs::msg::PointField::FLOAT32);

  modifier.resize(result.points.size());

  sensor_msgs::PointCloud2Iterator<float>
  output_x(output, "x");

  sensor_msgs::PointCloud2Iterator<float>
  output_y(output, "y");

  sensor_msgs::PointCloud2Iterator<float>
  output_z(output, "z");

  for (const auto & point : result.points) {
    *output_x = static_cast<float>(point.x);
    *output_y = static_cast<float>(point.y);
    *output_z = static_cast<float>(point.z);
    ++output_x;
    ++output_y;
    ++output_z;
  }

  output.height = 1U;
  output.width =
    static_cast<std::uint32_t>(
    result.points.size());

  output.row_step =
    output.point_step * output.width;

  cloud_publisher_->publish(output);

  ++clouds_published_;
  last_stats_ = result.stats;
  input_frame_ = message->header.frame_id;
  last_valid_input_time_ = now();
  have_valid_input_ = true;
  healthy_ = true;
  state_ = "ready";
  reason_ = "cloud_processed";

  publish_health_and_status();
}

void ObstacleCloudFilterNode::reject_cloud(
  const std::string & state,
  const std::string & reason,
  const bool transform_failure)
{
  healthy_ = false;
  state_ = state;
  reason_ = reason;

  if (transform_failure) {
    ++transform_failures_;
  } else {
    ++malformed_clouds_;
  }

  publish_health_and_status();
}

void ObstacleCloudFilterNode::publish_health_and_status()
{
  if (
    configuration_valid_ &&
    have_valid_input_)
  {
    double age_seconds = 0.0;

    try {
      age_seconds =
        (now() - last_valid_input_time_).seconds();
    } catch (const std::exception & error) {
      healthy_ = false;
      state_ = "stale";
      reason_ = error.what();
    }

    if (age_seconds > stale_timeout_s_) {
      healthy_ = false;
      state_ = "stale";
      reason_ = "input_cloud_stale";
    }
  }

  std_msgs::msg::Bool health;
  health.data = healthy_;
  health_publisher_->publish(health);

  std_msgs::msg::String status;
  status.data = make_status_json();
  status_publisher_->publish(status);
}

void ObstacleCloudFilterNode::publish_heartbeat()
{
  ++heartbeat_counter_;

  std_msgs::msg::String heartbeat;

  heartbeat.data =
    "{\"node\":\"obstacle_cloud_filter_node\","
    "\"counter\":" +
    std::to_string(heartbeat_counter_) +
    "}";

  heartbeat_publisher_->publish(heartbeat);
}

std::string ObstacleCloudFilterNode::make_status_json() const
{
  double age_seconds = -1.0;

  if (have_valid_input_) {
    try {
      age_seconds =
        (now() - last_valid_input_time_).seconds();
    } catch (const std::exception &) {
      age_seconds =
        std::numeric_limits<double>::infinity();
    }
  }

  std::ostringstream output;
  output << std::boolalpha;
  output << "{\"state\":\"" << json_escape(state_);
  output << "\",\"reason\":\"" << json_escape(reason_);
  output << "\",\"input_topic\":\"" << json_escape(input_topic_);
  output << "\",\"output_topic\":\"" << json_escape(output_topic_);
  output << "\",\"input_frame\":\"" << json_escape(input_frame_);
  output << "\",\"output_frame\":\"" << json_escape(output_frame_);
  output << "\",\"input_points\":" << last_stats_.input_points;
  output << ",\"output_points\":" << last_stats_.output_points;
  output << ",\"finite_points\":" << last_stats_.finite_points;
  output << ",\"range_rejected\":" << last_stats_.range_rejected;
  output << ",\"height_rejected\":" << last_stats_.height_rejected;
  output << ",\"self_rejected\":" << last_stats_.self_rejected;
  output << ",\"voxel_rejected\":" << last_stats_.voxel_rejected;
  output << ",\"output_limited\":" << last_stats_.output_limited;
  output << ",\"transform_failures\":" << transform_failures_;
  output << ",\"malformed_clouds\":" << malformed_clouds_;
  output << ",\"clouds_received\":" << clouds_received_;
  output << ",\"clouds_published\":" << clouds_published_;
  output << ",\"clouds_rate_limited\":" << clouds_rate_limited_;
  output << ",\"max_processing_hz\":" << max_processing_hz_;
  output << ",\"age_seconds\":" << age_seconds;
  output << ",\"clearing_supported\":false";
  output << ",\"semantics\":\"obstacle_only\"}";
  return output.str();
}

}  // namespace savo_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<
        savo_perception::ObstacleCloudFilterNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "obstacle_cloud_filter_node"),
      "Fatal startup error: %s",
      error.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
