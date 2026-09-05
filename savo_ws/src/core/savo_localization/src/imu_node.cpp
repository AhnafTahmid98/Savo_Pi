#include "savo_localization/imu_node.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "diagnostic_msgs/msg/key_value.hpp"

namespace savo_localization
{

namespace
{

constexpr double PI = 3.14159265358979323846;

std::string bool_text(bool value)
{
  return value ? "true" : "false";
}

std::string hex_u8(uint8_t value)
{
  std::ostringstream oss;
  oss << "0x" << std::uppercase << std::hex << std::setw(2)
      << std::setfill('0') << static_cast<int>(value);
  return oss.str();
}

diagnostic_msgs::msg::KeyValue key_value(
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

}  // namespace

ImuNode::ImuNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("imu_node", options)
{
  declare_parameters();
  load_parameters();
  configure_driver();
  create_calibration_save_service();

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    imu_topic_,
    rclcpp::SensorDataQoS());

  state_pub_ = create_publisher<std_msgs::msg::String>(
    imu_state_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  if (publish_diagnostics_) {
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  }

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ImuNode::timer_callback, this));

  RCLCPP_INFO(
    get_logger(),
    "imu_node started | topic=%s | state_topic=%s | bus=%d | address=0x%02X | mode=%s | rate=%.2f Hz",
    imu_topic_.c_str(),
    imu_state_topic_.c_str(),
    i2c_bus_,
    i2c_address_,
    mode_.c_str(),
    publish_rate_hz_);
}

ImuNode::~ImuNode()
{
  if (driver_) {
    driver_->close();
  }
}

void ImuNode::declare_parameters()
{
  declare_parameter<std::string>("frame_id", frame_id_);
  declare_parameter<std::string>("imu_topic", imu_topic_);
  declare_parameter<std::string>("imu_state_topic", imu_state_topic_);
  declare_parameter<std::string>("diagnostics_topic", diagnostics_topic_);

  declare_parameter<int>("i2c_bus", i2c_bus_);
  declare_parameter<int>("i2c_address", i2c_address_);
  declare_parameter<std::string>("mode", mode_);

  declare_parameter<double>("publish_rate_hz", publish_rate_hz_);
  declare_parameter<double>("health_publish_rate_hz", health_publish_rate_hz_);
  declare_parameter<double>(
    "diagnostics_publish_rate_hz", diagnostics_publish_rate_hz_);
  declare_parameter<double>("timestamp_fault_hold_s", timestamp_fault_hold_s_);
  declare_parameter<int>(
    "producer_rate_window_size", static_cast<int>(producer_rate_window_size_));
  declare_parameter<bool>("reset_on_start", reset_on_start_);
  declare_parameter<bool>(
    "calibration_restore_enabled",
    calibration_restore_enabled_);
  declare_parameter<std::string>(
    "calibration_profile_path",
    calibration_profile_path_);
  declare_parameter<bool>(
    "calibration_require_verified_restore",
    calibration_require_verified_restore_);
  declare_parameter<std::string>(
    "calibration_save_service",
    calibration_save_service_name_);

  declare_parameter<bool>("publish_orientation", publish_orientation_);
  declare_parameter<bool>("publish_magnetic_field", publish_magnetic_field_);
  declare_parameter<bool>("publish_temperature", publish_temperature_);
  declare_parameter<bool>("publish_diagnostics", publish_diagnostics_);

  declare_parameter<double>(
    "orientation_covariance_roll",
    orientation_covariance_roll_);
  declare_parameter<double>(
    "orientation_covariance_pitch",
    orientation_covariance_pitch_);
  declare_parameter<double>(
    "orientation_covariance_yaw",
    orientation_covariance_yaw_);

  declare_parameter<double>(
    "angular_velocity_covariance_x",
    angular_velocity_covariance_x_);
  declare_parameter<double>(
    "angular_velocity_covariance_y",
    angular_velocity_covariance_y_);
  declare_parameter<double>(
    "angular_velocity_covariance_z",
    angular_velocity_covariance_z_);

  declare_parameter<double>(
    "linear_acceleration_covariance_x",
    linear_acceleration_covariance_x_);
  declare_parameter<double>(
    "linear_acceleration_covariance_y",
    linear_acceleration_covariance_y_);
  declare_parameter<double>(
    "linear_acceleration_covariance_z",
    linear_acceleration_covariance_z_);
}

void ImuNode::load_parameters()
{
  frame_id_ = get_parameter("frame_id").as_string();
  imu_topic_ = get_parameter("imu_topic").as_string();
  imu_state_topic_ = get_parameter("imu_state_topic").as_string();
  diagnostics_topic_ = get_parameter("diagnostics_topic").as_string();

  i2c_bus_ = static_cast<int>(get_parameter("i2c_bus").as_int());
  i2c_address_ = static_cast<int>(get_parameter("i2c_address").as_int());
  mode_ = get_parameter("mode").as_string();

  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
  health_publish_rate_hz_ = get_parameter("health_publish_rate_hz").as_double();
  diagnostics_publish_rate_hz_ =
    get_parameter("diagnostics_publish_rate_hz").as_double();
  timestamp_fault_hold_s_ = get_parameter("timestamp_fault_hold_s").as_double();
  producer_rate_window_size_ = static_cast<std::size_t>(
    get_parameter("producer_rate_window_size").as_int());
  reset_on_start_ = get_parameter("reset_on_start").as_bool();
  calibration_restore_enabled_ =
    get_parameter("calibration_restore_enabled").as_bool();
  calibration_profile_path_ = get_parameter("calibration_profile_path").as_string();
  calibration_require_verified_restore_ =
    get_parameter("calibration_require_verified_restore").as_bool();
  calibration_save_service_name_ =
    get_parameter("calibration_save_service").as_string();

  publish_orientation_ = get_parameter("publish_orientation").as_bool();
  publish_magnetic_field_ = get_parameter("publish_magnetic_field").as_bool();
  publish_temperature_ = get_parameter("publish_temperature").as_bool();
  publish_diagnostics_ = get_parameter("publish_diagnostics").as_bool();

  orientation_covariance_roll_ =
    get_parameter("orientation_covariance_roll").as_double();
  orientation_covariance_pitch_ =
    get_parameter("orientation_covariance_pitch").as_double();
  orientation_covariance_yaw_ =
    get_parameter("orientation_covariance_yaw").as_double();

  angular_velocity_covariance_x_ =
    get_parameter("angular_velocity_covariance_x").as_double();
  angular_velocity_covariance_y_ =
    get_parameter("angular_velocity_covariance_y").as_double();
  angular_velocity_covariance_z_ =
    get_parameter("angular_velocity_covariance_z").as_double();

  linear_acceleration_covariance_x_ =
    get_parameter("linear_acceleration_covariance_x").as_double();
  linear_acceleration_covariance_y_ =
    get_parameter("linear_acceleration_covariance_y").as_double();
  linear_acceleration_covariance_z_ =
    get_parameter("linear_acceleration_covariance_z").as_double();

  if (frame_id_.empty()) {
    throw std::runtime_error("frame_id cannot be empty");
  }

  if (imu_topic_.empty()) {
    throw std::runtime_error("imu_topic cannot be empty");
  }

  if (imu_state_topic_.empty()) {
    throw std::runtime_error("imu_state_topic cannot be empty");
  }

  if (publish_rate_hz_ <= 0.0) {
    throw std::runtime_error("publish_rate_hz must be > 0.0");
  }
  if (health_publish_rate_hz_ <= 0.0 || diagnostics_publish_rate_hz_ <= 0.0) {
    throw std::runtime_error("health and diagnostics publish rates must be > 0.0");
  }
  if (timestamp_fault_hold_s_ <= 0.0) {
    throw std::runtime_error("timestamp_fault_hold_s must be > 0.0");
  }
  if (producer_rate_window_size_ < 3U) {
    throw std::runtime_error("producer_rate_window_size must be >= 3");
  }

  if (i2c_bus_ < 0) {
    throw std::runtime_error("i2c_bus must be >= 0");
  }

  if (i2c_address_ < 0x00 || i2c_address_ > 0x7F) {
    throw std::runtime_error("i2c_address must be a valid 7-bit I2C address");
  }

  if (calibration_profile_path_.empty()) {
    throw std::runtime_error("calibration_profile_path cannot be empty");
  }

  if (calibration_save_service_name_.empty()) {
    throw std::runtime_error("calibration_save_service cannot be empty");
  }
}

void ImuNode::configure_driver()
{
  driver_ = std::make_unique<BNO055Driver>(
    i2c_bus_,
    static_cast<uint8_t>(i2c_address_));

  const bool initialized = driver_->initialize_config_mode(reset_on_start_);

  if (!initialized) {
    throw std::runtime_error("failed to initialize BNO055 IMU");
  }

  calibration_runtime_ = std::make_unique<BNO055CalibrationRuntime>(
    calibration_profile_path_);
  calibration_runtime_->restore(
    *driver_,
    calibration_restore_enabled_,
    calibration_require_verified_restore_,
    calibration_metadata(false));

  driver_->set_mode(configured_mode());
  calibration_runtime_->verify_operational_status(*driver_);

  const auto & calibration_state = calibration_runtime_->state();
  if (calibration_state.restore_failed) {
    if (calibration_state.verification_required) {
      RCLCPP_ERROR(
        get_logger(),
        "BNO055 calibration restore failed: %s",
        calibration_state.error.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "BNO055 calibration restore failed in non-required mode: %s",
        calibration_state.error.c_str());
    }
  } else if (calibration_state.status == "profile_missing") {
    RCLCPP_WARN(
      get_logger(),
      "No saved BNO055 calibration profile at %s; continuing with live calibration",
      calibration_profile_path_.c_str());
  } else if (calibration_state.status == "restored") {
    RCLCPP_INFO(
      get_logger(),
      "BNO055 calibration profile restored and verified from %s",
      calibration_profile_path_.c_str());
  }

  RCLCPP_INFO(
    get_logger(),
    "BNO055 ready | chip_id=0x%02X | mode=%s",
    driver_->read_chip_id(),
    BNO055Driver::mode_name(configured_mode()).c_str());
}

void ImuNode::create_calibration_save_service()
{
  calibration_save_service_ = create_service<std_srvs::srv::Trigger>(
    calibration_save_service_name_,
    std::bind(
      &ImuNode::save_calibration_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}

void ImuNode::save_calibration_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!driver_ || !calibration_runtime_) {
    response->success = false;
    response->message = "BNO055 calibration runtime is unavailable";
    return;
  }
  if (!have_live_calibration_status_) {
    response->success = false;
    response->message = "calibration save rejected: no live BNO055 status received";
    return;
  }

  try {
    BNO055CalibrationMetadata metadata = calibration_metadata(true);
    const BNO055CalibrationCaptureResult result = calibration_runtime_->capture(
      *driver_, metadata);
    response->success = result.success;

    std::ostringstream message;
    message << result.message;
    if (result.success) {
      message << " | path=" << calibration_profile_path_
              << " | accel_offset=(" << result.profile.accel_offset_x << ','
              << result.profile.accel_offset_y << ','
              << result.profile.accel_offset_z << ')'
              << " | mag_offset=(" << result.profile.mag_offset_x << ','
              << result.profile.mag_offset_y << ','
              << result.profile.mag_offset_z << ')'
              << " | gyro_offset=(" << result.profile.gyro_offset_x << ','
              << result.profile.gyro_offset_y << ','
              << result.profile.gyro_offset_z << ')'
              << " | accel_radius=" << result.profile.accel_radius
              << " | mag_radius=" << result.profile.mag_radius;
    }
    response->message = message.str();
  } catch (const std::exception & exception) {
    response->success = false;
    response->message = "calibration save failed: " + std::string(exception.what());
  }
}

void ImuNode::timer_callback()
{
  try {
    const BNO055Sample sample = driver_->read_sample(
      publish_magnetic_field_,
      publish_orientation_,
      publish_temperature_);

    const auto imu_message = make_imu_msg(sample);
    imu_pub_->publish(imu_message);
    const std::int64_t publish_time_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    ++sample_count_;
    ++publish_count_;
    have_live_calibration_status_ = true;
    last_sample_ = sample;
    last_sample_data_valid_ = sample_data_valid(sample);
    read_error_active_ = false;

    if (producer_rate_tracker_.RecordSuccess(
        publish_time_ns, rclcpp::Time(imu_message.header.stamp).nanoseconds(),
        producer_rate_window_size_))
    {
      timestamp_fault_until_ns_ = publish_time_ns + static_cast<std::int64_t>(
        std::llround(timestamp_fault_hold_s_ * 1.0e9));
    }
    publish_health_outputs(publish_time_ns, false);
  } catch (const std::exception & exc) {
    ++error_count_;
    read_error_active_ = true;
    const std::int64_t failure_time_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    publish_health_outputs(failure_time_ns, false);

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "IMU read failed: %s",
      exc.what());
  }
}

void ImuNode::publish_health_outputs(
  const std::int64_t monotonic_time_ns,
  const bool force)
{
  const auto snapshot = make_health_snapshot(monotonic_time_ns);
  const bool transition = snapshot.health_state != last_health_state_ ||
    snapshot.reason != last_health_reason_;

  if (force || transition || output_due(
      monotonic_time_ns, last_health_publish_ns_, health_publish_rate_hz_))
  {
    publish_state(snapshot);
    last_health_publish_ns_ = monotonic_time_ns;
  }
  if (publish_diagnostics_ &&
    (force || transition || output_due(
      monotonic_time_ns, last_diagnostics_publish_ns_, diagnostics_publish_rate_hz_)))
  {
    publish_diagnostics(snapshot);
    last_diagnostics_publish_ns_ = monotonic_time_ns;
  }

  last_health_state_ = snapshot.health_state;
  last_health_reason_ = snapshot.reason;
}

void ImuNode::publish_state(const ProducerHealthSnapshot & snapshot)
{
  state_pub_->publish(make_state_msg(snapshot));
}

void ImuNode::publish_diagnostics(const ProducerHealthSnapshot & snapshot)
{
  if (diagnostics_pub_) {
    diagnostics_pub_->publish(make_diagnostic_msg(snapshot));
  }
}

sensor_msgs::msg::Imu ImuNode::make_imu_msg(const BNO055Sample & sample) const
{
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;

  if (publish_orientation_ && sample.euler_deg.available) {
    const double yaw_rad = deg_to_rad(sample.euler_deg.yaw_deg);
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = yaw_to_quaternion_z(yaw_rad);
    msg.orientation.w = yaw_to_quaternion_w(yaw_rad);
    msg.orientation_covariance = orientation_covariance();
  } else {
    msg.orientation.w = 1.0;
    msg.orientation_covariance = orientation_covariance();
    msg.orientation_covariance[0] = -1.0;
  }

  msg.angular_velocity.x = deg_to_rad(sample.gyro_dps.x);
  msg.angular_velocity.y = deg_to_rad(sample.gyro_dps.y);
  msg.angular_velocity.z = deg_to_rad(sample.gyro_dps.z);
  msg.angular_velocity_covariance = angular_velocity_covariance();

  msg.linear_acceleration.x = sample.accel_mps2.x;
  msg.linear_acceleration.y = sample.accel_mps2.y;
  msg.linear_acceleration.z = sample.accel_mps2.z;
  msg.linear_acceleration_covariance = linear_acceleration_covariance();

  return msg;
}

std_msgs::msg::String ImuNode::make_state_msg(
  const ProducerHealthSnapshot & snapshot) const
{
  std_msgs::msg::String msg;
  msg.data = SerializeProducerHealth(snapshot);
  return msg;
}

diagnostic_msgs::msg::DiagnosticArray ImuNode::make_diagnostic_msg(
  const ProducerHealthSnapshot & snapshot) const
{
  diagnostic_msgs::msg::DiagnosticArray array_msg;
  array_msg.header.stamp = now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "savo_localization/imu_node";
  status.hardware_id = "bno055";
  status.level = snapshot.health_state == "ERROR" ?
    diagnostic_msgs::msg::DiagnosticStatus::ERROR :
    snapshot.health_state == "DEGRADED" ?
    diagnostic_msgs::msg::DiagnosticStatus::WARN :
    diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = snapshot.reason;

  status.values.push_back(key_value("frame_id", frame_id_));
  status.values.push_back(key_value("imu_topic", imu_topic_));
  status.values.push_back(key_value("i2c_bus", std::to_string(i2c_bus_)));
  status.values.push_back(key_value("i2c_address", hex_u8(static_cast<uint8_t>(i2c_address_))));
  status.values.push_back(key_value("mode", mode_));
  status.values.push_back(key_value("chip_id", hex_u8(snapshot.chip_id)));
  status.values.push_back(key_value("system_status", std::to_string(snapshot.system_status)));
  status.values.push_back(key_value("system_error", std::to_string(snapshot.system_error)));
  status.values.push_back(key_value("calib_system", std::to_string(snapshot.calibration_system)));
  status.values.push_back(key_value("calib_gyro", std::to_string(snapshot.calibration_gyro)));
  status.values.push_back(key_value("calib_accel", std::to_string(snapshot.calibration_accel)));
  status.values.push_back(key_value("calib_mag", std::to_string(snapshot.calibration_mag)));
  const auto & calibration_state = calibration_runtime_->state();
  status.values.push_back(key_value(
    "calibration_restore_enabled", bool_text(calibration_state.restore_enabled)));
  status.values.push_back(key_value("calibration_profile_path", calibration_profile_path_));
  status.values.push_back(key_value(
    "calibration_profile_present", bool_text(calibration_state.profile_present)));
  status.values.push_back(key_value(
    "calibration_profile_loaded", bool_text(calibration_state.profile_loaded)));
  status.values.push_back(key_value(
    "calibration_restore_attempted", bool_text(calibration_state.restore_attempted)));
  status.values.push_back(key_value(
    "calibration_profile_verified", bool_text(calibration_state.profile_verified)));
  status.values.push_back(key_value(
    "calibration_operational_status_verified",
    bool_text(calibration_state.operational_status_verified)));
  status.values.push_back(key_value(
    "calibration_restore_failed", bool_text(calibration_state.restore_failed)));
  status.values.push_back(key_value("calibration_restore_status", calibration_state.status));
  status.values.push_back(key_value("calibration_restore_error", calibration_state.error));
  status.values.push_back(key_value("sample_count", std::to_string(sample_count_)));
  status.values.push_back(key_value("publish_count", std::to_string(publish_count_)));
  status.values.push_back(key_value("error_count", std::to_string(error_count_)));
  status.values.push_back(key_value(
    "producer_rate_hz", std::to_string(snapshot.producer_rate_hz)));
  status.values.push_back(key_value(
    "producer_rate_quality", snapshot.rate_quality));
  status.values.push_back(key_value(
    "last_success_age_s", std::to_string(snapshot.last_success_age_s)));

  array_msg.status.push_back(status);
  return array_msg;
}

ProducerHealthSnapshot ImuNode::make_health_snapshot(
  const std::int64_t monotonic_time_ns) const
{
  ProducerHealthSnapshot snapshot;
  snapshot.node = "imu_node";
  snapshot.frame_id = frame_id_;
  snapshot.frame_valid = !frame_id_.empty();
  snapshot.timestamp_valid = monotonic_time_ns >= timestamp_fault_until_ns_;
  snapshot.sample_count = sample_count_;
  snapshot.publish_count = publish_count_;
  snapshot.error_count = error_count_;

  const auto rate = producer_rate_tracker_.Observe(monotonic_time_ns, publish_rate_hz_);
  snapshot.producer_rate_available = rate.available;
  snapshot.producer_rate_hz = rate.rate_hz;
  snapshot.last_success_age_s = rate.last_success_age_s;
  snapshot.rate_quality = std::string(ProducerRateTracker::QualityString(rate.quality));

  if (!last_sample_) {
    snapshot.health_state = read_error_active_ ? "ERROR" : "INITIALIZING";
    snapshot.reason = read_error_active_ ? "imu_read_failed" : "waiting_for_first_success";
    return snapshot;
  }

  const auto & sample = *last_sample_;
  const auto & calibration_state = calibration_runtime_->state();
  snapshot.data_valid = last_sample_data_valid_;
  snapshot.chip_id = sample.status.chip_id;
  snapshot.system_status = sample.status.system_status;
  snapshot.system_error = sample.status.system_error;
  snapshot.calibration_system = sample.status.calibration.system;
  snapshot.calibration_gyro = sample.status.calibration.gyro;
  snapshot.calibration_accel = sample.status.calibration.accel;
  snapshot.calibration_mag = sample.status.calibration.mag;
  snapshot.motion_ready = sample.status.calibration.motion_ready();
  snapshot.hardware_ok = sample.status.chip_id == BNO055_CHIP_ID &&
    sample.status.system_error == 0 &&
    !(calibration_state.restore_failed && calibration_state.verification_required);

  if (read_error_active_) {
    snapshot.health_state = "ERROR";
    snapshot.reason = "imu_read_failed";
  } else if (!snapshot.timestamp_valid) {
    snapshot.health_state = "ERROR";
    snapshot.reason = "imu_timestamp_regression";
  } else if (!snapshot.data_valid) {
    snapshot.health_state = "ERROR";
    snapshot.reason = "imu_data_invalid";
  } else if (!snapshot.hardware_ok) {
    snapshot.health_state = "ERROR";
    snapshot.reason = diagnostic_message_from_sample(sample);
  } else if (diagnostic_level_from_sample(sample) ==
    diagnostic_msgs::msg::DiagnosticStatus::WARN)
  {
    snapshot.health_state = "DEGRADED";
    snapshot.reason = diagnostic_message_from_sample(sample);
  } else {
    snapshot.health_state = "OK";
    snapshot.reason = "IMU healthy";
  }
  return snapshot;
}

bool ImuNode::sample_data_valid(const BNO055Sample & sample) const
{
  return std::isfinite(sample.accel_mps2.x) &&
         std::isfinite(sample.accel_mps2.y) &&
         std::isfinite(sample.accel_mps2.z) &&
         std::isfinite(sample.gyro_dps.x) &&
         std::isfinite(sample.gyro_dps.y) &&
         std::isfinite(sample.gyro_dps.z) &&
         (!sample.euler_deg.available ||
         (std::isfinite(sample.euler_deg.yaw_deg) &&
         std::isfinite(sample.euler_deg.roll_deg) &&
         std::isfinite(sample.euler_deg.pitch_deg)));
}

bool ImuNode::output_due(
  const std::int64_t monotonic_time_ns,
  const std::int64_t last_publish_time_ns,
  const double rate_hz) const
{
  if (last_publish_time_ns < 0) {
    return true;
  }
  const auto period_ns = static_cast<std::int64_t>(std::llround(1.0e9 / rate_hz));
  return monotonic_time_ns - last_publish_time_ns >= period_ns;
}

BNO055Mode ImuNode::configured_mode() const
{
  if (mode_ == "imu" || mode_ == "IMU") {
    return BNO055Mode::IMU;
  }

  if (mode_ == "ndof" || mode_ == "NDOF") {
    return BNO055Mode::NDOF;
  }

  throw std::runtime_error("unsupported BNO055 mode: " + mode_);
}

BNO055CalibrationMetadata ImuNode::calibration_metadata(
  const bool include_timestamp) const
{
  BNO055CalibrationMetadata metadata;
  metadata.i2c_bus = i2c_bus_;
  metadata.i2c_address = static_cast<uint8_t>(i2c_address_);
  metadata.operational_mode = configured_mode();
  if (include_timestamp) {
    metadata.captured_at = bno055_calibration_timestamp_utc();
  }
  return metadata;
}

std::array<double, 9> ImuNode::orientation_covariance() const
{
  return covariance3_from_diagonal(
    orientation_covariance_roll_,
    orientation_covariance_pitch_,
    orientation_covariance_yaw_);
}

std::array<double, 9> ImuNode::angular_velocity_covariance() const
{
  return covariance3_from_diagonal(
    angular_velocity_covariance_x_,
    angular_velocity_covariance_y_,
    angular_velocity_covariance_z_);
}

std::array<double, 9> ImuNode::linear_acceleration_covariance() const
{
  return covariance3_from_diagonal(
    linear_acceleration_covariance_x_,
    linear_acceleration_covariance_y_,
    linear_acceleration_covariance_z_);
}

std::array<double, 9> ImuNode::covariance3_from_diagonal(
  double x,
  double y,
  double z)
{
  return {
    x, 0.0, 0.0,
    0.0, y, 0.0,
    0.0, 0.0, z,
  };
}

double ImuNode::deg_to_rad(double value_deg)
{
  return value_deg * PI / 180.0;
}

double ImuNode::yaw_to_quaternion_z(double yaw_rad)
{
  return std::sin(yaw_rad * 0.5);
}

double ImuNode::yaw_to_quaternion_w(double yaw_rad)
{
  return std::cos(yaw_rad * 0.5);
}

int ImuNode::diagnostic_level_from_sample(const BNO055Sample & sample) const
{
  if (sample.status.chip_id != BNO055_CHIP_ID || sample.status.system_error != 0) {
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }

  const auto & calibration_state = calibration_runtime_->state();
  if (calibration_state.restore_failed) {
    return calibration_state.verification_required ?
           diagnostic_msgs::msg::DiagnosticStatus::ERROR :
           diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  if (!sample.status.calibration.motion_ready()) {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  return diagnostic_msgs::msg::DiagnosticStatus::OK;
}

std::string ImuNode::diagnostic_message_from_sample(
  const BNO055Sample & sample) const
{
  if (sample.status.chip_id != BNO055_CHIP_ID) {
    return "unexpected BNO055 chip id";
  }

  if (sample.status.system_error != 0) {
    return "BNO055 system error";
  }

  const auto & calibration_state = calibration_runtime_->state();
  if (calibration_state.restore_failed) {
    return calibration_state.verification_required ?
           "BNO055 calibration restore failed" :
           "IMU usable, calibration restore failed";
  }

  if (!sample.status.calibration.motion_ready()) {
    return "IMU usable, calibration not fully ready";
  }

  return "IMU healthy";
}

}  // namespace savo_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<savo_localization::ImuNode>());
  } catch (const std::exception & exc) {
    RCLCPP_FATAL(
      rclcpp::get_logger("imu_node"),
      "Fatal error: %s",
      exc.what());
  }

  rclcpp::shutdown();
  return 0;
}
