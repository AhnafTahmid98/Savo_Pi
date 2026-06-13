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

std::string escape_json(const std::string & text)
{
  std::ostringstream oss;

  for (const char c : text) {
    switch (c) {
      case '"':
        oss << "\\\"";
        break;
      case '\\':
        oss << "\\\\";
        break;
      case '\n':
        oss << "\\n";
        break;
      case '\r':
        oss << "\\r";
        break;
      case '\t':
        oss << "\\t";
        break;
      default:
        oss << c;
        break;
    }
  }

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

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    imu_topic_,
    rclcpp::SensorDataQoS());

  state_pub_ = create_publisher<std_msgs::msg::String>(
    imu_state_topic_,
    rclcpp::QoS(10).reliable());

  if (publish_diagnostics_) {
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_,
      rclcpp::QoS(10).reliable());
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
  declare_parameter<bool>("reset_on_start", reset_on_start_);

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
  reset_on_start_ = get_parameter("reset_on_start").as_bool();

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

  if (i2c_bus_ < 0) {
    throw std::runtime_error("i2c_bus must be >= 0");
  }

  if (i2c_address_ < 0x00 || i2c_address_ > 0x7F) {
    throw std::runtime_error("i2c_address must be a valid 7-bit I2C address");
  }
}

void ImuNode::configure_driver()
{
  driver_ = std::make_unique<BNO055Driver>(
    i2c_bus_,
    static_cast<uint8_t>(i2c_address_));

  const bool initialized = driver_->initialize(
    configured_mode(),
    reset_on_start_);

  if (!initialized) {
    throw std::runtime_error("failed to initialize BNO055 IMU");
  }

  RCLCPP_INFO(
    get_logger(),
    "BNO055 ready | chip_id=0x%02X | mode=%s",
    driver_->read_chip_id(),
    BNO055Driver::mode_name(configured_mode()).c_str());
}

void ImuNode::timer_callback()
{
  try {
    const BNO055Sample sample = driver_->read_sample(
      publish_magnetic_field_,
      publish_orientation_,
      publish_temperature_);

    publish_imu(sample);
    publish_state(sample);

    if (publish_diagnostics_) {
      publish_diagnostics(sample);
    }

    ++sample_count_;
    ++publish_count_;
    last_sample_time_ = now();
    have_last_sample_ = true;
  } catch (const std::exception & exc) {
    ++error_count_;

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "IMU read failed: %s",
      exc.what());
  }
}

void ImuNode::publish_imu(const BNO055Sample & sample)
{
  imu_pub_->publish(make_imu_msg(sample));
}

void ImuNode::publish_state(const BNO055Sample & sample)
{
  state_pub_->publish(make_state_msg(sample));
}

void ImuNode::publish_diagnostics(const BNO055Sample & sample)
{
  if (diagnostics_pub_) {
    diagnostics_pub_->publish(make_diagnostic_msg(sample));
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

std_msgs::msg::String ImuNode::make_state_msg(const BNO055Sample & sample) const
{
  std_msgs::msg::String msg;
  msg.data = make_state_json(sample);
  return msg;
}

diagnostic_msgs::msg::DiagnosticArray ImuNode::make_diagnostic_msg(
  const BNO055Sample & sample) const
{
  diagnostic_msgs::msg::DiagnosticArray array_msg;
  array_msg.header.stamp = now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "savo_localization/imu_node";
  status.hardware_id = "bno055";
  status.level = diagnostic_level_from_sample(sample);
  status.message = diagnostic_message_from_sample(sample);

  status.values.push_back(key_value("frame_id", frame_id_));
  status.values.push_back(key_value("imu_topic", imu_topic_));
  status.values.push_back(key_value("i2c_bus", std::to_string(i2c_bus_)));
  status.values.push_back(key_value("i2c_address", hex_u8(static_cast<uint8_t>(i2c_address_))));
  status.values.push_back(key_value("mode", mode_));
  status.values.push_back(key_value("chip_id", hex_u8(sample.status.chip_id)));
  status.values.push_back(key_value("system_status", std::to_string(sample.status.system_status)));
  status.values.push_back(key_value("system_error", std::to_string(sample.status.system_error)));
  status.values.push_back(key_value("calib_system", std::to_string(sample.status.calibration.system)));
  status.values.push_back(key_value("calib_gyro", std::to_string(sample.status.calibration.gyro)));
  status.values.push_back(key_value("calib_accel", std::to_string(sample.status.calibration.accel)));
  status.values.push_back(key_value("calib_mag", std::to_string(sample.status.calibration.mag)));
  status.values.push_back(key_value("sample_count", std::to_string(sample_count_)));
  status.values.push_back(key_value("publish_count", std::to_string(publish_count_)));
  status.values.push_back(key_value("error_count", std::to_string(error_count_)));

  array_msg.status.push_back(status);
  return array_msg;
}

std::string ImuNode::make_state_json(const BNO055Sample & sample) const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6);

  oss << "{";
  oss << "\"node\":\"imu_node\",";
  oss << "\"status\":\"" << escape_json(diagnostic_message_from_sample(sample)) << "\",";
  oss << "\"frame_id\":\"" << escape_json(frame_id_) << "\",";
  oss << "\"imu_topic\":\"" << escape_json(imu_topic_) << "\",";
  oss << "\"i2c_bus\":" << i2c_bus_ << ",";
  oss << "\"i2c_address\":\"" << hex_u8(static_cast<uint8_t>(i2c_address_)) << "\",";
  oss << "\"mode\":\"" << escape_json(mode_) << "\",";
  oss << "\"chip_id\":\"" << hex_u8(sample.status.chip_id) << "\",";
  oss << "\"chip_ok\":" << bool_text(sample.status.chip_id == BNO055_CHIP_ID) << ",";
  oss << "\"system_status\":" << static_cast<int>(sample.status.system_status) << ",";
  oss << "\"system_error\":" << static_cast<int>(sample.status.system_error) << ",";

  oss << "\"calibration\":{";
  oss << "\"system\":" << sample.status.calibration.system << ",";
  oss << "\"gyro\":" << sample.status.calibration.gyro << ",";
  oss << "\"accel\":" << sample.status.calibration.accel << ",";
  oss << "\"mag\":" << sample.status.calibration.mag << ",";
  oss << "\"motion_ready\":" << bool_text(sample.status.calibration.motion_ready()) << ",";
  oss << "\"fully_calibrated\":" << bool_text(sample.status.calibration.fully_calibrated());
  oss << "},";

  oss << "\"accel_mps2\":{";
  oss << "\"x\":" << sample.accel_mps2.x << ",";
  oss << "\"y\":" << sample.accel_mps2.y << ",";
  oss << "\"z\":" << sample.accel_mps2.z;
  oss << "},";

  oss << "\"gyro_dps\":{";
  oss << "\"x\":" << sample.gyro_dps.x << ",";
  oss << "\"y\":" << sample.gyro_dps.y << ",";
  oss << "\"z\":" << sample.gyro_dps.z;
  oss << "},";

  oss << "\"euler_deg\":{";
  oss << "\"available\":" << bool_text(sample.euler_deg.available) << ",";
  oss << "\"yaw\":" << sample.euler_deg.yaw_deg << ",";
  oss << "\"roll\":" << sample.euler_deg.roll_deg << ",";
  oss << "\"pitch\":" << sample.euler_deg.pitch_deg;
  oss << "},";

  oss << "\"mag_ut\":{";
  oss << "\"available\":" << bool_text(sample.magnetic_available) << ",";
  oss << "\"x\":" << sample.mag_ut.x << ",";
  oss << "\"y\":" << sample.mag_ut.y << ",";
  oss << "\"z\":" << sample.mag_ut.z;
  oss << "},";

  oss << "\"temperature\":{";
  oss << "\"available\":" << bool_text(sample.temperature_available) << ",";
  oss << "\"c\":" << sample.temperature_c;
  oss << "},";

  oss << "\"sample_count\":" << sample_count_ << ",";
  oss << "\"publish_count\":" << publish_count_ << ",";
  oss << "\"error_count\":" << error_count_;
  oss << "}";

  return oss.str();
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

int ImuNode::diagnostic_level_from_sample(const BNO055Sample & sample)
{
  if (sample.status.chip_id != BNO055_CHIP_ID || sample.status.system_error != 0) {
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }

  if (!sample.status.calibration.motion_ready()) {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  return diagnostic_msgs::msg::DiagnosticStatus::OK;
}

std::string ImuNode::diagnostic_message_from_sample(const BNO055Sample & sample)
{
  if (sample.status.chip_id != BNO055_CHIP_ID) {
    return "unexpected BNO055 chip id";
  }

  if (sample.status.system_error != 0) {
    return "BNO055 system error";
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