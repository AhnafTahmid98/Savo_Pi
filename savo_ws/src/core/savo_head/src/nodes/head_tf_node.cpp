#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "savo_head/core/diagnostics.hpp"
#include "savo_head/core/head_types.hpp"

namespace savo_head
{

namespace
{

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

diagnostic_msgs::msg::KeyValue kv(const std::string & key, bool value)
{
  return kv(key, value ? "true" : "false");
}

diagnostic_msgs::msg::KeyValue kv(const std::string & key, double value)
{
  return kv(key, std::to_string(value));
}

std::uint8_t ros_level(DiagnosticLevel level)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  switch (level) {
    case DiagnosticLevel::kOk:
      return DiagnosticStatus::OK;
    case DiagnosticLevel::kWarn:
      return DiagnosticStatus::WARN;
    case DiagnosticLevel::kError:
      return DiagnosticStatus::ERROR;
  }

  return DiagnosticStatus::ERROR;
}

Quaternion quat_from_rpy(double roll, double pitch, double yaw)
{
  const auto cr = std::cos(roll * 0.5);
  const auto sr = std::sin(roll * 0.5);
  const auto cp = std::cos(pitch * 0.5);
  const auto sp = std::sin(pitch * 0.5);
  const auto cy = std::cos(yaw * 0.5);
  const auto sy = std::sin(yaw * 0.5);

  return Quaternion{
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy
  };
}

Quaternion quat_multiply(const Quaternion & a, const Quaternion & b)
{
  return Quaternion{
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
  };
}

Quaternion quat_from_axis_angle(const std::string & axis, double angle_rad)
{
  const auto half = angle_rad * 0.5;
  const auto s = std::sin(half);
  const auto c = std::cos(half);

  if (axis == "x") {
    return Quaternion{s, 0.0, 0.0, c};
  }

  if (axis == "y") {
    return Quaternion{0.0, s, 0.0, c};
  }

  if (axis == "z") {
    return Quaternion{0.0, 0.0, s, c};
  }

  throw std::invalid_argument("unsupported rotation axis: " + axis);
}

Quaternion combined_rotation(
  const std::vector<double> & base_rpy,
  const std::string & axis,
  double joint_angle_rad)
{
  if (base_rpy.size() != 3U) {
    throw std::invalid_argument("rpy parameter must have 3 values");
  }

  const auto base = quat_from_rpy(base_rpy[0], base_rpy[1], base_rpy[2]);
  const auto joint = quat_from_axis_angle(axis, joint_angle_rad);

  return quat_multiply(base, joint);
}

geometry_msgs::msg::TransformStamped make_transform(
  const rclcpp::Time & stamp,
  const std::string & parent,
  const std::string & child,
  const std::vector<double> & xyz,
  const Quaternion & quat)
{
  if (xyz.size() != 3U) {
    throw std::invalid_argument("xyz parameter must have 3 values");
  }

  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent;
  msg.child_frame_id = child;

  msg.transform.translation.x = xyz[0];
  msg.transform.translation.y = xyz[1];
  msg.transform.translation.z = xyz[2];

  msg.transform.rotation.x = quat.x;
  msg.transform.rotation.y = quat.y;
  msg.transform.rotation.z = quat.z;
  msg.transform.rotation.w = quat.w;

  return msg;
}

double degrees_to_radians(double degrees)
{
  return degrees * M_PI / 180.0;
}

}  // namespace

class HeadTfNode : public rclcpp::Node
{
public:
  HeadTfNode()
  : Node("head_tf_node")
  {
    declare_parameters();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("pan_tilt_state_topic").as_string(),
      rclcpp::QoS(10).reliable(),
      std::bind(&HeadTfNode::on_joint_state, this, std::placeholders::_1));

    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      get_parameter("status_topic").as_string(),
      rclcpp::QoS(10).reliable());

    dashboard_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("dashboard_text_topic").as_string(),
      rclcpp::QoS(10).reliable());

    const auto tf_rate_hz = std::max(1.0, get_parameter("tf_rate_hz").as_double());
    const auto status_hz = std::max(0.2, get_parameter("status_hz").as_double());

    tf_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / tf_rate_hz)),
      std::bind(&HeadTfNode::publish_tf, this));

    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / status_hz)),
      std::bind(&HeadTfNode::publish_status, this));

    RCLCPP_INFO(get_logger(), "head TF node started");
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("pan_tilt_state_topic", kTopicPanTiltState);
    declare_parameter<std::string>("status_topic", kTopicStatus);
    declare_parameter<std::string>("dashboard_text_topic", kTopicDashboardText);

    declare_parameter<bool>("publish_tf", true);
    declare_parameter<double>("tf_rate_hz", kTfHz);
    declare_parameter<double>("status_hz", kStatusHz);

    declare_parameter<std::string>("base_frame", kFrameBaseLink);
    declare_parameter<std::string>("pan_frame", kFramePanLink);
    declare_parameter<std::string>("tilt_frame", kFrameTiltLink);
    declare_parameter<std::string>("camera_frame", kFrameCameraLink);
    declare_parameter<std::string>("camera_optical_frame", kFrameCameraOptical);

    declare_parameter<std::string>("pan_joint_name", kJointPan);
    declare_parameter<std::string>("tilt_joint_name", kJointTilt);

    declare_parameter<std::string>("pan_axis", "z");
    declare_parameter<std::string>("tilt_axis", "y");
    declare_parameter<double>("pan_sign", 1.0);
    declare_parameter<double>("tilt_sign", 1.0);

    declare_parameter<double>("pan_zero_deg", static_cast<double>(kPanCenterDeg));
    declare_parameter<double>("tilt_zero_deg", static_cast<double>(kTiltCenterDeg));

    declare_parameter<std::vector<double>>("base_to_pan_xyz_m", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("base_to_pan_rpy_rad", std::vector<double>{0.0, 0.0, 0.0});

    declare_parameter<std::vector<double>>("pan_to_tilt_xyz_m", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("pan_to_tilt_rpy_rad", std::vector<double>{0.0, 0.0, 0.0});

    declare_parameter<std::vector<double>>("tilt_to_camera_xyz_m", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("tilt_to_camera_rpy_rad", std::vector<double>{0.0, 0.0, 0.0});

    declare_parameter<std::vector<double>>("camera_to_optical_xyz_m", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>(
      "camera_to_optical_rpy_rad",
      std::vector<double>{-1.57079632679, 0.0, -1.57079632679});

    declare_parameter<bool>("require_valid_pan_tilt_state", true);
    declare_parameter<double>("stale_state_timeout_s", 0.50);
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_state_ = *msg;
    last_state_s_ = now_s();
    last_error_.reset();
  }

  void publish_tf()
  {
    if (!get_parameter("publish_tf").as_bool()) {
      return;
    }

    try {
      const auto [pan_rad, tilt_rad] = current_joint_angles();
      const auto stamp = now();

      std::vector<geometry_msgs::msg::TransformStamped> transforms;
      transforms.reserve(4U);

      transforms.push_back(base_to_pan(stamp, pan_rad));
      transforms.push_back(pan_to_tilt(stamp, tilt_rad));
      transforms.push_back(tilt_to_camera(stamp));
      transforms.push_back(camera_to_optical(stamp));

      tf_broadcaster_->sendTransform(transforms);
      last_error_.reset();
    } catch (const std::exception & exc) {
      last_error_ = exc.what();
    }
  }

  std::pair<double, double> current_joint_angles() const
  {
    if (!last_joint_state_.has_value()) {
      if (get_parameter("require_valid_pan_tilt_state").as_bool()) {
        throw std::runtime_error("no pan-tilt joint state received");
      }

      return {0.0, 0.0};
    }

    const auto timeout_s = get_parameter("stale_state_timeout_s").as_double();
    if ((now_s() - last_state_s_) > timeout_s) {
      if (get_parameter("require_valid_pan_tilt_state").as_bool()) {
        throw std::runtime_error("pan-tilt joint state is stale");
      }
    }

    const auto & msg = last_joint_state_.value();

    const auto pan_abs = joint_position(
      msg,
      get_parameter("pan_joint_name").as_string(),
      0U);

    const auto tilt_abs = joint_position(
      msg,
      get_parameter("tilt_joint_name").as_string(),
      1U);

    const auto pan_zero = degrees_to_radians(get_parameter("pan_zero_deg").as_double());
    const auto tilt_zero = degrees_to_radians(get_parameter("tilt_zero_deg").as_double());

    const auto pan =
      get_parameter("pan_sign").as_double() * (pan_abs - pan_zero);

    const auto tilt =
      get_parameter("tilt_sign").as_double() * (tilt_abs - tilt_zero);

    return {pan, tilt};
  }

  static double joint_position(
    const sensor_msgs::msg::JointState & msg,
    const std::string & name,
    std::size_t fallback_index)
  {
    const auto found = std::find(msg.name.begin(), msg.name.end(), name);
    if (found != msg.name.end()) {
      const auto index = static_cast<std::size_t>(std::distance(msg.name.begin(), found));
      if (index < msg.position.size()) {
        return msg.position[index];
      }
    }

    if (fallback_index < msg.position.size()) {
      return msg.position[fallback_index];
    }

    throw std::runtime_error("joint position unavailable for " + name);
  }

  geometry_msgs::msg::TransformStamped base_to_pan(
    const rclcpp::Time & stamp,
    double pan_rad) const
  {
    const auto quat = combined_rotation(
      get_parameter("base_to_pan_rpy_rad").as_double_array(),
      get_parameter("pan_axis").as_string(),
      pan_rad);

    return make_transform(
      stamp,
      get_parameter("base_frame").as_string(),
      get_parameter("pan_frame").as_string(),
      get_parameter("base_to_pan_xyz_m").as_double_array(),
      quat);
  }

  geometry_msgs::msg::TransformStamped pan_to_tilt(
    const rclcpp::Time & stamp,
    double tilt_rad) const
  {
    const auto quat = combined_rotation(
      get_parameter("pan_to_tilt_rpy_rad").as_double_array(),
      get_parameter("tilt_axis").as_string(),
      tilt_rad);

    return make_transform(
      stamp,
      get_parameter("pan_frame").as_string(),
      get_parameter("tilt_frame").as_string(),
      get_parameter("pan_to_tilt_xyz_m").as_double_array(),
      quat);
  }

  geometry_msgs::msg::TransformStamped tilt_to_camera(const rclcpp::Time & stamp) const
  {
    const auto rpy = get_parameter("tilt_to_camera_rpy_rad").as_double_array();

    if (rpy.size() != 3U) {
      throw std::runtime_error("tilt_to_camera_rpy_rad must have 3 values");
    }

    return make_transform(
      stamp,
      get_parameter("tilt_frame").as_string(),
      get_parameter("camera_frame").as_string(),
      get_parameter("tilt_to_camera_xyz_m").as_double_array(),
      quat_from_rpy(rpy[0], rpy[1], rpy[2]));
  }

  geometry_msgs::msg::TransformStamped camera_to_optical(const rclcpp::Time & stamp) const
  {
    const auto rpy = get_parameter("camera_to_optical_rpy_rad").as_double_array();

    if (rpy.size() != 3U) {
      throw std::runtime_error("camera_to_optical_rpy_rad must have 3 values");
    }

    return make_transform(
      stamp,
      get_parameter("camera_frame").as_string(),
      get_parameter("camera_optical_frame").as_string(),
      get_parameter("camera_to_optical_xyz_m").as_double_array(),
      quat_from_rpy(rpy[0], rpy[1], rpy[2]));
  }

  void publish_status()
  {
    const auto health = current_health();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "savo_head.head_tf";
    status.hardware_id = "savo_head";
    status.level = ros_level(health.level);
    status.message = health.message;

    status.values = {
      kv("base_frame", get_parameter("base_frame").as_string()),
      kv("pan_frame", get_parameter("pan_frame").as_string()),
      kv("tilt_frame", get_parameter("tilt_frame").as_string()),
      kv("camera_frame", get_parameter("camera_frame").as_string()),
      kv("camera_optical_frame", get_parameter("camera_optical_frame").as_string()),
      kv("has_joint_state", last_joint_state_.has_value()),
      kv("last_state_age_s", last_state_age_s()),
      kv("last_error", last_error_.value_or(""))
    };

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status = {status};

    status_pub_->publish(array);

    std_msgs::msg::String dashboard;
    dashboard.data = dashboard_text(health) + " " + tf_chain_text();
    dashboard_pub_->publish(dashboard);
  }

  HeadHealthSummary current_health() const
  {
    std::vector<ComponentHealth> components;

    const auto stale = check_stale(
      now_s(),
      last_state_s_,
      get_parameter("stale_state_timeout_s").as_double(),
      get_parameter("require_valid_pan_tilt_state").as_bool());

    auto state_health = health_from_stale_check(
      "savo_head.tf_state",
      stale,
      get_parameter("require_valid_pan_tilt_state").as_bool());

    if (last_error_.has_value() && !last_error_->empty()) {
      state_health = error_component("savo_head.tf", *last_error_);
    }

    state_health.add_value("publish_tf", get_parameter("publish_tf").as_bool());
    state_health.add_value("last_state_age_s", last_state_age_s());

    components.push_back(state_health);

    return summarize_health(components, true);
  }

  std::string tf_chain_text() const
  {
    std::ostringstream stream;
    stream << get_parameter("base_frame").as_string()
           << "->" << get_parameter("pan_frame").as_string()
           << "->" << get_parameter("tilt_frame").as_string()
           << "->" << get_parameter("camera_frame").as_string()
           << "->" << get_parameter("camera_optical_frame").as_string();

    return stream.str();
  }

  double last_state_age_s() const
  {
    if (!last_joint_state_.has_value() || last_state_s_ <= 0.0) {
      return 0.0;
    }

    return std::max(0.0, now_s() - last_state_s_);
  }

  double now_s() const
  {
    return now().seconds();
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{};
  std::optional<sensor_msgs::msg::JointState> last_joint_state_{};
  std::optional<std::string> last_error_{};
  double last_state_s_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_{};

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dashboard_pub_{};

  rclcpp::TimerBase::SharedPtr tf_timer_{};
  rclcpp::TimerBase::SharedPtr status_timer_{};
};

}  // namespace savo_head

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<savo_head::HeadTfNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    std::cerr << "head_tf_node failed: " << exc.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
