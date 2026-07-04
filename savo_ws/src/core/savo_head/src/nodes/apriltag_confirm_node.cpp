#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_head/core/diagnostics.hpp"
#include "savo_head/core/head_types.hpp"

namespace savo_head
{

namespace
{

inline constexpr const char * kAprilTagFamilyDefault = "tag36h11";
inline constexpr const char * kSemanticConfirmationSource = "savo_head";
inline constexpr const char * kConfirmationTypeKnownLocation = "known_location";

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct TagRegistration
{
  int tag_id{-1};
  std::string label{};
  std::string confirmation_type{kConfirmationTypeKnownLocation};
  bool enabled{true};
  bool save_as_summon_point{false};
  std::vector<std::string> aliases{};

  [[nodiscard]] bool valid() const
  {
    return tag_id >= 0 && enabled;
  }
};

struct AprilTagObservation
{
  int tag_id{-1};
  std::string family{kAprilTagFamilyDefault};
  double confidence{0.0};
  double distance_m{0.0};
  int stable_frames{0};
  double stamp_s{0.0};
  std::string frame_id{kFrameCameraOptical};
  Vec3 pose_camera_xyz_m{};
  Vec3 pose_camera_rpy_rad{};
};

struct RobotPoseSnapshot
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  std::string frame_id{"map"};
  std::string base_frame{kFrameBaseLink};
  double stamp_s{0.0};
  double linear_speed_mps{0.0};
  double angular_speed_radps{0.0};
  double pose_covariance_xy{0.0};
  double yaw_covariance{0.0};
  bool localization_ok{true};
  bool lidar_map_pose_ok{true};
  bool tf_ok{true};
};

struct ConfirmationPolicy
{
  int min_stable_frames{5};
  double min_detection_confidence{0.70};
  double max_detection_distance_m{3.0};
  double max_detection_age_s{0.50};

  bool require_tf_available{true};
  bool require_robot_stationary{true};
  double max_robot_linear_speed_mps{0.03};
  double max_robot_angular_speed_radps{0.05};

  bool require_localization_ok{true};
  double max_pose_covariance_xy{0.25};
  double max_yaw_covariance{0.20};

  bool require_lidar_map_pose{true};
  bool require_semantic_label{true};

  [[nodiscard]] std::vector<std::string> rejection_reasons(
    const AprilTagObservation & observation,
    const std::optional<RobotPoseSnapshot> & robot_pose,
    const std::optional<TagRegistration> & registration,
    double now_s) const
  {
    std::vector<std::string> reasons;

    if (!registration.has_value()) {
      reasons.emplace_back("unknown_tag");
    } else {
      if (!registration->enabled) {
        reasons.emplace_back("tag_disabled");
      }

      if (require_semantic_label && registration->label.empty()) {
        reasons.emplace_back("semantic_label_missing");
      }
    }

    if (observation.stable_frames < min_stable_frames) {
      reasons.emplace_back("not_enough_stable_frames");
    }

    if (observation.confidence < min_detection_confidence) {
      reasons.emplace_back("low_detection_confidence");
    }

    if (observation.distance_m > max_detection_distance_m) {
      reasons.emplace_back("detection_too_far");
    }

    if (observation.stamp_s <= 0.0 || (now_s - observation.stamp_s) > max_detection_age_s) {
      reasons.emplace_back("detection_stale");
    }

    if (!robot_pose.has_value()) {
      reasons.emplace_back("robot_pose_missing");
      return reasons;
    }

    const auto & pose = robot_pose.value();

    if (require_tf_available && !pose.tf_ok) {
      reasons.emplace_back("tf_unavailable");
    }

    if (require_robot_stationary) {
      if (std::abs(pose.linear_speed_mps) > max_robot_linear_speed_mps ||
        std::abs(pose.angular_speed_radps) > max_robot_angular_speed_radps)
      {
        reasons.emplace_back("robot_not_stationary");
      }
    }

    if (require_localization_ok && !pose.localization_ok) {
      reasons.emplace_back("localization_not_ok");
    }

    if (pose.pose_covariance_xy > max_pose_covariance_xy) {
      reasons.emplace_back("pose_covariance_too_high");
    }

    if (pose.yaw_covariance > max_yaw_covariance) {
      reasons.emplace_back("yaw_covariance_too_high");
    }

    if (require_lidar_map_pose && !pose.lidar_map_pose_ok) {
      reasons.emplace_back("lidar_map_pose_missing");
    }

    return reasons;
  }
};

diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

diagnostic_msgs::msg::KeyValue kv(const std::string & key, int value)
{
  return kv(key, std::to_string(value));
}

diagnostic_msgs::msg::KeyValue kv(const std::string & key, bool value)
{
  return kv(key, std::string(value ? "true" : "false"));
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

std::chrono::nanoseconds seconds_to_period(double seconds)
{
  const auto safe_seconds = std::max(0.001, seconds);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(safe_seconds));
}

std::string escape_json(const std::string & value)
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

std::string json_array(const std::vector<std::string> & values)
{
  std::ostringstream out;
  out << "[";

  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0U) {
      out << ",";
    }
    out << "\"" << escape_json(values[i]) << "\"";
  }

  out << "]";
  return out.str();
}

std::string json_array(const std::vector<int> & values)
{
  std::ostringstream out;
  out << "[";

  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0U) {
      out << ",";
    }
    out << values[i];
  }

  out << "]";
  return out.str();
}

std::optional<std::string> json_string_value(const std::string & text, const std::string & key)
{
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
  std::smatch match;

  if (std::regex_search(text, match, pattern) && match.size() >= 2U) {
    return match[1].str();
  }

  return std::nullopt;
}

std::optional<double> json_number_value(const std::string & text, const std::string & key)
{
  const std::regex pattern(
    "\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)");
  std::smatch match;

  if (!std::regex_search(text, match, pattern) || match.size() < 2U) {
    return std::nullopt;
  }

  try {
    return std::stod(match[1].str());
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<bool> json_bool_value(const std::string & text, const std::string & key)
{
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*(true|false|1|0)");
  std::smatch match;

  if (!std::regex_search(text, match, pattern) || match.size() < 2U) {
    return std::nullopt;
  }

  const auto value = match[1].str();
  return value == "true" || value == "1";
}

std::optional<Vec3> json_vec3_value(const std::string & text, const std::string & key)
{
  const std::regex pattern(
    "\\\"" + key +
    "\\\"\\s*:\\s*\\[\\s*"
    "(-?[0-9]+(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)\\s*,\\s*"
    "(-?[0-9]+(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)\\s*,\\s*"
    "(-?[0-9]+(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)\\s*\\]");
  std::smatch match;

  if (!std::regex_search(text, match, pattern) || match.size() < 4U) {
    return std::nullopt;
  }

  try {
    return Vec3{
      std::stod(match[1].str()),
      std::stod(match[2].str()),
      std::stod(match[3].str())
    };
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::string join_csv(const std::vector<std::string> & values)
{
  std::ostringstream out;

  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0U) {
      out << ",";
    }
    out << values[i];
  }

  return out.str();
}

std::vector<std::string> split_csv_strings(const std::string & raw)
{
  std::vector<std::string> out;
  std::stringstream stream(raw);
  std::string item;

  while (std::getline(stream, item, ',')) {
    item.erase(
      item.begin(),
      std::find_if(
        item.begin(),
        item.end(),
        [](unsigned char c) {return !std::isspace(c);}));

    item.erase(
      std::find_if(
        item.rbegin(),
        item.rend(),
        [](unsigned char c) {return !std::isspace(c);}).base(),
      item.end());

    if (!item.empty()) {
      out.push_back(item);
    }
  }

  return out;
}

std::vector<int> split_csv_ints(const std::string & raw)
{
  std::vector<int> out;

  for (const auto & item : split_csv_strings(raw)) {
    try {
      const auto value = std::stoi(item);
      if (value >= 0 && std::find(out.begin(), out.end(), value) == out.end()) {
        out.push_back(value);
      }
    } catch (...) {
    }
  }

  return out;
}

}  // namespace

class AprilTagConfirmNode : public rclcpp::Node
{
public:
  AprilTagConfirmNode()
  : Node("apriltag_confirm_node")
  {
    declare_parameters();

    registrations_ = load_registrations();
    policy_ = load_policy();

    detection_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("apriltag_detections_topic").as_string(),
      rclcpp::QoS(10).best_effort(),
      std::bind(&AprilTagConfirmNode::on_detection, this, std::placeholders::_1));

    robot_pose_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("robot_pose_snapshot_topic").as_string(),
      rclcpp::QoS(10).reliable(),
      std::bind(&AprilTagConfirmNode::on_robot_pose, this, std::placeholders::_1));

    semantic_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("semantic_confirmations_topic").as_string(),
      rclcpp::QoS(10).reliable());

    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      get_parameter("status_topic").as_string(),
      rclcpp::QoS(10).reliable());

    dashboard_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("dashboard_text_topic").as_string(),
      rclcpp::QoS(10).reliable());

    const auto status_hz = std::max(0.2, get_parameter("status_hz").as_double());
    status_timer_ = create_wall_timer(
      seconds_to_period(1.0 / status_hz),
      std::bind(&AprilTagConfirmNode::publish_status, this));

    RCLCPP_INFO(
      get_logger(),
      "apriltag confirmation node started with %zu registered tags",
      registrations_.size());
  }

private:
  void declare_parameters()
  {
    declare_parameter<bool>("apriltag_enabled", true);
    declare_parameter<std::string>("apriltag_family", kAprilTagFamilyDefault);

    declare_parameter<std::string>("apriltag_detections_topic", kTopicAprilTagDetections);
    declare_parameter<std::string>("semantic_confirmations_topic", kTopicSemanticConfirmations);
    declare_parameter<std::string>("robot_pose_snapshot_topic", "/savo_head/robot_pose_snapshot");

    declare_parameter<std::string>("status_topic", kTopicStatus);
    declare_parameter<std::string>("dashboard_text_topic", kTopicDashboardText);
    declare_parameter<double>("status_hz", kStatusHz);

    declare_parameter<std::string>("camera_optical_frame", kFrameCameraOptical);
    declare_parameter<std::string>("robot_base_frame", kFrameBaseLink);
    declare_parameter<std::string>("map_frame", "map");

    declare_parameter<std::string>("confirmation_source", kSemanticConfirmationSource);
    declare_parameter<std::string>("default_confirmation_type", kConfirmationTypeKnownLocation);

    declare_parameter<bool>("allow_unknown_tags", false);
    declare_parameter<std::string>("registered_tag_ids_csv", "");
    declare_parameter<std::string>("tag_param_prefix", "tag_");

    declare_parameter<int>("min_stable_frames", 5);
    declare_parameter<double>("min_detection_confidence", 0.70);
    declare_parameter<double>("max_detection_distance_m", 3.0);
    declare_parameter<double>("max_detection_age_s", 0.50);

    declare_parameter<bool>("require_tf_available", true);
    declare_parameter<double>("tf_timeout_s", 0.25);

    declare_parameter<bool>("require_robot_stationary", true);
    declare_parameter<double>("max_robot_linear_speed_mps", 0.03);
    declare_parameter<double>("max_robot_angular_speed_radps", 0.05);

    declare_parameter<bool>("require_localization_ok", true);
    declare_parameter<double>("max_pose_covariance_xy", 0.25);
    declare_parameter<double>("max_yaw_covariance", 0.20);

    declare_parameter<bool>("require_lidar_map_pose", true);
    declare_parameter<bool>("require_semantic_label", true);
    declare_parameter<bool>("save_as_summon_point_by_default", false);

    declare_parameter<bool>("publish_rejections", true);
    declare_parameter<bool>("require_robot_pose", true);

    for (const auto tag_id : registered_tag_ids_from_param()) {
      const auto prefix = tag_prefix(tag_id);
      declare_parameter<bool>(prefix + "enabled", true);
      declare_parameter<std::string>(prefix + "label", "");
      declare_parameter<std::string>(prefix + "type", kConfirmationTypeKnownLocation);
      declare_parameter<bool>(prefix + "save_as_summon_point", false);
      declare_parameter<std::string>(prefix + "aliases_csv", "");
    }
  }

  std::vector<int> registered_tag_ids_from_param() const
  {
    return split_csv_ints(get_parameter("registered_tag_ids_csv").as_string());
  }

  std::string tag_prefix(int tag_id) const
  {
    return get_parameter("tag_param_prefix").as_string() + std::to_string(tag_id) + "_";
  }

  std::map<int, TagRegistration> load_registrations()
  {
    std::map<int, TagRegistration> registrations;

    for (const auto tag_id : registered_tag_ids_from_param()) {
      const auto prefix = tag_prefix(tag_id);

      TagRegistration registration;
      registration.tag_id = tag_id;
      registration.enabled = get_parameter(prefix + "enabled").as_bool();
      registration.label = get_parameter(prefix + "label").as_string();
      registration.confirmation_type = get_parameter(prefix + "type").as_string();
      registration.save_as_summon_point =
        get_parameter(prefix + "save_as_summon_point").as_bool();
      registration.aliases =
        split_csv_strings(get_parameter(prefix + "aliases_csv").as_string());

      if (registration.valid()) {
        registrations[tag_id] = registration;
      } else {
        RCLCPP_WARN(get_logger(), "ignoring invalid AprilTag registration id=%d", tag_id);
      }
    }

    return registrations;
  }

  ConfirmationPolicy load_policy() const
  {
    ConfirmationPolicy policy;
    policy.min_stable_frames = get_parameter("min_stable_frames").as_int();
    policy.min_detection_confidence = get_parameter("min_detection_confidence").as_double();
    policy.max_detection_distance_m = get_parameter("max_detection_distance_m").as_double();
    policy.max_detection_age_s = get_parameter("max_detection_age_s").as_double();

    policy.require_tf_available = get_parameter("require_tf_available").as_bool();
    policy.require_robot_stationary = get_parameter("require_robot_stationary").as_bool();
    policy.max_robot_linear_speed_mps = get_parameter("max_robot_linear_speed_mps").as_double();
    policy.max_robot_angular_speed_radps =
      get_parameter("max_robot_angular_speed_radps").as_double();

    policy.require_localization_ok = get_parameter("require_localization_ok").as_bool();
    policy.max_pose_covariance_xy = get_parameter("max_pose_covariance_xy").as_double();
    policy.max_yaw_covariance = get_parameter("max_yaw_covariance").as_double();

    policy.require_lidar_map_pose = get_parameter("require_lidar_map_pose").as_bool();
    policy.require_semantic_label = get_parameter("require_semantic_label").as_bool();

    return policy;
  }

  void on_robot_pose(const std_msgs::msg::String::SharedPtr msg)
  {
    try {
      latest_robot_pose_ = parse_robot_pose(msg->data);
      last_error_.reset();
    } catch (const std::exception & exc) {
      last_error_ = std::string("robot_pose_parse_failed: ") + exc.what();
      RCLCPP_WARN(get_logger(), "%s", last_error_->c_str());
    }
  }

  void on_detection(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!get_parameter("apriltag_enabled").as_bool()) {
      return;
    }

    const auto stamp = now_s();

    try {
      const auto observation = parse_observation(msg->data, stamp);
      auto registration = registration_for(observation.tag_id);
      auto pose = latest_robot_pose_;

      std::vector<std::string> reasons;

      if (!pose.has_value() && get_parameter("require_robot_pose").as_bool()) {
        reasons.emplace_back("robot_pose_missing");
      }

      const auto policy_reasons = policy_.rejection_reasons(
        observation,
        pose,
        registration,
        stamp);

      reasons.insert(reasons.end(), policy_reasons.begin(), policy_reasons.end());

      if (!reasons.empty()) {
        reject_count_ += 1U;
        last_error_ = join_csv(reasons);

        if (get_parameter("publish_rejections").as_bool()) {
          publish_confirmation(make_rejection_json(observation, reasons, stamp));
        }

        return;
      }

      if (!pose.has_value() || !registration.has_value()) {
        throw std::runtime_error("internal confirmation state missing pose or registration");
      }

      confirm_count_ += 1U;
      last_error_.reset();

      publish_confirmation(make_confirmation_json(
        observation,
        pose.value(),
        registration.value(),
        stamp));
    } catch (const std::exception & exc) {
      reject_count_ += 1U;
      last_error_ = exc.what();
      RCLCPP_ERROR(get_logger(), "AprilTag confirmation failed: %s", exc.what());
    }
  }

  RobotPoseSnapshot parse_robot_pose(const std::string & text) const
  {
    RobotPoseSnapshot pose;

    pose.x_m = json_number_value(text, "x_m").value_or(0.0);
    pose.y_m = json_number_value(text, "y_m").value_or(0.0);
    pose.yaw_rad = json_number_value(text, "yaw_rad").value_or(0.0);
    pose.frame_id = json_string_value(text, "frame_id").value_or(get_parameter("map_frame").as_string());
    pose.base_frame =
      json_string_value(text, "base_frame").value_or(get_parameter("robot_base_frame").as_string());
    pose.stamp_s = json_number_value(text, "stamp_s").value_or(now_s());
    pose.linear_speed_mps = json_number_value(text, "linear_speed_mps").value_or(0.0);
    pose.angular_speed_radps = json_number_value(text, "angular_speed_radps").value_or(0.0);
    pose.pose_covariance_xy = json_number_value(text, "pose_covariance_xy").value_or(0.0);
    pose.yaw_covariance = json_number_value(text, "yaw_covariance").value_or(0.0);
    pose.localization_ok = json_bool_value(text, "localization_ok").value_or(true);
    pose.lidar_map_pose_ok = json_bool_value(text, "lidar_map_pose_ok").value_or(true);
    pose.tf_ok = json_bool_value(text, "tf_ok").value_or(true);

    return pose;
  }

  AprilTagObservation parse_observation(const std::string & text, double default_stamp_s) const
  {
    const auto tag_id = json_number_value(text, "tag_id");
    if (!tag_id.has_value()) {
      throw std::runtime_error("AprilTag observation missing tag_id");
    }

    AprilTagObservation observation;
    observation.tag_id = static_cast<int>(std::llround(tag_id.value()));
    observation.family =
      json_string_value(text, "family").value_or(get_parameter("apriltag_family").as_string());
    observation.confidence = json_number_value(text, "confidence").value_or(0.0);
    observation.distance_m = json_number_value(text, "distance_m").value_or(0.0);
    observation.stable_frames =
      static_cast<int>(std::llround(json_number_value(text, "stable_frames").value_or(0.0)));
    observation.stamp_s = json_number_value(text, "stamp_s").value_or(default_stamp_s);
    observation.frame_id =
      json_string_value(text, "frame_id").value_or(get_parameter("camera_optical_frame").as_string());
    observation.pose_camera_xyz_m =
      json_vec3_value(text, "pose_camera_xyz_m").value_or(Vec3{0.0, 0.0, 0.0});
    observation.pose_camera_rpy_rad =
      json_vec3_value(text, "pose_camera_rpy_rad").value_or(Vec3{0.0, 0.0, 0.0});

    return observation;
  }

  std::optional<TagRegistration> registration_for(int tag_id) const
  {
    const auto found = registrations_.find(tag_id);
    if (found != registrations_.end()) {
      return found->second;
    }

    if (!get_parameter("allow_unknown_tags").as_bool()) {
      return std::nullopt;
    }

    TagRegistration registration;
    registration.tag_id = tag_id;
    registration.label = "tag_" + std::to_string(tag_id);
    registration.confirmation_type = get_parameter("default_confirmation_type").as_string();
    registration.enabled = true;
    registration.save_as_summon_point =
      get_parameter("save_as_summon_point_by_default").as_bool();

    return registration;
  }

  std::string make_confirmation_json(
    const AprilTagObservation & observation,
    const RobotPoseSnapshot & pose,
    const TagRegistration & registration,
    double stamp_s) const
  {
    std::ostringstream out;
    out << "{";
    out << "\"state\":\"confirmed\",";
    out << "\"source\":\"" << escape_json(get_parameter("confirmation_source").as_string()) << "\",";
    out << "\"tag_id\":" << observation.tag_id << ",";
    out << "\"family\":\"" << escape_json(observation.family) << "\",";
    out << "\"label\":\"" << escape_json(registration.label) << "\",";
    out << "\"confirmation_type\":\"" << escape_json(registration.confirmation_type) << "\",";
    out << "\"save_as_summon_point\":" << (registration.save_as_summon_point ? "true" : "false") << ",";
    out << "\"aliases\":" << json_array(registration.aliases) << ",";
    out << "\"confidence\":" << observation.confidence << ",";
    out << "\"distance_m\":" << observation.distance_m << ",";
    out << "\"stable_frames\":" << observation.stable_frames << ",";
    out << "\"stamp_s\":" << stamp_s << ",";
    out << "\"robot_pose\":{";
    out << "\"x_m\":" << pose.x_m << ",";
    out << "\"y_m\":" << pose.y_m << ",";
    out << "\"yaw_rad\":" << pose.yaw_rad << ",";
    out << "\"frame_id\":\"" << escape_json(pose.frame_id) << "\",";
    out << "\"base_frame\":\"" << escape_json(pose.base_frame) << "\"";
    out << "},";
    out << "\"observation_frame\":\"" << escape_json(observation.frame_id) << "\",";
    out << "\"reason\":\"apriltag_confirm_node\"";
    out << "}";

    return out.str();
  }

  std::string make_rejection_json(
    const AprilTagObservation & observation,
    const std::vector<std::string> & reasons,
    double stamp_s) const
  {
    std::ostringstream out;
    out << "{";
    out << "\"state\":\"rejected\",";
    out << "\"source\":\"" << escape_json(get_parameter("confirmation_source").as_string()) << "\",";
    out << "\"tag_id\":" << observation.tag_id << ",";
    out << "\"family\":\"" << escape_json(observation.family) << "\",";
    out << "\"confidence\":" << observation.confidence << ",";
    out << "\"distance_m\":" << observation.distance_m << ",";
    out << "\"stable_frames\":" << observation.stable_frames << ",";
    out << "\"stamp_s\":" << stamp_s << ",";
    out << "\"rejection_reasons\":" << json_array(reasons);
    out << "}";

    return out.str();
  }

  void publish_confirmation(const std::string & json)
  {
    std_msgs::msg::String msg;
    msg.data = json;
    semantic_pub_->publish(msg);
  }

  void publish_status()
  {
    const auto health = current_health();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "savo_head.apriltag_confirm";
    status.hardware_id = "savo_head";
    status.level = ros_level(health.level);
    status.message = health.message;

    std::vector<int> registered_ids;
    registered_ids.reserve(registrations_.size());
    for (const auto & [tag_id, registration] : registrations_) {
      (void)registration;
      registered_ids.push_back(tag_id);
    }

    status.values = {
      kv("registered_tags", json_array(registered_ids)),
      kv("confirm_count", static_cast<int>(confirm_count_)),
      kv("reject_count", static_cast<int>(reject_count_)),
      kv("has_robot_pose", latest_robot_pose_.has_value()),
      kv("allow_unknown_tags", get_parameter("allow_unknown_tags").as_bool()),
      kv("last_error", last_error_.value_or(""))
    };

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status = {status};

    status_pub_->publish(array);

    std_msgs::msg::String dashboard;
    dashboard.data = dashboard_text(health) + " apriltag_confirmed=" +
      std::to_string(confirm_count_) + " rejected=" + std::to_string(reject_count_);
    dashboard_pub_->publish(dashboard);
  }

  HeadHealthSummary current_health() const
  {
    std::vector<ComponentHealth> components;

    if (last_error_.has_value() && !last_error_->empty()) {
      auto health = warn_component("savo_head.apriltag_confirm", HeadStatus::kStale, *last_error_);
      health.add_value("confirm_count", static_cast<int>(confirm_count_));
      health.add_value("reject_count", static_cast<int>(reject_count_));
      components.push_back(health);
    } else if (registrations_.empty() && !get_parameter("allow_unknown_tags").as_bool()) {
      auto health = warn_component(
        "savo_head.apriltag_confirm",
        HeadStatus::kDryrun,
        "NO_REGISTERED_TAGS");
      health.add_value("confirm_count", static_cast<int>(confirm_count_));
      health.add_value("reject_count", static_cast<int>(reject_count_));
      components.push_back(health);
    } else {
      auto health = ok_component("savo_head.apriltag_confirm", "OK");
      health.add_value("registered_count", static_cast<int>(registrations_.size()));
      health.add_value("confirm_count", static_cast<int>(confirm_count_));
      health.add_value("reject_count", static_cast<int>(reject_count_));
      components.push_back(health);
    }

    return summarize_health(components, true);
  }

  double now_s() const
  {
    return now().seconds();
  }

  std::map<int, TagRegistration> registrations_{};
  ConfirmationPolicy policy_{};
  std::optional<RobotPoseSnapshot> latest_robot_pose_{};
  std::optional<std::string> last_error_{};
  std::size_t confirm_count_{0};
  std::size_t reject_count_{0};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detection_sub_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_pose_sub_{};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr semantic_pub_{};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dashboard_pub_{};

  rclcpp::TimerBase::SharedPtr status_timer_{};
};

}  // namespace savo_head

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<savo_head::AprilTagConfirmNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    std::cerr << "apriltag_confirm_node failed: " << exc.what() << std::endl;
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
