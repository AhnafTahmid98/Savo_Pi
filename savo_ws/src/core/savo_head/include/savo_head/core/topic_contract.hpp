#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include "savo_head/core/head_types.hpp"

namespace savo_head
{

inline constexpr const char * kSavoHeadNamespace = "/savo_head";

struct HeadTopicContract
{
  std::string pan_tilt_cmd{kTopicPanTiltCmd};
  std::string pan_tilt_state{kTopicPanTiltState};

  std::string scan_cmd{kTopicScanCmd};
  std::string scan_state{kTopicScanState};

  std::string status{kTopicStatus};
  std::string diagnostics{"/diagnostics"};
  std::string dashboard_text{kTopicDashboardText};

  std::string emergency_center{kTopicEmergencyCenter};

  std::string camera_stream_cmd{"/savo_head/camera_stream_cmd"};
  std::string camera_stream_state{"/savo_head/camera_stream_state"};

  std::string camera_status{kTopicCameraStatus};
  std::string image_raw{kTopicCameraImageRaw};
  std::string camera_info{kTopicCameraInfo};

  std::string apriltag_detections{kTopicAprilTagDetections};
  std::string semantic_confirmations{kTopicSemanticConfirmations};
  std::string robot_pose_snapshot{"/savo_head/robot_pose_snapshot"};

  std::string center_service{"/savo_head/center"};
  std::string health_check_service{"/savo_head/health_check"};

  std::string start_scan_service{"/savo_head/start_scan"};
  std::string stop_scan_service{"/savo_head/stop_scan"};
  std::string pause_scan_service{"/savo_head/pause_scan"};
  std::string resume_scan_service{"/savo_head/resume_scan"};

  [[nodiscard]] std::vector<std::string> all_topics() const
  {
    return {
      pan_tilt_cmd,
      pan_tilt_state,
      scan_cmd,
      scan_state,
      status,
      diagnostics,
      dashboard_text,
      emergency_center,
      camera_stream_cmd,
      camera_stream_state,
      camera_status,
      image_raw,
      camera_info,
      apriltag_detections,
      semantic_confirmations,
      robot_pose_snapshot
    };
  }

  [[nodiscard]] std::vector<std::string> all_services() const
  {
    return {
      center_service,
      health_check_service,
      start_scan_service,
      stop_scan_service,
      pause_scan_service,
      resume_scan_service
    };
  }

  [[nodiscard]] std::vector<std::string> validation_errors() const
  {
    std::vector<std::string> errors;

    for (const auto & topic : all_topics()) {
      if (!is_absolute_ros_name(topic)) {
        errors.emplace_back("topic is not absolute ROS name: " + topic);
      }
    }

    for (const auto & service : all_services()) {
      if (!is_absolute_ros_name(service)) {
        errors.emplace_back("service is not absolute ROS name: " + service);
      }
    }

    if (has_duplicates(all_topics())) {
      errors.emplace_back("duplicate topic names found");
    }

    if (has_duplicates(all_services())) {
      errors.emplace_back("duplicate service names found");
    }

    return errors;
  }

  [[nodiscard]] bool valid() const
  {
    return validation_errors().empty();
  }

  [[nodiscard]] static bool is_absolute_ros_name(const std::string & name)
  {
    return !name.empty() && name.front() == '/';
  }

  [[nodiscard]] static bool has_duplicates(std::vector<std::string> values)
  {
    std::sort(values.begin(), values.end());
    return std::adjacent_find(values.begin(), values.end()) != values.end();
  }
};

struct HeadQosContract
{
  std::string command_reliability{"reliable"};
  std::string state_reliability{"reliable"};
  std::string sensor_reliability{"best_effort"};
  std::string status_reliability{"reliable"};

  int command_depth{10};
  int state_depth{10};
  int sensor_depth{5};
  int status_depth{10};

  [[nodiscard]] std::vector<std::string> validation_errors() const
  {
    std::vector<std::string> errors;

    validate_reliability(command_reliability, "command_reliability", errors);
    validate_reliability(state_reliability, "state_reliability", errors);
    validate_reliability(sensor_reliability, "sensor_reliability", errors);
    validate_reliability(status_reliability, "status_reliability", errors);

    validate_depth(command_depth, "command_depth", errors);
    validate_depth(state_depth, "state_depth", errors);
    validate_depth(sensor_depth, "sensor_depth", errors);
    validate_depth(status_depth, "status_depth", errors);

    return errors;
  }

  [[nodiscard]] bool valid() const
  {
    return validation_errors().empty();
  }

  [[nodiscard]] static bool is_valid_reliability(const std::string & reliability)
  {
    return reliability == "reliable" || reliability == "best_effort";
  }

private:
  static void validate_reliability(
    const std::string & value,
    const std::string & key,
    std::vector<std::string> & errors)
  {
    if (!is_valid_reliability(value)) {
      errors.emplace_back(key + " must be reliable or best_effort");
    }
  }

  static void validate_depth(
    int value,
    const std::string & key,
    std::vector<std::string> & errors)
  {
    if (value <= 0) {
      errors.emplace_back(key + " must be positive");
    }
  }
};

[[nodiscard]] inline HeadTopicContract default_head_topic_contract()
{
  return HeadTopicContract{};
}

[[nodiscard]] inline HeadQosContract default_head_qos_contract()
{
  return HeadQosContract{};
}

[[nodiscard]] inline bool is_camera_like_topic(const std::string & topic)
{
  return topic.find("image") != std::string::npos ||
    topic.find("camera") != std::string::npos ||
    topic.find("apriltag_detections") != std::string::npos;
}

[[nodiscard]] inline bool should_use_best_effort_for_topic(const std::string & topic)
{
  return topic == kTopicCameraImageRaw ||
    topic == kTopicCameraInfo ||
    topic == kTopicAprilTagDetections;
}

}  // namespace savo_head
