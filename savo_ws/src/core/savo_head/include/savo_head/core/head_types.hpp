#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace savo_head
{

inline constexpr const char * kPackageName = "savo_head";
inline constexpr const char * kRobotName = "Robot Savo";

inline constexpr int kPanMinDeg = 0;
inline constexpr int kPanCenterDeg = 72;
inline constexpr int kPanMaxDeg = 170;

inline constexpr int kTiltMinDeg = 45;
inline constexpr int kTiltCenterDeg = 55;
inline constexpr int kTiltMaxDeg = 130;

inline constexpr int kManualStepDeg = 2;

inline constexpr const char * kPanLogicalChannel = "7";
inline constexpr const char * kTiltLogicalChannel = "6";
inline constexpr int kPanPca9685Channel = 15;
inline constexpr int kTiltPca9685Channel = 14;

inline constexpr double kControlHz = 30.0;
inline constexpr double kStatusHz = 2.0;
inline constexpr double kTfHz = 30.0;
inline constexpr double kWatchdogTimeoutS = 0.50;

inline constexpr const char * kTopicPanTiltCmd = "/savo_head/pan_tilt_cmd";
inline constexpr const char * kTopicPanTiltState = "/savo_head/pan_tilt_state";
inline constexpr const char * kTopicScanCmd = "/savo_head/scan_cmd";
inline constexpr const char * kTopicScanState = "/savo_head/scan_state";
inline constexpr const char * kTopicStatus = "/savo_head/status";
inline constexpr const char * kTopicDashboardText = "/savo_head/dashboard_text";
inline constexpr const char * kTopicEmergencyCenter = "/savo_head/emergency_center";
inline constexpr const char * kTopicCameraImageRaw = "/savo_head/camera/image_raw";
inline constexpr const char * kTopicCameraInfo = "/savo_head/camera/camera_info";
inline constexpr const char * kTopicCameraStatus = "/savo_head/camera/status";
inline constexpr const char * kTopicAprilTagDetections = "/savo_head/apriltag_detections";
inline constexpr const char * kTopicSemanticConfirmations = "/savo_head/semantic_confirmations";

inline constexpr const char * kFrameBaseLink = "base_link";
inline constexpr const char * kFramePanLink = "pantilt_pan_link";
inline constexpr const char * kFrameTiltLink = "pantilt_tilt_link";
inline constexpr const char * kFrameCameraLink = "pi_camera_link";
inline constexpr const char * kFrameCameraOptical = "pi_camera_optical_frame";

inline constexpr const char * kJointPan = "head_pan_joint";
inline constexpr const char * kJointTilt = "head_tilt_joint";

enum class HeadMode
{
  kIdle,
  kManual,
  kAuto,
  kCentering
};

enum class HeadStatus
{
  kOk,
  kDryrun,
  kStale,
  kDisabled,
  kError
};

enum class CommandType
{
  kAbsolute,
  kDelta,
  kCenter,
  kHold,
  kStop
};

enum class CommandSource
{
  kTopic,
  kManual,
  kScan,
  kService,
  kEmergency,
  kWatchdog,
  kSystem
};

struct PanTiltLimits
{
  int pan_min_deg{kPanMinDeg};
  int pan_center_deg{kPanCenterDeg};
  int pan_max_deg{kPanMaxDeg};

  int tilt_min_deg{kTiltMinDeg};
  int tilt_center_deg{kTiltCenterDeg};
  int tilt_max_deg{kTiltMaxDeg};

  [[nodiscard]] int clamp_pan(int value) const
  {
    return std::clamp(value, pan_min_deg, pan_max_deg);
  }

  [[nodiscard]] int clamp_tilt(int value) const
  {
    return std::clamp(value, tilt_min_deg, tilt_max_deg);
  }

  [[nodiscard]] bool contains_pan(int value) const
  {
    return value >= pan_min_deg && value <= pan_max_deg;
  }

  [[nodiscard]] bool contains_tilt(int value) const
  {
    return value >= tilt_min_deg && value <= tilt_max_deg;
  }

  [[nodiscard]] std::vector<std::string> validation_errors() const
  {
    std::vector<std::string> errors;

    if (pan_min_deg >= pan_center_deg || pan_center_deg >= pan_max_deg) {
      errors.emplace_back("pan center must be inside pan min/max range");
    }

    if (tilt_min_deg > tilt_center_deg || tilt_center_deg > tilt_max_deg) {
      errors.emplace_back("tilt center must be inside tilt min/max range");
    }

    return errors;
  }

  [[nodiscard]] bool valid() const
  {
    return validation_errors().empty();
  }
};

struct PanTiltState
{
  int pan_deg{kPanCenterDeg};
  int tilt_deg{kTiltCenterDeg};
  HeadMode mode{HeadMode::kIdle};
  HeadStatus status{HeadStatus::kOk};
  double stamp_s{0.0};
  std::string source{"unknown"};

  [[nodiscard]] PanTiltState normalized(const PanTiltLimits & limits = PanTiltLimits{}) const
  {
    PanTiltState out = *this;
    out.pan_deg = limits.clamp_pan(pan_deg);
    out.tilt_deg = limits.clamp_tilt(tilt_deg);

    if (!std::isfinite(out.stamp_s) || out.stamp_s < 0.0) {
      out.stamp_s = 0.0;
    }

    if (out.source.empty()) {
      out.source = "unknown";
    }

    return out;
  }

  [[nodiscard]] bool stale(double now_s, double timeout_s) const
  {
    if (stamp_s <= 0.0) {
      return true;
    }
    return (now_s - stamp_s) > timeout_s;
  }

  [[nodiscard]] std::tuple<int, int> as_tuple() const
  {
    return {pan_deg, tilt_deg};
  }
};

struct PanTiltCommand
{
  CommandType type{CommandType::kAbsolute};

  std::optional<int> pan_deg{};
  std::optional<int> tilt_deg{};

  int pan_delta_deg{0};
  int tilt_delta_deg{0};

  CommandSource source{CommandSource::kTopic};
  double stamp_s{0.0};
  int priority{0};
  std::string reason{};

  [[nodiscard]] std::vector<std::string> validation_errors() const
  {
    std::vector<std::string> errors;

    if (type == CommandType::kAbsolute && !pan_deg.has_value() && !tilt_deg.has_value()) {
      errors.emplace_back("absolute command requires pan or tilt target");
    }

    if (type == CommandType::kDelta && pan_delta_deg == 0 && tilt_delta_deg == 0) {
      errors.emplace_back("delta command requires non-zero pan or tilt delta");
    }

    if (!std::isfinite(stamp_s) || stamp_s < 0.0) {
      errors.emplace_back("stamp_s must be finite and non-negative");
    }

    return errors;
  }

  [[nodiscard]] bool valid() const
  {
    return validation_errors().empty();
  }
};

[[nodiscard]] inline const char * to_string(HeadMode mode)
{
  switch (mode) {
    case HeadMode::kIdle:
      return "idle";
    case HeadMode::kManual:
      return "manual";
    case HeadMode::kAuto:
      return "auto";
    case HeadMode::kCentering:
      return "centering";
  }

  return "unknown";
}

[[nodiscard]] inline const char * to_string(HeadStatus status)
{
  switch (status) {
    case HeadStatus::kOk:
      return "OK";
    case HeadStatus::kDryrun:
      return "DRYRUN";
    case HeadStatus::kStale:
      return "STALE";
    case HeadStatus::kDisabled:
      return "DISABLED";
    case HeadStatus::kError:
      return "ERROR";
  }

  return "UNKNOWN";
}

[[nodiscard]] inline const char * to_string(CommandType type)
{
  switch (type) {
    case CommandType::kAbsolute:
      return "absolute";
    case CommandType::kDelta:
      return "delta";
    case CommandType::kCenter:
      return "center";
    case CommandType::kHold:
      return "hold";
    case CommandType::kStop:
      return "stop";
  }

  return "unknown";
}

[[nodiscard]] inline const char * to_string(CommandSource source)
{
  switch (source) {
    case CommandSource::kTopic:
      return "topic";
    case CommandSource::kManual:
      return "manual";
    case CommandSource::kScan:
      return "scan";
    case CommandSource::kService:
      return "service";
    case CommandSource::kEmergency:
      return "emergency";
    case CommandSource::kWatchdog:
      return "watchdog";
    case CommandSource::kSystem:
      return "system";
  }

  return "unknown";
}

[[nodiscard]] inline int clamp_servo_angle(int value)
{
  return std::clamp(value, 0, 180);
}

[[nodiscard]] inline PanTiltState centered_state(double stamp_s = 0.0, std::string source = "center")
{
  return PanTiltState{
    kPanCenterDeg,
    kTiltCenterDeg,
    HeadMode::kCentering,
    HeadStatus::kOk,
    stamp_s,
    std::move(source)
  };
}

[[nodiscard]] inline PanTiltCommand absolute_command(
  std::optional<int> pan_deg,
  std::optional<int> tilt_deg,
  double stamp_s = 0.0,
  CommandSource source = CommandSource::kTopic,
  std::string reason = "absolute")
{
  return PanTiltCommand{
    CommandType::kAbsolute,
    pan_deg,
    tilt_deg,
    0,
    0,
    source,
    stamp_s,
    0,
    std::move(reason)
  };
}

[[nodiscard]] inline PanTiltCommand delta_command(
  int pan_delta_deg,
  int tilt_delta_deg,
  double stamp_s = 0.0,
  CommandSource source = CommandSource::kTopic,
  std::string reason = "delta")
{
  return PanTiltCommand{
    CommandType::kDelta,
    std::nullopt,
    std::nullopt,
    pan_delta_deg,
    tilt_delta_deg,
    source,
    stamp_s,
    0,
    std::move(reason)
  };
}

[[nodiscard]] inline PanTiltCommand center_command(
  double stamp_s = 0.0,
  CommandSource source = CommandSource::kSystem,
  std::string reason = "center")
{
  return PanTiltCommand{
    CommandType::kCenter,
    std::nullopt,
    std::nullopt,
    0,
    0,
    source,
    stamp_s,
    10,
    std::move(reason)
  };
}

[[nodiscard]] inline PanTiltCommand hold_command(
  double stamp_s = 0.0,
  CommandSource source = CommandSource::kSystem,
  std::string reason = "hold")
{
  return PanTiltCommand{
    CommandType::kHold,
    std::nullopt,
    std::nullopt,
    0,
    0,
    source,
    stamp_s,
    5,
    std::move(reason)
  };
}

[[nodiscard]] inline PanTiltCommand stop_command(
  double stamp_s = 0.0,
  CommandSource source = CommandSource::kSystem,
  std::string reason = "stop")
{
  return PanTiltCommand{
    CommandType::kStop,
    std::nullopt,
    std::nullopt,
    0,
    0,
    source,
    stamp_s,
    20,
    std::move(reason)
  };
}

[[nodiscard]] inline PanTiltState target_from_command(
  const PanTiltCommand & command,
  const PanTiltState & current,
  const PanTiltLimits & limits = PanTiltLimits{})
{
  PanTiltState target = current;

  switch (command.type) {
    case CommandType::kCenter:
      target.pan_deg = limits.pan_center_deg;
      target.tilt_deg = limits.tilt_center_deg;
      target.mode = HeadMode::kCentering;
      break;

    case CommandType::kHold:
    case CommandType::kStop:
      break;

    case CommandType::kDelta:
      target.pan_deg = current.pan_deg + command.pan_delta_deg;
      target.tilt_deg = current.tilt_deg + command.tilt_delta_deg;
      break;

    case CommandType::kAbsolute:
      if (command.pan_deg.has_value()) {
        target.pan_deg = command.pan_deg.value();
      }
      if (command.tilt_deg.has_value()) {
        target.tilt_deg = command.tilt_deg.value();
      }
      break;
  }

  target.pan_deg = limits.clamp_pan(clamp_servo_angle(target.pan_deg));
  target.tilt_deg = limits.clamp_tilt(clamp_servo_angle(target.tilt_deg));
  target.stamp_s = command.stamp_s;
  target.source = to_string(command.source);

  return target;
}

}  // namespace savo_head
