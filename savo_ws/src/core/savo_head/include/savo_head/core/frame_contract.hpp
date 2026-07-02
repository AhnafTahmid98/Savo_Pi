#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "savo_head/core/head_types.hpp"

namespace savo_head
{

inline constexpr double kPi = 3.14159265358979323846;

using Vec3d = std::array<double, 3>;

struct FrameTransformSpec
{
  std::string parent;
  std::string child;
  Vec3d xyz_m{0.0, 0.0, 0.0};
  Vec3d rpy_rad{0.0, 0.0, 0.0};

  [[nodiscard]] bool valid() const
  {
    return validation_errors().empty();
  }

  [[nodiscard]] std::vector<std::string> validation_errors() const
  {
    std::vector<std::string> errors;

    if (!is_valid_tf_frame(parent)) {
      errors.emplace_back("invalid parent frame: " + parent);
    }

    if (!is_valid_tf_frame(child)) {
      errors.emplace_back("invalid child frame: " + child);
    }

    if (parent == child) {
      errors.emplace_back("parent and child frame must be different");
    }

    for (const auto value : xyz_m) {
      if (!std::isfinite(value)) {
        errors.emplace_back("xyz_m contains non-finite value");
        break;
      }
    }

    for (const auto value : rpy_rad) {
      if (!std::isfinite(value)) {
        errors.emplace_back("rpy_rad contains non-finite value");
        break;
      }
    }

    return errors;
  }

  [[nodiscard]] static bool is_valid_tf_frame(const std::string & frame)
  {
    if (frame.empty()) {
      return false;
    }

    if (frame.front() == '/') {
      return false;
    }

    return frame.find(' ') == std::string::npos;
  }
};

struct HeadFrameContract
{
  std::string base_frame{kFrameBaseLink};
  std::string pan_frame{kFramePanLink};
  std::string tilt_frame{kFrameTiltLink};
  std::string camera_frame{kFrameCameraLink};
  std::string camera_optical_frame{kFrameCameraOptical};

  std::string pan_joint{kJointPan};
  std::string tilt_joint{kJointTilt};

  std::string pan_axis{"z"};
  std::string tilt_axis{"y"};

  double pan_sign{1.0};
  double tilt_sign{1.0};

  double pan_zero_deg{static_cast<double>(kPanCenterDeg)};
  double tilt_zero_deg{static_cast<double>(kTiltCenterDeg)};

  FrameTransformSpec base_to_pan{
    kFrameBaseLink,
    kFramePanLink,
    Vec3d{0.0, 0.0, 0.0},
    Vec3d{0.0, 0.0, 0.0}
  };

  FrameTransformSpec pan_to_tilt{
    kFramePanLink,
    kFrameTiltLink,
    Vec3d{0.0, 0.0, 0.0},
    Vec3d{0.0, 0.0, 0.0}
  };

  FrameTransformSpec tilt_to_camera{
    kFrameTiltLink,
    kFrameCameraLink,
    Vec3d{0.0, 0.0, 0.0},
    Vec3d{0.0, 0.0, 0.0}
  };

  FrameTransformSpec camera_to_optical{
    kFrameCameraLink,
    kFrameCameraOptical,
    Vec3d{0.0, 0.0, 0.0},
    Vec3d{-kPi / 2.0, 0.0, -kPi / 2.0}
  };

  [[nodiscard]] std::vector<std::string> frames() const
  {
    return {
      base_frame,
      pan_frame,
      tilt_frame,
      camera_frame,
      camera_optical_frame
    };
  }

  [[nodiscard]] std::vector<std::string> joints() const
  {
    return {
      pan_joint,
      tilt_joint
    };
  }

  [[nodiscard]] std::vector<FrameTransformSpec> transforms() const
  {
    return {
      base_to_pan,
      pan_to_tilt,
      tilt_to_camera,
      camera_to_optical
    };
  }

  [[nodiscard]] std::vector<std::string> validation_errors() const
  {
    std::vector<std::string> errors;

    for (const auto & frame : frames()) {
      if (!FrameTransformSpec::is_valid_tf_frame(frame)) {
        errors.emplace_back("invalid frame name: " + frame);
      }
    }

    if (has_duplicates(frames())) {
      errors.emplace_back("duplicate frame names found");
    }

    for (const auto & joint : joints()) {
      if (joint.empty() || joint.find(' ') != std::string::npos) {
        errors.emplace_back("invalid joint name: " + joint);
      }
    }

    if (pan_axis != "x" && pan_axis != "y" && pan_axis != "z") {
      errors.emplace_back("pan_axis must be x, y, or z");
    }

    if (tilt_axis != "x" && tilt_axis != "y" && tilt_axis != "z") {
      errors.emplace_back("tilt_axis must be x, y, or z");
    }

    if (!std::isfinite(pan_sign) || std::abs(pan_sign) < 1e-9) {
      errors.emplace_back("pan_sign must be finite and non-zero");
    }

    if (!std::isfinite(tilt_sign) || std::abs(tilt_sign) < 1e-9) {
      errors.emplace_back("tilt_sign must be finite and non-zero");
    }

    if (!std::isfinite(pan_zero_deg)) {
      errors.emplace_back("pan_zero_deg must be finite");
    }

    if (!std::isfinite(tilt_zero_deg)) {
      errors.emplace_back("tilt_zero_deg must be finite");
    }

    for (const auto & transform : transforms()) {
      for (const auto & error : transform.validation_errors()) {
        errors.emplace_back(transform.parent + "->" + transform.child + ": " + error);
      }
    }

    return errors;
  }

  [[nodiscard]] bool valid() const
  {
    return validation_errors().empty();
  }

  [[nodiscard]] static bool has_duplicates(std::vector<std::string> values)
  {
    std::sort(values.begin(), values.end());
    return std::adjacent_find(values.begin(), values.end()) != values.end();
  }
};

[[nodiscard]] inline HeadFrameContract default_head_frame_contract()
{
  return HeadFrameContract{};
}

[[nodiscard]] inline double degrees_to_radians(double degrees)
{
  return degrees * kPi / 180.0;
}

[[nodiscard]] inline double radians_to_degrees(double radians)
{
  return radians * 180.0 / kPi;
}

[[nodiscard]] inline double pan_joint_offset_rad(double pan_deg, const HeadFrameContract & frames)
{
  return frames.pan_sign * degrees_to_radians(pan_deg - frames.pan_zero_deg);
}

[[nodiscard]] inline double tilt_joint_offset_rad(double tilt_deg, const HeadFrameContract & frames)
{
  return frames.tilt_sign * degrees_to_radians(tilt_deg - frames.tilt_zero_deg);
}

}  // namespace savo_head
