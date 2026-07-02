#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "savo_head/core/head_types.hpp"

namespace savo_head
{

inline constexpr int kI2cBusDefault = 1;
inline constexpr int kPca9685AddressDefault = 0x40;
inline constexpr double kPca9685PwmFrequencyHzDefault = 50.0;

inline constexpr double kServoPulseMinUs = 500.0;
inline constexpr double kServoPulseMaxUs = 2500.0;
inline constexpr double kServoPeriodUs = 20000.0;
inline constexpr int kPca9685TicksPerCycle = 4096;

inline constexpr int kServoAngleMinDeg = 0;
inline constexpr int kServoAngleMaxDeg = 180;
inline constexpr int kServoErrorDegDefault = 10;

enum class ServoDirection
{
  kNormal,
  kReversed
};

struct ServoChannelCalibration
{
  std::string logical_channel;
  int pca9685_channel{0};
  int min_deg{kServoAngleMinDeg};
  int center_deg{kServoAngleMinDeg};
  int max_deg{kServoAngleMaxDeg};
  int error_deg{kServoErrorDegDefault};
  ServoDirection direction{ServoDirection::kNormal};

  [[nodiscard]] ServoChannelCalibration normalized() const
  {
    ServoChannelCalibration out = *this;
    out.pca9685_channel = logical_channel_to_pca9685_channel(out.logical_channel);

    out.min_deg = std::clamp(out.min_deg, kServoAngleMinDeg, kServoAngleMaxDeg);
    out.max_deg = std::clamp(out.max_deg, kServoAngleMinDeg, kServoAngleMaxDeg);

    if (out.min_deg > out.max_deg) {
      std::swap(out.min_deg, out.max_deg);
    }

    out.center_deg = std::clamp(out.center_deg, out.min_deg, out.max_deg);
    return out;
  }

  [[nodiscard]] int clamp_angle(int angle_deg) const
  {
    const auto item = normalized();
    return std::clamp(angle_deg, item.min_deg, item.max_deg);
  }

  [[nodiscard]] std::vector<std::string> validation_errors() const
  {
    std::vector<std::string> errors;

    try {
      const auto expected = logical_channel_to_pca9685_channel(logical_channel);
      if (expected != pca9685_channel) {
        errors.emplace_back("pca9685 channel does not match Freenove logical mapping");
      }
    } catch (const std::exception & exc) {
      errors.emplace_back(exc.what());
    }

    if (min_deg < kServoAngleMinDeg) {
      errors.emplace_back("min_deg is below servo angle minimum");
    }

    if (max_deg > kServoAngleMaxDeg) {
      errors.emplace_back("max_deg is above servo angle maximum");
    }

    if (min_deg >= max_deg) {
      errors.emplace_back("min_deg must be lower than max_deg");
    }

    if (center_deg < min_deg || center_deg > max_deg) {
      errors.emplace_back("center_deg must be inside min/max range");
    }

    return errors;
  }

  [[nodiscard]] bool valid() const
  {
    return validation_errors().empty();
  }

  [[nodiscard]] static int logical_channel_to_pca9685_channel(const std::string & logical)
  {
    if (logical == "0") {
      return 0;
    }
    if (logical == "1") {
      return 1;
    }
    if (logical == "2") {
      return 2;
    }
    if (logical == "3") {
      return 3;
    }
    if (logical == "4") {
      return 4;
    }
    if (logical == "5") {
      return 5;
    }
    if (logical == "6") {
      return 14;
    }
    if (logical == "7") {
      return 15;
    }

    throw std::invalid_argument("invalid Freenove logical servo channel: " + logical);
  }
};

[[nodiscard]] inline int logical_channel_to_pca9685_channel(const std::string & logical)
{
  return ServoChannelCalibration::logical_channel_to_pca9685_channel(logical);
}

struct HeadServoCalibration
{
  ServoChannelCalibration pan{
    kPanLogicalChannel,
    kPanPca9685Channel,
    kPanMinDeg,
    kPanCenterDeg,
    kPanMaxDeg,
    kServoErrorDegDefault,
    ServoDirection::kNormal
  };

  ServoChannelCalibration tilt{
    kTiltLogicalChannel,
    kTiltPca9685Channel,
    kTiltMinDeg,
    kTiltCenterDeg,
    kTiltMaxDeg,
    kServoErrorDegDefault,
    ServoDirection::kNormal
  };

  [[nodiscard]] HeadServoCalibration normalized() const
  {
    HeadServoCalibration out = *this;
    out.pan = out.pan.normalized();
    out.tilt = out.tilt.normalized();
    return out;
  }

  [[nodiscard]] PanTiltLimits limits() const
  {
    const auto item = normalized();
    return PanTiltLimits{
      item.pan.min_deg,
      item.pan.center_deg,
      item.pan.max_deg,
      item.tilt.min_deg,
      item.tilt.center_deg,
      item.tilt.max_deg
    };
  }

  [[nodiscard]] std::vector<std::string> validation_errors() const
  {
    std::vector<std::string> errors;

    for (const auto & error : pan.validation_errors()) {
      errors.emplace_back("pan: " + error);
    }

    for (const auto & error : tilt.validation_errors()) {
      errors.emplace_back("tilt: " + error);
    }

    return errors;
  }

  [[nodiscard]] bool valid() const
  {
    return validation_errors().empty();
  }
};

struct ServoOutput
{
  std::string axis;
  std::string logical_channel;
  int pca9685_channel{0};
  int angle_deg{0};
  double pulse_us{0.0};
  int ticks{0};
};

[[nodiscard]] inline HeadServoCalibration default_servo_calibration()
{
  return HeadServoCalibration{}.normalized();
}

[[nodiscard]] inline double clamp_pulse_us(double pulse_us)
{
  if (!std::isfinite(pulse_us)) {
    return kServoPulseMinUs;
  }

  return std::clamp(pulse_us, kServoPulseMinUs, kServoPulseMaxUs);
}

[[nodiscard]] inline double angle_to_pulse_us(
  int angle_deg,
  int error_deg = kServoErrorDegDefault,
  ServoDirection direction = ServoDirection::kNormal)
{
  const auto angle = clamp_servo_angle(angle_deg);
  double pulse = 0.0;

  if (direction == ServoDirection::kReversed) {
    pulse = 2500.0 - static_cast<double>(angle + error_deg) / 0.09;
  } else {
    pulse = 500.0 + static_cast<double>(angle + error_deg) / 0.09;
  }

  return clamp_pulse_us(pulse);
}

[[nodiscard]] inline int pulse_us_to_ticks(double pulse_us)
{
  const auto pulse = clamp_pulse_us(pulse_us);
  const auto ticks = pulse * static_cast<double>(kPca9685TicksPerCycle) / kServoPeriodUs;
  return static_cast<int>(ticks);
}

[[nodiscard]] inline int angle_to_ticks(
  int angle_deg,
  int error_deg = kServoErrorDegDefault,
  ServoDirection direction = ServoDirection::kNormal)
{
  return pulse_us_to_ticks(angle_to_pulse_us(angle_deg, error_deg, direction));
}

[[nodiscard]] inline const ServoChannelCalibration & calibration_for_axis(
  const std::string & axis,
  const HeadServoCalibration & calibration)
{
  if (axis == "pan") {
    return calibration.pan;
  }

  if (axis == "tilt") {
    return calibration.tilt;
  }

  throw std::invalid_argument("unknown head axis: " + axis);
}

[[nodiscard]] inline ServoOutput angle_to_servo_output(
  const std::string & axis,
  int angle_deg,
  const HeadServoCalibration & calibration = default_servo_calibration())
{
  const auto normalized = calibration.normalized();
  const auto & item = calibration_for_axis(axis, normalized);
  const auto angle = item.clamp_angle(angle_deg);
  const auto pulse_us = angle_to_pulse_us(angle, item.error_deg, item.direction);
  const auto ticks = pulse_us_to_ticks(pulse_us);

  return ServoOutput{
    axis,
    item.logical_channel,
    item.pca9685_channel,
    angle,
    pulse_us,
    ticks
  };
}

}  // namespace savo_head
