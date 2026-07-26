// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace savo_localization
{

enum class LocalizationHealthState : std::uint8_t
{
  kUnknown = 0,
  kInitializing,
  kOk,
  kDegraded,
  kStale,
  kError
};

struct SourceHealthObservation
{
  std::string name{};
  bool enabled{false};
  bool required{false};
  bool received{false};
  bool fresh{false};
  bool data_valid{true};
  bool frame_valid{true};
  bool timestamp_valid{true};
  bool rate_valid{true};
  bool diagnostic_warning{false};
  bool diagnostic_error{false};
  double age_s{-1.0};
  double rate_hz{0.0};
  std::string detail{};
};

struct TransformHealthObservation
{
  std::string name{};
  bool required{false};
  bool available{false};
  bool fresh{false};
  double age_s{-1.0};
  std::string detail{};
};

struct LocalizationHealthInputs
{
  double startup_age_s{0.0};
  double startup_grace_s{3.0};

  SourceHealthObservation imu{};
  SourceHealthObservation wheel_odom{};
  SourceHealthObservation filtered_odom{};
  SourceHealthObservation vo_odom{};

  TransformHealthObservation odom_to_base{};
  TransformHealthObservation base_to_imu{};

  bool filtered_pose_jump_detected{false};
  bool filtered_yaw_jump_detected{false};
};

struct LocalizationHealthResult
{
  LocalizationHealthState state{LocalizationHealthState::kUnknown};
  bool ready{false};
  bool degraded{false};
  std::string reason_code{"not_evaluated"};
  std::vector<std::string> reasons{};
};

class LocalizationHealthCore
{
public:
  [[nodiscard]] LocalizationHealthResult Evaluate(
    const LocalizationHealthInputs & inputs) const;

  [[nodiscard]] static std::string_view ToString(
    LocalizationHealthState state) noexcept;

private:
  static void AppendSourceErrors(
    const SourceHealthObservation & source,
    std::vector<std::string> & errors);

  static void AppendSourceStaleReasons(
    const SourceHealthObservation & source,
    std::vector<std::string> & stale_reasons);

  static void AppendSourceDegradedReasons(
    const SourceHealthObservation & source,
    std::vector<std::string> & degraded_reasons);
};

}  // namespace savo_localization
