#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace savo_vo
{

enum class VOHealthLevel : std::uint8_t
{
  kWaiting = 0,
  kOk = 1,
  kStale = 2,
  kDegraded = 3,
  kError = 4,
};

enum class VOTrackingState : std::uint8_t
{
  kWaitingForReference = 0,
  kTracking = 1,
  kRejected = 2,
  kLost = 3,
  kError = 4,
};

struct CameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  int width{0};
  int height{0};

  bool is_valid() const;
};

struct TrackingQuality
{
  int feature_count{0};
  int tracked_count{0};
  double score{0.0};
  VOTrackingState state{VOTrackingState::kWaitingForReference};
  std::string message{"waiting for visual odometry reference frame"};

  bool is_good(double minimum_score, int minimum_features) const;
};

struct TimestampSyncStatus
{
  double color_stamp_s{0.0};
  double depth_stamp_s{0.0};
  double camera_info_stamp_s{0.0};
  double max_delta_s{0.0};
  bool synchronized{false};
};

struct VOCovarianceConfig
{
  double position_variance{0.05};
  double yaw_variance{0.10};
  double linear_velocity_variance{0.10};
  double angular_velocity_variance{0.20};
};

using Covariance36 = std::array<double, 36>;

std::string health_level_to_string(VOHealthLevel level);
std::string tracking_state_to_string(VOTrackingState state);

}  // namespace savo_vo
