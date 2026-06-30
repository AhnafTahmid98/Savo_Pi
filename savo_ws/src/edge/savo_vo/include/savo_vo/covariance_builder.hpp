#pragma once

#include "savo_vo/vo_types.hpp"

namespace savo_vo
{

class CovarianceBuilder
{
public:
  explicit CovarianceBuilder(
    const VOCovarianceConfig & config = VOCovarianceConfig{});

  Covariance36 pose_covariance() const;
  Covariance36 twist_covariance() const;

private:
  VOCovarianceConfig config_;
};

Covariance36 make_zero_covariance();

Covariance36 make_pose_covariance(
  double position_variance,
  double yaw_variance);

Covariance36 make_twist_covariance(
  double linear_velocity_variance,
  double angular_velocity_variance);

}  // namespace savo_vo
