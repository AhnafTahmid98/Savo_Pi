#include "savo_vo/covariance_builder.hpp"

namespace savo_vo
{

CovarianceBuilder::CovarianceBuilder(
  const VOCovarianceConfig & config)
: config_(config)
{
}

Covariance36 CovarianceBuilder::pose_covariance() const
{
  return make_pose_covariance(
    config_.position_variance,
    config_.yaw_variance);
}

Covariance36 CovarianceBuilder::twist_covariance() const
{
  return make_twist_covariance(
    config_.linear_velocity_variance,
    config_.angular_velocity_variance);
}

Covariance36 make_zero_covariance()
{
  Covariance36 covariance{};
  covariance.fill(0.0);
  return covariance;
}

Covariance36 make_pose_covariance(
  const double position_variance,
  const double yaw_variance)
{
  Covariance36 covariance = make_zero_covariance();

  covariance[0] = position_variance;   // x
  covariance[7] = position_variance;   // y
  covariance[14] = 999.0;              // z unused for 2D VO
  covariance[21] = 999.0;              // roll unused
  covariance[28] = 999.0;              // pitch unused
  covariance[35] = yaw_variance;       // yaw

  return covariance;
}

Covariance36 make_twist_covariance(
  const double linear_velocity_variance,
  const double angular_velocity_variance)
{
  Covariance36 covariance = make_zero_covariance();

  covariance[0] = linear_velocity_variance;    // vx
  covariance[7] = linear_velocity_variance;    // vy
  covariance[14] = 999.0;                      // vz unused
  covariance[21] = 999.0;                      // vroll unused
  covariance[28] = 999.0;                      // vpitch unused
  covariance[35] = angular_velocity_variance;  // vyaw

  return covariance;
}

}  // namespace savo_vo
