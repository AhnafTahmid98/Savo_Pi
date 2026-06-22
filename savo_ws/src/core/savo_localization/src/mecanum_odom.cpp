#include "savo_localization/mecanum_odom.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace savo_localization
{

bool MecanumGeometry::valid() const
{
  return wheelbase_m > 0.0 &&
         track_m > 0.0 &&
         wheel_diameter_m > 0.0 &&
         counts_per_wheel_rev > 0;
}

double MecanumGeometry::radius_sum_m() const
{
  return (wheelbase_m + track_m) / 2.0;
}

double MecanumGeometry::wheel_circumference_m() const
{
  return PI * wheel_diameter_m;
}

double MecanumGeometry::metres_per_count() const
{
  if (counts_per_wheel_rev <= 0) {
    throw std::invalid_argument("counts_per_wheel_rev must be > 0");
  }

  return wheel_circumference_m() / static_cast<double>(counts_per_wheel_rev);
}

std::array<double, 4> WheelSpeeds::as_array() const
{
  return {fl_mps, fr_mps, rl_mps, rr_mps};
}

double WheelSpeeds::max_abs_speed() const
{
  return std::max(
    std::max(std::abs(fl_mps), std::abs(fr_mps)),
    std::max(std::abs(rl_mps), std::abs(rr_mps)));
}

bool BodyVelocity::near_zero(
  double linear_tolerance_mps,
  double angular_tolerance_rad_s) const
{
  return std::abs(vx_mps) <= linear_tolerance_mps &&
         std::abs(vy_mps) <= linear_tolerance_mps &&
         std::abs(omega_rad_s) <= angular_tolerance_rad_s;
}

MecanumOdom::MecanumOdom(MecanumGeometry geometry)
: geometry_(geometry)
{
  if (!geometry_.valid()) {
    throw std::invalid_argument("invalid mecanum geometry");
  }
}

void MecanumOdom::reset(
  double x_m,
  double y_m,
  double yaw_rad)
{
  pose_.x_m = x_m;
  pose_.y_m = y_m;
  pose_.yaw_rad = normalize_yaw_rad(yaw_rad);

  twist_ = Twist2D{};
  total_distance_m_ = 0.0;
  total_rotation_rad_ = 0.0;
  sample_count_ = 0;
}

void MecanumOdom::set_geometry(const MecanumGeometry & geometry)
{
  if (!geometry.valid()) {
    throw std::invalid_argument("invalid mecanum geometry");
  }

  geometry_ = geometry;
}

MecanumGeometry MecanumOdom::geometry() const
{
  return geometry_;
}

Pose2D MecanumOdom::pose() const
{
  return pose_;
}

Twist2D MecanumOdom::twist() const
{
  return twist_;
}

double MecanumOdom::total_distance_m() const
{
  return total_distance_m_;
}

double MecanumOdom::total_rotation_rad() const
{
  return total_rotation_rad_;
}

std::uint64_t MecanumOdom::sample_count() const
{
  return sample_count_;
}

WheelOdomSample MecanumOdom::update_from_wheel_speeds(
  const WheelSpeeds & wheel_speeds,
  double dt_s,
  double stamp_s,
  int active_wheel_count,
  std::uint64_t total_illegal_transitions)
{
  if (!wheel_speeds_finite(wheel_speeds)) {
    throw std::invalid_argument("wheel speeds contain non-finite values");
  }

  const double dt = clamp_dt_s(dt_s);
  const BodyVelocity body_velocity = forward_kinematics(wheel_speeds, geometry_);

  twist_.vx_mps = body_velocity.vx_mps;
  twist_.vy_mps = body_velocity.vy_mps;
  twist_.omega_rad_s = body_velocity.omega_rad_s;

  pose_ = integrate_pose(pose_, twist_, dt);

  const double linear_speed = std::hypot(twist_.vx_mps, twist_.vy_mps);
  total_distance_m_ += linear_speed * dt;
  total_rotation_rad_ += std::abs(twist_.omega_rad_s) * dt;
  ++sample_count_;

  return WheelOdomSample{
    stamp_s,
    dt,
    pose_,
    twist_,
    wheel_speeds,
    active_wheel_count,
    total_illegal_transitions,
  };
}

WheelOdomSample MecanumOdom::update_from_encoder_sample(
  const EncoderSample & encoder_sample)
{
  const WheelSpeeds wheel_speeds = wheel_speeds_from_encoder_sample(encoder_sample);

  return update_from_wheel_speeds(
    wheel_speeds,
    encoder_sample.dt_s,
    encoder_sample.stamp_s,
    encoder_sample.active_wheel_count(),
    encoder_sample.total_illegal_transitions());
}

Pose2D MecanumOdom::integrate_pose(
  const Pose2D & pose,
  const Twist2D & twist,
  double dt_s) const
{
  const double dt = clamp_dt_s(dt_s);

  double world_vx_mps = 0.0;
  double world_vy_mps = 0.0;

  body_velocity_to_world(
    twist.vx_mps,
    twist.vy_mps,
    pose.yaw_rad,
    world_vx_mps,
    world_vy_mps);

  Pose2D next{};
  next.x_m = pose.x_m + world_vx_mps * dt;
  next.y_m = pose.y_m + world_vy_mps * dt;
  next.yaw_rad = normalize_yaw_rad(pose.yaw_rad + twist.omega_rad_s * dt);

  return next;
}

double normalize_yaw_rad(double yaw_rad)
{
  double angle = std::fmod(yaw_rad + PI, 2.0 * PI);

  if (angle < 0.0) {
    angle += 2.0 * PI;
  }

  return angle - PI;
}

BodyVelocity forward_kinematics(
  const WheelSpeeds & wheel_speeds,
  const MecanumGeometry & geometry)
{
  if (!geometry.valid()) {
    throw std::invalid_argument("invalid mecanum geometry");
  }

  const double fl = wheel_speeds.fl_mps;
  const double fr = wheel_speeds.fr_mps;
  const double rl = wheel_speeds.rl_mps;
  const double rr = wheel_speeds.rr_mps;
  const double radius_sum = geometry.radius_sum_m();

  return BodyVelocity{
    (fl + fr + rl + rr) / 4.0,
    (-fl + fr + rl - rr) / 4.0,
    (-fl + fr - rl + rr) / (4.0 * radius_sum),
  };
}

WheelSpeeds inverse_kinematics(
  const BodyVelocity & body_velocity,
  const MecanumGeometry & geometry)
{
  if (!geometry.valid()) {
    throw std::invalid_argument("invalid mecanum geometry");
  }

  const double rotation = geometry.radius_sum_m() * body_velocity.omega_rad_s;
  const double vx = body_velocity.vx_mps;
  const double vy = body_velocity.vy_mps;

  return WheelSpeeds{
    vx - vy - rotation,
    vx + vy + rotation,
    vx + vy - rotation,
    vx - vy + rotation,
  };
}

WheelSpeeds wheel_speeds_from_encoder_sample(
  const EncoderSample & sample)
{
  return WheelSpeeds{
    sample.fl.speed_mps,
    sample.fr.speed_mps,
    sample.rl.speed_mps,
    sample.rr.speed_mps,
  };
}

WheelSpeeds wheel_speeds_from_array(
  const std::array<double, 4> & speeds_mps)
{
  return WheelSpeeds{
    speeds_mps[0],
    speeds_mps[1],
    speeds_mps[2],
    speeds_mps[3],
  };
}

BodyVelocity body_velocity_from_encoder_sample(
  const EncoderSample & sample,
  const MecanumGeometry & geometry)
{
  return forward_kinematics(
    wheel_speeds_from_encoder_sample(sample),
    geometry);
}

void body_velocity_to_world(
  double vx_mps,
  double vy_mps,
  double yaw_rad,
  double & world_vx_mps,
  double & world_vy_mps)
{
  const double yaw = normalize_yaw_rad(yaw_rad);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  world_vx_mps = vx_mps * cos_yaw - vy_mps * sin_yaw;
  world_vy_mps = vx_mps * sin_yaw + vy_mps * cos_yaw;
}

double clamp_dt_s(double dt_s)
{
  if (!std::isfinite(dt_s)) {
    return 0.0;
  }

  return std::max(0.0, dt_s);
}

bool pose_finite(const Pose2D & pose)
{
  return std::isfinite(pose.x_m) &&
         std::isfinite(pose.y_m) &&
         std::isfinite(pose.yaw_rad);
}

bool twist_finite(const Twist2D & twist)
{
  return std::isfinite(twist.vx_mps) &&
         std::isfinite(twist.vy_mps) &&
         std::isfinite(twist.omega_rad_s);
}

bool wheel_speeds_finite(const WheelSpeeds & wheel_speeds)
{
  return std::isfinite(wheel_speeds.fl_mps) &&
         std::isfinite(wheel_speeds.fr_mps) &&
         std::isfinite(wheel_speeds.rl_mps) &&
         std::isfinite(wheel_speeds.rr_mps);
}

}  // namespace savo_localization
