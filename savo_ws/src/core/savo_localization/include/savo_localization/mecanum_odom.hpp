#pragma once

#include <array>
#include <cstdint>

#include "savo_localization/encoder_state.hpp"

namespace savo_localization
{

constexpr double PI = 3.14159265358979323846;

struct MecanumGeometry
{
  double wheelbase_m{0.165};
  double track_m{0.165};
  double wheel_diameter_m{0.065};
  int counts_per_wheel_rev{80};

  bool valid() const;
  double radius_sum_m() const;
  double wheel_circumference_m() const;
  double metres_per_count() const;
};

struct WheelSpeeds
{
  double fl_mps{0.0};
  double fr_mps{0.0};
  double rl_mps{0.0};
  double rr_mps{0.0};

  std::array<double, 4> as_array() const;
  double max_abs_speed() const;
};

struct BodyVelocity
{
  double vx_mps{0.0};
  double vy_mps{0.0};
  double omega_rad_s{0.0};

  bool near_zero(
    double linear_tolerance_mps = 1e-6,
    double angular_tolerance_rad_s = 1e-6) const;
};

struct Pose2D
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

struct Twist2D
{
  double vx_mps{0.0};
  double vy_mps{0.0};
  double omega_rad_s{0.0};
};

struct WheelOdomSample
{
  double stamp_s{0.0};
  double dt_s{0.0};

  Pose2D pose{};
  Twist2D twist{};
  WheelSpeeds wheel_speeds{};

  int active_wheel_count{0};
  std::uint64_t total_illegal_transitions{0};
};

class MecanumOdom
{
public:
  explicit MecanumOdom(MecanumGeometry geometry = MecanumGeometry{});

  void reset(
    double x_m = 0.0,
    double y_m = 0.0,
    double yaw_rad = 0.0);

  void set_geometry(const MecanumGeometry & geometry);
  MecanumGeometry geometry() const;

  Pose2D pose() const;
  Twist2D twist() const;

  double total_distance_m() const;
  double total_rotation_rad() const;
  std::uint64_t sample_count() const;

  WheelOdomSample update_from_wheel_speeds(
    const WheelSpeeds & wheel_speeds,
    double dt_s,
    double stamp_s,
    int active_wheel_count = 0,
    std::uint64_t total_illegal_transitions = 0);

  WheelOdomSample update_from_encoder_sample(
    const EncoderSample & encoder_sample);

private:
  MecanumGeometry geometry_{};

  Pose2D pose_{};
  Twist2D twist_{};

  double total_distance_m_{0.0};
  double total_rotation_rad_{0.0};
  std::uint64_t sample_count_{0};

  Pose2D integrate_pose(
    const Pose2D & pose,
    const Twist2D & twist,
    double dt_s) const;
};

double normalize_yaw_rad(double yaw_rad);

BodyVelocity forward_kinematics(
  const WheelSpeeds & wheel_speeds,
  const MecanumGeometry & geometry);

WheelSpeeds inverse_kinematics(
  const BodyVelocity & body_velocity,
  const MecanumGeometry & geometry);

WheelSpeeds wheel_speeds_from_encoder_sample(
  const EncoderSample & sample);

WheelSpeeds wheel_speeds_from_array(
  const std::array<double, 4> & speeds_mps);

BodyVelocity body_velocity_from_encoder_sample(
  const EncoderSample & sample,
  const MecanumGeometry & geometry);

void body_velocity_to_world(
  double vx_mps,
  double vy_mps,
  double yaw_rad,
  double & world_vx_mps,
  double & world_vy_mps);

double clamp_dt_s(double dt_s);

bool pose_finite(const Pose2D & pose);
bool twist_finite(const Twist2D & twist);
bool wheel_speeds_finite(const WheelSpeeds & wheel_speeds);

}  // namespace savo_localization