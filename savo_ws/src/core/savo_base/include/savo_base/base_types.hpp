#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace savo_base
{

struct AxisSigns
{
  int forward{1};
  int strafe{1};
  int rotate{1};
};

struct WheelInverts
{
  bool fl{true};
  bool rl{true};
  bool fr{true};
  bool rr{true};
};

struct WheelDuty
{
  int fl{0};
  int rl{0};
  int fr{0};
  int rr{0};
};

struct WheelChannels
{
  std::array<int, 2> fl{0, 1};
  std::array<int, 2> rl{3, 2};
  std::array<int, 2> fr{6, 7};
  std::array<int, 2> rr{4, 5};
};

struct MotorLimits
{
  int max_duty{3500};
  bool enable_breakaway_compensation{true};
  int min_motion_duty{850};
  int breakaway_trigger_duty{120};
};

struct VelocityLimits
{
  double vx{0.8};
  double vy{0.7};
  double wz{0.7};
};

struct BoardConfig
{
  std::string backend{"freenove"};
  std::string name{"robot_savo_freenove_mecanum"};
  int i2c_bus{1};
  int address{0x40};
  double pwm_freq_hz{50.0};
  int quench_ms{18};
  bool dryrun{false};
  bool debug{false};
};

struct BaseDriverConfig
{
  std::string robot_name{"Robot Savo"};
  std::string cmd_topic{"/cmd_vel_safe"};
  std::string safety_stop_topic{"/safety/stop"};
  std::string slowdown_topic{"/safety/slowdown_factor"};
  std::string base_state_topic{"/savo_base/base_state"};

  bool use_safety_stop{false};
  bool use_slowdown_factor{false};

  double loop_hz{30.0};
  double watchdog_timeout_s{0.30};
  double turn_gain{1.0};

  AxisSigns signs{};
  WheelInverts inverts{};
  WheelChannels channels{};
  MotorLimits motor{};
  VelocityLimits velocity{};
  BoardConfig board{};
};

}  // namespace savo_base
