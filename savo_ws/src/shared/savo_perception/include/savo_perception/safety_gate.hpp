#ifndef SAVO_PERCEPTION__SAFETY_GATE_HPP_
#define SAVO_PERCEPTION__SAFETY_GATE_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <utility>

#include "savo_perception/constants.hpp"
#include "savo_perception/perception_types.hpp"
#include "savo_perception/visibility_control.hpp"

namespace savo_perception
{

struct SAVO_PERCEPTION_PUBLIC VelocityCommand
{
  double linear_x_mps{0.0};
  double linear_y_mps{0.0};
  double angular_z_radps{0.0};

  std::chrono::steady_clock::time_point stamp{std::chrono::steady_clock::now()};
  bool valid{false};
  std::string source;

  [[nodiscard]] double age_s(
    const std::chrono::steady_clock::time_point & now = std::chrono::steady_clock::now()) const
  {
    const auto dt = std::chrono::duration<double>(now - stamp);
    return std::max(0.0, dt.count());
  }

  [[nodiscard]] bool stale(
    const double timeout_s,
    const std::chrono::steady_clock::time_point & now = std::chrono::steady_clock::now()) const
  {
    return age_s(now) > timeout_s;
  }
};

struct SAVO_PERCEPTION_PUBLIC SafetyGateConfig
{
  double stale_timeout_s{constants::kSensorStaleTimeoutSDefault};
  double cmd_timeout_s{constants::kCmdTimeoutSDefault};

  bool fail_safe_on_stale{true};
  bool publish_zero_on_stop{true};
  bool publish_zero_on_stale{true};

  bool apply_slowdown_to_linear{true};
  bool apply_slowdown_to_angular{true};

  double slowdown_min{0.0};
  double slowdown_max{1.0};

  double max_linear_x_mps{constants::kMaxLinearXMpsDefault};
  double max_linear_y_mps{constants::kMaxLinearYMpsDefault};
  double max_angular_z_radps{constants::kMaxAngularZRadpsDefault};

  bool allow_recovery_when_stopped{false};
};

struct SAVO_PERCEPTION_PUBLIC SafetyGateInput
{
  VelocityCommand command;
  bool has_command{false};

  SafetyDecision safety_decision;
  bool has_safety_decision{false};

  std::optional<double> slowdown_factor;
  bool slowdown_stale{true};

  bool safety_stop{true};
  bool safety_stop_stale{true};

  bool recovery_requested{false};
  std::optional<VelocityCommand> recovery_command;
};

struct SAVO_PERCEPTION_PUBLIC SafetyGateOutput
{
  VelocityCommand output_command;

  bool allowed{false};
  bool stopped{true};
  bool stale{true};

  double applied_slowdown{0.0};
  std::string reason{"initial"};
};

inline VelocityCommand make_zero_velocity_command(std::string source = "safety_gate")
{
  VelocityCommand command;
  command.linear_x_mps = 0.0;
  command.linear_y_mps = 0.0;
  command.angular_z_radps = 0.0;
  command.stamp = std::chrono::steady_clock::now();
  command.valid = true;
  command.source = std::move(source);
  return command;
}

inline VelocityCommand make_velocity_command(
  const double linear_x_mps,
  const double linear_y_mps,
  const double angular_z_radps,
  std::string source = {})
{
  VelocityCommand command;
  command.linear_x_mps = linear_x_mps;
  command.linear_y_mps = linear_y_mps;
  command.angular_z_radps = angular_z_radps;
  command.stamp = std::chrono::steady_clock::now();
  command.valid = true;
  command.source = std::move(source);
  return command;
}

inline bool finite_velocity_command(const VelocityCommand & command)
{
  return std::isfinite(command.linear_x_mps) &&
    std::isfinite(command.linear_y_mps) &&
    std::isfinite(command.angular_z_radps);
}

SAVO_PERCEPTION_PUBLIC VelocityCommand clamp_velocity_command(
  const VelocityCommand & command,
  const SafetyGateConfig & config);

SAVO_PERCEPTION_PUBLIC VelocityCommand apply_slowdown(
  const VelocityCommand & command,
  double slowdown_factor,
  const SafetyGateConfig & config);

SAVO_PERCEPTION_PUBLIC SafetyGateOutput gate_velocity_command(
  const SafetyGateInput & input,
  const SafetyGateConfig & config);

}  // namespace savo_perception

#endif  // SAVO_PERCEPTION__SAFETY_GATE_HPP_