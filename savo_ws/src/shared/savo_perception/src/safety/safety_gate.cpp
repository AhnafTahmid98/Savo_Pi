#include "savo_perception/safety_gate.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <utility>

namespace savo_perception
{
namespace
{

bool command_is_zero(const VelocityCommand & command)
{
  constexpr double eps = 1e-9;

  return std::abs(command.linear_x_mps) <= eps &&
    std::abs(command.linear_y_mps) <= eps &&
    std::abs(command.angular_z_radps) <= eps;
}

SafetyGateOutput stopped_output(std::string reason)
{
  SafetyGateOutput output;
  output.output_command = make_zero_velocity_command("safety_gate");
  output.allowed = false;
  output.stopped = true;
  output.stale = false;
  output.applied_slowdown = 0.0;
  output.reason = std::move(reason);
  return output;
}

SafetyGateOutput stale_output(std::string reason)
{
  auto output = stopped_output(std::move(reason));
  output.stale = true;
  return output;
}

bool recovery_command_usable(const SafetyGateInput & input, const SafetyGateConfig & config)
{
  if (!config.allow_recovery_when_stopped || !input.recovery_requested) {
    return false;
  }

  if (!input.recovery_command.has_value()) {
    return false;
  }

  const auto & cmd = *input.recovery_command;
  return cmd.valid && finite_velocity_command(cmd) && !cmd.stale(config.cmd_timeout_s);
}

double effective_slowdown(const SafetyGateInput & input, const SafetyGateConfig & config)
{
  auto slowdown = input.slowdown_factor.value_or(input.safety_decision.slowdown_factor);

  if (input.has_safety_decision) {
    slowdown = std::min(slowdown, input.safety_decision.slowdown_factor);
  }

  return clamp_slowdown(slowdown, config.slowdown_min, config.slowdown_max);
}

}  // namespace

VelocityCommand clamp_velocity_command(
  const VelocityCommand & command,
  const SafetyGateConfig & config)
{
  auto out = command;

  out.linear_x_mps = clamp_value(
    out.linear_x_mps,
    -std::abs(config.max_linear_x_mps),
    std::abs(config.max_linear_x_mps));

  out.linear_y_mps = clamp_value(
    out.linear_y_mps,
    -std::abs(config.max_linear_y_mps),
    std::abs(config.max_linear_y_mps));

  out.angular_z_radps = clamp_value(
    out.angular_z_radps,
    -std::abs(config.max_angular_z_radps),
    std::abs(config.max_angular_z_radps));

  return out;
}

VelocityCommand apply_slowdown(
  const VelocityCommand & command,
  const double slowdown_factor,
  const SafetyGateConfig & config)
{
  auto out = command;
  const auto factor = clamp_slowdown(
    slowdown_factor,
    config.slowdown_min,
    config.slowdown_max);

  if (config.apply_slowdown_to_linear) {
    out.linear_x_mps *= factor;
    out.linear_y_mps *= factor;
  }

  if (config.apply_slowdown_to_angular) {
    out.angular_z_radps *= factor;
  }

  return out;
}

SafetyGateOutput gate_velocity_command(
  const SafetyGateInput & input,
  const SafetyGateConfig & config)
{
  if (!input.has_command && !recovery_command_usable(input, config)) {
    return stale_output("no_command");
  }

  if (config.fail_safe_on_stale && input.safety_stop_stale) {
    return stale_output("safety_stop_stale");
  }

  if (config.fail_safe_on_stale && input.slowdown_stale) {
    return stale_output("slowdown_stale");
  }

  const auto safety_stop_active =
    input.safety_stop ||
    (input.has_safety_decision && input.safety_decision.stop_required);

  if (safety_stop_active && !recovery_command_usable(input, config)) {
    return stopped_output("safety_stop");
  }

  auto selected_command = input.command;

  if (safety_stop_active && recovery_command_usable(input, config)) {
    selected_command = *input.recovery_command;
  }

  if (!selected_command.valid || !finite_velocity_command(selected_command)) {
    return stopped_output("invalid_command");
  }

  if (selected_command.stale(config.cmd_timeout_s)) {
    if (config.publish_zero_on_stale) {
      return stale_output("command_stale");
    }

    SafetyGateOutput output;
    output.output_command = selected_command;
    output.allowed = false;
    output.stopped = command_is_zero(selected_command);
    output.stale = true;
    output.applied_slowdown = 0.0;
    output.reason = "command_stale";
    return output;
  }

  const auto slowdown = effective_slowdown(input, config);
  auto safe_command = clamp_velocity_command(selected_command, config);
  safe_command = apply_slowdown(safe_command, slowdown, config);
  safe_command = clamp_velocity_command(safe_command, config);
  safe_command.source = safety_stop_active ? "safety_gate_recovery" : "safety_gate";

  SafetyGateOutput output;
  output.output_command = safe_command;
  output.allowed = true;
  output.stopped = command_is_zero(safe_command);
  output.stale = false;
  output.applied_slowdown = slowdown;
  output.reason = safety_stop_active ? "recovery_allowed" : "command_allowed";

  if (output.stopped && slowdown <= 0.0) {
    output.reason = "slowdown_zero";
  }

  return output;
}

}  // namespace savo_perception
