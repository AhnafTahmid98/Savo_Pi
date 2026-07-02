#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "savo_head/core/head_types.hpp"

namespace savo_head
{

inline constexpr double kVectorCmdAbsolute = 0.0;
inline constexpr double kVectorCmdDelta = 1.0;
inline constexpr double kVectorCmdCenter = 2.0;
inline constexpr double kVectorCmdHold = 3.0;
inline constexpr double kVectorCmdStop = 4.0;

inline constexpr const char * kManualKeyTiltUp = "w";
inline constexpr const char * kManualKeyTiltDown = "s";
inline constexpr const char * kManualKeyPanLeft = "a";
inline constexpr const char * kManualKeyPanRight = "d";
inline constexpr const char * kManualKeyCenter = "c";
inline constexpr const char * kManualKeyHold = " ";
inline constexpr const char * kManualKeyStop = "q";

struct VectorPanTiltCommand
{
  double x{0.0};
  double y{0.0};
  double z{kVectorCmdAbsolute};
};

[[nodiscard]] inline bool close_mode(double lhs, double rhs)
{
  return std::isfinite(lhs) && std::isfinite(rhs) && std::abs(lhs - rhs) < 1e-6;
}

[[nodiscard]] inline double vector_mode_from_command_type(CommandType type)
{
  switch (type) {
    case CommandType::kAbsolute:
      return kVectorCmdAbsolute;
    case CommandType::kDelta:
      return kVectorCmdDelta;
    case CommandType::kCenter:
      return kVectorCmdCenter;
    case CommandType::kHold:
      return kVectorCmdHold;
    case CommandType::kStop:
      return kVectorCmdStop;
  }

  return kVectorCmdStop;
}

[[nodiscard]] inline CommandType command_type_from_vector_mode(double mode)
{
  if (close_mode(mode, kVectorCmdDelta)) {
    return CommandType::kDelta;
  }

  if (close_mode(mode, kVectorCmdCenter)) {
    return CommandType::kCenter;
  }

  if (close_mode(mode, kVectorCmdHold)) {
    return CommandType::kHold;
  }

  if (close_mode(mode, kVectorCmdStop)) {
    return CommandType::kStop;
  }

  return CommandType::kAbsolute;
}

[[nodiscard]] inline VectorPanTiltCommand command_to_vector(const PanTiltCommand & command)
{
  VectorPanTiltCommand out;
  out.z = vector_mode_from_command_type(command.type);

  switch (command.type) {
    case CommandType::kAbsolute:
      out.x = static_cast<double>(command.pan_deg.value_or(0));
      out.y = static_cast<double>(command.tilt_deg.value_or(0));
      break;

    case CommandType::kDelta:
      out.x = static_cast<double>(command.pan_delta_deg);
      out.y = static_cast<double>(command.tilt_delta_deg);
      break;

    case CommandType::kCenter:
    case CommandType::kHold:
    case CommandType::kStop:
      out.x = 0.0;
      out.y = 0.0;
      break;
  }

  return out;
}

[[nodiscard]] inline PanTiltCommand command_from_vector(
  const VectorPanTiltCommand & vector,
  double stamp_s = 0.0,
  CommandSource source = CommandSource::kTopic)
{
  const auto type = command_type_from_vector_mode(vector.z);

  switch (type) {
    case CommandType::kDelta:
      return delta_command(
        static_cast<int>(std::llround(vector.x)),
        static_cast<int>(std::llround(vector.y)),
        stamp_s,
        source,
        "vector_delta");

    case CommandType::kCenter:
      return center_command(stamp_s, source, "vector_center");

    case CommandType::kHold:
      return hold_command(stamp_s, source, "vector_hold");

    case CommandType::kStop:
      return stop_command(stamp_s, source, "vector_stop");

    case CommandType::kAbsolute:
      return absolute_command(
        std::optional<int>{static_cast<int>(std::llround(vector.x))},
        std::optional<int>{static_cast<int>(std::llround(vector.y))},
        stamp_s,
        source,
        "vector_absolute");
  }

  return stop_command(stamp_s, source, "vector_unknown");
}

[[nodiscard]] inline std::string trim_lower(std::string value)
{
  value.erase(
    value.begin(),
    std::find_if(
      value.begin(),
      value.end(),
      [](unsigned char ch) {return !std::isspace(ch);}));

  value.erase(
    std::find_if(
      value.rbegin(),
      value.rend(),
      [](unsigned char ch) {return !std::isspace(ch);}).base(),
    value.end());

  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](unsigned char ch) {return static_cast<char>(std::tolower(ch));});

  return value;
}

[[nodiscard]] inline std::optional<PanTiltCommand> manual_key_command(
  std::string key,
  int step_deg = kManualStepDeg,
  double stamp_s = 0.0)
{
  if (key == kManualKeyHold) {
    return hold_command(stamp_s, CommandSource::kManual, "manual_hold");
  }

  key = trim_lower(std::move(key));
  const auto step = std::max(1, step_deg);

  if (key == kManualKeyPanRight) {
    return delta_command(step, 0, stamp_s, CommandSource::kManual, "manual_pan_right");
  }

  if (key == kManualKeyPanLeft) {
    return delta_command(-step, 0, stamp_s, CommandSource::kManual, "manual_pan_left");
  }

  if (key == kManualKeyTiltUp) {
    return delta_command(0, step, stamp_s, CommandSource::kManual, "manual_tilt_up");
  }

  if (key == kManualKeyTiltDown) {
    return delta_command(0, -step, stamp_s, CommandSource::kManual, "manual_tilt_down");
  }

  if (key == kManualKeyCenter) {
    return center_command(stamp_s, CommandSource::kManual, "manual_center");
  }

  if (key == kManualKeyStop) {
    return stop_command(stamp_s, CommandSource::kManual, "manual_stop");
  }

  return std::nullopt;
}

[[nodiscard]] inline std::vector<std::string> command_contract_errors(
  const PanTiltCommand & command,
  const PanTiltLimits & limits = PanTiltLimits{})
{
  std::vector<std::string> errors = command.validation_errors();

  if (command.type == CommandType::kAbsolute) {
    if (command.pan_deg.has_value() && !limits.contains_pan(command.pan_deg.value())) {
      errors.emplace_back("absolute pan target is outside configured limits");
    }

    if (command.tilt_deg.has_value() && !limits.contains_tilt(command.tilt_deg.value())) {
      errors.emplace_back("absolute tilt target is outside configured limits");
    }
  }

  return errors;
}

[[nodiscard]] inline bool command_contract_valid(
  const PanTiltCommand & command,
  const PanTiltLimits & limits = PanTiltLimits{})
{
  return command_contract_errors(command, limits).empty();
}

[[nodiscard]] inline bool command_is_motion(const PanTiltCommand & command)
{
  return command.type == CommandType::kAbsolute ||
    command.type == CommandType::kDelta ||
    command.type == CommandType::kCenter;
}

[[nodiscard]] inline bool command_is_safety_action(const PanTiltCommand & command)
{
  return command.type == CommandType::kStop ||
    command.type == CommandType::kHold ||
    command.source == CommandSource::kEmergency ||
    command.source == CommandSource::kWatchdog;
}

[[nodiscard]] inline int command_priority(const PanTiltCommand & command)
{
  if (command.source == CommandSource::kEmergency) {
    return 100;
  }

  if (command.type == CommandType::kStop) {
    return 90;
  }

  if (command.source == CommandSource::kWatchdog) {
    return 80;
  }

  if (command.type == CommandType::kCenter) {
    return 60;
  }

  return command.priority;
}

}  // namespace savo_head
