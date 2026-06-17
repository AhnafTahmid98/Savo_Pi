#include "savo_base/base_state.hpp"

#include <iomanip>
#include <sstream>

namespace savo_base
{

std::string BaseStateJson::make(
  const BaseDriverConfig & config,
  const BaseRuntimeState & state)
{
  std::ostringstream ss;

  ss << "{";
  ss << "\"node\":\"" << escape_json(state.node_name) << "\",";
  ss << "\"robot_name\":\"" << escape_json(config.robot_name) << "\",";
  ss << "\"status_level\":\"" << status_level(state) << "\",";

  ss << "\"backend\":{";
  ss << "\"board_backend\":\"" << escape_json(config.board.backend) << "\",";
  ss << "\"board_name\":\"" << escape_json(config.board.name) << "\",";
  ss << "\"connected\":" << bool_text(state.connected) << ",";
  ss << "\"i2c_bus\":" << config.board.i2c_bus << ",";
  ss << "\"pca9685_addr\":\"" << hex_addr(config.board.address) << "\",";
  ss << "\"pwm_freq_hz\":" << config.board.pwm_freq_hz << ",";
  ss << "\"quench_ms\":" << config.board.quench_ms;
  ss << "},";

  ss << "\"inputs\":{";
  ss << "\"cmd_topic\":\"" << escape_json(config.cmd_topic) << "\",";
  ss << "\"safety_stop_topic\":\"" << escape_json(config.safety_stop_topic) << "\",";
  ss << "\"slowdown_topic\":\"" << escape_json(config.slowdown_topic) << "\"";
  ss << "},";

  ss << "\"command\":{";
  ss << "\"seq\":" << state.counters.command_seq << ",";
  ss << "\"vx\":" << state.command.vx << ",";
  ss << "\"vy\":" << state.command.vy << ",";
  ss << "\"wz\":" << state.command.wz << ",";
  ss << "\"age_s\":" << state.command.age_s << ",";
  ss << "\"stale\":" << bool_text(state.command.stale);
  ss << "},";

  ss << "\"safety\":{";
  ss << "\"safety_stop\":" << bool_text(state.safety_stop) << ",";
  ss << "\"slowdown_factor\":" << state.safety_decision.slowdown_factor << ",";
  ss << "\"watchdog_timeout_s\":" << config.watchdog_timeout_s << ",";
  ss << "\"watchdog\":{";
  ss << "\"tripped\":" << bool_text(state.watchdog.tripped) << ",";
  ss << "\"reason\":\"" << escape_json(state.watchdog.reason) << "\"";
  ss << "},";
  ss << "\"policy_decision\":{";
  ss << "\"force_zero\":" << bool_text(state.safety_decision.force_zero) << ",";
  ss << "\"blocked\":" << bool_text(state.safety_decision.blocked) << ",";
  ss << "\"reason\":\"" << escape_json(state.safety_decision.reason) << "\"";
  ss << "}";
  ss << "},";

  ss << "\"limits\":{";
  ss << "\"vx_limit\":" << config.velocity.vx << ",";
  ss << "\"vy_limit\":" << config.velocity.vy << ",";
  ss << "\"wz_limit\":" << config.velocity.wz << ",";
  ss << "\"max_duty\":" << config.motor.max_duty << ",";
  ss << "\"enable_breakaway_compensation\":"
     << bool_text(config.motor.enable_breakaway_compensation) << ",";
  ss << "\"min_motion_duty\":" << config.motor.min_motion_duty << ",";
  ss << "\"breakaway_trigger_duty\":" << config.motor.breakaway_trigger_duty << ",";
  ss << "\"turn_gain\":" << config.turn_gain;
  ss << "},";

  ss << "\"conventions\":{";
  ss << "\"forward_sign\":" << config.signs.forward << ",";
  ss << "\"strafe_sign\":" << config.signs.strafe << ",";
  ss << "\"rotate_sign\":" << config.signs.rotate << ",";
  ss << "\"invert_fl\":" << bool_text(config.inverts.fl) << ",";
  ss << "\"invert_rl\":" << bool_text(config.inverts.rl) << ",";
  ss << "\"invert_fr\":" << bool_text(config.inverts.fr) << ",";
  ss << "\"invert_rr\":" << bool_text(config.inverts.rr);
  ss << "},";

  ss << "\"last_duty\":{";
  ss << "\"fl\":" << state.last_duty.fl << ",";
  ss << "\"rl\":" << state.last_duty.rl << ",";
  ss << "\"fr\":" << state.last_duty.fr << ",";
  ss << "\"rr\":" << state.last_duty.rr;
  ss << "},";

  ss << "\"counters\":{";
  ss << "\"loop_count\":" << state.counters.loop_count << ",";
  ss << "\"board_write_count\":" << state.counters.board_write_count << ",";
  ss << "\"zero_count\":" << state.counters.zero_count << ",";
  ss << "\"trip_count\":" << state.counters.trip_count;
  ss << "},";

  ss << "\"diagnostics\":{";
  ss << "\"last_board_error\":\"" << escape_json(state.last_board_error) << "\"";
  ss << "}";

  ss << "}";
  return ss.str();
}

std::string BaseStateJson::bool_text(const bool value)
{
  return value ? "true" : "false";
}

std::string BaseStateJson::hex_addr(const int address)
{
  std::ostringstream ss;
  ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << address;
  return ss.str();
}

std::string BaseStateJson::status_level(const BaseRuntimeState & state)
{
  if (!state.last_board_error.empty()) {
    return "ERROR";
  }

  if (state.safety_decision.blocked) {
    return "BLOCKED";
  }

  if (state.safety_decision.force_zero) {
    return "STALE";
  }

  return "OK";
}

std::string BaseStateJson::escape_json(const std::string & text)
{
  std::ostringstream ss;

  for (const char ch : text) {
    switch (ch) {
      case '\\':
        ss << "\\\\";
        break;
      case '"':
        ss << "\\\"";
        break;
      case '\n':
        ss << "\\n";
        break;
      case '\r':
        ss << "\\r";
        break;
      case '\t':
        ss << "\\t";
        break;
      default:
        ss << ch;
        break;
    }
  }

  return ss.str();
}

}  // namespace savo_base
