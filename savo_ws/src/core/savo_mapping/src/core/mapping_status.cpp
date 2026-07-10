#include "savo_mapping/mapping_status.hpp"

#include <sstream>
#include <string>

namespace savo_mapping
{

namespace
{

const char * bool_text(bool value)
{
  return value ? "true" : "false";
}

std::string json_escape(const std::string & input)
{
  std::string output;
  output.reserve(input.size());

  for (const char ch : input) {
    switch (ch) {
      case '\\':
        output += "\\\\";
        break;
      case '"':
        output += "\\\"";
        break;
      case '\n':
        output += "\\n";
        break;
      case '\r':
        output += "\\r";
        break;
      case '\t':
        output += "\\t";
        break;
      default:
        output += ch;
        break;
    }
  }

  return output;
}

}  // namespace

MappingStatus make_default_status()
{
  return MappingStatus{};
}

bool has_core_sensor_inputs(const MappingStatus & status)
{
  return status.scan_received;
}

bool has_core_localization_inputs(const MappingStatus & status)
{
  return status.tf_ok && status.odom_ok;
}

bool has_map_data(const MappingStatus & status)
{
  return status.map_received;
}

bool is_mapping_ready(const MappingStatus & status)
{
  return status.healthy &&
         status.ready &&
         status.slam_active &&
         has_core_sensor_inputs(status) &&
         has_core_localization_inputs(status) &&
         has_map_data(status) &&
         is_quality_score_valid(status.quality_score);
}

bool is_navigation_handoff_ready(const MappingStatus & status, double quality_threshold)
{
  return is_mapping_ready(status) &&
         !status.active_map_name.empty() &&
         is_quality_score_navigation_ready(status.quality_score, quality_threshold);
}

bool is_status_consistent(const MappingStatus & status)
{
  if (!is_quality_score_valid(status.quality_score)) {
    return false;
  }

  if (status.ready && !status.healthy) {
    return false;
  }

  if (status.ready && !status.slam_active) {
    return false;
  }

  if (status.ready && !status.scan_received) {
    return false;
  }

  if (status.ready && !status.tf_ok) {
    return false;
  }

  if (status.ready && !status.odom_ok) {
    return false;
  }

  if (is_session_finished(status.session_state) &&
      is_active_phase(status.workflow_phase)) {
    return false;
  }

  return true;
}

std::string readiness_text(const MappingStatus & status)
{
  if (!status.healthy) {
    return "not_ready: unhealthy";
  }

  if (!status.slam_active) {
    return "not_ready: slam_inactive";
  }

  if (!status.scan_received) {
    return "not_ready: missing_scan";
  }

  if (!status.tf_ok) {
    return "not_ready: tf_not_ok";
  }

  if (!status.odom_ok) {
    return "not_ready: odom_not_ok";
  }

  if (!status.map_received) {
    return "not_ready: missing_map";
  }

  if (!is_quality_score_valid(status.quality_score)) {
    return "not_ready: invalid_quality_score";
  }

  return status.ready ? "ready" : "not_ready";
}

std::string make_status_json(const MappingStatus & status)
{
  std::ostringstream out;

  out << "{";
  out << "\"mode\":\"" << std::string{to_string(status.mode)} << "\",";
  out << "\"exploration_mode\":\"" << std::string{to_string(status.exploration_mode)} << "\",";
  out << "\"workflow_phase\":\"" << std::string{to_string(status.workflow_phase)} << "\",";
  out << "\"session_state\":\"" << std::string{to_string(status.session_state)} << "\",";
  out << "\"healthy\":" << bool_text(status.healthy) << ",";
  out << "\"ready\":" << bool_text(status.ready) << ",";
  out << "\"slam_active\":" << bool_text(status.slam_active) << ",";
  out << "\"map_received\":" << bool_text(status.map_received) << ",";
  out << "\"scan_received\":" << bool_text(status.scan_received) << ",";
  out << "\"tf_ok\":" << bool_text(status.tf_ok) << ",";
  out << "\"odom_ok\":" << bool_text(status.odom_ok) << ",";
  out << "\"quality_score\":" << status.quality_score << ",";
  out << "\"heartbeat_seq\":" << status.heartbeat_seq << ",";
  out << "\"active_map_name\":\"" << json_escape(status.active_map_name) << "\",";
  out << "\"message\":\"" << json_escape(status.message) << "\"";
  out << "}";

  return out.str();
}

}  // namespace savo_mapping
