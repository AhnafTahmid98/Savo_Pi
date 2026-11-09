// Transport-free differential harness; stdin is one test scenario, stdout one
// JSON string per line (the string contains the actual serialized summary).
#include "savo_base/state/state_summary.hpp"
#include <iostream>
#include <iterator>
#include <string>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

int main()
{
  const std::string input{std::istreambuf_iterator<char>(std::cin), {}};
  rapidjson::Document request;
  request.Parse<rapidjson::kParseNanAndInfFlag>(input.c_str());
  if (request.HasParseError()) {return 2;}
  savo_base::summary::Parameters p;
  const auto & params = request["params"];
  if (params.HasMember("robot_name")) {p.robot_name = params["robot_name"].GetString();}
  if (params.HasMember("cmd_topic")) {p.cmd_topic = params["cmd_topic"].GetString();}
  if (params.HasMember("safety_stop_topic")) {p.safety_stop_topic = params["safety_stop_topic"].GetString();}
  if (params.HasMember("slowdown_topic")) {p.slowdown_topic = params["slowdown_topic"].GetString();}
  if (params.HasMember("watchdog_topic")) {p.watchdog_topic = params["watchdog_topic"].GetString();}
  if (params.HasMember("base_state_topic")) {p.base_state_topic = params["base_state_topic"].GetString();}
  if (params.HasMember("motor_board_status_topic")) {p.motor_board_status_topic = params["motor_board_status_topic"].GetString();}
  if (params.HasMember("subscribe_cmd")) {p.subscribe_cmd = params["subscribe_cmd"].GetBool();}
  if (params.HasMember("subscribe_safety_stop")) {p.subscribe_safety_stop = params["subscribe_safety_stop"].GetBool();}
  if (params.HasMember("subscribe_slowdown")) {p.subscribe_slowdown = params["subscribe_slowdown"].GetBool();}
  if (params.HasMember("subscribe_watchdog")) {p.subscribe_watchdog = params["subscribe_watchdog"].GetBool();}
  if (params.HasMember("subscribe_base_state")) {p.subscribe_base_state = params["subscribe_base_state"].GetBool();}
  if (params.HasMember("subscribe_motor_board_status")) {p.subscribe_motor_board_status = params["subscribe_motor_board_status"].GetBool();}
  if (params.HasMember("summary_topic")) {p.summary_topic = params["summary_topic"].GetString();}
  if (params.HasMember("heartbeat_topic")) {p.heartbeat_topic = params["heartbeat_topic"].GetString();}
  if (params.HasMember("publish_hz")) {p.publish_hz = params["publish_hz"].GetDouble();}
  if (params.HasMember("heartbeat_hz")) {p.heartbeat_hz = params["heartbeat_hz"].GetDouble();}
  if (params.HasMember("cmd_stale_s")) {p.cmd_stale_s = params["cmd_stale_s"].GetDouble();}
  if (params.HasMember("safety_stale_s")) {p.safety_stale_s = params["safety_stale_s"].GetDouble();}
  if (params.HasMember("watchdog_stale_s")) {p.watchdog_stale_s = params["watchdog_stale_s"].GetDouble();}
  if (params.HasMember("base_state_stale_s")) {p.base_state_stale_s = params["base_state_stale_s"].GetDouble();}
  if (params.HasMember("motor_board_status_stale_s")) {p.motor_board_status_stale_s = params["motor_board_status_stale_s"].GetDouble();}
  if (params.HasMember("pretty_json")) {p.pretty_json = params["pretty_json"].GetBool();}
  savo_base::summary::StateSummary state(p, 100.0);
  std::string output;
  for (const auto & event : request["events"].GetArray()) {
    const std::string op = event["op"].GetString();
    const double now = event["t"].GetDouble();
    if (op == "summary") {
      state.serialize(now, output);
      // Escape pretty-mode newlines for the line-oriented test protocol.
      rapidjson::StringBuffer buffer;
      rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
      writer.String(output.data(), static_cast<rapidjson::SizeType>(output.size()));
      std::cout << buffer.GetString() << '\n';
      state.summary_published();
    } else if (op == "cmd") {
      const auto & v = event["value"];
      state.command(v[0].GetDouble(), v[1].GetDouble(), v[2].GetDouble(), now);
    } else if (op == "stop") {
      state.safety_stop(event["value"].GetBool(), now);
    } else if (op == "slowdown") {
      state.slowdown(event["value"].GetDouble(), now);
    } else {
      const auto & v = event["value"];
      const std::string value(v.GetString(), v.GetStringLength());
      if (op == "watchdog") {state.watchdog(value, now);}
      else if (op == "base") {state.base_state(value, now);}
      else if (op == "board") {state.motor_board(value, now);}
      else {return 3;}
    }
  }
}
