#ifndef SAVO_BASE__STATE__STATE_SUMMARY_HPP_
#define SAVO_BASE__STATE__STATE_SUMMARY_HPP_

#include <array>
#include <cstdint>
#include <string>

namespace savo_base::summary
{
struct Parameters
{
  std::string robot_name{"Robot Savo"};
  std::string cmd_topic{"/cmd_vel_safe"};
  std::string safety_stop_topic{"/safety/stop"};
  std::string slowdown_topic{"/safety/slowdown_factor"};
  std::string watchdog_topic{"/savo_base/watchdog_state"};
  std::string base_state_topic{"/savo_base/base_state"};
  std::string motor_board_status_topic{"/savo_base/motor_board_status"};
  bool subscribe_cmd{true};
  bool subscribe_safety_stop{true};
  bool subscribe_slowdown{true};
  bool subscribe_watchdog{true};
  bool subscribe_base_state{true};
  bool subscribe_motor_board_status{false};
  std::string summary_topic{"/savo_base/state_summary"};
  std::string heartbeat_topic{"/savo_base/state_heartbeat"};
  double publish_hz{5.0};
  double heartbeat_hz{1.0};
  double cmd_stale_s{0.30};
  double safety_stale_s{0.50};
  double watchdog_stale_s{1.50};
  double base_state_stale_s{1.50};
  double motor_board_status_stale_s{1.50};
  bool pretty_json{false};
  void clamp();
};

struct Sample
{
  double time{0.0};
  std::uint64_t count{0};
  void receive(double now) {time = now; ++count;}
  double age(double now) const;
  bool stale(double now, double timeout) const {return age(now) >= timeout;}
};

struct JsonSample : Sample
{
  std::string data{"null"};
  std::string status;
  std::uint64_t parse_error_count{0};
  // Only the immediately previous wire value is retained. Even invalid repeats
  // must still refresh receive time and increment the error/receive counters.
  std::string previous_raw;
  bool previous_valid{false};
  void receive_json(const std::string & raw, double now);
};

class StateSummary
{
public:
  explicit StateSummary(Parameters parameters, double started);
  const Parameters & parameters() const {return params_;}
  void command(double vx, double vy, double wz, double now);
  void safety_stop(bool value, double now);
  void slowdown(double value, double now);
  void watchdog(const std::string & raw, double now) {json_[0].receive_json(raw, now);}
  void base_state(const std::string & raw, double now) {json_[1].receive_json(raw, now);}
  void motor_board(const std::string & raw, double now) {json_[2].receive_json(raw, now);}
  std::string status(double now) const;
  // The caller reuses its message string across ticks. A single-threaded executor
  // owns this core; callbacks and summary generation must not run concurrently.
  void serialize(double now, std::string & output);
  void summary_published() {++summary_count_;}

private:
  Parameters params_;
  double started_;
  Sample cmd_, stop_, slowdown_;
  double vx_{0.0}, vy_{0.0}, wz_{0.0};
  bool stop_value_{false};
  double slowdown_value_{1.0};
  std::array<JsonSample, 3> json_;
  std::uint64_t summary_count_{0};
  std::string topics_json_;
  std::string compact_buffer_;
};
}  // namespace savo_base::summary
#endif  // SAVO_BASE__STATE__STATE_SUMMARY_HPP_
