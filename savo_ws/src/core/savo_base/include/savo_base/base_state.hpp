#pragma once

#include "savo_base/base_safety.hpp"
#include "savo_base/base_types.hpp"
#include "savo_base/timeout_watchdog.hpp"

#include <cstdint>
#include <string>

namespace savo_base
{

struct BaseCounters
{
  uint64_t command_seq{0};
  uint64_t loop_count{0};
  uint64_t board_write_count{0};
  uint64_t zero_count{0};
  uint64_t trip_count{0};
};

struct BaseCommandState
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  double age_s{1.0e9};
  bool stale{true};
};

struct BaseRuntimeState
{
  std::string node_name{"base_driver_node_cpp"};
  std::string last_board_error{};

  bool connected{false};
  bool safety_stop{false};

  BaseCommandState command{};
  SafetyDecision safety_decision{};
  WatchdogStatus watchdog{};
  WheelDuty last_duty{};
  BaseCounters counters{};
};

class BaseStateJson
{
public:
  static std::string make(
    const BaseDriverConfig & config,
    const BaseRuntimeState & state);

private:
  static std::string bool_text(bool value);
  static std::string hex_addr(int address);
  static std::string status_level(const BaseRuntimeState & state);
  static std::string escape_json(const std::string & text);
};

}  // namespace savo_base
