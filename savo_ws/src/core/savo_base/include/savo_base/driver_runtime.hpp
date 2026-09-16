#pragma once

#include "savo_base/base_state.hpp"

#include <exception>
#include <string>

namespace savo_base::driver_detail
{
template<typename Operation, typename Stop>
void run_board_operation(
  Operation operation, Stop stop, SafetyDecision & decision, WheelDuty & duty,
  BaseCounters & counters, std::string & primary_error, std::string & recovery_error)
{
  const auto hold_after_fault = [&]() {
      decision.force_zero = true;
      decision.blocked = true;
      decision.reason = "board_error";
      // Requested safe output, never evidence that failed I2C writes reached the board.
      duty = {};
      ++counters.zero_count;
      ++counters.trip_count;
      try {
        stop();
      } catch (const std::exception & stop_exc) {
        recovery_error = stop_exc.what();
      } catch (...) {
        recovery_error = "unknown recovery stop exception";
      }
    };

  // A successful brake retry is not authorization to resume the old motion command.
  // Keep the original fault visible until this driver is explicitly restarted.
  if (!primary_error.empty()) {
    hold_after_fault();
    return;
  }
  try {
    operation();
  } catch (const std::exception & exc) {
    primary_error = exc.what();
    if (primary_error.empty()) {primary_error = "board operation exception";}
    hold_after_fault();
  }
}

template<typename Run, typename Fatal, typename Shutdown>
int run_main(Run run, Fatal fatal, Shutdown shutdown)
{
  int exit_code = 0;
  try {
    run();
  } catch (const std::exception & exc) {
    exit_code = 1;
    fatal(exc.what());
  }
  shutdown();
  return exit_code;
}
}  // namespace savo_base::driver_detail
