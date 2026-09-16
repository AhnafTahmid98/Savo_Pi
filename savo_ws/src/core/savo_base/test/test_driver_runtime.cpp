#include "gtest/gtest.h"

#include "savo_base/driver_runtime.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace
{
struct FaultingBoard
{
  bool write_fails{true};
  bool stop_fails{false};
  int writes{0};
  int stops{0};

  void write()
  {
    ++writes;
    if (write_fails) {throw std::runtime_error("primary PWM write failed");}
  }
  void stop()
  {
    ++stops;
    if (stop_fails) {throw std::runtime_error("recovery brake write failed");}
  }
};

struct Cycle
{
  savo_base::SafetyDecision decision{false, false, "motion_allowed", 1.0};
  savo_base::WheelDuty duty{100, 100, 100, 100};
  savo_base::BaseCounters counters;
  std::string primary;
  std::string recovery;

  void run(FaultingBoard & board)
  {
    savo_base::driver_detail::run_board_operation(
      [&]() {board.write(); ++counters.board_write_count;},
      [&]() {board.stop();}, decision, duty, counters, primary, recovery);
  }
};

void expect_zero(const Cycle & cycle)
{
  EXPECT_TRUE(cycle.decision.force_zero);
  EXPECT_TRUE(cycle.decision.blocked);
  EXPECT_EQ(cycle.decision.reason, "board_error");
  EXPECT_EQ(cycle.duty.fl, 0);
  EXPECT_EQ(cycle.duty.rl, 0);
  EXPECT_EQ(cycle.duty.fr, 0);
  EXPECT_EQ(cycle.duty.rr, 0);
}

TEST(BaseDriverFailure, PrimaryFailureRecordsErrorAndReportsZeroRatherThanAttemptedDuty)
{
  FaultingBoard board;
  Cycle cycle;
  EXPECT_NO_THROW(cycle.run(board));
  EXPECT_EQ(cycle.primary, "primary PWM write failed");
  EXPECT_TRUE(cycle.recovery.empty());
  EXPECT_EQ(board.stops, 1);
  EXPECT_EQ(cycle.counters.zero_count, 1U);
  EXPECT_EQ(cycle.counters.trip_count, 1U);
  EXPECT_EQ(cycle.counters.board_write_count, 0U);
  expect_zero(cycle);
}

TEST(BaseDriverFailure, RecoveryStopExceptionCannotEscapeOrOverwritePrimaryError)
{
  FaultingBoard board;
  board.stop_fails = true;
  Cycle cycle;
  EXPECT_NO_THROW(cycle.run(board));
  EXPECT_EQ(cycle.primary, "primary PWM write failed");
  EXPECT_EQ(cycle.recovery, "recovery brake write failed");
  EXPECT_EQ(cycle.counters.zero_count, 1U);
  EXPECT_EQ(cycle.counters.trip_count, 1U);
  EXPECT_EQ(cycle.counters.board_write_count, 0U);
  expect_zero(cycle);
}

TEST(BaseDriverFailure, SubsequentHealthyCycleCannotResumeMotionOrEraseFault)
{
  FaultingBoard board;
  Cycle cycle;
  cycle.run(board);
  board.write_fails = false;
  cycle.decision = {false, false, "motion_allowed", 1.0};
  cycle.duty = {200, 200, 200, 200};
  cycle.run(board);
  EXPECT_EQ(board.writes, 1);
  EXPECT_EQ(board.stops, 2);
  EXPECT_EQ(cycle.primary, "primary PWM write failed");
  EXPECT_EQ(cycle.counters.board_write_count, 0U);
  EXPECT_EQ(cycle.counters.zero_count, 2U);
  EXPECT_EQ(cycle.counters.trip_count, 2U);
  expect_zero(cycle);
}

TEST(BaseDriverFailure, PersistentStopFaultRemainsContainedOnLaterTicks)
{
  FaultingBoard board;
  Cycle cycle;
  cycle.run(board);
  board.stop_fails = true;
  EXPECT_NO_THROW(cycle.run(board));
  EXPECT_NO_THROW(cycle.run(board));
  EXPECT_EQ(board.writes, 1);
  EXPECT_EQ(board.stops, 3);
  EXPECT_EQ(cycle.primary, "primary PWM write failed");
  EXPECT_EQ(cycle.recovery, "recovery brake write failed");
  expect_zero(cycle);
}

TEST(BaseDriverFailure, NormalBoardOperationIsUnchanged)
{
  FaultingBoard board;
  board.write_fails = false;
  Cycle cycle;
  cycle.run(board);
  EXPECT_EQ(board.writes, 1);
  EXPECT_EQ(board.stops, 0);
  EXPECT_EQ(cycle.counters.board_write_count, 1U);
  EXPECT_EQ(cycle.counters.zero_count, 0U);
  EXPECT_FALSE(cycle.decision.force_zero);
  EXPECT_TRUE(cycle.primary.empty());
}

TEST(BaseDriverFailure, FailedNormalStopIsContainedJustLikeFailedWrite)
{
  FaultingBoard board;
  board.stop_fails = true;
  Cycle cycle;
  EXPECT_NO_THROW(savo_base::driver_detail::run_board_operation(
      [&]() {board.stop();}, [&]() {board.stop();}, cycle.decision, cycle.duty,
      cycle.counters, cycle.primary, cycle.recovery));
  EXPECT_EQ(board.writes, 0);
  EXPECT_EQ(board.stops, 2);
  EXPECT_EQ(cycle.primary, "recovery brake write failed");
  EXPECT_EQ(cycle.recovery, "recovery brake write failed");
  EXPECT_EQ(cycle.counters.zero_count, 1U);
  expect_zero(cycle);
}

TEST(BaseDriverFailure, NonStandardSecondaryExceptionIsContained)
{
  Cycle cycle;
  EXPECT_NO_THROW(savo_base::driver_detail::run_board_operation(
      []() {throw std::runtime_error("primary");}, []() {throw 42;},
      cycle.decision, cycle.duty, cycle.counters, cycle.primary, cycle.recovery));
  EXPECT_EQ(cycle.primary, "primary");
  EXPECT_EQ(cycle.recovery, "unknown recovery stop exception");
  expect_zero(cycle);
}

TEST(BaseDriverFailure, EmptyExceptionMessageCannotBypassFaultHold)
{
  Cycle cycle;
  savo_base::driver_detail::run_board_operation(
    []() {throw std::runtime_error("");}, []() {},
    cycle.decision, cycle.duty, cycle.counters, cycle.primary, cycle.recovery);
  EXPECT_FALSE(cycle.primary.empty());
  FaultingBoard board;
  board.write_fails = false;
  cycle.run(board);
  EXPECT_EQ(board.writes, 0);
  expect_zero(cycle);
}

TEST(BaseDriverFailure, DiagnosticJsonKeepsPrimaryAndSecondaryErrorsDistinct)
{
  FaultingBoard board;
  board.stop_fails = true;
  Cycle cycle;
  cycle.run(board);
  savo_base::BaseRuntimeState state;
  state.last_board_error = cycle.primary;
  state.last_recovery_stop_error = cycle.recovery;
  state.safety_decision = cycle.decision;
  state.last_duty = cycle.duty;
  state.counters = cycle.counters;
  const auto json = savo_base::BaseStateJson::make(savo_base::BaseDriverConfig{}, state);
  EXPECT_NE(json.find("\"status_level\":\"ERROR\""), std::string::npos);
  EXPECT_NE(json.find("\"last_board_error\":\"primary PWM write failed\""), std::string::npos);
  EXPECT_NE(json.find("\"last_recovery_stop_error\":\"recovery brake write failed\""),
    std::string::npos);
  EXPECT_NE(json.find("\"last_duty\":{\"fl\":0,\"rl\":0,\"fr\":0,\"rr\":0}"),
    std::string::npos);
}

TEST(BaseDriverFailure, NormalIdleStillWritesStopOnEveryTick)
{
  FaultingBoard board;
  Cycle cycle;
  for (int tick = 0; tick < 30; ++tick) {
    savo_base::driver_detail::run_board_operation(
      [&]() {board.stop(); ++cycle.counters.zero_count;}, [&]() {board.stop();},
      cycle.decision, cycle.duty, cycle.counters, cycle.primary, cycle.recovery);
  }
  EXPECT_EQ(board.stops, 30);
  EXPECT_EQ(board.writes, 0);
  EXPECT_EQ(cycle.counters.zero_count, 30U);
  EXPECT_TRUE(cycle.primary.empty());
}

TEST(BaseDriverMain, NormalShutdownReturnsZero)
{
  std::vector<std::string> events;
  const int result = savo_base::driver_detail::run_main(
    [&]() {events.push_back("run");},
    [&](const char *) {events.push_back("fatal");},
    [&]() {events.push_back("shutdown");});
  EXPECT_EQ(result, 0);
  EXPECT_EQ(events, (std::vector<std::string>{"run", "shutdown"}));
}

TEST(BaseDriverMain, FatalExceptionReturnsFailureAndStillShutsDown)
{
  std::vector<std::string> events;
  const int result = savo_base::driver_detail::run_main(
    []() {throw std::runtime_error("fatal board startup");},
    [&](const char * message) {events.push_back(message);},
    [&]() {events.push_back("shutdown");});
  EXPECT_EQ(result, 1);
  EXPECT_EQ(events, (std::vector<std::string>{"fatal board startup", "shutdown"}));
}
}  // namespace
