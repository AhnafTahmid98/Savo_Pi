#include "gtest/gtest.h"

#include "savo_control/recovery_manager.hpp"

namespace
{

constexpr double kLargeTimestamp = 1.78795e9;

savo_control::RecoveryManagerConfig timing_test_config()
{
  savo_control::RecoveryManagerConfig config;
  config.backup_duration_s = 1.0;
  config.max_recovery_duration_s = 8.0;
  config.stop_before_motion = false;
  config.stop_after_motion = false;
  return config;
}

TEST(RecoveryManagerTiming, IdleAndResetDoNotAccumulateElapsedTime)
{
  savo_control::RecoveryManager manager(timing_test_config());

  const auto initial = manager.update(kLargeTimestamp);
  EXPECT_EQ(initial.state, savo_control::RecoveryState::IDLE);
  EXPECT_FALSE(initial.active);
  EXPECT_DOUBLE_EQ(initial.elapsed_s, 0.0);
  EXPECT_DOUBLE_EQ(initial.phase_elapsed_s, 0.0);

  ASSERT_TRUE(manager.request(
    kLargeTimestamp,
    savo_control::RecoveryTrigger::MANUAL_REQUEST,
    savo_control::RecoveryAction::BACKUP));

  const auto active = manager.update(kLargeTimestamp + 0.25);
  EXPECT_EQ(active.state, savo_control::RecoveryState::BACKING_UP);
  EXPECT_TRUE(active.active);
  EXPECT_DOUBLE_EQ(active.elapsed_s, 0.25);
  EXPECT_DOUBLE_EQ(active.phase_elapsed_s, 0.25);

  manager.reset();
  const auto reset = manager.update(kLargeTimestamp + 1000.0);
  EXPECT_EQ(reset.state, savo_control::RecoveryState::IDLE);
  EXPECT_FALSE(reset.active);
  EXPECT_DOUBLE_EQ(reset.elapsed_s, 0.0);
  EXPECT_DOUBLE_EQ(reset.phase_elapsed_s, 0.0);

  ASSERT_TRUE(manager.request(
    0.0,
    savo_control::RecoveryTrigger::MANUAL_REQUEST,
    savo_control::RecoveryAction::BACKUP));
  const auto zero_started = manager.update(0.25);
  EXPECT_EQ(zero_started.state, savo_control::RecoveryState::BACKING_UP);
  EXPECT_TRUE(zero_started.active);
  EXPECT_DOUBLE_EQ(zero_started.elapsed_s, 0.25);
  EXPECT_DOUBLE_EQ(zero_started.phase_elapsed_s, 0.25);
}

TEST(RecoveryManagerTiming, ActiveAndTerminalTimingRemainRelativeToRequest)
{
  auto config = timing_test_config();
  config.backup_duration_s = 0.5;
  savo_control::RecoveryManager manager(config);

  ASSERT_TRUE(manager.request(
    kLargeTimestamp,
    savo_control::RecoveryTrigger::MANUAL_REQUEST,
    savo_control::RecoveryAction::BACKUP));

  const auto active = manager.update(kLargeTimestamp + 0.25);
  EXPECT_EQ(active.state, savo_control::RecoveryState::BACKING_UP);
  EXPECT_TRUE(active.active);
  EXPECT_DOUBLE_EQ(active.elapsed_s, 0.25);
  EXPECT_DOUBLE_EQ(active.phase_elapsed_s, 0.25);

  const auto completed = manager.update(kLargeTimestamp + 0.5);
  EXPECT_EQ(completed.state, savo_control::RecoveryState::COMPLETED);
  EXPECT_TRUE(completed.finished);
  EXPECT_FALSE(completed.active);
  EXPECT_DOUBLE_EQ(completed.elapsed_s, 0.5);
  EXPECT_DOUBLE_EQ(completed.phase_elapsed_s, 0.5);

  const auto terminal = manager.update(kLargeTimestamp + 0.75);
  EXPECT_EQ(terminal.state, savo_control::RecoveryState::COMPLETED);
  EXPECT_TRUE(terminal.finished);
  EXPECT_FALSE(terminal.active);
  EXPECT_DOUBLE_EQ(terminal.elapsed_s, 0.75);
  EXPECT_DOUBLE_EQ(terminal.phase_elapsed_s, 0.75);
}

}  // namespace
