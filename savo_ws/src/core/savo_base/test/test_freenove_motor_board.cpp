#include "gtest/gtest.h"

#include "savo_base/freenove_motor_board.hpp"

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace
{

enum class EventKind
{
  WRITE_REGISTER,
  READ_REGISTER,
  WRITE_BRAKE_FRAME,
  SLEEP,
  STOP,
  FORCE_OUTPUTS_OFF,
  CLOSE
};

struct Event
{
  EventKind kind;
  int reg{0};
  int value{0};
};

TEST(FreenoveMotorBoardStartup, MasksOutputsBeforeConfigurationAndRestart)
{
  using namespace savo_base::freenove_detail;
  std::vector<Event> events;

  initialize_pca9685_fail_safe(
    50.0,
    [&events](const int reg, const int value) {
      events.push_back({EventKind::WRITE_REGISTER, reg, value});
    },
    [&events](const int reg) {
      events.push_back({EventKind::READ_REGISTER, reg, 0});
      return kMode1AllCall | kMode1AutoIncrement;
    },
    [&events]() {events.push_back({EventKind::WRITE_BRAKE_FRAME});},
    [&events](const int milliseconds) {
      events.push_back({EventKind::SLEEP, 0, milliseconds});
    });

  ASSERT_EQ(events.size(), 11U);
  EXPECT_EQ(events[0].kind, EventKind::WRITE_REGISTER);
  EXPECT_EQ(events[0].reg, kPcaAllLedOffH);
  EXPECT_EQ(events[0].value, kPcaFullOff);

  EXPECT_EQ(events[1].reg, kPcaMode2);
  EXPECT_EQ(events[1].value, kMode2OutDrv);
  EXPECT_EQ(events[2].reg, kPcaMode1);
  EXPECT_EQ(events[2].value, kMode1AllCall | kMode1AutoIncrement);

  EXPECT_EQ(events[4].kind, EventKind::READ_REGISTER);
  EXPECT_EQ(events[4].reg, kPcaMode1);
  EXPECT_EQ(events[5].reg, kPcaMode1);
  EXPECT_EQ(events[5].value, kMode1AllCall | kMode1AutoIncrement | kMode1Sleep);
  EXPECT_EQ(events[6].reg, kPcaPrescale);
  EXPECT_EQ(events[6].value, 121);
  EXPECT_EQ(events[9].reg, kPcaMode1);
  EXPECT_EQ(
    events[9].value,
    kMode1AllCall | kMode1AutoIncrement | kMode1Restart);

  EXPECT_EQ(events[10].kind, EventKind::WRITE_BRAKE_FRAME);
}

TEST(FreenoveMotorBoardStartup, ReassertsOutputMaskWhenConfigurationFails)
{
  using namespace savo_base::freenove_detail;
  std::vector<Event> events;

  EXPECT_THROW(
    initialize_pca9685_fail_safe(
      50.0,
      [&events](const int reg, const int value) {
        events.push_back({EventKind::WRITE_REGISTER, reg, value});
        if (reg == kPcaPrescale) {
          throw std::runtime_error("simulated prescale failure");
        }
      },
      [](const int) {return kMode1AllCall | kMode1AutoIncrement;},
      [&events]() {events.push_back({EventKind::WRITE_BRAKE_FRAME});},
      [](const int) {}),
    std::runtime_error);

  const auto mask_writes = std::count_if(
    events.begin(),
    events.end(),
    [](const Event & event) {
      return event.kind == EventKind::WRITE_REGISTER &&
             event.reg == kPcaAllLedOffH &&
             event.value == kPcaFullOff;
    });
  EXPECT_EQ(mask_writes, 2);
  EXPECT_EQ(events.back().reg, kPcaAllLedOffH);
  EXPECT_EQ(events.back().value, kPcaFullOff);
  EXPECT_EQ(
    std::count_if(
      events.begin(),
      events.end(),
      [](const Event & event) {return event.kind == EventKind::WRITE_BRAKE_FRAME;}),
    0);
}

TEST(FreenoveMotorBoardStartup, BrakeFrameIsSafeForMotorAndUnusedChannels)
{
  using namespace savo_base::freenove_detail;

  const auto registers = startup_brake_registers(savo_base::WheelChannels{});
  for (int channel = 0; channel < 8; ++channel) {
    EXPECT_EQ(registers[4 * channel], 0x00);
    EXPECT_EQ(registers[4 * channel + 1], 0x10);
    EXPECT_EQ(registers[4 * channel + 2], 0x00);
    EXPECT_EQ(registers[4 * channel + 3], 0x00);
  }
  for (int channel = 8; channel < 16; ++channel) {
    EXPECT_EQ(registers[4 * channel], 0x00);
    EXPECT_EQ(registers[4 * channel + 1], 0x00);
    EXPECT_EQ(registers[4 * channel + 2], 0x00);
    EXPECT_EQ(registers[4 * channel + 3], kPcaFullOff);
  }
}

TEST(FreenoveMotorBoardOutputs, PreservesDirectionAndActiveBrakeEncodings)
{
  using namespace savo_base::freenove_detail;

  const auto positive = motor_pair_pwm(750);
  EXPECT_EQ(positive.channel_a, 0);
  EXPECT_EQ(positive.channel_b, 750);

  const auto negative = motor_pair_pwm(-750);
  EXPECT_EQ(negative.channel_a, 750);
  EXPECT_EQ(negative.channel_b, 0);

  const auto stopped = motor_pair_pwm(0);
  EXPECT_EQ(stopped.channel_a, 4095);
  EXPECT_EQ(stopped.channel_b, 4095);

  const auto full_on = pwm_channel_registers(4095);
  EXPECT_EQ(full_on.on_l, 0x00);
  EXPECT_EQ(full_on.on_h, 0x10);
  EXPECT_EQ(full_on.off_l, 0x00);
  EXPECT_EQ(full_on.off_h, 0x00);

  const auto ordinary_pwm = pwm_channel_registers(750);
  EXPECT_EQ(ordinary_pwm.on_h, 0x00);
  EXPECT_EQ(ordinary_pwm.off_l, 0xEE);
  EXPECT_EQ(ordinary_pwm.off_h, 0x02);
}

TEST(FreenoveMotorBoardDryrun, NeverRequiresHardware)
{
  savo_base::BoardConfig config;
  config.dryrun = true;
  savo_base::FreenoveMotorBoard board(
    config, savo_base::WheelChannels{}, savo_base::WheelInverts{});

  EXPECT_TRUE(board.connected());
  EXPECT_TRUE(board.ping());
  EXPECT_NO_THROW(board.write({100, -100, 200, -200}));
  EXPECT_NO_THROW(board.stop());
  EXPECT_NO_THROW(board.close());
}

TEST(FreenoveMotorBoardShutdown, FallsBackToFullOffAndAlwaysCloses)
{
  std::vector<Event> events;

  savo_base::freenove_detail::shutdown_fail_safe(
    [&events]() {
      events.push_back({EventKind::STOP});
      throw std::runtime_error("simulated stop failure");
    },
    [&events]() {events.push_back({EventKind::FORCE_OUTPUTS_OFF});},
    [&events]() {events.push_back({EventKind::CLOSE});});

  ASSERT_EQ(events.size(), 3U);
  EXPECT_EQ(events[0].kind, EventKind::STOP);
  EXPECT_EQ(events[1].kind, EventKind::FORCE_OUTPUTS_OFF);
  EXPECT_EQ(events[2].kind, EventKind::CLOSE);
}

TEST(FreenoveMotorBoardShutdown, PreservesNormalBrakeStopBeforeClose)
{
  std::vector<Event> events;

  savo_base::freenove_detail::shutdown_fail_safe(
    [&events]() {events.push_back({EventKind::STOP});},
    [&events]() {events.push_back({EventKind::FORCE_OUTPUTS_OFF});},
    [&events]() {events.push_back({EventKind::CLOSE});});

  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].kind, EventKind::STOP);
  EXPECT_EQ(events[1].kind, EventKind::CLOSE);
}

}  // namespace
