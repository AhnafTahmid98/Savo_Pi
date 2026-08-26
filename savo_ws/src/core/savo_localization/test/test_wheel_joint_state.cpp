#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "savo_localization/wheel_joint_state.hpp"

namespace savo_localization
{
namespace
{

constexpr int COUNTS_PER_WHEEL_REV = 80;

EncoderSample make_sample(
  const std::array<std::int64_t, 4> & counts,
  const std::array<double, 4> & counts_per_second)
{
  EncoderSample sample{};
  sample.fl.count = counts[0];
  sample.fr.count = counts[1];
  sample.rl.count = counts[2];
  sample.rr.count = counts[3];
  sample.fl.counts_per_second = counts_per_second[0];
  sample.fr.counts_per_second = counts_per_second[1];
  sample.rl.counts_per_second = counts_per_second[2];
  sample.rr.counts_per_second = counts_per_second[3];
  return sample;
}

TEST(WheelJointStateTest, UsesExactUrdfWheelJointNames)
{
  static_assert(WHEEL_JOINT_NAMES.size() == 4);
  EXPECT_EQ(std::string(WHEEL_JOINT_NAMES[0]), "wheel_fl_link_joint");
  EXPECT_EQ(std::string(WHEEL_JOINT_NAMES[1]), "wheel_fr_link_joint");
  EXPECT_EQ(std::string(WHEEL_JOINT_NAMES[2]), "wheel_rl_link_joint");
  EXPECT_EQ(std::string(WHEEL_JOINT_NAMES[3]), "wheel_rr_link_joint");
}

TEST(WheelJointStateTest, ZeroCountStartupProducesFourZeroValues)
{
  const WheelJointState state = wheel_joint_state_from_encoder_sample(
    EncoderSample{}, COUNTS_PER_WHEEL_REV);

  EXPECT_EQ(state.position_rad.size(), 4U);
  EXPECT_EQ(state.velocity_rad_s.size(), 4U);
  EXPECT_EQ(state.position_rad, (std::array<double, 4>{0.0, 0.0, 0.0, 0.0}));
  EXPECT_EQ(state.velocity_rad_s, (std::array<double, 4>{0.0, 0.0, 0.0, 0.0}));
}

TEST(WheelJointStateTest, ConvertsSignedCumulativeCountsToRadians)
{
  const EncoderSample sample = make_sample(
    {80, -80, 40, -40},
    {0.0, 0.0, 0.0, 0.0});
  const WheelJointState state = wheel_joint_state_from_encoder_sample(
    sample, COUNTS_PER_WHEEL_REV);

  EXPECT_DOUBLE_EQ(state.position_rad[0], TWO_PI);
  EXPECT_DOUBLE_EQ(state.position_rad[1], -TWO_PI);
  EXPECT_DOUBLE_EQ(state.position_rad[2], TWO_PI / 2.0);
  EXPECT_DOUBLE_EQ(state.position_rad[3], -TWO_PI / 2.0);
}

TEST(WheelJointStateTest, ConvertsSignedCountRateToRadiansPerSecond)
{
  const EncoderSample sample = make_sample(
    {0, 0, 0, 0},
    {80.0, -80.0, 20.0, -20.0});
  const WheelJointState state = wheel_joint_state_from_encoder_sample(
    sample, COUNTS_PER_WHEEL_REV);

  EXPECT_DOUBLE_EQ(state.velocity_rad_s[0], TWO_PI);
  EXPECT_DOUBLE_EQ(state.velocity_rad_s[1], -TWO_PI);
  EXPECT_DOUBLE_EQ(state.velocity_rad_s[2], TWO_PI / 4.0);
  EXPECT_DOUBLE_EQ(state.velocity_rad_s[3], -TWO_PI / 4.0);
}

TEST(WheelJointStateTest, PreservesLargeCumulativeCountsWithoutWrapping)
{
  constexpr std::int64_t LARGE_COUNT = 1'000'000'000;
  const EncoderSample sample = make_sample(
    {LARGE_COUNT, -LARGE_COUNT, LARGE_COUNT, -LARGE_COUNT},
    {0.0, 0.0, 0.0, 0.0});
  const WheelJointState state = wheel_joint_state_from_encoder_sample(
    sample, COUNTS_PER_WHEEL_REV);
  const double expected =
    static_cast<double>(LARGE_COUNT) * TWO_PI / COUNTS_PER_WHEEL_REV;

  EXPECT_DOUBLE_EQ(state.position_rad[0], expected);
  EXPECT_DOUBLE_EQ(state.position_rad[1], -expected);
  EXPECT_DOUBLE_EQ(state.position_rad[2], expected);
  EXPECT_DOUBLE_EQ(state.position_rad[3], -expected);
}

TEST(WheelJointStateTest, ConversionDoesNotMutateEncoderInput)
{
  EncoderSample sample = make_sample(
    {10, -20, 30, -40},
    {1.0, -2.0, 3.0, -4.0});
  const EncoderSample before = sample;

  (void)wheel_joint_state_from_encoder_sample(sample, COUNTS_PER_WHEEL_REV);

  EXPECT_EQ(sample.fl.count, before.fl.count);
  EXPECT_EQ(sample.fr.count, before.fr.count);
  EXPECT_EQ(sample.rl.count, before.rl.count);
  EXPECT_EQ(sample.rr.count, before.rr.count);
  EXPECT_DOUBLE_EQ(sample.fl.counts_per_second, before.fl.counts_per_second);
  EXPECT_DOUBLE_EQ(sample.fr.counts_per_second, before.fr.counts_per_second);
  EXPECT_DOUBLE_EQ(sample.rl.counts_per_second, before.rl.counts_per_second);
  EXPECT_DOUBLE_EQ(sample.rr.counts_per_second, before.rr.counts_per_second);
}

TEST(WheelJointStateTest, RejectsInvalidEffectiveEncoderResolution)
{
  EXPECT_THROW(
    wheel_joint_state_from_encoder_sample(EncoderSample{}, 0),
    std::invalid_argument);
  EXPECT_THROW(
    wheel_joint_state_from_encoder_sample(EncoderSample{}, -1),
    std::invalid_argument);
}

}  // namespace
}  // namespace savo_localization
