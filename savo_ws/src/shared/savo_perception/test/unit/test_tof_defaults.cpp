#include <gtest/gtest.h>

#include "savo_perception/vl53l1x_driver.hpp"

TEST(TofDefaults, UsesRewiredLeftChannelWithoutChangingBusOrAddresses)
{
  const savo_perception::Vl53MuxPairConfig config;
  EXPECT_EQ(config.left_channel, 7);
  EXPECT_EQ(config.right_channel, 3);
  EXPECT_EQ(config.bus, 1);
  EXPECT_EQ(config.tca_address, 0x70);
  EXPECT_EQ(config.sensor_address, 0x29);
}
