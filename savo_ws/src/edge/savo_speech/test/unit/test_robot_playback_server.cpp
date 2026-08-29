// Copyright 2026 Ahnaf Tahmid
#include <gtest/gtest.h>

#include <string>

#include "savo_speech/transport/robot_playback_server.hpp"

namespace savo_speech::transport
{

TEST(RobotPlaybackServerConfigTest, HasSafeDefaults)
{
  const RobotPlaybackServerConfig config;
  EXPECT_EQ(config.socket_path, "/run/savo_speech/playback.sock");
  EXPECT_EQ(config.maximum_wav_bytes, 16U * 1024U * 1024U);
  EXPECT_FALSE(config.require_peer_uid);
  EXPECT_EQ(config.peer_uid, 0U);
  EXPECT_TRUE(config.is_valid());
}

TEST(RobotPlaybackServerConfigTest, AcceptsProductionConfiguration)
{
  RobotPlaybackServerConfig config;
  config.socket_path = "/run/savo_speech/playback.sock";
  config.maximum_wav_bytes = 16U * 1024U * 1024U;
  config.require_peer_uid = true;
  config.peer_uid = 10001U;
  EXPECT_TRUE(config.is_valid());
}

TEST(RobotPlaybackServerConfigTest, RetainsConfiguredPeerUid)
{
  RobotPlaybackServerConfig config;
  config.require_peer_uid = true;
  config.peer_uid = 10001U;
  EXPECT_TRUE(config.require_peer_uid);
  EXPECT_EQ(config.peer_uid, 10001U);
  EXPECT_TRUE(config.is_valid());
}

TEST(RobotPlaybackServerConfigTest, RejectsRelativeAndOversizedPaths)
{
  RobotPlaybackServerConfig config;
  config.socket_path = "playback.sock";
  EXPECT_FALSE(config.is_valid());
  config.socket_path = "/" + std::string(108U, 'x');
  EXPECT_FALSE(config.is_valid());
}

TEST(RobotPlaybackServerConfigTest, RejectsEmptySocketPath)
{
  RobotPlaybackServerConfig config;
  config.socket_path.clear();
  EXPECT_FALSE(config.is_valid());
}

TEST(RobotPlaybackServerConfigTest, RejectsUnsafePayloadLimits)
{
  RobotPlaybackServerConfig config;
  config.maximum_wav_bytes = 43U;
  EXPECT_FALSE(config.is_valid());
  config.maximum_wav_bytes = 64U * 1024U * 1024U + 1U;
  EXPECT_FALSE(config.is_valid());
}

TEST(RobotPlaybackServerConfigTest, AcceptsMaximumPayloadLimit)
{
  RobotPlaybackServerConfig config;
  config.maximum_wav_bytes = 64U * 1024U * 1024U;
  EXPECT_TRUE(config.is_valid());
}

}  // namespace savo_speech::transport
