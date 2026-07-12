#include <gtest/gtest.h>

#include "savo_speech/constants.hpp"
#include "savo_speech/speech_types.hpp"
#include "savo_speech/topic_names.hpp"

TEST(SpeechTypes, StableStringConversions)
{
  EXPECT_EQ(
    savo_speech::to_string(savo_speech::SpeechState::SLEEPING),
    "SLEEPING");

  EXPECT_EQ(
    savo_speech::to_string(
      savo_speech::SpeechEvent::PLAYBACK_FINISHED),
    "PLAYBACK_FINISHED");

  EXPECT_EQ(
    savo_speech::to_string(savo_speech::WakeState::AWAKE),
    "AWAKE");

  EXPECT_EQ(
    savo_speech::to_string(savo_speech::ProviderState::READY),
    "READY");
}

TEST(SpeechTypes, NormalizedParsingAndFallbacks)
{
  EXPECT_EQ(
    savo_speech::speech_state_from_string("awake idle"),
    savo_speech::SpeechState::AWAKE_IDLE);

  EXPECT_EQ(
    savo_speech::speech_event_from_string("playback-finished"),
    savo_speech::SpeechEvent::PLAYBACK_FINISHED);

  EXPECT_EQ(
    savo_speech::speech_state_from_string("invalid"),
    savo_speech::SpeechState::ERROR);
}

TEST(SpeechTypes, StateCategories)
{
  EXPECT_TRUE(
    savo_speech::is_busy_state(
      savo_speech::SpeechState::LISTENING));

  EXPECT_TRUE(
    savo_speech::is_capture_state(
      savo_speech::SpeechState::UTTERANCE_READY));

  EXPECT_TRUE(
    savo_speech::is_output_state(
      savo_speech::SpeechState::SPEAKING));

  EXPECT_FALSE(
    savo_speech::is_busy_state(
      savo_speech::SpeechState::SLEEPING));
}

TEST(SpeechTopics, PublicClassification)
{
  EXPECT_TRUE(
    savo_speech::topics::is_public_topic(
      savo_speech::topics::STATE));

  EXPECT_TRUE(
    savo_speech::topics::is_event_topic(
      savo_speech::topics::TTS_FINISHED));

  EXPECT_TRUE(
    savo_speech::topics::is_public_service(
      savo_speech::topics::SERVICE_CANCEL));

  EXPECT_FALSE(
    savo_speech::topics::is_public_topic("/cmd_vel_safe"));
}

TEST(SpeechTopics, JoinTopicHandlesSlashes)
{
  EXPECT_EQ(
    savo_speech::topics::join_topic(
      "/savo_speech",
      "state"),
    "/savo_speech/state");

  EXPECT_EQ(
    savo_speech::topics::join_topic(
      "/savo_speech/",
      "/state"),
    "/savo_speech/state");
}

TEST(SpeechOwnership, MotionTopicsRemainForbidden)
{
  for (const auto topic :
    savo_speech::constants::FORBIDDEN_MOTION_TOPICS)
  {
    EXPECT_FALSE(
      savo_speech::topics::is_public_topic(topic));
  }
}
