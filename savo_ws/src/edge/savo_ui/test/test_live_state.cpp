// Copyright 2026 Ahnaf Tahmid
#include <gtest/gtest.h>

#include <chrono>

#include "savo_ui/app/live_state.hpp"

TEST(LiveState, ClassifiesLifecycleWithoutInventingAuthority)
{
  using savo_ui::SpeechLifecycle;
  EXPECT_EQ(savo_ui::classify_speech_lifecycle("phase=recording"), SpeechLifecycle::Recording);
  EXPECT_EQ(savo_ui::classify_speech_lifecycle("state=synthesizing"), SpeechLifecycle::Thinking);
  EXPECT_EQ(savo_ui::classify_speech_lifecycle("playback=playing"), SpeechLifecycle::Playing);
  EXPECT_EQ(savo_ui::classify_speech_lifecycle("reason=connection_failed"),
    SpeechLifecycle::Failed);
}

TEST(LiveState, BoundsUntrustedDisplayText)
{
  EXPECT_EQ(savo_ui::bounded_ui_text("abcdefgh", 6U), "abc...");
  EXPECT_EQ(savo_ui::status_field("state=ready;map_id=main", "map_id"), "main");
}

TEST(LiveState, FailsStale)
{
  const auto now = std::chrono::steady_clock::now();
  EXPECT_TRUE(savo_ui::feed_is_stale(false, now, now, std::chrono::seconds(5)));
  EXPECT_FALSE(savo_ui::feed_is_stale(true, now, now, std::chrono::seconds(5)));
  EXPECT_TRUE(savo_ui::feed_is_stale(true, now - std::chrono::seconds(6), now,
    std::chrono::seconds(5)));
}
