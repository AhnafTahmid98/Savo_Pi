#include <chrono>
#include <stdexcept>

#include "gtest/gtest.h"

#include "savo_speech/audio/microphone_gate.hpp"

TEST(MicrophoneGate, RejectsNegativeHoldDuration)
{
  savo_speech::audio::MicrophoneGateConfig config;
  config.post_playback_hold =
    std::chrono::milliseconds{-1};

  EXPECT_THROW(
    savo_speech::audio::MicrophoneGate{config},
    std::invalid_argument);
}

TEST(MicrophoneGate, PlaybackImmediatelyGatesMicrophone)
{
  savo_speech::audio::MicrophoneGate gate;

  gate.begin_playback();

  const auto state = gate.state();

  EXPECT_TRUE(state.gated);
  EXPECT_TRUE(state.playback_active);
  EXPECT_EQ(
    state.reason,
    savo_speech::audio::MicrophoneGateReason::Playback);

  EXPECT_EQ(state.playback_sessions, 1U);
}

TEST(MicrophoneGate, PostPlaybackHoldExpires)
{
  savo_speech::audio::MicrophoneGateConfig config;
  config.post_playback_hold =
    std::chrono::milliseconds{250};

  savo_speech::audio::MicrophoneGate gate{config};

  const auto start =
    savo_speech::audio::MicrophoneGate::Clock::now();

  gate.begin_playback();
  gate.end_playback(start);

  const auto held_state =
    gate.state(start + std::chrono::milliseconds{100});

  EXPECT_TRUE(held_state.gated);
  EXPECT_EQ(
    held_state.reason,
    savo_speech::audio::MicrophoneGateReason::
    PostPlaybackHold);

  const auto open_state =
    gate.state(start + std::chrono::milliseconds{251});

  EXPECT_FALSE(open_state.gated);
  EXPECT_EQ(
    open_state.reason,
    savo_speech::audio::MicrophoneGateReason::Open);
}

TEST(MicrophoneGate, ManualGateOverridesPostPlaybackHold)
{
  savo_speech::audio::MicrophoneGate gate;

  gate.begin_playback();
  gate.end_playback();
  gate.set_manual_gate(true);

  const auto state = gate.state();

  EXPECT_TRUE(state.gated);
  EXPECT_EQ(
    state.reason,
    savo_speech::audio::MicrophoneGateReason::Manual);
}

TEST(MicrophoneGate, ShutdownGateHasHighestPriority)
{
  savo_speech::audio::MicrophoneGate gate;

  gate.begin_playback();
  gate.set_manual_gate(true);
  gate.set_shutdown_gate(true);

  const auto state = gate.state();

  EXPECT_TRUE(state.gated);
  EXPECT_EQ(
    state.reason,
    savo_speech::audio::MicrophoneGateReason::Shutdown);
}
