// Copyright 2026 Ahnaf Tahmid
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "savo_speech/session/utterance_session_core.hpp"

namespace
{

using namespace std::chrono_literals;

using Core =
  savo_speech::session::UtteranceSessionCore;

using State =
  savo_speech::session::UtteranceSessionState;

using CompletionReason =
  savo_speech::session::UtteranceCompletionReason;

using CancellationReason =
  savo_speech::session::UtteranceCancellationReason;

[[nodiscard]] Core::Clock::time_point
time_at(const std::int64_t milliseconds)
{
  return
    Core::Clock::time_point{} +
    std::chrono::milliseconds{milliseconds};
}

[[nodiscard]] savo_speech::audio::AudioFrame
make_frame(
  const std::uint64_t sequence,
  const std::int16_t sample,
  const std::int64_t milliseconds,
  const std::uint32_t sample_rate = 16000U)
{
  savo_speech::audio::AudioFrame frame;

  frame.sequence = sequence;
  frame.captured_at = time_at(milliseconds);

  frame.format = {
    sample_rate,
    1U,
    savo_speech::audio::PcmSampleFormat::
    Signed16LittleEndian};

  frame.interleaved_samples = {
    sample,
    sample,
    sample,
    sample};

  return frame;
}

[[nodiscard]]
savo_speech::wake_word::WakeWordEvent
make_wake(
  const std::uint64_t event_id,
  const std::uint64_t sequence,
  const std::int64_t milliseconds,
  const std::string & phrase = "hey savo")
{
  savo_speech::wake_word::WakeWordEvent event;

  event.event_id = event_id;
  event.frame_sequence = sequence;
  event.detected_at = time_at(milliseconds);
  event.phrase = phrase;
  event.confidence = 0.90;

  return event;
}

[[nodiscard]] savo_speech::vad::VadEvent
make_vad(
  const std::uint64_t event_id,
  const std::uint64_t segment_id,
  const std::uint64_t sequence,
  const std::int64_t milliseconds,
  const savo_speech::vad::VadEventType type)
{
  savo_speech::vad::VadEvent event;

  event.event_id = event_id;
  event.segment_id = segment_id;

  event.frame_sequence = sequence;
  event.occurred_at = time_at(milliseconds);

  event.type = type;

  event.speech_score =
    type ==
    savo_speech::vad::VadEventType::SpeechStarted ?
    0.90 :
    0.10;

  return event;
}

[[nodiscard]]
savo_speech::session::UtteranceSessionConfig
make_config()
{
  savo_speech::session::UtteranceSessionConfig config;

  config.pre_roll_duration = 1ms;
  config.speech_start_timeout = 100ms;
  config.minimum_speech_duration = 1ms;
  config.maximum_utterance_duration = 100ms;

  config.completed_queue_capacity = 2U;

  return config;
}

void complete_initial_utterance(
  Core & core,
  const std::uint64_t wake_id,
  const std::uint64_t vad_start_id,
  const std::uint64_t vad_end_id,
  const std::uint64_t segment_id,
  const std::uint64_t wake_sequence,
  const std::int64_t base_time)
{
  core.process_audio_frame(
    make_frame(
      wake_sequence,
      100,
      base_time));

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(
        wake_id,
        wake_sequence,
        base_time)));

  core.process_audio_frame(
    make_frame(
      wake_sequence + 1U,
      200,
      base_time + 20));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        vad_start_id,
        segment_id,
        wake_sequence + 1U,
        base_time + 20,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        vad_end_id,
        segment_id,
        wake_sequence + 1U,
        base_time + 21,
        savo_speech::vad::VadEventType::
        SpeechEnded)));

  ASSERT_EQ(
    core.snapshot().state,
    State::AwaitingResponse);
}

void complete_follow_up_utterance(
  Core & core,
  const std::uint64_t vad_start_id,
  const std::uint64_t vad_end_id,
  const std::uint64_t segment_id,
  const std::uint64_t frame_sequence,
  const std::int64_t follow_up_started_at)
{
  ASSERT_EQ(
    core.snapshot().state,
    State::AwaitingResponse);

  ASSERT_TRUE(
    core.begin_follow_up(
      time_at(follow_up_started_at)));

  ASSERT_EQ(
    core.snapshot().state,
    State::Armed);

  core.process_audio_frame(
    make_frame(
      frame_sequence,
      300,
      follow_up_started_at + 20));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        vad_start_id,
        segment_id,
        frame_sequence,
        follow_up_started_at + 20,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        vad_end_id,
        segment_id,
        frame_sequence,
        follow_up_started_at + 21,
        savo_speech::vad::VadEventType::
        SpeechEnded)));

  ASSERT_EQ(
    core.snapshot().state,
    State::AwaitingResponse);
}

}  // namespace

TEST(UtteranceSessionCore, RejectsInvalidConfiguration)
{
  auto config = make_config();
  config.pre_roll_duration = 0ms;

  EXPECT_THROW(
    static_cast<void>(Core{config}),
    std::invalid_argument);

  config = make_config();
  config.speech_start_timeout = 0ms;

  EXPECT_THROW(
    static_cast<void>(Core{config}),
    std::invalid_argument);

  config = make_config();
  config.minimum_speech_duration = 0ms;

  EXPECT_THROW(
    static_cast<void>(Core{config}),
    std::invalid_argument);

  config = make_config();
  config.minimum_speech_duration = -1ms;

  EXPECT_THROW(
    static_cast<void>(Core{config}),
    std::invalid_argument);

  config = make_config();
  config.minimum_speech_duration = 101ms;

  EXPECT_THROW(
    static_cast<void>(Core{config}),
    std::invalid_argument);

  config = make_config();
  config.completed_queue_capacity = 0U;

  EXPECT_THROW(
    static_cast<void>(Core{config}),
    std::invalid_argument);
}

TEST(UtteranceSessionCore, WakeWordArmsIdleSession)
{
  Core core{make_config()};

  core.process_audio_frame(
    make_frame(1U, 100, 0));

  EXPECT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  const auto snapshot = core.snapshot();

  EXPECT_EQ(snapshot.state, State::Armed);
  EXPECT_EQ(snapshot.active_wake_phrase, "hey savo");
  EXPECT_EQ(snapshot.statistics.sessions_armed, 1U);
}

TEST(UtteranceSessionCore, RejectsVadStartOnWakeFrame)
{
  Core core{make_config()};

  core.process_audio_frame(
    make_frame(1U, 100, 0));

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  EXPECT_FALSE(
    core.handle_vad_event(
      make_vad(
        1U,
        1U,
        1U,
        0,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  const auto snapshot = core.snapshot();

  EXPECT_EQ(snapshot.state, State::Armed);
  EXPECT_EQ(snapshot.active_audio_samples, 0U);
  EXPECT_EQ(snapshot.pre_roll_samples, 0U);

  EXPECT_FALSE(
    core.try_pop_completed().has_value());
}

TEST(UtteranceSessionCore, RejectsDuplicateWakeEvent)
{
  Core core{make_config()};

  EXPECT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  EXPECT_FALSE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  EXPECT_EQ(
    core.snapshot().statistics.stale_wake_events,
    1U);
}

TEST(UtteranceSessionCore, IgnoresNewWakeWhileArmed)
{
  Core core{make_config()};

  EXPECT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  EXPECT_FALSE(
    core.handle_wake_word_event(
      make_wake(2U, 2U, 20)));

  const auto snapshot = core.snapshot();

  EXPECT_EQ(snapshot.state, State::Armed);

  EXPECT_EQ(
    snapshot.statistics.
    wake_events_ignored_while_active,
    1U);
}

TEST(UtteranceSessionCore, SpeechStartTimeoutCancelsSession)
{
  Core core{make_config()};

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  core.advance_time(time_at(99));

  EXPECT_EQ(
    core.snapshot().state,
    State::Armed);

  core.advance_time(time_at(100));

  const auto snapshot = core.snapshot();

  EXPECT_EQ(snapshot.state, State::Idle);

  EXPECT_EQ(
    snapshot.last_cancellation_reason,
    CancellationReason::SpeechStartTimeout);

  EXPECT_EQ(
    snapshot.statistics.speech_start_timeouts,
    1U);
}

TEST(UtteranceSessionCore, VadStartBeginsRecording)
{
  Core core{make_config()};

  core.process_audio_frame(
    make_frame(1U, 100, 0));

  core.process_audio_frame(
    make_frame(2U, 200, 20));

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 2U, 20)));

  // First command frame after the wake boundary.
  core.process_audio_frame(
    make_frame(3U, 300, 40));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        1U,
        7U,
        3U,
        40,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  const auto snapshot = core.snapshot();

  EXPECT_EQ(snapshot.state, State::Recording);
  EXPECT_EQ(snapshot.active_vad_segment_id, 7U);

  // Frames 1 and 2 were pre-wake/wake audio and were discarded.
  EXPECT_EQ(snapshot.active_audio_samples, 4U);
}

TEST(UtteranceSessionCore, RetainsOnlyPostWakePreRoll)
{
  Core core{make_config()};

  // Ambient/pre-wake audio.
  core.process_audio_frame(
    make_frame(1U, 100, 0));

  // Wake phrase frame.
  core.process_audio_frame(
    make_frame(2U, 200, 20));

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 2U, 20)));

  // Post-wake command onset enters pre-roll.
  core.process_audio_frame(
    make_frame(3U, 300, 40));

  core.process_audio_frame(
    make_frame(4U, 400, 60));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        1U,
        1U,
        4U,
        60,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  core.process_audio_frame(
    make_frame(5U, 500, 80));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        2U,
        1U,
        5U,
        80,
        savo_speech::vad::VadEventType::
        SpeechEnded)));

  const auto completed =
    core.try_pop_completed();

  ASSERT_TRUE(completed.has_value());

  const std::vector<std::int16_t> expected{
    300, 300, 300, 300,
    400, 400, 400, 400,
    500, 500, 500, 500};

  EXPECT_EQ(
    completed->audio.interleaved_samples,
    expected);

  // Only frame 3 occurred before the detected speech-start frame.
  EXPECT_EQ(completed->pre_roll_samples, 4U);
}

TEST(UtteranceSessionCore, RecordingAppendsLaterFrames)
{
  Core core{make_config()};

  core.process_audio_frame(
    make_frame(1U, 100, 0));

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  core.process_audio_frame(
    make_frame(2U, 200, 20));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        1U,
        1U,
        2U,
        20,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  core.process_audio_frame(
    make_frame(3U, 300, 40));

  EXPECT_EQ(
    core.snapshot().active_audio_samples,
    8U);
}

TEST(UtteranceSessionCore, SpeechEndCompletesUtterance)
{
  Core core{make_config()};

  core.process_audio_frame(
    make_frame(1U, 100, 0));

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(
        1U,
        1U,
        0,
        "hi savo")));

  core.process_audio_frame(
    make_frame(2U, 200, 20));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        1U,
        4U,
        2U,
        20,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  core.process_audio_frame(
    make_frame(3U, 300, 40));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        2U,
        4U,
        3U,
        40,
        savo_speech::vad::VadEventType::
        SpeechEnded)));

  const auto completed =
    core.try_pop_completed();

  ASSERT_TRUE(completed.has_value());

  EXPECT_EQ(completed->utterance_id, 1U);
  EXPECT_EQ(completed->wake_event_id, 1U);
  EXPECT_EQ(completed->wake_phrase, "hi savo");
  EXPECT_EQ(completed->vad_segment_id, 4U);

  EXPECT_EQ(
    completed->completion_reason,
    CompletionReason::SpeechEnded);

  EXPECT_TRUE(
    completed->audio.is_consistent());

  const auto snapshot = core.snapshot();

  EXPECT_EQ(
    snapshot.state,
    State::AwaitingResponse);

  EXPECT_EQ(
    snapshot.active_wake_phrase,
    "hi savo");
}

TEST(
  UtteranceSessionCore,
  AwaitingResponseDoesNotConsumeListeningTimeout)
{
  Core core{make_config()};

  complete_initial_utterance(
    core,
    1U,
    1U,
    2U,
    1U,
    1U,
    0);

  // Simulate a long SavoMind/TTS operation. The listening timeout
  // must not run while the robot is processing or speaking.
  core.advance_time(
    time_at(5000));

  EXPECT_EQ(
    core.snapshot().state,
    State::AwaitingResponse);

  EXPECT_EQ(
    core.snapshot().statistics.
    speech_start_timeouts,
    0U);
}

TEST(
  UtteranceSessionCore,
  FollowUpTimeoutStartsAfterPlaybackCompletion)
{
  Core core{make_config()};

  complete_initial_utterance(
    core,
    1U,
    1U,
    2U,
    1U,
    1U,
    0);

  ASSERT_TRUE(
    core.begin_follow_up(
      time_at(1000)));

  EXPECT_EQ(
    core.snapshot().state,
    State::Armed);

  core.advance_time(
    time_at(1099));

  EXPECT_EQ(
    core.snapshot().state,
    State::Armed);

  core.advance_time(
    time_at(1100));

  const auto snapshot =
    core.snapshot();

  EXPECT_EQ(
    snapshot.state,
    State::Idle);

  EXPECT_EQ(
    snapshot.last_cancellation_reason,
    CancellationReason::SpeechStartTimeout);

  EXPECT_TRUE(
    snapshot.active_wake_phrase.empty());
}

TEST(
  UtteranceSessionCore,
  FollowUpPreservesWakeConversationAndClearsOldAudio)
{
  Core core{make_config()};

  complete_initial_utterance(
    core,
    1U,
    1U,
    2U,
    1U,
    1U,
    0);

  const auto first =
    core.try_pop_completed();

  ASSERT_TRUE(first.has_value());

  // Audio received while SavoMind is processing must not leak into
  // the next user command.
  core.process_audio_frame(
    make_frame(
      3U,
      999,
      500));

  ASSERT_TRUE(
    core.begin_follow_up(
      time_at(1000)));

  core.process_audio_frame(
    make_frame(
      4U,
      400,
      1020));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        3U,
        2U,
        4U,
        1020,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        4U,
        2U,
        4U,
        1021,
        savo_speech::vad::VadEventType::
        SpeechEnded)));

  const auto second =
    core.try_pop_completed();

  ASSERT_TRUE(second.has_value());

  EXPECT_EQ(
    second->utterance_id,
    2U);

  EXPECT_EQ(
    second->wake_event_id,
    first->wake_event_id);

  EXPECT_EQ(
    second->wake_phrase,
    first->wake_phrase);

  const std::vector<std::int16_t> expected{
    400, 400, 400, 400};

  EXPECT_EQ(
    second->audio.interleaved_samples,
    expected);

  EXPECT_EQ(
    core.snapshot().state,
    State::AwaitingResponse);
}

TEST(
  UtteranceSessionCore,
  RejectsFollowUpOutsideAwaitingResponse)
{
  Core core{make_config()};

  EXPECT_FALSE(
    core.begin_follow_up(
      time_at(0)));

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(
        1U,
        1U,
        0)));

  EXPECT_FALSE(
    core.begin_follow_up(
      time_at(20)));

  EXPECT_EQ(
    core.snapshot().state,
    State::Armed);
}

TEST(
  UtteranceSessionCore,
  RejectsShortSpeechAfterInitialWake)
{
  auto config = make_config();
  config.minimum_speech_duration = 30ms;

  Core core{config};

  core.process_audio_frame(
    make_frame(1U, 100, 0));

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  core.process_audio_frame(
    make_frame(2U, 200, 20));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        1U,
        1U,
        2U,
        20,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  core.process_audio_frame(
    make_frame(3U, 300, 40));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        2U,
        1U,
        3U,
        40,
        savo_speech::vad::VadEventType::
        SpeechEnded)));

  const auto snapshot = core.snapshot();

  EXPECT_EQ(snapshot.state, State::Armed);

  EXPECT_EQ(
    snapshot.statistics.short_speech_rejections,
    1U);

  EXPECT_EQ(
    snapshot.active_wake_phrase,
    "hey savo");

  EXPECT_EQ(snapshot.active_audio_samples, 0U);
  EXPECT_EQ(snapshot.pre_roll_samples, 0U);

  EXPECT_FALSE(
    core.try_pop_completed().has_value());
}

TEST(
  UtteranceSessionCore,
  ShortSpeechDoesNotResetInitialListeningDeadline)
{
  auto config = make_config();
  config.minimum_speech_duration = 30ms;

  Core core{config};

  core.process_audio_frame(
    make_frame(1U, 100, 0));

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  core.process_audio_frame(
    make_frame(2U, 200, 20));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        1U,
        1U,
        2U,
        20,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  core.process_audio_frame(
    make_frame(3U, 300, 40));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        2U,
        1U,
        3U,
        40,
        savo_speech::vad::VadEventType::
        SpeechEnded)));

  ASSERT_EQ(
    core.snapshot().state,
    State::Armed);

  core.advance_time(time_at(99));

  EXPECT_EQ(
    core.snapshot().state,
    State::Armed);

  core.advance_time(time_at(100));

  const auto snapshot = core.snapshot();

  EXPECT_EQ(snapshot.state, State::Idle);

  EXPECT_EQ(
    snapshot.last_cancellation_reason,
    CancellationReason::SpeechStartTimeout);

  EXPECT_EQ(
    snapshot.statistics.short_speech_rejections,
    1U);
}

TEST(
  UtteranceSessionCore,
  ShortSpeechDoesNotResetFollowUpDeadline)
{
  auto config = make_config();
  config.minimum_speech_duration = 30ms;

  Core core{config};

  // Valid initial user turn: 40 ms >= 30 ms.
  core.process_audio_frame(
    make_frame(1U, 100, 0));

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  core.process_audio_frame(
    make_frame(2U, 200, 20));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        1U,
        1U,
        2U,
        20,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  core.process_audio_frame(
    make_frame(3U, 300, 60));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        2U,
        1U,
        3U,
        60,
        savo_speech::vad::VadEventType::
        SpeechEnded)));

  ASSERT_TRUE(
    core.try_pop_completed().has_value());

  ASSERT_TRUE(
    core.begin_follow_up(
      time_at(1000)));

  // False follow-up burst: only 10 ms.
  core.process_audio_frame(
    make_frame(4U, 400, 1010));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        3U,
        2U,
        4U,
        1010,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  core.process_audio_frame(
    make_frame(5U, 500, 1020));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        4U,
        2U,
        5U,
        1020,
        savo_speech::vad::VadEventType::
        SpeechEnded)));

  ASSERT_EQ(
    core.snapshot().state,
    State::Armed);

  EXPECT_FALSE(
    core.try_pop_completed().has_value());

  core.advance_time(time_at(1099));

  EXPECT_EQ(
    core.snapshot().state,
    State::Armed);

  core.advance_time(time_at(1100));

  const auto snapshot = core.snapshot();

  EXPECT_EQ(snapshot.state, State::Idle);

  EXPECT_EQ(
    snapshot.statistics.short_speech_rejections,
    1U);

  EXPECT_EQ(
    snapshot.last_cancellation_reason,
    CancellationReason::SpeechStartTimeout);
}

TEST(
  UtteranceSessionCore,
  ValidSpeechAfterRejectedFollowUpNoiseStillCompletes)
{
  auto config = make_config();
  config.minimum_speech_duration = 30ms;

  Core core{config};

  // Valid initial turn.
  core.process_audio_frame(
    make_frame(1U, 100, 0));

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  core.process_audio_frame(
    make_frame(2U, 200, 20));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        1U,
        1U,
        2U,
        20,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  core.process_audio_frame(
    make_frame(3U, 300, 60));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        2U,
        1U,
        3U,
        60,
        savo_speech::vad::VadEventType::
        SpeechEnded)));

  const auto initial =
    core.try_pop_completed();

  ASSERT_TRUE(initial.has_value());

  ASSERT_TRUE(
    core.begin_follow_up(
      time_at(1000)));

  // Rejected 10 ms noise segment.
  core.process_audio_frame(
    make_frame(4U, 400, 1010));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        3U,
        2U,
        4U,
        1010,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  core.process_audio_frame(
    make_frame(5U, 500, 1020));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        4U,
        2U,
        5U,
        1020,
        savo_speech::vad::VadEventType::
        SpeechEnded)));

  ASSERT_EQ(
    core.snapshot().state,
    State::Armed);

  // Genuine follow-up within the original 100 ms window.
  core.process_audio_frame(
    make_frame(6U, 600, 1040));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        5U,
        3U,
        6U,
        1040,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  core.process_audio_frame(
    make_frame(7U, 700, 1080));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        6U,
        3U,
        7U,
        1080,
        savo_speech::vad::VadEventType::
        SpeechEnded)));

  const auto follow_up =
    core.try_pop_completed();

  ASSERT_TRUE(follow_up.has_value());

  EXPECT_FALSE(
    core.try_pop_completed().has_value());

  EXPECT_EQ(
    follow_up->wake_event_id,
    initial->wake_event_id);

  EXPECT_EQ(
    core.snapshot().statistics.short_speech_rejections,
    1U);

  EXPECT_EQ(
    core.snapshot().state,
    State::AwaitingResponse);
}

TEST(
  UtteranceSessionCore,
  SpeechAtMinimumDurationIsAccepted)
{
  auto config = make_config();
  config.minimum_speech_duration = 30ms;

  Core core{config};

  core.process_audio_frame(
    make_frame(1U, 100, 0));

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  core.process_audio_frame(
    make_frame(2U, 200, 20));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        1U,
        1U,
        2U,
        20,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  core.process_audio_frame(
    make_frame(3U, 300, 50));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        2U,
        1U,
        3U,
        50,
        savo_speech::vad::VadEventType::
        SpeechEnded)));

  EXPECT_TRUE(
    core.try_pop_completed().has_value());

  const auto snapshot = core.snapshot();

  EXPECT_EQ(
    snapshot.statistics.short_speech_rejections,
    0U);

  EXPECT_EQ(
    snapshot.state,
    State::AwaitingResponse);
}

TEST(UtteranceSessionCore, AssignsStableUtteranceIds)
{
  Core core{make_config()};

  complete_initial_utterance(
    core,
    1U,
    1U,
    2U,
    1U,
    1U,
    0);

  complete_follow_up_utterance(
    core,
    3U,
    4U,
    2U,
    3U,
    100);

  const auto first =
    core.try_pop_completed();

  const auto second =
    core.try_pop_completed();

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());

  EXPECT_EQ(first->utterance_id, 1U);
  EXPECT_EQ(second->utterance_id, 2U);

  // Both turns belong to the same wake conversation.
  EXPECT_EQ(
    first->wake_event_id,
    second->wake_event_id);

  EXPECT_EQ(
    first->wake_phrase,
    second->wake_phrase);
}

TEST(UtteranceSessionCore, MaximumDurationCompletesRecording)
{
  Core core{make_config()};

  core.process_audio_frame(
    make_frame(1U, 100, 0));

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  core.process_audio_frame(
    make_frame(2U, 200, 20));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        1U,
        1U,
        2U,
        20,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  core.advance_time(time_at(120));

  const auto completed =
    core.try_pop_completed();

  ASSERT_TRUE(completed.has_value());

  EXPECT_EQ(
    completed->completion_reason,
    CompletionReason::MaximumDurationReached);

  EXPECT_EQ(
    core.snapshot().statistics.
    maximum_duration_completions,
    1U);
}

TEST(UtteranceSessionCore, ExplicitCancellationReturnsToIdle)
{
  Core core{make_config()};

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  EXPECT_TRUE(
    core.cancel(
      CancellationReason::ExplicitCancellation));

  const auto snapshot = core.snapshot();

  EXPECT_EQ(snapshot.state, State::Idle);

  EXPECT_EQ(
    snapshot.last_cancellation_reason,
    CancellationReason::ExplicitCancellation);

  EXPECT_FALSE(
    core.try_pop_completed().has_value());
}

TEST(UtteranceSessionCore, ResetClearsAllRuntimeState)
{
  Core core{make_config()};

  core.process_audio_frame(
    make_frame(1U, 100, 0));

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  core.reset();

  const auto snapshot = core.snapshot();

  EXPECT_EQ(snapshot.state, State::Idle);
  EXPECT_FALSE(snapshot.audio_format_initialized);
  EXPECT_EQ(snapshot.pre_roll_samples, 0U);
  EXPECT_EQ(snapshot.statistics.audio_frames_processed, 0U);
  EXPECT_TRUE(snapshot.active_wake_phrase.empty());
}

TEST(UtteranceSessionCore, RejectsDuplicateAndOlderAudio)
{
  Core core{make_config()};

  core.process_audio_frame(
    make_frame(5U, 100, 0));

  EXPECT_THROW(
    core.process_audio_frame(
      make_frame(5U, 100, 20)),
    std::invalid_argument);

  EXPECT_THROW(
    core.process_audio_frame(
      make_frame(4U, 100, 20)),
    std::invalid_argument);

  const auto statistics =
    core.snapshot().statistics;

  EXPECT_EQ(statistics.duplicate_audio_frames, 1U);
  EXPECT_EQ(statistics.out_of_order_audio_frames, 1U);
}

TEST(UtteranceSessionCore, RecordsSequenceGapMetadata)
{
  Core core{make_config()};

  core.process_audio_frame(
    make_frame(1U, 100, 0));

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  core.process_audio_frame(
    make_frame(2U, 200, 20));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        1U,
        1U,
        2U,
        20,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  // Sequence 3 is intentionally missing.
  core.process_audio_frame(
    make_frame(4U, 400, 40));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        2U,
        1U,
        4U,
        40,
        savo_speech::vad::VadEventType::
        SpeechEnded)));

  const auto completed =
    core.try_pop_completed();

  ASSERT_TRUE(completed.has_value());

  EXPECT_EQ(completed->sequence_gap_count, 1U);
  EXPECT_EQ(completed->missing_audio_frames, 1U);
}

TEST(UtteranceSessionCore, RejectsStaleVadEvent)
{
  Core core{make_config()};

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        2U,
        1U,
        2U,
        20,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  EXPECT_FALSE(
    core.handle_vad_event(
      make_vad(
        1U,
        1U,
        2U,
        20,
        savo_speech::vad::VadEventType::
        SpeechEnded)));

  EXPECT_EQ(
    core.snapshot().statistics.stale_vad_events,
    1U);
}

TEST(UtteranceSessionCore, RejectsInvalidVadTransitions)
{
  Core core{make_config()};

  EXPECT_FALSE(
    core.handle_vad_event(
      make_vad(
        1U,
        1U,
        1U,
        0,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  EXPECT_FALSE(
    core.handle_vad_event(
      make_vad(
        2U,
        1U,
        1U,
        0,
        savo_speech::vad::VadEventType::
        SpeechEnded)));

  EXPECT_EQ(
    core.snapshot().statistics.
    invalid_vad_transitions,
    2U);
}

TEST(UtteranceSessionCore, DropsOldestCompletedUtterance)
{
  Core core{make_config()};

  complete_initial_utterance(
    core,
    1U,
    1U,
    2U,
    1U,
    1U,
    0);

  complete_follow_up_utterance(
    core,
    3U,
    4U,
    2U,
    3U,
    100);

  complete_follow_up_utterance(
    core,
    5U,
    6U,
    3U,
    4U,
    200);

  const auto first =
    core.try_pop_completed();

  const auto second =
    core.try_pop_completed();

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());

  EXPECT_EQ(first->utterance_id, 2U);
  EXPECT_EQ(second->utterance_id, 3U);

  // Queueing multiple follow-up turns must not create new wake
  // conversations.
  EXPECT_EQ(
    first->wake_event_id,
    1U);

  EXPECT_EQ(
    second->wake_event_id,
    1U);

  const auto statistics =
    core.snapshot().statistics;

  EXPECT_EQ(
    statistics.completed_queue_overflows,
    1U);

  EXPECT_EQ(
    statistics.completed_utterances_dropped,
    1U);
}

TEST(UtteranceSessionCore, RejectsAudioFormatChange)
{
  Core core{make_config()};

  core.process_audio_frame(
    make_frame(1U, 100, 0, 16000U));

  EXPECT_THROW(
    core.process_audio_frame(
      make_frame(2U, 100, 20, 8000U)),
    std::invalid_argument);

  EXPECT_EQ(
    core.snapshot().statistics.
    audio_format_mismatches,
    1U);
}

TEST(UtteranceSessionCore, PendingEndWaitsForMatchingAudio)
{
  Core core{make_config()};

  core.process_audio_frame(
    make_frame(1U, 100, 0));

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        1U,
        1U,
        2U,
        20,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        2U,
        1U,
        3U,
        40,
        savo_speech::vad::VadEventType::
        SpeechEnded)));

  EXPECT_TRUE(
    core.snapshot().pending_speech_end);

  core.process_audio_frame(
    make_frame(2U, 200, 20));

  EXPECT_FALSE(
    core.try_pop_completed().has_value());

  core.process_audio_frame(
    make_frame(3U, 300, 40));

  EXPECT_TRUE(
    core.try_pop_completed().has_value());
}

TEST(UtteranceSessionCore, RejectsBackwardTimeAdvance)
{
  Core core{make_config()};

  core.advance_time(time_at(100));

  EXPECT_THROW(
    core.advance_time(time_at(99)),
    std::invalid_argument);
}

TEST(UtteranceSessionCore, MaximumDurationWithoutAudioCancels)
{
  Core core{make_config()};

  ASSERT_TRUE(
    core.handle_wake_word_event(
      make_wake(1U, 1U, 0)));

  // Post-wake VAD starts, but no corresponding audio frame arrives.
  ASSERT_TRUE(
    core.handle_vad_event(
      make_vad(
        1U,
        1U,
        2U,
        20,
        savo_speech::vad::VadEventType::
        SpeechStarted)));

  core.advance_time(time_at(120));

  const auto snapshot = core.snapshot();

  EXPECT_EQ(snapshot.state, State::Idle);

  EXPECT_EQ(
    snapshot.last_cancellation_reason,
    CancellationReason::NoAudioCaptured);

  EXPECT_FALSE(
    core.try_pop_completed().has_value());
}

TEST(UtteranceSessionCore, WaitCompletedTimesOutWhenEmpty)
{
  Core core{make_config()};

  EXPECT_FALSE(
    core.wait_completed_for(1ms).has_value());

  EXPECT_THROW(
    static_cast<void>(
      core.wait_completed_for(-1ms)),
    std::invalid_argument);
}
