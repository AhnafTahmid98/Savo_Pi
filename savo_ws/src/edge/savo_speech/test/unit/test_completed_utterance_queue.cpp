#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include "gtest/gtest.h"

#include "savo_speech/audio/audio_buffer.hpp"
#include "savo_speech/audio/wav_writer.hpp"
#include "savo_speech/session/completed_utterance_queue.hpp"
#include "savo_speech/session/serialized_utterance.hpp"

namespace
{

using namespace std::chrono_literals;

[[nodiscard]]
savo_speech::session::SerializedUtterance
make_serialized_utterance(
  const std::uint64_t utterance_id,
  const std::size_t sample_count = 8U)
{
  savo_speech::audio::AudioBuffer audio;

  audio.format = {
    16000U,
    1U,
    savo_speech::audio::PcmSampleFormat::
    Signed16LittleEndian};

  audio.interleaved_samples.resize(
    sample_count,
    static_cast<std::int16_t>(100));

  savo_speech::session::SerializedUtterance utterance;

  utterance.utterance_id = utterance_id;
  utterance.wake_event_id = utterance_id + 100U;
  utterance.wake_phrase = "savo";
  utterance.wake_confidence = 0.9;

  utterance.vad_segment_id =
    utterance_id + 200U;

  utterance.audio_format = audio.format;
  utterance.sample_count =
    audio.interleaved_samples.size();

  utterance.pre_roll_samples = 2U;

  utterance.wav_bytes =
    savo_speech::audio::WavWriter::encode(audio);

  return utterance;
}

}  // namespace

TEST(
  CompletedUtteranceQueue,
  RejectsInvalidCapacity)
{
  EXPECT_THROW(
    static_cast<void>(
      savo_speech::session::
      CompletedUtteranceQueue{0U}),
    std::invalid_argument);

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::session::
      CompletedUtteranceQueue{1025U}),
    std::invalid_argument);
}

TEST(
  CompletedUtteranceQueue,
  AcceptsAndPopsSerializedUtterance)
{
  savo_speech::session::CompletedUtteranceQueue
    queue{2U};

  EXPECT_EQ(
    queue.push(make_serialized_utterance(1U)),
    savo_speech::session::
    SerializedUtterancePushResult::Accepted);

  EXPECT_EQ(queue.size(), 1U);
  EXPECT_FALSE(queue.empty());
  EXPECT_FALSE(queue.full());

  const auto result = queue.try_pop();

  ASSERT_TRUE(result.has_value());

  EXPECT_EQ(result->utterance_id, 1U);
  EXPECT_EQ(result->wake_phrase, "savo");
  EXPECT_TRUE(result->is_valid());

  EXPECT_TRUE(queue.empty());

  const auto statistics = queue.statistics();

  EXPECT_EQ(statistics.accepted, 1U);
  EXPECT_EQ(statistics.popped, 1U);
  EXPECT_EQ(statistics.size, 0U);
  EXPECT_EQ(statistics.capacity, 2U);
}

TEST(
  CompletedUtteranceQueue,
  RejectsInvalidSerializedUtterance)
{
  savo_speech::session::CompletedUtteranceQueue
    queue{2U};

  auto utterance = make_serialized_utterance(1U);

  utterance.wav_bytes.clear();

  EXPECT_EQ(
    queue.push(std::move(utterance)),
    savo_speech::session::
    SerializedUtterancePushResult::
    RejectedInvalid);

  EXPECT_TRUE(queue.empty());

  EXPECT_EQ(
    queue.statistics().rejected_invalid,
    1U);
}

TEST(
  CompletedUtteranceQueue,
  RejectsDuplicatePendingUtteranceId)
{
  savo_speech::session::CompletedUtteranceQueue
    queue{2U};

  EXPECT_EQ(
    queue.push(make_serialized_utterance(50U)),
    savo_speech::session::
    SerializedUtterancePushResult::Accepted);

  EXPECT_EQ(
    queue.push(make_serialized_utterance(50U)),
    savo_speech::session::
    SerializedUtterancePushResult::
    RejectedDuplicateId);

  EXPECT_EQ(queue.size(), 1U);

  EXPECT_EQ(
    queue.statistics().rejected_duplicate_id,
    1U);
}

TEST(
  CompletedUtteranceQueue,
  EnforcesConfiguredCapacity)
{
  savo_speech::session::CompletedUtteranceQueue
    queue{1U};

  EXPECT_EQ(
    queue.push(make_serialized_utterance(1U)),
    savo_speech::session::
    SerializedUtterancePushResult::Accepted);

  EXPECT_TRUE(queue.full());

  EXPECT_EQ(
    queue.push(make_serialized_utterance(2U)),
    savo_speech::session::
    SerializedUtterancePushResult::
    RejectedFull);

  EXPECT_EQ(queue.size(), 1U);

  EXPECT_EQ(
    queue.statistics().rejected_full,
    1U);
}

TEST(
  CompletedUtteranceQueue,
  ClearRemovesQueuedUtterances)
{
  savo_speech::session::CompletedUtteranceQueue
    queue{3U};

  ASSERT_EQ(
    queue.push(make_serialized_utterance(1U)),
    savo_speech::session::
    SerializedUtterancePushResult::Accepted);

  ASSERT_EQ(
    queue.push(make_serialized_utterance(2U)),
    savo_speech::session::
    SerializedUtterancePushResult::Accepted);

  queue.clear();

  EXPECT_TRUE(queue.empty());
  EXPECT_FALSE(queue.try_pop().has_value());

  const auto statistics = queue.statistics();

  EXPECT_EQ(statistics.cleared, 2U);
  EXPECT_EQ(statistics.accepted, 2U);
  EXPECT_EQ(statistics.popped, 0U);
}

TEST(
  CompletedUtteranceQueue,
  TimedWaitReturnsEmptyAndRejectsNegativeTimeout)
{
  savo_speech::session::CompletedUtteranceQueue
    queue{2U};

  EXPECT_FALSE(
    queue.wait_pop_for(2ms).has_value());

  EXPECT_THROW(
    static_cast<void>(
      queue.wait_pop_for(-1ms)),
    std::invalid_argument);
}
