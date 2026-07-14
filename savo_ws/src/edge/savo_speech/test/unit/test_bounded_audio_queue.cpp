#include <chrono>
#include <cstdint>
#include <stdexcept>

#include "gtest/gtest.h"

#include "savo_speech/audio/bounded_audio_queue.hpp"

namespace
{

savo_speech::audio::AudioFrame make_frame(
  const std::uint64_t sequence)
{
  savo_speech::audio::AudioFrame frame;

  frame.sequence = sequence;
  frame.format = {
    16000U,
    1U,
    savo_speech::audio::PcmSampleFormat::Signed16LittleEndian};

  frame.interleaved_samples = {
    static_cast<std::int16_t>(sequence)};

  return frame;
}

}  // namespace

TEST(BoundedAudioQueue, RejectsZeroCapacity)
{
  EXPECT_THROW(
    savo_speech::audio::BoundedAudioQueue{0U},
    std::invalid_argument);
}

TEST(BoundedAudioQueue, PreservesFifoOrdering)
{
  savo_speech::audio::BoundedAudioQueue queue{3U};

  EXPECT_EQ(
    queue.push(make_frame(1U)),
    savo_speech::audio::QueuePushResult::Accepted);

  EXPECT_EQ(
    queue.push(make_frame(2U)),
    savo_speech::audio::QueuePushResult::Accepted);

  const auto first = queue.try_pop();
  const auto second = queue.try_pop();

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());

  EXPECT_EQ(first->sequence, 1U);
  EXPECT_EQ(second->sequence, 2U);
  EXPECT_TRUE(queue.empty());
}

TEST(BoundedAudioQueue, RejectNewestPolicyPreservesExistingFrames)
{
  savo_speech::audio::BoundedAudioQueue queue{
    2U,
    savo_speech::audio::QueueOverflowPolicy::RejectNewest};

  static_cast<void>(queue.push(make_frame(1U)));
  static_cast<void>(queue.push(make_frame(2U)));

  EXPECT_EQ(
    queue.push(make_frame(3U)),
    savo_speech::audio::QueuePushResult::RejectedFull);

  const auto first = queue.try_pop();
  const auto second = queue.try_pop();

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());

  EXPECT_EQ(first->sequence, 1U);
  EXPECT_EQ(second->sequence, 2U);

  const auto statistics = queue.statistics();

  EXPECT_EQ(statistics.accepted_frames, 2U);
  EXPECT_EQ(statistics.rejected_full_frames, 1U);
}

TEST(BoundedAudioQueue, DropOldestPolicyKeepsNewestFrames)
{
  savo_speech::audio::BoundedAudioQueue queue{
    2U,
    savo_speech::audio::QueueOverflowPolicy::DropOldest};

  static_cast<void>(queue.push(make_frame(1U)));
  static_cast<void>(queue.push(make_frame(2U)));

  EXPECT_EQ(
    queue.push(make_frame(3U)),
    savo_speech::audio::QueuePushResult::
    AcceptedAfterDroppingOldest);

  const auto first = queue.try_pop();
  const auto second = queue.try_pop();

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());

  EXPECT_EQ(first->sequence, 2U);
  EXPECT_EQ(second->sequence, 3U);

  const auto statistics = queue.statistics();

  EXPECT_EQ(statistics.accepted_frames, 3U);
  EXPECT_EQ(statistics.dropped_oldest_frames, 1U);
}

TEST(BoundedAudioQueue, ClosedQueueRejectsNewFramesButDrainsExisting)
{
  savo_speech::audio::BoundedAudioQueue queue{2U};

  static_cast<void>(queue.push(make_frame(1U)));
  queue.close();

  EXPECT_TRUE(queue.closed());

  EXPECT_EQ(
    queue.push(make_frame(2U)),
    savo_speech::audio::QueuePushResult::RejectedClosed);

  const auto existing = queue.try_pop();

  ASSERT_TRUE(existing.has_value());
  EXPECT_EQ(existing->sequence, 1U);

  EXPECT_FALSE(queue.try_pop().has_value());

  const auto statistics = queue.statistics();
  EXPECT_EQ(statistics.rejected_closed_frames, 1U);
}

TEST(BoundedAudioQueue, ClearRemovesQueuedFrames)
{
  savo_speech::audio::BoundedAudioQueue queue{3U};

  static_cast<void>(queue.push(make_frame(1U)));
  static_cast<void>(queue.push(make_frame(2U)));

  queue.clear();

  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.size(), 0U);
  EXPECT_EQ(queue.capacity(), 3U);
}

TEST(BoundedAudioQueue, WaitPopTimesOutWhenNoFrameArrives)
{
  using namespace std::chrono_literals;

  savo_speech::audio::BoundedAudioQueue queue{2U};

  const auto frame = queue.wait_pop_for(5ms);

  EXPECT_FALSE(frame.has_value());
}
