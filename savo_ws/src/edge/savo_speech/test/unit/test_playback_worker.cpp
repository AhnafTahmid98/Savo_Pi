#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <span>
#include <stdexcept>
#include <thread>

#include "gtest/gtest.h"

#include "savo_speech/audio/audio_buffer.hpp"
#include "savo_speech/audio/microphone_gate.hpp"
#include "savo_speech/audio/playback_sink.hpp"
#include "savo_speech/audio/playback_worker.hpp"

namespace
{

using namespace std::chrono_literals;

class ControlledPlaybackSink final :
  public savo_speech::audio::PlaybackSink
{
public:
  ControlledPlaybackSink()
  {
    format_ = {
      16000U,
      1U,
      savo_speech::audio::PcmSampleFormat::
      Signed16LittleEndian};
  }

  void open() override
  {
    const std::scoped_lock lock{mutex_};
    open_ = true;
  }

  void close() noexcept override
  {
    const std::scoped_lock lock{mutex_};
    open_ = false;
  }

  [[nodiscard]] bool is_open() const noexcept override
  {
    const std::scoped_lock lock{mutex_};
    return open_;
  }

  [[nodiscard]] savo_speech::audio::AudioFormat
  format() const noexcept override
  {
    return format_;
  }

  void write(
    const std::span<const std::int16_t>) override
  {
    std::unique_lock lock{mutex_};

    ++write_calls_;
    write_entered_ = true;

    condition_.notify_all();

    if (throw_on_write_) {
      throw std::runtime_error{
              "scripted playback failure"};
    }

    condition_.wait(
      lock,
      [this]() {
        return !block_writes_ || release_writes_;
      });
  }

  void drain() override
  {
    const std::scoped_lock lock{mutex_};
    ++drain_calls_;
  }

  void stop() noexcept override
  {
    const std::scoped_lock lock{mutex_};
    ++stop_calls_;
  }

  void set_block_writes(const bool enabled)
  {
    const std::scoped_lock lock{mutex_};

    block_writes_ = enabled;
    release_writes_ = !enabled;
    write_entered_ = false;
  }

  void release_writes()
  {
    {
      const std::scoped_lock lock{mutex_};
      release_writes_ = true;
    }

    condition_.notify_all();
  }

  void set_throw_on_write(const bool enabled)
  {
    const std::scoped_lock lock{mutex_};
    throw_on_write_ = enabled;
  }

  [[nodiscard]] bool wait_for_write(
    const std::chrono::milliseconds timeout = 500ms)
  {
    std::unique_lock lock{mutex_};

    return condition_.wait_for(
      lock,
      timeout,
      [this]() {
        return write_entered_;
      });
  }

  [[nodiscard]] std::size_t write_calls() const noexcept
  {
    const std::scoped_lock lock{mutex_};
    return write_calls_;
  }

  [[nodiscard]] std::size_t drain_calls() const noexcept
  {
    const std::scoped_lock lock{mutex_};
    return drain_calls_;
  }

  [[nodiscard]] std::size_t stop_calls() const noexcept
  {
    const std::scoped_lock lock{mutex_};
    return stop_calls_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;

  savo_speech::audio::AudioFormat format_{};

  bool open_{false};

  bool block_writes_{false};
  bool release_writes_{true};
  bool write_entered_{false};

  bool throw_on_write_{false};

  std::size_t write_calls_{0U};
  std::size_t drain_calls_{0U};
  std::size_t stop_calls_{0U};
};

[[nodiscard]] savo_speech::audio::AudioBuffer make_audio(
  const std::size_t frames)
{
  savo_speech::audio::AudioBuffer audio;

  audio.format = {
    16000U,
    1U,
    savo_speech::audio::PcmSampleFormat::
    Signed16LittleEndian};

  audio.interleaved_samples.resize(
    frames,
    static_cast<std::int16_t>(100));

  return audio;
}

[[nodiscard]] savo_speech::audio::PlaybackRequest
make_request(
  const std::uint64_t request_id,
  const std::size_t frames = 32U)
{
  savo_speech::audio::PlaybackRequest request;

  request.request_id = request_id;
  request.audio = make_audio(frames);

  return request;
}

}  // namespace

TEST(PlaybackWorker, RejectsInvalidConfiguration)
{
  ControlledPlaybackSink sink;
  savo_speech::audio::MicrophoneGate gate;

  savo_speech::audio::PlaybackWorkerConfig config;
  config.queue_capacity = 0U;

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::audio::PlaybackWorker(
        sink,
        gate,
        config)),
    std::invalid_argument);
}

TEST(PlaybackWorker, CompletesQueuedRequest)
{
  ControlledPlaybackSink sink;
  savo_speech::audio::MicrophoneGate gate;

  savo_speech::audio::PlaybackWorkerConfig config;
  config.chunk_frames = 8U;

  savo_speech::audio::PlaybackWorker worker{
    sink,
    gate,
    config};

  EXPECT_TRUE(worker.start());

  EXPECT_EQ(
    worker.enqueue(make_request(1U, 24U)),
    savo_speech::audio::PlaybackEnqueueResult::Accepted);

  const auto completion =
    worker.wait_completion_for(500ms);

  ASSERT_TRUE(completion.has_value());

  EXPECT_EQ(completion->request_id, 1U);
  EXPECT_EQ(
    completion->status,
    savo_speech::audio::PlaybackCompletionStatus::
    Completed);

  EXPECT_EQ(completion->frames_submitted, 24U);
  EXPECT_EQ(completion->chunks_submitted, 3U);

  worker.stop();

  const auto statistics = worker.statistics();

  EXPECT_EQ(statistics.completed_requests, 1U);
  EXPECT_EQ(statistics.failed_requests, 0U);
}

TEST(PlaybackWorker, RejectsDuplicateRequestId)
{
  ControlledPlaybackSink sink;
  sink.set_block_writes(true);

  savo_speech::audio::MicrophoneGate gate;

  savo_speech::audio::PlaybackWorker worker{
    sink,
    gate};

  EXPECT_TRUE(worker.start());

  EXPECT_EQ(
    worker.enqueue(make_request(20U)),
    savo_speech::audio::PlaybackEnqueueResult::Accepted);

  ASSERT_TRUE(sink.wait_for_write());

  EXPECT_EQ(
    worker.enqueue(make_request(20U)),
    savo_speech::audio::PlaybackEnqueueResult::
    RejectedDuplicateId);

  EXPECT_TRUE(worker.cancel_current());
  sink.release_writes();

  ASSERT_TRUE(
    worker.wait_completion_for(500ms).has_value());

  worker.stop();
}

TEST(PlaybackWorker, EnforcesBoundedRequestQueue)
{
  ControlledPlaybackSink sink;
  sink.set_block_writes(true);

  savo_speech::audio::MicrophoneGate gate;

  savo_speech::audio::PlaybackWorkerConfig config;
  config.queue_capacity = 1U;
  config.chunk_frames = 8U;

  savo_speech::audio::PlaybackWorker worker{
    sink,
    gate,
    config};

  EXPECT_TRUE(worker.start());

  EXPECT_EQ(
    worker.enqueue(make_request(1U)),
    savo_speech::audio::PlaybackEnqueueResult::Accepted);

  ASSERT_TRUE(sink.wait_for_write());

  EXPECT_EQ(
    worker.enqueue(make_request(2U)),
    savo_speech::audio::PlaybackEnqueueResult::Accepted);

  EXPECT_EQ(
    worker.enqueue(make_request(3U)),
    savo_speech::audio::PlaybackEnqueueResult::
    RejectedQueueFull);

  worker.cancel_all();
  sink.release_writes();

  ASSERT_TRUE(
    worker.wait_completion_for(500ms).has_value());

  ASSERT_TRUE(
    worker.wait_completion_for(500ms).has_value());

  worker.stop();
}

TEST(PlaybackWorker, CancelsCurrentRequestBetweenChunks)
{
  ControlledPlaybackSink sink;
  sink.set_block_writes(true);

  savo_speech::audio::MicrophoneGate gate;

  savo_speech::audio::PlaybackWorkerConfig config;
  config.chunk_frames = 8U;

  savo_speech::audio::PlaybackWorker worker{
    sink,
    gate,
    config};

  EXPECT_TRUE(worker.start());

  EXPECT_EQ(
    worker.enqueue(make_request(9U, 64U)),
    savo_speech::audio::PlaybackEnqueueResult::Accepted);

  ASSERT_TRUE(sink.wait_for_write());

  EXPECT_TRUE(worker.cancel_current());

  sink.release_writes();

  const auto completion =
    worker.wait_completion_for(500ms);

  ASSERT_TRUE(completion.has_value());

  EXPECT_EQ(completion->request_id, 9U);
  EXPECT_EQ(
    completion->status,
    savo_speech::audio::PlaybackCompletionStatus::
    Cancelled);

  EXPECT_LT(completion->frames_submitted, 64U);
  EXPECT_GE(sink.stop_calls(), 1U);

  worker.stop();
}

TEST(PlaybackWorker, RuntimeFailureProducesFailedCompletion)
{
  ControlledPlaybackSink sink;
  sink.set_throw_on_write(true);

  savo_speech::audio::MicrophoneGate gate;

  savo_speech::audio::PlaybackWorker worker{
    sink,
    gate};

  EXPECT_TRUE(worker.start());

  EXPECT_EQ(
    worker.enqueue(make_request(44U)),
    savo_speech::audio::PlaybackEnqueueResult::Accepted);

  const auto completion =
    worker.wait_completion_for(500ms);

  ASSERT_TRUE(completion.has_value());

  EXPECT_EQ(completion->request_id, 44U);
  EXPECT_EQ(
    completion->status,
    savo_speech::audio::PlaybackCompletionStatus::
    Failed);

  EXPECT_NE(
    completion->error.find("scripted playback failure"),
    std::string::npos);

  EXPECT_EQ(
    worker.state(),
    savo_speech::audio::PlaybackWorkerState::Faulted);

  EXPECT_NE(
    worker.last_error().find("scripted playback failure"),
    std::string::npos);

  worker.stop();
}

TEST(PlaybackWorker, RejectsFormatMismatch)
{
  ControlledPlaybackSink sink;
  savo_speech::audio::MicrophoneGate gate;

  savo_speech::audio::PlaybackWorker worker{
    sink,
    gate};

  EXPECT_TRUE(worker.start());

  auto request = make_request(30U);
  request.audio.format.sample_rate_hz = 48000U;

  EXPECT_EQ(
    worker.enqueue(std::move(request)),
    savo_speech::audio::PlaybackEnqueueResult::
    RejectedFormatMismatch);

  worker.stop();
}
