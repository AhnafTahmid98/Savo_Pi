#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "savo_speech/audio/audio_runtime.hpp"

namespace
{

using namespace std::chrono_literals;

class RuntimeCaptureSource final :
  public savo_speech::audio::CaptureSource
{
public:
  explicit RuntimeCaptureSource(
    const bool fail_on_open = false)
  : fail_on_open_{fail_on_open}
  {
  }

  void open() override
  {
    ++open_calls_;

    if (fail_on_open_) {
      throw std::runtime_error{
              "scripted capture open failure"};
    }

    open_.store(true);
  }

  void close() noexcept override
  {
    open_.store(false);
    ++close_calls_;
  }

  [[nodiscard]] bool is_open() const noexcept override
  {
    return open_.load();
  }

  [[nodiscard]] savo_speech::audio::AudioFormat
  format() const noexcept override
  {
    return {
      16000U,
      2U,
      savo_speech::audio::PcmSampleFormat::
      Signed16LittleEndian};
  }

  [[nodiscard]] savo_speech::audio::AudioFrame
  read_frame() override
  {
    if (!is_open()) {
      throw std::logic_error{
              "runtime capture source is closed"};
    }

    std::this_thread::sleep_for(1ms);

    const std::uint64_t sequence =
      sequence_.fetch_add(1U) + 1U;

    savo_speech::audio::AudioFrame frame;

    frame.sequence = sequence;

    frame.captured_at =
      savo_speech::audio::AudioFrame::Clock::now();

    frame.format = format();

    const auto value =
      static_cast<std::int16_t>(
      sequence % 1000U);

    frame.interleaved_samples = {
      value,
      static_cast<std::int16_t>(value + 100),
      static_cast<std::int16_t>(value + 1),
      static_cast<std::int16_t>(value + 101)};

    return frame;
  }

  [[nodiscard]] std::size_t open_calls() const noexcept
  {
    return open_calls_.load();
  }

private:
  bool fail_on_open_{false};

  std::atomic_bool open_{false};

  std::atomic_size_t open_calls_{0U};
  std::atomic_size_t close_calls_{0U};

  std::atomic_uint64_t sequence_{0U};
};

class RuntimePlaybackSink final :
  public savo_speech::audio::PlaybackSink
{
public:
  RuntimePlaybackSink()
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
    ++open_calls_;
  }

  void close() noexcept override
  {
    {
      const std::scoped_lock lock{mutex_};

      open_ = false;
      release_writes_ = true;

      ++close_calls_;
    }

    condition_.notify_all();
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
              "scripted runtime playback failure"};
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
    {
      const std::scoped_lock lock{mutex_};

      ++stop_calls_;
      release_writes_ = true;
    }

    condition_.notify_all();
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

  [[nodiscard]] std::size_t open_calls() const noexcept
  {
    const std::scoped_lock lock{mutex_};
    return open_calls_;
  }

  [[nodiscard]] std::size_t close_calls() const noexcept
  {
    const std::scoped_lock lock{mutex_};
    return close_calls_;
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

  std::size_t open_calls_{0U};
  std::size_t close_calls_{0U};

  std::size_t write_calls_{0U};
  std::size_t drain_calls_{0U};
  std::size_t stop_calls_{0U};
};

[[nodiscard]] savo_speech::audio::AudioRuntimeConfig
make_runtime_config()
{
  savo_speech::audio::AudioRuntimeConfig config;

  config.microphone_gate.post_playback_hold =
    10ms;

  config.capture_pipeline.selected_channel = 1U;
  config.capture_pipeline.pre_roll_samples = 128U;
  config.capture_pipeline.queue_capacity_frames = 128U;

  config.playback_worker.queue_capacity = 4U;
  config.playback_worker.chunk_frames = 8U;

  return config;
}

[[nodiscard]] savo_speech::audio::PlaybackRequest
make_request(
  const std::uint64_t request_id,
  const std::size_t frames = 32U)
{
  savo_speech::audio::PlaybackRequest request;

  request.request_id = request_id;

  request.audio.format = {
    16000U,
    1U,
    savo_speech::audio::PcmSampleFormat::
    Signed16LittleEndian};

  request.audio.interleaved_samples.resize(
    frames,
    static_cast<std::int16_t>(100));

  return request;
}

[[nodiscard]] bool wait_until(
  const std::function<bool()> & predicate,
  const std::chrono::milliseconds timeout = 500ms)
{
  const auto deadline =
    std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }

    std::this_thread::sleep_for(1ms);
  }

  return predicate();
}

}  // namespace

TEST(AudioRuntime, RejectsInvalidConfiguration)
{
  RuntimeCaptureSource capture_source;
  RuntimePlaybackSink playback_sink;

  auto config = make_runtime_config();
  config.playback_worker.queue_capacity = 0U;

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::audio::AudioRuntime(
        capture_source,
        playback_sink,
        config)),
    std::invalid_argument);
}

TEST(AudioRuntime, StartsAndStopsBothWorkers)
{
  RuntimeCaptureSource capture_source;
  RuntimePlaybackSink playback_sink;

  savo_speech::audio::AudioRuntime runtime{
    capture_source,
    playback_sink,
    make_runtime_config()};

  EXPECT_TRUE(runtime.start());
  EXPECT_TRUE(runtime.ready());

  const auto running_snapshot =
    runtime.health_snapshot();

  EXPECT_TRUE(running_snapshot.ready);

  EXPECT_EQ(
    running_snapshot.state,
    savo_speech::audio::AudioRuntimeState::Running);

  EXPECT_FALSE(
    running_snapshot.microphone_gate.shutdown_gate);

  runtime.stop();

  EXPECT_EQ(
    runtime.state(),
    savo_speech::audio::AudioRuntimeState::Stopped);

  const auto stopped_snapshot =
    runtime.health_snapshot();

  EXPECT_TRUE(
    stopped_snapshot.microphone_gate.shutdown_gate);
}

TEST(AudioRuntime, RollsBackPlaybackWhenCaptureStartupFails)
{
  RuntimeCaptureSource capture_source{true};
  RuntimePlaybackSink playback_sink;

  savo_speech::audio::AudioRuntime runtime{
    capture_source,
    playback_sink,
    make_runtime_config()};

  EXPECT_THROW(
    static_cast<void>(runtime.start()),
    std::runtime_error);

  EXPECT_EQ(
    runtime.state(),
    savo_speech::audio::AudioRuntimeState::Faulted);

  EXPECT_FALSE(playback_sink.is_open());

  EXPECT_EQ(capture_source.open_calls(), 1U);
  EXPECT_EQ(playback_sink.open_calls(), 1U);
  EXPECT_GE(playback_sink.close_calls(), 1U);

  const auto statistics = runtime.statistics();

  EXPECT_EQ(statistics.start_failures, 1U);
  EXPECT_EQ(statistics.startup_rollbacks, 1U);

  EXPECT_NE(
    runtime.last_error().find(
      "scripted capture open failure"),
    std::string::npos);
}

TEST(AudioRuntime, RoutesCapturedFramesThroughPipeline)
{
  RuntimeCaptureSource capture_source;
  RuntimePlaybackSink playback_sink;

  savo_speech::audio::AudioRuntime runtime{
    capture_source,
    playback_sink,
    make_runtime_config()};

  ASSERT_TRUE(runtime.start());

  const auto frame =
    runtime.wait_captured_frame_for(500ms);

  ASSERT_TRUE(frame.has_value());

  EXPECT_EQ(frame->format.channels, 1U);
  EXPECT_EQ(frame->interleaved_samples.size(), 2U);

  runtime.stop();
}

TEST(AudioRuntime, RoutesPlaybackRequestAndCompletion)
{
  RuntimeCaptureSource capture_source;
  RuntimePlaybackSink playback_sink;

  savo_speech::audio::AudioRuntime runtime{
    capture_source,
    playback_sink,
    make_runtime_config()};

  ASSERT_TRUE(runtime.start());

  EXPECT_EQ(
    runtime.enqueue_playback(
      make_request(1U, 24U)),
    savo_speech::audio::PlaybackEnqueueResult::
    Accepted);

  const auto completion =
    runtime.wait_playback_completion_for(500ms);

  ASSERT_TRUE(completion.has_value());

  EXPECT_EQ(completion->request_id, 1U);

  EXPECT_EQ(
    completion->status,
    savo_speech::audio::PlaybackCompletionStatus::
    Completed);

  EXPECT_EQ(completion->frames_submitted, 24U);
  EXPECT_EQ(completion->chunks_submitted, 3U);

  runtime.stop();
}

TEST(AudioRuntime, GatesCaptureDuringPlayback)
{
  RuntimeCaptureSource capture_source;
  RuntimePlaybackSink playback_sink;

  playback_sink.set_block_writes(true);

  savo_speech::audio::AudioRuntime runtime{
    capture_source,
    playback_sink,
    make_runtime_config()};

  ASSERT_TRUE(runtime.start());

  EXPECT_EQ(
    runtime.enqueue_playback(
      make_request(9U, 64U)),
    savo_speech::audio::PlaybackEnqueueResult::
    Accepted);

  ASSERT_TRUE(playback_sink.wait_for_write());

  ASSERT_TRUE(
    wait_until(
      [&runtime]() {
        return
          runtime.health_snapshot().
          capture_worker_statistics.gated_frames >= 2U;
      }));

  EXPECT_TRUE(runtime.cancel_current_playback());

  playback_sink.release_writes();

  const auto completion =
    runtime.wait_playback_completion_for(500ms);

  ASSERT_TRUE(completion.has_value());

  EXPECT_EQ(
    completion->status,
    savo_speech::audio::PlaybackCompletionStatus::
    Cancelled);

  runtime.stop();
}

TEST(AudioRuntime, PropagatesPlaybackWorkerFault)
{
  RuntimeCaptureSource capture_source;
  RuntimePlaybackSink playback_sink;

  playback_sink.set_throw_on_write(true);

  savo_speech::audio::AudioRuntime runtime{
    capture_source,
    playback_sink,
    make_runtime_config()};

  ASSERT_TRUE(runtime.start());

  EXPECT_EQ(
    runtime.enqueue_playback(make_request(44U)),
    savo_speech::audio::PlaybackEnqueueResult::
    Accepted);

  const auto completion =
    runtime.wait_playback_completion_for(500ms);

  ASSERT_TRUE(completion.has_value());

  EXPECT_EQ(
    completion->status,
    savo_speech::audio::PlaybackCompletionStatus::
    Failed);

  ASSERT_TRUE(
    wait_until(
      [&runtime]() {
        return
          runtime.state() ==
          savo_speech::audio::AudioRuntimeState::Faulted;
      }));

  EXPECT_NE(
    runtime.last_error().find(
      "scripted runtime playback failure"),
    std::string::npos);

  runtime.stop();
}

TEST(AudioRuntime, SupportsCleanRestart)
{
  RuntimeCaptureSource capture_source;
  RuntimePlaybackSink playback_sink;

  savo_speech::audio::AudioRuntime runtime{
    capture_source,
    playback_sink,
    make_runtime_config()};

  EXPECT_TRUE(runtime.start());
  runtime.stop();

  EXPECT_TRUE(runtime.start());
  runtime.stop();

  const auto statistics = runtime.statistics();

  EXPECT_EQ(statistics.starts, 2U);
  EXPECT_EQ(statistics.stops, 2U);

  EXPECT_EQ(playback_sink.open_calls(), 2U);
}
