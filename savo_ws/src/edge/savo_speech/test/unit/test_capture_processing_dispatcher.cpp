#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>

#include "gtest/gtest.h"

#include "savo_speech/audio/capture_processing_dispatcher.hpp"

namespace
{

using namespace std::chrono_literals;

class FakeCapturedFrameSource final :
  public savo_speech::audio::CapturedFrameSource
{
public:
  [[nodiscard]] std::optional<
    savo_speech::audio::AudioFrame>
  wait_captured_frame_for(
    const std::chrono::milliseconds timeout) override
  {
    std::unique_lock lock{mutex_};

    const bool ready = condition_.wait_for(
      lock,
      timeout,
      [this]() {
        return !frames_.empty();
      });

    if (!ready || frames_.empty()) {
      return std::nullopt;
    }

    auto frame = std::move(frames_.front());
    frames_.pop_front();

    return frame;
  }

  void push(savo_speech::audio::AudioFrame frame)
  {
    {
      const std::scoped_lock lock{mutex_};
      frames_.push_back(std::move(frame));
    }

    condition_.notify_one();
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;

  std::deque<savo_speech::audio::AudioFrame>
  frames_;
};

class FakeCapturedAudioProcessor final :
  public savo_speech::audio::CapturedAudioProcessor
{
public:
  void process(
    const savo_speech::audio::AudioFrame &) override
  {
    const std::scoped_lock lock{mutex_};

    if (throw_on_process_) {
      throw std::runtime_error{
              "scripted processor failure"};
    }

    ++processed_;
  }

  void set_throw_on_process(const bool enabled)
  {
    const std::scoped_lock lock{mutex_};
    throw_on_process_ = enabled;
  }

  [[nodiscard]] std::uint64_t processed() const
  {
    const std::scoped_lock lock{mutex_};
    return processed_;
  }

private:
  mutable std::mutex mutex_;

  bool throw_on_process_{false};
  std::uint64_t processed_{0U};
};

[[nodiscard]] savo_speech::audio::AudioFrame
make_frame(const std::uint64_t sequence)
{
  savo_speech::audio::AudioFrame frame;

  frame.sequence = sequence;
  frame.captured_at =
    savo_speech::audio::AudioFrame::Clock::now();

  frame.format = {
    16000U,
    1U,
    savo_speech::audio::PcmSampleFormat::
    Signed16LittleEndian};

  frame.interleaved_samples = {
    10,
    20,
    30,
    40};

  return frame;
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

[[nodiscard]]
savo_speech::audio::CaptureProcessingConfig
make_config()
{
  savo_speech::audio::CaptureProcessingConfig config;

  config.wait_timeout = 5ms;
  config.freshness_timeout = 100ms;
  config.fault_on_processor_error = true;

  return config;
}

}  // namespace

TEST(CaptureProcessingDispatcher, RejectsInvalidConfiguration)
{
  FakeCapturedFrameSource source;
  FakeCapturedAudioProcessor processor;

  auto config = make_config();

  config.wait_timeout = 0ms;

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::audio::CaptureProcessingDispatcher(
        source,
        processor,
        config)),
    std::invalid_argument);
}

TEST(CaptureProcessingDispatcher, ProcessesFramesAndBecomesReady)
{
  FakeCapturedFrameSource source;
  FakeCapturedAudioProcessor processor;

  savo_speech::audio::CaptureProcessingDispatcher
  dispatcher{
    source,
    processor,
    make_config()};

  EXPECT_TRUE(dispatcher.start());

  source.push(make_frame(1U));

  ASSERT_TRUE(
    wait_until(
      [&dispatcher]() {
        return
          dispatcher.statistics().
          frames_processed == 1U;
      }));

  const auto health = dispatcher.health_snapshot();

  EXPECT_TRUE(health.ready);
  EXPECT_TRUE(health.fresh);

  EXPECT_EQ(processor.processed(), 1U);
  EXPECT_EQ(health.statistics.frames_received, 1U);
  EXPECT_EQ(health.statistics.frames_processed, 1U);

  dispatcher.stop();

  EXPECT_EQ(
    dispatcher.state(),
    savo_speech::audio::CaptureProcessingState::
    Stopped);
}

TEST(CaptureProcessingDispatcher, DetectsSequenceGap)
{
  FakeCapturedFrameSource source;
  FakeCapturedAudioProcessor processor;

  savo_speech::audio::CaptureProcessingDispatcher
  dispatcher{
    source,
    processor,
    make_config()};

  ASSERT_TRUE(dispatcher.start());

  source.push(make_frame(1U));
  source.push(make_frame(4U));

  ASSERT_TRUE(
    wait_until(
      [&dispatcher]() {
        return
          dispatcher.statistics().
          frames_processed == 2U;
      }));

  const auto statistics = dispatcher.statistics();

  EXPECT_EQ(statistics.missing_sequence_frames, 2U);
  EXPECT_EQ(statistics.out_of_order_frames, 0U);
  EXPECT_EQ(statistics.last_sequence, 4U);

  dispatcher.stop();
}

TEST(CaptureProcessingDispatcher, ProcessorFailureFaultsWorker)
{
  FakeCapturedFrameSource source;
  FakeCapturedAudioProcessor processor;

  processor.set_throw_on_process(true);

  savo_speech::audio::CaptureProcessingDispatcher
  dispatcher{
    source,
    processor,
    make_config()};

  ASSERT_TRUE(dispatcher.start());

  source.push(make_frame(1U));

  ASSERT_TRUE(
    wait_until(
      [&dispatcher]() {
        return
          dispatcher.state() ==
          savo_speech::audio::CaptureProcessingState::
          Faulted;
      }));

  const auto health = dispatcher.health_snapshot();

  EXPECT_FALSE(health.ready);
  EXPECT_EQ(health.statistics.processor_failures, 1U);
  EXPECT_EQ(health.statistics.faults, 1U);

  EXPECT_NE(
    health.last_error.find(
      "scripted processor failure"),
    std::string::npos);

  dispatcher.stop();
}

TEST(CaptureProcessingDispatcher, ReportsStaleProcessing)
{
  FakeCapturedFrameSource source;
  FakeCapturedAudioProcessor processor;

  auto config = make_config();
  config.freshness_timeout = 15ms;

  savo_speech::audio::CaptureProcessingDispatcher
  dispatcher{
    source,
    processor,
    config};

  ASSERT_TRUE(dispatcher.start());

  source.push(make_frame(1U));

  ASSERT_TRUE(
    wait_until(
      [&dispatcher]() {
        return dispatcher.health_snapshot().ready;
      }));

  std::this_thread::sleep_for(30ms);

  const auto health = dispatcher.health_snapshot();

  EXPECT_FALSE(health.fresh);
  EXPECT_FALSE(health.ready);
  EXPECT_GE(health.last_frame_age_ms, 15);

  dispatcher.stop();
}
