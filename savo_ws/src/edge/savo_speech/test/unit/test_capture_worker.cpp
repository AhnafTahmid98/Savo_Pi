#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "savo_speech/audio/capture_pipeline.hpp"
#include "savo_speech/audio/capture_source.hpp"
#include "savo_speech/audio/capture_worker.hpp"
#include "savo_speech/audio/microphone_gate.hpp"

namespace
{

using namespace std::chrono_literals;

class FakeCaptureSource final :
  public savo_speech::audio::CaptureSource
{
public:
  explicit FakeCaptureSource(
    const std::size_t fail_after_reads =
      std::numeric_limits<std::size_t>::max())
  : fail_after_reads_{fail_after_reads}
  {
  }

  void open() override
  {
    open_.store(true);
    ++open_calls_;
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
              "fake capture source is closed"};
    }

    std::this_thread::sleep_for(1ms);

    const std::size_t read_index =
      read_calls_.fetch_add(1U);

    if (read_index >= fail_after_reads_) {
      throw std::runtime_error{
              "scripted capture failure"};
    }

    savo_speech::audio::AudioFrame frame;

    frame.sequence =
      static_cast<std::uint64_t>(read_index + 1U);

    frame.captured_at =
      savo_speech::audio::AudioFrame::Clock::now();

    frame.format = format();

    const auto value =
      static_cast<std::int16_t>(read_index);

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

  [[nodiscard]] std::size_t close_calls() const noexcept
  {
    return close_calls_.load();
  }

private:
  std::size_t fail_after_reads_;

  std::atomic_bool open_{false};

  std::atomic_size_t open_calls_{0U};
  std::atomic_size_t close_calls_{0U};
  std::atomic_size_t read_calls_{0U};
};

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

[[nodiscard]] savo_speech::audio::CapturePipelineConfig
make_pipeline_config()
{
  savo_speech::audio::CapturePipelineConfig config;

  config.selected_channel = 0U;
  config.pre_roll_samples = 128U;
  config.queue_capacity_frames = 128U;

  return config;
}

}  // namespace

TEST(CaptureWorker, StartsRoutesFramesAndStops)
{
  savo_speech::audio::MicrophoneGate gate;

  savo_speech::audio::CapturePipeline pipeline{
    make_pipeline_config(),
    gate};

  FakeCaptureSource source;

  savo_speech::audio::CaptureWorker worker{
    source,
    pipeline};

  EXPECT_TRUE(worker.start());

  ASSERT_TRUE(
    wait_until(
      [&worker]() {
        return worker.statistics().frames_read >= 3U;
      }));

  worker.stop();

  EXPECT_EQ(
    worker.state(),
    savo_speech::audio::CaptureWorkerState::Stopped);

  EXPECT_FALSE(source.is_open());
  EXPECT_EQ(source.open_calls(), 1U);
  EXPECT_GE(source.close_calls(), 1U);

  const auto statistics = worker.statistics();

  EXPECT_EQ(statistics.starts, 1U);
  EXPECT_EQ(statistics.stops, 1U);
  EXPECT_GE(statistics.frames_read, 3U);
  EXPECT_GE(statistics.accepted_frames, 3U);
  EXPECT_EQ(statistics.faults, 0U);
}

TEST(CaptureWorker, StartIsIdempotentWhileRunning)
{
  savo_speech::audio::MicrophoneGate gate;

  savo_speech::audio::CapturePipeline pipeline{
    make_pipeline_config(),
    gate};

  FakeCaptureSource source;

  savo_speech::audio::CaptureWorker worker{
    source,
    pipeline};

  EXPECT_TRUE(worker.start());
  EXPECT_FALSE(worker.start());

  worker.stop();

  EXPECT_EQ(source.open_calls(), 1U);
  EXPECT_EQ(worker.statistics().starts, 1U);
}

TEST(CaptureWorker, CountsFramesDroppedByMicrophoneGate)
{
  savo_speech::audio::MicrophoneGate gate;
  gate.set_manual_gate(true);

  savo_speech::audio::CapturePipeline pipeline{
    make_pipeline_config(),
    gate};

  FakeCaptureSource source;

  savo_speech::audio::CaptureWorker worker{
    source,
    pipeline};

  EXPECT_TRUE(worker.start());

  ASSERT_TRUE(
    wait_until(
      [&worker]() {
        return worker.statistics().gated_frames >= 3U;
      }));

  worker.stop();

  const auto statistics = worker.statistics();

  EXPECT_GE(statistics.gated_frames, 3U);
  EXPECT_EQ(statistics.accepted_frames, 0U);
  EXPECT_FALSE(pipeline.try_pop().has_value());
}

TEST(CaptureWorker, CaptureFailureTransitionsToFaulted)
{
  savo_speech::audio::MicrophoneGate gate;

  savo_speech::audio::CapturePipeline pipeline{
    make_pipeline_config(),
    gate};

  FakeCaptureSource source{2U};

  savo_speech::audio::CaptureWorker worker{
    source,
    pipeline};

  EXPECT_TRUE(worker.start());

  ASSERT_TRUE(
    wait_until(
      [&worker]() {
        return
          worker.state() ==
          savo_speech::audio::CaptureWorkerState::Faulted;
      }));

  EXPECT_FALSE(worker.running());
  EXPECT_FALSE(source.is_open());

  EXPECT_NE(
    worker.last_error().find("scripted capture failure"),
    std::string::npos);

  EXPECT_EQ(worker.statistics().faults, 1U);

  worker.stop();

  EXPECT_EQ(
    worker.state(),
    savo_speech::audio::CaptureWorkerState::Stopped);
}

TEST(CaptureWorker, SupportsRestartAfterCleanStop)
{
  savo_speech::audio::MicrophoneGate gate;

  savo_speech::audio::CapturePipeline pipeline{
    make_pipeline_config(),
    gate};

  FakeCaptureSource source;

  savo_speech::audio::CaptureWorker worker{
    source,
    pipeline};

  EXPECT_TRUE(worker.start());

  ASSERT_TRUE(
    wait_until(
      [&worker]() {
        return worker.statistics().frames_read >= 2U;
      }));

  worker.stop();

  EXPECT_TRUE(worker.start());

  ASSERT_TRUE(
    wait_until(
      [&worker]() {
        return worker.statistics().starts >= 2U;
      }));

  worker.stop();

  const auto statistics = worker.statistics();

  EXPECT_EQ(statistics.starts, 2U);
  EXPECT_EQ(statistics.stops, 2U);
  EXPECT_EQ(source.open_calls(), 2U);
}
