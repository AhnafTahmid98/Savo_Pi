#include <chrono>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "savo_speech/wake_word/wake_word_processor.hpp"

namespace
{

using namespace std::chrono_literals;

class FakeWakeWordBackend final :
  public savo_speech::wake_word::WakeWordBackend
{
public:
  [[nodiscard]]
  savo_speech::wake_word::WakeWordBackendResult
  analyze(
    const savo_speech::audio::AudioFrame &) override
  {
    ++analyze_calls_;

    if (throw_on_analyze_) {
      throw std::runtime_error{
              "scripted wake-word backend failure"};
    }

    if (results_.empty()) {
      return {};
    }

    auto result = std::move(results_.front());
    results_.pop_front();

    return result;
  }

  void reset() noexcept override
  {
    results_.clear();

    throw_on_analyze_ = false;
    analyze_calls_ = 0U;
  }

  void push(
    savo_speech::wake_word::WakeWordBackendResult result)
  {
    results_.push_back(std::move(result));
  }

  void set_throw_on_analyze(const bool enabled) noexcept
  {
    throw_on_analyze_ = enabled;
  }

private:
  std::deque<
    savo_speech::wake_word::WakeWordBackendResult>
  results_{};

  bool throw_on_analyze_{false};
  std::uint64_t analyze_calls_{0U};
};

[[nodiscard]] savo_speech::audio::AudioFrame
make_frame(
  const std::uint64_t sequence,
  const std::chrono::milliseconds offset = 0ms)
{
  savo_speech::audio::AudioFrame frame;

  frame.sequence = sequence;

  frame.captured_at =
    savo_speech::audio::AudioFrame::Clock::time_point{} +
    offset;

  frame.format = {
    16000U,
    1U,
    savo_speech::audio::PcmSampleFormat::
    Signed16LittleEndian};

  frame.interleaved_samples = {
    100,
    -100,
    200,
    -200};

  return frame;
}

[[nodiscard]]
savo_speech::wake_word::WakeWordBackendResult
detection(
  std::string phrase = "hey_savo",
  const double confidence = 0.9)
{
  savo_speech::wake_word::WakeWordBackendResult result;

  result.detected = true;
  result.phrase = std::move(phrase);
  result.confidence = confidence;

  return result;
}

[[nodiscard]]
savo_speech::wake_word::WakeWordProcessorConfig
make_config()
{
  savo_speech::wake_word::WakeWordProcessorConfig config;

  config.confidence_threshold = 0.65;
  config.required_consecutive_detections = 1U;
  config.cooldown = 2000ms;
  config.event_queue_capacity = 4U;

  return config;
}

}  // namespace

TEST(WakeWordProcessor, RejectsInvalidConfiguration)
{
  FakeWakeWordBackend backend;

  auto config = make_config();
  config.event_queue_capacity = 0U;

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::wake_word::WakeWordProcessor(
        backend,
        config)),
    std::invalid_argument);
}

TEST(WakeWordProcessor, EmitsAcceptedDetection)
{
  FakeWakeWordBackend backend;
  backend.push(detection("hey_savo", 0.91));

  savo_speech::wake_word::WakeWordProcessor processor{
    backend,
    make_config()};

  processor.process(make_frame(1U, 1000ms));

  const auto event = processor.try_pop_event();

  ASSERT_TRUE(event.has_value());

  EXPECT_EQ(event->event_id, 1U);
  EXPECT_EQ(event->frame_sequence, 1U);
  EXPECT_EQ(event->phrase, "hey_savo");
  EXPECT_DOUBLE_EQ(event->confidence, 0.91);

  const auto snapshot = processor.snapshot();

  EXPECT_EQ(
    snapshot.statistics.accepted_detections,
    1U);

  EXPECT_EQ(snapshot.queued_events, 0U);
}

TEST(WakeWordProcessor, RejectsBelowThresholdDetection)
{
  FakeWakeWordBackend backend;
  backend.push(detection("hey_savo", 0.4));

  savo_speech::wake_word::WakeWordProcessor processor{
    backend,
    make_config()};

  processor.process(make_frame(1U, 1000ms));

  EXPECT_FALSE(processor.try_pop_event().has_value());

  const auto snapshot = processor.snapshot();

  EXPECT_EQ(snapshot.statistics.backend_detections, 1U);
  EXPECT_EQ(snapshot.statistics.below_threshold, 1U);
  EXPECT_EQ(snapshot.statistics.accepted_detections, 0U);
}

TEST(WakeWordProcessor, RequiresConsecutiveDetections)
{
  FakeWakeWordBackend backend;

  backend.push(detection("hey_savo", 0.8));
  backend.push(detection("hey_savo", 0.85));

  auto config = make_config();
  config.required_consecutive_detections = 2U;

  savo_speech::wake_word::WakeWordProcessor processor{
    backend,
    config};

  processor.process(make_frame(1U, 1000ms));

  EXPECT_FALSE(processor.try_pop_event().has_value());

  processor.process(make_frame(2U, 1020ms));

  const auto event = processor.try_pop_event();

  ASSERT_TRUE(event.has_value());

  EXPECT_EQ(event->frame_sequence, 2U);
  EXPECT_DOUBLE_EQ(event->confidence, 0.85);
}

TEST(WakeWordProcessor, PhraseChangeResetsDebounce)
{
  FakeWakeWordBackend backend;

  backend.push(detection("hey_savo", 0.8));
  backend.push(detection("savo", 0.9));
  backend.push(detection("savo", 0.92));

  auto config = make_config();
  config.required_consecutive_detections = 2U;

  savo_speech::wake_word::WakeWordProcessor processor{
    backend,
    config};

  processor.process(make_frame(1U, 1000ms));
  processor.process(make_frame(2U, 1020ms));

  EXPECT_FALSE(processor.try_pop_event().has_value());

  processor.process(make_frame(3U, 1040ms));

  const auto event = processor.try_pop_event();

  ASSERT_TRUE(event.has_value());

  EXPECT_EQ(event->phrase, "savo");

  const auto snapshot = processor.snapshot();

  EXPECT_EQ(snapshot.statistics.debounce_resets, 1U);
}

TEST(WakeWordProcessor, SuppressesDetectionDuringCooldown)
{
  FakeWakeWordBackend backend;

  backend.push(detection());
  backend.push(detection());
  backend.push(detection());

  auto config = make_config();
  config.cooldown = 2000ms;

  savo_speech::wake_word::WakeWordProcessor processor{
    backend,
    config};

  processor.process(make_frame(1U, 1000ms));

  ASSERT_TRUE(processor.try_pop_event().has_value());

  processor.process(make_frame(2U, 2000ms));

  EXPECT_FALSE(processor.try_pop_event().has_value());

  processor.process(make_frame(3U, 3100ms));

  ASSERT_TRUE(processor.try_pop_event().has_value());

  const auto snapshot = processor.snapshot();

  EXPECT_EQ(snapshot.statistics.accepted_detections, 2U);
  EXPECT_EQ(snapshot.statistics.cooldown_suppressed, 1U);
}

TEST(WakeWordProcessor, DropsOldestEventWhenQueueIsFull)
{
  FakeWakeWordBackend backend;

  backend.push(detection());
  backend.push(detection());
  backend.push(detection());

  auto config = make_config();

  config.cooldown = 0ms;
  config.event_queue_capacity = 2U;

  savo_speech::wake_word::WakeWordProcessor processor{
    backend,
    config};

  processor.process(make_frame(1U, 1000ms));
  processor.process(make_frame(2U, 1020ms));
  processor.process(make_frame(3U, 1040ms));

  const auto first = processor.try_pop_event();
  const auto second = processor.try_pop_event();

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());

  EXPECT_EQ(first->event_id, 2U);
  EXPECT_EQ(second->event_id, 3U);

  const auto snapshot = processor.snapshot();

  EXPECT_EQ(snapshot.statistics.queue_overflows, 1U);
  EXPECT_EQ(snapshot.statistics.events_dropped, 1U);
}

TEST(WakeWordProcessor, BackendFailureIsPropagated)
{
  FakeWakeWordBackend backend;

  backend.set_throw_on_analyze(true);

  savo_speech::wake_word::WakeWordProcessor processor{
    backend,
    make_config()};

  EXPECT_THROW(
    processor.process(make_frame(1U)),
    std::runtime_error);

  const auto snapshot = processor.snapshot();

  EXPECT_EQ(snapshot.statistics.backend_failures, 1U);

  EXPECT_NE(
    snapshot.last_error.find(
      "scripted wake-word backend failure"),
    std::string::npos);
}
