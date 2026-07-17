#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "savo_speech/audio/captured_audio_processor_chain.hpp"

namespace
{

class RecordingProcessor final :
  public savo_speech::audio::CapturedAudioProcessor
{
public:
  RecordingProcessor(
    std::string name,
    std::vector<std::string> & execution_order)
  : name_{std::move(name)},
    execution_order_{execution_order}
  {
  }

  void process(
    const savo_speech::audio::AudioFrame & frame) override
  {
    execution_order_.push_back(name_);

    ++invocations_;
    last_sequence_ = frame.sequence;

    if (throw_on_process_) {
      throw std::runtime_error{
              "scripted processor failure"};
    }
  }

  void set_throw_on_process(const bool enabled) noexcept
  {
    throw_on_process_ = enabled;
  }

  [[nodiscard]] std::uint64_t invocations() const noexcept
  {
    return invocations_;
  }

  [[nodiscard]] std::uint64_t last_sequence() const noexcept
  {
    return last_sequence_;
  }

private:
  std::string name_;

  std::vector<std::string> & execution_order_;

  bool throw_on_process_{false};

  std::uint64_t invocations_{0U};
  std::uint64_t last_sequence_{0U};
};

[[nodiscard]] savo_speech::audio::AudioFrame
make_frame(const std::uint64_t sequence = 1U)
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

}  // namespace

TEST(
  CapturedAudioProcessorChain,
  RejectsEmptyProcessorName)
{
  std::vector<std::string> execution_order;

  RecordingProcessor processor{
    "processor",
    execution_order};

  savo_speech::audio::CapturedAudioProcessorChain chain;

  EXPECT_THROW(
    chain.add_processor(
      "",
      processor),
    std::invalid_argument);
}

TEST(
  CapturedAudioProcessorChain,
  RejectsDuplicateProcessorName)
{
  std::vector<std::string> execution_order;

  RecordingProcessor first{
    "first",
    execution_order};

  RecordingProcessor second{
    "second",
    execution_order};

  savo_speech::audio::CapturedAudioProcessorChain chain;

  chain.add_processor(
    "duplicate",
    first);

  EXPECT_THROW(
    chain.add_processor(
      "duplicate",
      second),
    std::invalid_argument);
}

TEST(
  CapturedAudioProcessorChain,
  RejectsProcessingBeforeSeal)
{
  std::vector<std::string> execution_order;

  RecordingProcessor processor{
    "processor",
    execution_order};

  savo_speech::audio::CapturedAudioProcessorChain chain;

  chain.add_processor(
    "processor",
    processor);

  EXPECT_THROW(
    chain.process(make_frame()),
    std::logic_error);
}

TEST(
  CapturedAudioProcessorChain,
  RejectsSealingEmptyChain)
{
  savo_speech::audio::CapturedAudioProcessorChain chain;

  EXPECT_THROW(
    static_cast<void>(chain.seal()),
    std::logic_error);
}

TEST(
  CapturedAudioProcessorChain,
  ProcessesInRegistrationOrder)
{
  std::vector<std::string> execution_order;

  RecordingProcessor first{
    "first",
    execution_order};

  RecordingProcessor second{
    "second",
    execution_order};

  RecordingProcessor third{
    "third",
    execution_order};

  savo_speech::audio::CapturedAudioProcessorChain chain;

  chain.add_processor("first", first);
  chain.add_processor("second", second);
  chain.add_processor("third", third);

  ASSERT_TRUE(chain.seal());
  EXPECT_FALSE(chain.seal());

  chain.process(make_frame(27U));

  const std::vector<std::string> expected{
    "first",
    "second",
    "third"};

  EXPECT_EQ(execution_order, expected);

  EXPECT_EQ(first.last_sequence(), 27U);
  EXPECT_EQ(second.last_sequence(), 27U);
  EXPECT_EQ(third.last_sequence(), 27U);

  const auto snapshot = chain.snapshot();

  EXPECT_TRUE(snapshot.sealed);
  EXPECT_EQ(snapshot.processor_count, 3U);

  EXPECT_EQ(snapshot.statistics.frames_received, 1U);
  EXPECT_EQ(snapshot.statistics.frames_completed, 1U);
  EXPECT_EQ(snapshot.statistics.frames_failed, 0U);

  EXPECT_EQ(
    snapshot.statistics.processor_invocations,
    3U);

  EXPECT_EQ(
    snapshot.statistics.processor_successes,
    3U);
}

TEST(
  CapturedAudioProcessorChain,
  RequiredFailureStopsLaterProcessors)
{
  std::vector<std::string> execution_order;

  RecordingProcessor first{
    "first",
    execution_order};

  RecordingProcessor failing{
    "failing",
    execution_order};

  RecordingProcessor last{
    "last",
    execution_order};

  failing.set_throw_on_process(true);

  savo_speech::audio::CapturedAudioProcessorChain chain;

  chain.add_processor(
    "first",
    first,
    true);

  chain.add_processor(
    "failing",
    failing,
    true);

  chain.add_processor(
    "last",
    last,
    true);

  ASSERT_TRUE(chain.seal());

  EXPECT_THROW(
    chain.process(make_frame(9U)),
    std::runtime_error);

  EXPECT_EQ(first.invocations(), 1U);
  EXPECT_EQ(failing.invocations(), 1U);
  EXPECT_EQ(last.invocations(), 0U);

  const auto snapshot = chain.snapshot();

  EXPECT_EQ(snapshot.statistics.frames_received, 1U);
  EXPECT_EQ(snapshot.statistics.frames_completed, 0U);
  EXPECT_EQ(snapshot.statistics.frames_failed, 1U);

  EXPECT_EQ(
    snapshot.statistics.required_processor_failures,
    1U);

  EXPECT_EQ(
    snapshot.statistics.optional_processor_failures,
    0U);

  EXPECT_NE(
    snapshot.last_error.find("failing"),
    std::string::npos);
}

TEST(
  CapturedAudioProcessorChain,
  OptionalFailureContinuesProcessing)
{
  std::vector<std::string> execution_order;

  RecordingProcessor optional{
    "optional",
    execution_order};

  RecordingProcessor required{
    "required",
    execution_order};

  optional.set_throw_on_process(true);

  savo_speech::audio::CapturedAudioProcessorChain chain;

  chain.add_processor(
    "optional",
    optional,
    false);

  chain.add_processor(
    "required",
    required,
    true);

  ASSERT_TRUE(chain.seal());

  EXPECT_NO_THROW(
    chain.process(make_frame(11U)));

  EXPECT_EQ(optional.invocations(), 1U);
  EXPECT_EQ(required.invocations(), 1U);

  const std::vector<std::string> expected{
    "optional",
    "required"};

  EXPECT_EQ(execution_order, expected);

  const auto snapshot = chain.snapshot();

  EXPECT_EQ(snapshot.statistics.frames_received, 1U);
  EXPECT_EQ(snapshot.statistics.frames_completed, 1U);
  EXPECT_EQ(snapshot.statistics.frames_failed, 0U);

  EXPECT_EQ(
    snapshot.statistics.frames_with_optional_failures,
    1U);

  EXPECT_EQ(
    snapshot.statistics.optional_processor_failures,
    1U);

  EXPECT_EQ(
    snapshot.statistics.required_processor_failures,
    0U);

  EXPECT_EQ(
    snapshot.statistics.processor_successes,
    1U);

  EXPECT_EQ(
    snapshot.statistics.processor_failures,
    1U);
}

TEST(
  CapturedAudioProcessorChain,
  ResetClearsStatisticsButPreservesRegistration)
{
  std::vector<std::string> execution_order;

  RecordingProcessor processor{
    "processor",
    execution_order};

  savo_speech::audio::CapturedAudioProcessorChain chain;

  chain.add_processor(
    "processor",
    processor,
    true);

  ASSERT_TRUE(chain.seal());

  chain.process(make_frame(3U));
  chain.reset_statistics();

  const auto snapshot = chain.snapshot();

  EXPECT_TRUE(snapshot.sealed);
  EXPECT_EQ(snapshot.processor_count, 1U);

  EXPECT_EQ(snapshot.statistics.frames_received, 0U);
  EXPECT_EQ(snapshot.statistics.frames_completed, 0U);
  EXPECT_EQ(snapshot.statistics.processor_invocations, 0U);

  ASSERT_EQ(snapshot.processors.size(), 1U);

  EXPECT_EQ(
    snapshot.processors.front().name,
    "processor");

  EXPECT_TRUE(
    snapshot.processors.front().required);

  EXPECT_EQ(
    snapshot.processors.front().invocations,
    0U);

  EXPECT_TRUE(snapshot.last_error.empty());
}
