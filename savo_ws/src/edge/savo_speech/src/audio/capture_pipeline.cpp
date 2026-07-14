#include "savo_speech/audio/capture_pipeline.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>

#include "savo_speech/audio/channel_selector.hpp"

namespace savo_speech::audio
{

CapturePipeline::CapturePipeline(
  CapturePipelineConfig config,
  MicrophoneGate & microphone_gate)
: config_{config},
  microphone_gate_{microphone_gate},
  pre_roll_{config.pre_roll_samples},
  queue_{
    config.queue_capacity_frames,
    config.queue_overflow_policy}
{
  if (!config_.is_valid()) {
    throw std::invalid_argument{
            "invalid capture-pipeline configuration"};
  }
}

CapturePipelineResult CapturePipeline::process(
  AudioFrame frame,
  const Clock::time_point now)
{
  {
    const std::scoped_lock lock{state_mutex_};
    ++statistics_.received_frames;
  }

  const bool gated = microphone_gate_.is_gated(now);

  if (gated) {
    bool should_flush{false};

    {
      const std::scoped_lock lock{state_mutex_};

      ++statistics_.gated_frames;

      if (!previously_gated_) {
        previously_gated_ = true;
        ++statistics_.gate_flushes;
        should_flush = true;
      }
    }

    if (should_flush) {
      enter_gated_state();
    }

    return CapturePipelineResult::DroppedWhileGated;
  }

  {
    const std::scoped_lock lock{state_mutex_};
    previously_gated_ = false;
  }

  if (!frame.is_consistent()) {
    const std::scoped_lock lock{state_mutex_};
    ++statistics_.invalid_frames;

    return CapturePipelineResult::RejectedInvalidFrame;
  }

  if (config_.selected_channel >= frame.format.channels) {
    const std::scoped_lock lock{state_mutex_};
    ++statistics_.channel_errors;

    return CapturePipelineResult::RejectedChannelUnavailable;
  }

  AudioFrame mono_frame;

  mono_frame.sequence = frame.sequence;
  mono_frame.captured_at = frame.captured_at;

  mono_frame.format = AudioFormat{
    frame.format.sample_rate_hz,
    1U,
    frame.format.sample_format};

  mono_frame.interleaved_samples =
    ChannelSelector::extract(
    frame.interleaved_samples,
    frame.format.channels,
    config_.selected_channel);

  pre_roll_.append(mono_frame.interleaved_samples);

  {
    const std::scoped_lock lock{state_mutex_};
    output_sample_rate_hz_ = mono_frame.format.sample_rate_hz;
  }

  const QueuePushResult push_result =
    queue_.push(std::move(mono_frame));

  switch (push_result) {
    case QueuePushResult::Accepted:
    {
      const std::scoped_lock lock{state_mutex_};
      ++statistics_.accepted_frames;

      return CapturePipelineResult::Accepted;
    }

    case QueuePushResult::AcceptedAfterDroppingOldest:
    {
      const std::scoped_lock lock{state_mutex_};

      ++statistics_.accepted_frames;
      ++statistics_.queue_drop_events;

      return CapturePipelineResult::
             AcceptedAfterDroppingOldest;
    }

    case QueuePushResult::RejectedFull:
    {
      const std::scoped_lock lock{state_mutex_};
      ++statistics_.queue_rejections;

      return CapturePipelineResult::RejectedQueueFull;
    }

    case QueuePushResult::RejectedClosed:
    {
      const std::scoped_lock lock{state_mutex_};
      ++statistics_.queue_rejections;

      return CapturePipelineResult::RejectedQueueClosed;
    }
  }

  throw std::logic_error{
          "unknown bounded-queue push result"};
}

std::optional<AudioFrame> CapturePipeline::try_pop()
{
  return queue_.try_pop();
}

std::optional<AudioFrame> CapturePipeline::wait_pop_for(
  const std::chrono::milliseconds timeout)
{
  return queue_.wait_pop_for(timeout);
}

AudioBuffer CapturePipeline::pre_roll_snapshot() const
{
  AudioBuffer result;

  {
    const std::scoped_lock lock{state_mutex_};

    if (output_sample_rate_hz_ == 0U) {
      return result;
    }

    result.format = AudioFormat{
      output_sample_rate_hz_,
      1U,
      PcmSampleFormat::Signed16LittleEndian};
  }

  result.interleaved_samples = pre_roll_.snapshot();

  return result;
}

void CapturePipeline::clear() noexcept
{
  queue_.clear();
  pre_roll_.clear();
}

void CapturePipeline::close() noexcept
{
  queue_.close();
}

CapturePipelineStatistics
CapturePipeline::statistics() const noexcept
{
  const std::scoped_lock lock{state_mutex_};
  return statistics_;
}

BoundedAudioQueueStatistics
CapturePipeline::queue_statistics() const noexcept
{
  return queue_.statistics();
}

AudioRingBufferStatistics
CapturePipeline::pre_roll_statistics() const noexcept
{
  return pre_roll_.statistics();
}

void CapturePipeline::enter_gated_state()
{
  queue_.clear();
  pre_roll_.clear();
}

}  // namespace savo_speech::audio
