#include "savo_speech/drivers/alsa_capture_stream.hpp"

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <alsa/asoundlib.h>

#include "savo_speech/drivers/alsa_error.hpp"

namespace savo_speech::drivers
{

namespace
{

[[nodiscard]] std::string alsa_error_text(
  const int error_code)
{
  const char * text = snd_strerror(error_code);

  return text == nullptr ?
         std::string{"unknown ALSA error"} :
         std::string{text};
}

[[noreturn]] void throw_alsa_error(
  const std::string & operation,
  const int error_code)
{
  throw AlsaError{
          operation,
          error_code,
          alsa_error_text(error_code)};
}

[[nodiscard]] snd_pcm_format_t to_alsa_format(
  const audio::PcmSampleFormat format)
{
  switch (format) {
    case audio::PcmSampleFormat::Signed16LittleEndian:
      return SND_PCM_FORMAT_S16_LE;
  }

  throw std::invalid_argument{
          "unsupported PCM sample format"};
}

}  // namespace

class AlsaCaptureStream::Impl final
{
public:
  explicit Impl(AlsaCaptureConfig config)
  : config_{std::move(config)}
  {
    if (!config_.is_valid()) {
      throw std::invalid_argument{
              "invalid ALSA capture configuration"};
    }
  }

  ~Impl()
  {
    close();
  }

  void open()
  {
    if (handle_ != nullptr) {
      return;
    }

    snd_pcm_t * candidate = nullptr;

    const int open_result = snd_pcm_open(
      &candidate,
      config_.device_name.c_str(),
      SND_PCM_STREAM_CAPTURE,
      0);

    if (open_result < 0) {
      throw_alsa_error(
        "snd_pcm_open(" + config_.device_name + ")",
        open_result);
    }

    try {
      configure(candidate);
      handle_ = candidate;
    } catch (...) {
      static_cast<void>(snd_pcm_close(candidate));
      throw;
    }
  }

  void close() noexcept
  {
    if (handle_ == nullptr) {
      return;
    }

    static_cast<void>(snd_pcm_drop(handle_));
    static_cast<void>(snd_pcm_close(handle_));

    handle_ = nullptr;
  }

  [[nodiscard]] bool is_open() const noexcept
  {
    return handle_ != nullptr;
  }

  [[nodiscard]] audio::AudioFormat format() const noexcept
  {
    return actual_format_;
  }

  [[nodiscard]] std::size_t period_frames() const noexcept
  {
    return actual_period_frames_;
  }

  [[nodiscard]] std::size_t buffer_frames() const noexcept
  {
    return actual_buffer_frames_;
  }

  [[nodiscard]] AlsaCaptureStatistics statistics() const noexcept
  {
    return statistics_;
  }

  [[nodiscard]] audio::AudioFrame read_frame()
  {
    if (handle_ == nullptr) {
      throw std::logic_error{
              "ALSA capture stream is not open"};
    }

    if (
      actual_period_frames_ == 0U ||
      !actual_format_.is_valid())
    {
      throw std::logic_error{
              "ALSA capture stream has no valid negotiated format"};
    }

    const std::size_t channel_count =
      static_cast<std::size_t>(actual_format_.channels);

    if (
      actual_period_frames_ >
      std::numeric_limits<std::size_t>::max() /
      channel_count)
    {
      throw std::length_error{
              "capture period sample count overflows size_t"};
    }

    const std::size_t sample_count =
      actual_period_frames_ * channel_count;

    std::vector<std::int16_t> samples(sample_count);

    std::size_t captured_frames{0U};
    std::size_t recovery_attempts{0U};

    while (captured_frames < actual_period_frames_) {
      const std::size_t remaining_frames =
        actual_period_frames_ - captured_frames;

      const std::size_t sample_offset =
        captured_frames * channel_count;

      ++statistics_.alsa_read_calls;

      const snd_pcm_sframes_t result = snd_pcm_readi(
        handle_,
        samples.data() + sample_offset,
        static_cast<snd_pcm_uframes_t>(remaining_frames));

      if (result == -EINTR) {
        ++statistics_.interrupted_reads;
        continue;
      }

      if (result == -EAGAIN) {
        const int wait_result = snd_pcm_wait(handle_, 1000);

        if (wait_result < 0) {
          recover(wait_result, recovery_attempts);
        }

        continue;
      }

      if (result < 0) {
        recover(
          static_cast<int>(result),
          recovery_attempts);

        continue;
      }

      if (result == 0) {
        continue;
      }

      const std::size_t returned_frames =
        static_cast<std::size_t>(result);

      if (returned_frames < remaining_frames) {
        ++statistics_.short_reads;
      }

      captured_frames += returned_frames;
    }

    ++sequence_;
    ++statistics_.capture_calls;

    statistics_.frames_captured +=
      static_cast<std::uint64_t>(captured_frames);

    audio::AudioFrame frame;

    frame.sequence = sequence_;
    frame.captured_at = audio::AudioFrame::Clock::now();
    frame.format = actual_format_;
    frame.interleaved_samples = std::move(samples);

    return frame;
  }

private:
  void configure(snd_pcm_t * handle)
  {
    snd_pcm_hw_params_t * parameters = nullptr;
    snd_pcm_hw_params_alloca(&parameters);

    int result = snd_pcm_hw_params_any(
      handle,
      parameters);

    if (result < 0) {
      throw_alsa_error(
        "snd_pcm_hw_params_any",
        result);
    }

    result = snd_pcm_hw_params_set_access(
      handle,
      parameters,
      SND_PCM_ACCESS_RW_INTERLEAVED);

    if (result < 0) {
      throw_alsa_error(
        "snd_pcm_hw_params_set_access",
        result);
    }

    result = snd_pcm_hw_params_set_format(
      handle,
      parameters,
      to_alsa_format(
        config_.requested_format.sample_format));

    if (result < 0) {
      throw_alsa_error(
        "snd_pcm_hw_params_set_format",
        result);
    }

    result = snd_pcm_hw_params_set_channels(
      handle,
      parameters,
      static_cast<unsigned int>(
        config_.requested_format.channels));

    if (result < 0) {
      throw_alsa_error(
        "snd_pcm_hw_params_set_channels",
        result);
    }

    unsigned int negotiated_rate =
      config_.requested_format.sample_rate_hz;

    int rate_direction = 0;

    result = snd_pcm_hw_params_set_rate_near(
      handle,
      parameters,
      &negotiated_rate,
      &rate_direction);

    if (result < 0) {
      throw_alsa_error(
        "snd_pcm_hw_params_set_rate_near",
        result);
    }

    if (
      config_.require_exact_sample_rate &&
      negotiated_rate !=
      config_.requested_format.sample_rate_hz)
    {
      throw std::runtime_error{
              "ALSA device negotiated sample rate " +
              std::to_string(negotiated_rate) +
              " Hz instead of required rate " +
              std::to_string(
                config_.requested_format.sample_rate_hz) +
              " Hz"};
    }

    snd_pcm_uframes_t negotiated_period =
      static_cast<snd_pcm_uframes_t>(
      config_.period_frames);

    int period_direction = 0;

    result = snd_pcm_hw_params_set_period_size_near(
      handle,
      parameters,
      &negotiated_period,
      &period_direction);

    if (result < 0) {
      throw_alsa_error(
        "snd_pcm_hw_params_set_period_size_near",
        result);
    }

    if (
      negotiated_period >
      std::numeric_limits<snd_pcm_uframes_t>::max() /
      static_cast<snd_pcm_uframes_t>(config_.periods))
    {
      throw std::length_error{
              "requested ALSA buffer size overflows"};
    }

    snd_pcm_uframes_t negotiated_buffer =
      negotiated_period *
      static_cast<snd_pcm_uframes_t>(config_.periods);

    result = snd_pcm_hw_params_set_buffer_size_near(
      handle,
      parameters,
      &negotiated_buffer);

    if (result < 0) {
      throw_alsa_error(
        "snd_pcm_hw_params_set_buffer_size_near",
        result);
    }

    result = snd_pcm_hw_params(
      handle,
      parameters);

    if (result < 0) {
      throw_alsa_error(
        "snd_pcm_hw_params",
        result);
    }

    unsigned int actual_channels{0U};
    unsigned int actual_rate{0U};
    int actual_rate_direction{0};

    snd_pcm_uframes_t actual_period{0U};
    int actual_period_direction{0};

    snd_pcm_uframes_t actual_buffer{0U};

    result = snd_pcm_hw_params_get_channels(
      parameters,
      &actual_channels);

    if (result < 0) {
      throw_alsa_error(
        "snd_pcm_hw_params_get_channels",
        result);
    }

    result = snd_pcm_hw_params_get_rate(
      parameters,
      &actual_rate,
      &actual_rate_direction);

    if (result < 0) {
      throw_alsa_error(
        "snd_pcm_hw_params_get_rate",
        result);
    }

    result = snd_pcm_hw_params_get_period_size(
      parameters,
      &actual_period,
      &actual_period_direction);

    if (result < 0) {
      throw_alsa_error(
        "snd_pcm_hw_params_get_period_size",
        result);
    }

    result = snd_pcm_hw_params_get_buffer_size(
      parameters,
      &actual_buffer);

    if (result < 0) {
      throw_alsa_error(
        "snd_pcm_hw_params_get_buffer_size",
        result);
    }

    if (
      actual_channels == 0U ||
      actual_channels >
      static_cast<unsigned int>(
        std::numeric_limits<std::uint16_t>::max()))
    {
      throw std::runtime_error{
              "ALSA returned an invalid channel count"};
    }

    if (actual_period == 0U || actual_buffer == 0U) {
      throw std::runtime_error{
              "ALSA returned an invalid period or buffer size"};
    }

    actual_format_ = audio::AudioFormat{
      static_cast<std::uint32_t>(actual_rate),
      static_cast<std::uint16_t>(actual_channels),
      audio::PcmSampleFormat::Signed16LittleEndian};

    actual_period_frames_ =
      static_cast<std::size_t>(actual_period);

    actual_buffer_frames_ =
      static_cast<std::size_t>(actual_buffer);

    result = snd_pcm_prepare(handle);

    if (result < 0) {
      throw_alsa_error(
        "snd_pcm_prepare",
        result);
    }
  }

  void recover(
    const int error_code,
    std::size_t & recovery_attempts)
  {
    if (error_code == -EPIPE) {
      ++statistics_.xrun_recoveries;
    } else if (error_code == -ESTRPIPE) {
      ++statistics_.suspend_recoveries;
    }

    ++recovery_attempts;

    if (
      recovery_attempts >
      config_.maximum_recovery_attempts)
    {
      ++statistics_.recovery_failures;

      throw AlsaError{
              "capture recovery limit exceeded",
              error_code,
              alsa_error_text(error_code)};
    }

    const int recovery_result =
      snd_pcm_recover(
      handle_,
      error_code,
      1);

    if (recovery_result < 0) {
      ++statistics_.recovery_failures;

      throw_alsa_error(
        "snd_pcm_recover",
        recovery_result);
    }
  }

  AlsaCaptureConfig config_;

  snd_pcm_t * handle_{nullptr};

  audio::AudioFormat actual_format_{};

  std::size_t actual_period_frames_{0U};
  std::size_t actual_buffer_frames_{0U};

  std::uint64_t sequence_{0U};

  AlsaCaptureStatistics statistics_{};
};

AlsaCaptureStream::AlsaCaptureStream(
  AlsaCaptureConfig config)
: impl_{std::make_unique<Impl>(std::move(config))}
{
}

AlsaCaptureStream::~AlsaCaptureStream() = default;

void AlsaCaptureStream::open()
{
  impl_->open();
}

void AlsaCaptureStream::close() noexcept
{
  impl_->close();
}

bool AlsaCaptureStream::is_open() const noexcept
{
  return impl_->is_open();
}

audio::AudioFormat AlsaCaptureStream::format() const noexcept
{
  return impl_->format();
}

audio::AudioFrame AlsaCaptureStream::read_frame()
{
  return impl_->read_frame();
}

std::size_t AlsaCaptureStream::period_frames() const noexcept
{
  return impl_->period_frames();
}

std::size_t AlsaCaptureStream::buffer_frames() const noexcept
{
  return impl_->buffer_frames();
}

AlsaCaptureStatistics
AlsaCaptureStream::statistics() const noexcept
{
  return impl_->statistics();
}

}  // namespace savo_speech::drivers
