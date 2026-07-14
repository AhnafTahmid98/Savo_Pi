#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "savo_speech/audio/audio_buffer.hpp"
#include "savo_speech/audio/audio_level_meter.hpp"
#include "savo_speech/audio/channel_selector.hpp"
#include "savo_speech/audio/wav_writer.hpp"
#include "savo_speech/drivers/alsa_capture_stream.hpp"

namespace
{

struct Options
{
  std::string device{};

  std::uint32_t sample_rate_hz{16000U};
  std::uint16_t channels{1U};

  std::size_t period_frames{320U};
  std::size_t periods{4U};

  double seconds{3.0};

  std::optional<std::uint16_t> selected_channel{};

  std::filesystem::path output{
    "/tmp/savo_speech_capture.wav"};

  bool require_exact_sample_rate{true};
};

void print_usage(const char * executable)
{
  std::cout
    << "Usage:\n"
    << "  " << executable << " --device <ALSA PCM> [options]\n\n"
    << "Required:\n"
    << "  --device <name>           ALSA PCM name\n\n"
    << "Options:\n"
    << "  --rate <Hz>               Sample rate; default 16000\n"
    << "  --channels <count>        Capture channels; default 1\n"
    << "  --period-frames <count>   Frames per read; default 320\n"
    << "  --periods <count>         ALSA buffer periods; default 4\n"
    << "  --seconds <duration>      Capture duration; default 3\n"
    << "  --channel <index>         Extract one channel to mono\n"
    << "  --output <path>           Output WAV path\n"
    << "  --allow-near-rate         Allow a negotiated nearby rate\n"
    << "  --help                    Show this help\n\n"
    << "Examples:\n"
    << "  " << executable
    << " --device default --rate 16000 --channels 1\n\n"
    << "  " << executable
    << " --device plughw:CARD=Array,DEV=0"
    << " --rate 16000 --channels 6 --channel 0\n";
}

[[nodiscard]] const char * require_value(
  const int argc,
  char * argv[],
  int & index,
  const std::string_view option)
{
  if (index + 1 >= argc) {
    throw std::invalid_argument{
            std::string{option} + " requires a value"};
  }

  ++index;
  return argv[index];
}

[[nodiscard]] std::uint64_t parse_unsigned(
  const std::string & value,
  const std::string_view option)
{
  std::size_t consumed{0U};

  const unsigned long long parsed =
    std::stoull(value, &consumed, 10);

  if (consumed != value.size()) {
    throw std::invalid_argument{
            std::string{option} +
            " contains invalid characters"};
  }

  return static_cast<std::uint64_t>(parsed);
}

[[nodiscard]] double parse_positive_double(
  const std::string & value,
  const std::string_view option)
{
  std::size_t consumed{0U};

  const double parsed = std::stod(value, &consumed);

  if (
    consumed != value.size() ||
    !std::isfinite(parsed) ||
    parsed <= 0.0)
  {
    throw std::invalid_argument{
            std::string{option} +
            " must be a finite positive number"};
  }

  return parsed;
}

[[nodiscard]] Options parse_options(
  const int argc,
  char * argv[])
{
  Options options;

  for (int index = 1; index < argc; ++index) {
    const std::string_view argument{argv[index]};

    if (argument == "--help" || argument == "-h") {
      print_usage(argv[0]);
      std::exit(EXIT_SUCCESS);
    }

    if (argument == "--device") {
      options.device = require_value(
        argc,
        argv,
        index,
        argument);

      continue;
    }

    if (argument == "--rate") {
      const auto value = parse_unsigned(
        require_value(argc, argv, index, argument),
        argument);

      if (value < 8000U || value > 192000U) {
        throw std::invalid_argument{
                "--rate must be in the range [8000, 192000]"};
      }

      options.sample_rate_hz =
        static_cast<std::uint32_t>(value);

      continue;
    }

    if (argument == "--channels") {
      const auto value = parse_unsigned(
        require_value(argc, argv, index, argument),
        argument);

      if (value == 0U || value > 32U) {
        throw std::invalid_argument{
                "--channels must be in the range [1, 32]"};
      }

      options.channels =
        static_cast<std::uint16_t>(value);

      continue;
    }

    if (argument == "--period-frames") {
      const auto value = parse_unsigned(
        require_value(argc, argv, index, argument),
        argument);

      if (value == 0U || value > 65536U) {
        throw std::invalid_argument{
                "--period-frames must be in the range [1, 65536]"};
      }

      options.period_frames =
        static_cast<std::size_t>(value);

      continue;
    }

    if (argument == "--periods") {
      const auto value = parse_unsigned(
        require_value(argc, argv, index, argument),
        argument);

      if (value < 2U || value > 32U) {
        throw std::invalid_argument{
                "--periods must be in the range [2, 32]"};
      }

      options.periods =
        static_cast<std::size_t>(value);

      continue;
    }

    if (argument == "--seconds") {
      options.seconds = parse_positive_double(
        require_value(argc, argv, index, argument),
        argument);

      if (options.seconds > 60.0) {
        throw std::invalid_argument{
                "--seconds must not exceed 60"};
      }

      continue;
    }

    if (argument == "--channel") {
      const auto value = parse_unsigned(
        require_value(argc, argv, index, argument),
        argument);

      if (value > 31U) {
        throw std::invalid_argument{
                "--channel must be in the range [0, 31]"};
      }

      options.selected_channel =
        static_cast<std::uint16_t>(value);

      continue;
    }

    if (argument == "--output") {
      options.output = require_value(
        argc,
        argv,
        index,
        argument);

      continue;
    }

    if (argument == "--allow-near-rate") {
      options.require_exact_sample_rate = false;
      continue;
    }

    throw std::invalid_argument{
            "unknown option: " + std::string{argument}};
  }

  if (options.device.empty()) {
    throw std::invalid_argument{
            "--device is required"};
  }

  return options;
}

}  // namespace

int main(int argc, char * argv[])
{
  try {
    const Options options = parse_options(argc, argv);

    savo_speech::drivers::AlsaCaptureConfig config;

    config.device_name = options.device;

    config.requested_format = {
      options.sample_rate_hz,
      options.channels,
      savo_speech::audio::PcmSampleFormat::
      Signed16LittleEndian};

    config.period_frames = options.period_frames;
    config.periods = options.periods;

    config.require_exact_sample_rate =
      options.require_exact_sample_rate;

    savo_speech::drivers::AlsaCaptureStream capture{
      config};

    capture.open();

    const auto actual_format = capture.format();

    if (
      options.selected_channel.has_value() &&
      options.selected_channel.value() >=
      actual_format.channels)
    {
      throw std::out_of_range{
              "selected channel index is outside the "
              "negotiated channel count"};
    }

    const auto target_frames =
      static_cast<std::size_t>(
      std::ceil(
        options.seconds *
        static_cast<double>(actual_format.sample_rate_hz)));

    std::vector<std::int16_t> captured_samples;

    captured_samples.reserve(
      target_frames *
      static_cast<std::size_t>(actual_format.channels));

    std::size_t captured_frames{0U};

    std::cout
      << "Capture started\n"
      << "Device:          " << options.device << '\n'
      << "Sample rate:     "
      << actual_format.sample_rate_hz << " Hz\n"
      << "Channels:        "
      << actual_format.channels << '\n'
      << "Period frames:   "
      << capture.period_frames() << '\n'
      << "Buffer frames:   "
      << capture.buffer_frames() << '\n'
      << "Target frames:   "
      << target_frames << '\n';

    while (captured_frames < target_frames) {
      auto frame = capture.read_frame();

      const std::size_t remaining_frames =
        target_frames - captured_frames;

      const std::size_t frames_to_copy =
        frame.frame_count() < remaining_frames ?
        frame.frame_count() :
        remaining_frames;

      const std::size_t samples_to_copy =
        frames_to_copy *
        static_cast<std::size_t>(actual_format.channels);

      captured_samples.insert(
        captured_samples.end(),
        frame.interleaved_samples.begin(),
        frame.interleaved_samples.begin() +
        static_cast<std::ptrdiff_t>(samples_to_copy));

      captured_frames += frames_to_copy;
    }

    capture.close();

    savo_speech::audio::AudioBuffer output_audio;

    if (options.selected_channel.has_value()) {
      output_audio.format = {
        actual_format.sample_rate_hz,
        1U,
        actual_format.sample_format};

      output_audio.interleaved_samples =
        savo_speech::audio::ChannelSelector::extract(
        captured_samples,
        actual_format.channels,
        options.selected_channel.value());
    } else {
      output_audio.format = actual_format;
      output_audio.interleaved_samples =
        std::move(captured_samples);
    }

    const auto level =
      savo_speech::audio::AudioLevelMeter::measure(
      output_audio.interleaved_samples);

    savo_speech::audio::WavWriter::write_file(
      options.output,
      output_audio);

    const auto statistics = capture.statistics();

    std::cout
      << "Capture completed\n"
      << "Output:          "
      << options.output.string() << '\n'
      << "Frames:          "
      << output_audio.frame_count() << '\n'
      << "Duration:        "
      << static_cast<double>(
        output_audio.duration().count()) /
        1000000000.0
      << " seconds\n"
      << "RMS level:       "
      << level.rms << '\n'
      << "Peak level:      "
      << level.peak << '\n'
      << "Clipping:        "
      << (level.clipping ? "true" : "false") << '\n'
      << "ALSA read calls: "
      << statistics.alsa_read_calls << '\n'
      << "Short reads:     "
      << statistics.short_reads << '\n'
      << "XRUN recoveries: "
      << statistics.xrun_recoveries << '\n'
      << "Suspend recovery:"
      << ' ' << statistics.suspend_recoveries << '\n';

    return EXIT_SUCCESS;
  } catch (const std::exception & exception) {
    std::cerr
      << "savo_speech capture test failed: "
      << exception.what()
      << '\n';

    return EXIT_FAILURE;
  }
}
