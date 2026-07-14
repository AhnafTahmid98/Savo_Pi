#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "savo_speech/audio/audio_level_meter.hpp"
#include "savo_speech/audio/microphone_gate.hpp"
#include "savo_speech/audio/playback_controller.hpp"
#include "savo_speech/audio/wav_reader.hpp"
#include "savo_speech/drivers/alsa_playback_stream.hpp"

namespace
{

struct Options
{
  std::string device{};
  std::filesystem::path input{};

  std::size_t period_frames{320U};
  std::size_t periods{4U};
};

void print_usage(const char * executable)
{
  std::cout
    << "Usage:\n"
    << "  " << executable
    << " --device <ALSA PCM> --input <WAV> [options]\n\n"
    << "Required:\n"
    << "  --device <name>           ALSA playback PCM name\n"
    << "  --input <path>            S16_LE PCM WAV input\n\n"
    << "Options:\n"
    << "  --period-frames <count>   Frames per period; default 320\n"
    << "  --periods <count>         ALSA buffer periods; default 4\n"
    << "  --help                    Show this help\n\n"
    << "Example:\n"
    << "  " << executable
    << " --device default"
    << " --input /tmp/savo_speech_pc_capture.wav\n";
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

[[nodiscard]] std::size_t parse_size(
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

  return static_cast<std::size_t>(parsed);
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

    if (argument == "--input") {
      options.input = require_value(
        argc,
        argv,
        index,
        argument);

      continue;
    }

    if (argument == "--period-frames") {
      options.period_frames = parse_size(
        require_value(argc, argv, index, argument),
        argument);

      if (
        options.period_frames == 0U ||
        options.period_frames > 65536U)
      {
        throw std::invalid_argument{
                "--period-frames must be in the range [1, 65536]"};
      }

      continue;
    }

    if (argument == "--periods") {
      options.periods = parse_size(
        require_value(argc, argv, index, argument),
        argument);

      if (options.periods < 2U || options.periods > 32U) {
        throw std::invalid_argument{
                "--periods must be in the range [2, 32]"};
      }

      continue;
    }

    throw std::invalid_argument{
            "unknown option: " + std::string{argument}};
  }

  if (options.device.empty()) {
    throw std::invalid_argument{
            "--device is required"};
  }

  if (options.input.empty()) {
    throw std::invalid_argument{
            "--input is required"};
  }

  return options;
}

}  // namespace

int main(int argc, char * argv[])
{
  try {
    const Options options = parse_options(argc, argv);

    const auto audio =
      savo_speech::audio::WavReader::read_file(
      options.input);

    const auto level =
      savo_speech::audio::AudioLevelMeter::measure(
      audio.interleaved_samples);

    savo_speech::drivers::AlsaPlaybackConfig config;

    config.device_name = options.device;
    config.requested_format = audio.format;
    config.period_frames = options.period_frames;
    config.periods = options.periods;
    config.require_exact_sample_rate = true;

    savo_speech::drivers::AlsaPlaybackStream playback{
      config};

    playback.open();

    const auto actual_format = playback.format();

    if (actual_format != audio.format) {
      throw std::runtime_error{
              "negotiated ALSA playback format differs from WAV format; "
              "resampling and channel conversion are not implemented yet"};
    }

    std::cout
      << "Playback started\n"
      << "Device:          " << options.device << '\n'
      << "Input:           " << options.input.string() << '\n'
      << "Sample rate:     "
      << actual_format.sample_rate_hz << " Hz\n"
      << "Channels:        "
      << actual_format.channels << '\n'
      << "Period frames:   "
      << playback.period_frames() << '\n'
      << "Buffer frames:   "
      << playback.buffer_frames() << '\n'
      << "Audio frames:    "
      << audio.frame_count() << '\n'
      << "Duration:        "
      << static_cast<double>(audio.duration().count()) /
        1000000000.0
      << " seconds\n"
      << "RMS level:       "
      << level.rms << '\n'
      << "Peak level:      "
      << level.peak << '\n';

    savo_speech::audio::MicrophoneGateConfig gate_config;
    gate_config.post_playback_hold =
      std::chrono::milliseconds{250};

    savo_speech::audio::MicrophoneGate microphone_gate{
      gate_config};

    savo_speech::audio::PlaybackController controller{
      microphone_gate,
      playback.period_frames()};

    const auto execution =
      controller.play(audio, playback);

    const auto gate_state = microphone_gate.state();
    const auto statistics = playback.statistics();

    playback.close();

    std::cout
      << "Playback completed\n"
      << "Frames written:  "
      << statistics.frames_written << '\n'
      << "Chunks written:  "
      << execution.chunks_submitted << '\n'
      << "Outcome:         "
      << (
        execution.outcome ==
        savo_speech::audio::PlaybackOutcome::Completed ?
        "completed" : "cancelled")
      << '\n'
      << "Microphone gate: "
      << savo_speech::audio::to_string(gate_state.reason)
      << '\n'
      << "ALSA writes:     "
      << statistics.alsa_write_calls << '\n'
      << "Short writes:    "
      << statistics.short_writes << '\n'
      << "XRUN recoveries: "
      << statistics.xrun_recoveries << '\n'
      << "Suspend recovery:"
      << ' ' << statistics.suspend_recoveries << '\n'
      << "Drain calls:     "
      << statistics.drain_calls << '\n';

    return EXIT_SUCCESS;
  } catch (const std::exception & exception) {
    std::cerr
      << "savo_speech playback test failed: "
      << exception.what()
      << '\n';

    return EXIT_FAILURE;
  }
}
