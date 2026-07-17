#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

#include "savo_speech/audio/audio_runtime.hpp"
#include "savo_speech/audio/wav_reader.hpp"
#include "savo_speech/drivers/alsa_capture_stream.hpp"
#include "savo_speech/drivers/alsa_playback_stream.hpp"

namespace
{

struct Options
{
  std::string capture_device{};
  std::string playback_device{};

  std::filesystem::path input{};

  std::size_t capture_channels{1U};
  std::size_t selected_channel{0U};

  std::size_t period_frames{320U};
  std::size_t periods{4U};
  std::size_t chunk_frames{320U};

  std::chrono::milliseconds warmup{300};
  std::chrono::milliseconds post_playback_hold{250};
  std::chrono::milliseconds timeout{30000};
};

void print_usage(const char * executable)
{
  std::cout
    << "Usage:\n"
    << "  " << executable
    << " --capture-device <PCM>"
    << " --playback-device <PCM>"
    << " --input <WAV> [options]\n\n"
    << "Required:\n"
    << "  --capture-device <name>   ALSA capture PCM\n"
    << "  --playback-device <name>  ALSA playback PCM\n"
    << "  --input <path>            S16_LE PCM WAV\n\n"
    << "Options:\n"
    << "  --capture-channels <n>    Default 1\n"
    << "  --selected-channel <n>    Default 0\n"
    << "  --period-frames <n>       Default 320\n"
    << "  --periods <n>             Default 4\n"
    << "  --chunk-frames <n>        Default 320\n"
    << "  --warmup-ms <n>           Default 300\n"
    << "  --post-hold-ms <n>        Default 250\n"
    << "  --timeout-ms <n>          Default 30000\n"
    << "  --help                    Show this help\n";
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
  const std::string_view option,
  const bool allow_zero = false)
{
  std::size_t consumed{0U};

  const unsigned long long parsed =
    std::stoull(value, &consumed, 10);

  if (
    consumed != value.size() ||
    (!allow_zero && parsed == 0U) ||
    parsed >
    static_cast<unsigned long long>(
      std::numeric_limits<std::size_t>::max()))
  {
    throw std::invalid_argument{
            std::string{option} +
            " contains an invalid integer"};
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

    if (argument == "--capture-device") {
      options.capture_device = require_value(
        argc,
        argv,
        index,
        argument);

      continue;
    }

    if (argument == "--playback-device") {
      options.playback_device = require_value(
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

    if (argument == "--capture-channels") {
      options.capture_channels = parse_size(
        require_value(argc, argv, index, argument),
        argument);

      continue;
    }

    if (argument == "--selected-channel") {
      options.selected_channel = parse_size(
        require_value(argc, argv, index, argument),
        argument,
        true);

      continue;
    }

    if (argument == "--period-frames") {
      options.period_frames = parse_size(
        require_value(argc, argv, index, argument),
        argument);

      continue;
    }

    if (argument == "--periods") {
      options.periods = parse_size(
        require_value(argc, argv, index, argument),
        argument);

      continue;
    }

    if (argument == "--chunk-frames") {
      options.chunk_frames = parse_size(
        require_value(argc, argv, index, argument),
        argument);

      continue;
    }

    if (argument == "--warmup-ms") {
      options.warmup = std::chrono::milliseconds{
        parse_size(
          require_value(argc, argv, index, argument),
          argument,
          true)};

      continue;
    }

    if (argument == "--post-hold-ms") {
      options.post_playback_hold =
        std::chrono::milliseconds{
        parse_size(
          require_value(argc, argv, index, argument),
          argument,
          true)};

      continue;
    }

    if (argument == "--timeout-ms") {
      options.timeout = std::chrono::milliseconds{
        parse_size(
          require_value(argc, argv, index, argument),
          argument)};

      continue;
    }

    throw std::invalid_argument{
            "unknown option: " +
            std::string{argument}};
  }

  if (options.capture_device.empty()) {
    throw std::invalid_argument{
            "--capture-device is required"};
  }

  if (options.playback_device.empty()) {
    throw std::invalid_argument{
            "--playback-device is required"};
  }

  if (options.input.empty()) {
    throw std::invalid_argument{
            "--input is required"};
  }

  if (
    options.capture_channels >
    std::numeric_limits<std::uint16_t>::max())
  {
    throw std::invalid_argument{
            "--capture-channels is too large"};
  }

  if (
    options.selected_channel >=
    options.capture_channels)
  {
    throw std::invalid_argument{
            "--selected-channel must be lower than "
            "--capture-channels"};
  }

  return options;
}

}  // namespace

int main(int argc, char * argv[])
{
  try {
    const Options options = parse_options(argc, argv);

    auto playback_audio =
      savo_speech::audio::WavReader::read_file(
      options.input);

    const std::size_t audio_frames =
      playback_audio.frame_count();

    const double duration_seconds =
      static_cast<double>(
      playback_audio.duration().count()) /
      1000000000.0;

    savo_speech::drivers::AlsaCaptureConfig
    capture_config;

    capture_config.device_name =
      options.capture_device;

    capture_config.requested_format = {
      playback_audio.format.sample_rate_hz,
      static_cast<std::uint16_t>(
        options.capture_channels),
      savo_speech::audio::PcmSampleFormat::
      Signed16LittleEndian};

    capture_config.period_frames =
      options.period_frames;

    capture_config.periods = options.periods;
    capture_config.require_exact_sample_rate = true;

    savo_speech::drivers::AlsaPlaybackConfig
    playback_config;

    playback_config.device_name =
      options.playback_device;

    playback_config.requested_format =
      playback_audio.format;

    playback_config.period_frames =
      options.period_frames;

    playback_config.periods = options.periods;
    playback_config.require_exact_sample_rate = true;

    savo_speech::drivers::AlsaCaptureStream
    capture_stream{capture_config};

    savo_speech::drivers::AlsaPlaybackStream
    playback_stream{playback_config};

    savo_speech::audio::AudioRuntimeConfig
    runtime_config;

    runtime_config.microphone_gate.
    post_playback_hold =
      options.post_playback_hold;

    runtime_config.capture_pipeline.selected_channel =
      static_cast<std::uint16_t>(
      options.selected_channel);

    runtime_config.capture_pipeline.pre_roll_samples =
      playback_audio.format.sample_rate_hz;

    runtime_config.capture_pipeline.
    queue_capacity_frames = 256U;

    runtime_config.capture_pipeline.
    queue_overflow_policy =
      savo_speech::audio::QueueOverflowPolicy::
      DropOldest;

    runtime_config.playback_worker.queue_capacity =
      4U;

    runtime_config.playback_worker.chunk_frames =
      options.chunk_frames;

    savo_speech::audio::AudioRuntime runtime{
      capture_stream,
      playback_stream,
      runtime_config};

    const bool started = runtime.start();

    if (!started) {
      throw std::runtime_error{
              "audio runtime was already active"};
    }

    std::this_thread::sleep_for(options.warmup);

    const auto warmup_snapshot =
      runtime.health_snapshot();

    savo_speech::audio::PlaybackRequest request;

    request.request_id = 1U;
    request.audio = std::move(playback_audio);

    const auto enqueue_result =
      runtime.enqueue_playback(std::move(request));

    if (
      enqueue_result !=
      savo_speech::audio::PlaybackEnqueueResult::
      Accepted)
    {
      runtime.stop();

      throw std::runtime_error{
              "audio runtime rejected playback request: " +
              std::string{
                savo_speech::audio::to_string(
                  enqueue_result)}};
    }

    std::cout
      << "Unified audio runtime started\n"
      << "Capture device:  "
      << options.capture_device << '\n'
      << "Playback device: "
      << options.playback_device << '\n'
      << "Input:           "
      << options.input.string() << '\n'
      << "Audio frames:    "
      << audio_frames << '\n'
      << "Duration:        "
      << duration_seconds << " seconds\n"
      << "Chunk frames:    "
      << options.chunk_frames << '\n'
      << "Warmup accepted: "
      << warmup_snapshot.
        capture_worker_statistics.accepted_frames
      << '\n';

    const auto completion =
      runtime.wait_playback_completion_for(
      options.timeout);

    if (!completion.has_value()) {
      runtime.cancel_all_playback();
      runtime.stop();

      throw std::runtime_error{
              "timed out waiting for audio-runtime "
              "playback completion"};
    }

    const auto completion_snapshot =
      runtime.health_snapshot();

    std::this_thread::sleep_for(
      options.post_playback_hold +
      std::chrono::milliseconds{100});

    const auto resumed_snapshot =
      runtime.health_snapshot();

    runtime.stop();

    std::cout
      << "Unified audio runtime finished\n"
      << "Runtime state:   "
      << savo_speech::audio::to_string(
        completion_snapshot.state)
      << '\n'
      << "Request ID:      "
      << completion->request_id << '\n'
      << "Status:          "
      << savo_speech::audio::to_string(
        completion->status)
      << '\n'
      << "Frames submitted:"
      << ' ' << completion->frames_submitted << '\n'
      << "Chunks submitted:"
      << ' ' << completion->chunks_submitted << '\n'
      << "Gate at finish:  "
      << savo_speech::audio::to_string(
        completion_snapshot.microphone_gate.reason)
      << '\n'
      << "Capture accepted:"
      << ' ' << completion_snapshot.
        capture_worker_statistics.accepted_frames
      << '\n'
      << "Capture gated:   "
      << completion_snapshot.
        capture_worker_statistics.gated_frames
      << '\n'
      << "Gate flushes:    "
      << completion_snapshot.
        capture_pipeline_statistics.gate_flushes
      << '\n'
      << "Accepted resumed:"
      << ' ' << resumed_snapshot.
        capture_worker_statistics.accepted_frames
      << '\n'
      << "Worker completed:"
      << ' ' << completion_snapshot.
        playback_worker_statistics.completed_requests
      << '\n'
      << "Worker failed:   "
      << completion_snapshot.
        playback_worker_statistics.failed_requests
      << '\n';

    if (
      completion->status !=
      savo_speech::audio::PlaybackCompletionStatus::
      Completed)
    {
      std::cerr
        << "Playback error: "
        << completion->error << '\n';

      return EXIT_FAILURE;
    }

    if (
      completion_snapshot.
      capture_worker_statistics.gated_frames == 0U)
    {
      std::cerr
        << "No capture frames were gated during playback\n";

      return EXIT_FAILURE;
    }

    if (
      resumed_snapshot.
      capture_worker_statistics.accepted_frames <=
      completion_snapshot.
      capture_worker_statistics.accepted_frames)
    {
      std::cerr
        << "Capture did not resume after the echo hold\n";

      return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
  } catch (const std::exception & exception) {
    std::cerr
      << "savo_speech audio-runtime test failed: "
      << exception.what()
      << '\n';

    return EXIT_FAILURE;
  }
}
