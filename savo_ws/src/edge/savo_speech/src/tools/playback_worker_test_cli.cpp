#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "savo_speech/audio/microphone_gate.hpp"
#include "savo_speech/audio/playback_worker.hpp"
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
  std::size_t chunk_frames{320U};

  std::chrono::milliseconds timeout{30000};
};

void print_usage(const char * executable)
{
  std::cout
    << "Usage:\n"
    << "  " << executable
    << " --device <ALSA PCM> --input <WAV> [options]\n\n"
    << "Required:\n"
    << "  --device <name>           ALSA playback PCM\n"
    << "  --input <path>            S16_LE PCM WAV input\n\n"
    << "Options:\n"
    << "  --period-frames <count>   ALSA period; default 320\n"
    << "  --periods <count>         ALSA periods; default 4\n"
    << "  --chunk-frames <count>    Worker chunk; default 320\n"
    << "  --timeout-ms <count>      Completion timeout; default 30000\n"
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
  const std::string_view option)
{
  std::size_t consumed{0U};

  const unsigned long long parsed =
    std::stoull(value, &consumed, 10);

  if (
    consumed != value.size() ||
    parsed == 0U)
  {
    throw std::invalid_argument{
            std::string{option} +
            " must be a positive integer"};
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

    if (argument == "--timeout-ms") {
      const std::size_t value = parse_size(
        require_value(argc, argv, index, argument),
        argument);

      options.timeout =
        std::chrono::milliseconds{value};

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

    auto audio =
      savo_speech::audio::WavReader::read_file(
      options.input);

    const std::size_t audio_frames =
      audio.frame_count();

    const double duration_seconds =
      static_cast<double>(audio.duration().count()) /
      1000000000.0;

    savo_speech::drivers::AlsaPlaybackConfig sink_config;

    sink_config.device_name = options.device;
    sink_config.requested_format = audio.format;
    sink_config.period_frames = options.period_frames;
    sink_config.periods = options.periods;

    savo_speech::drivers::AlsaPlaybackStream sink{
      sink_config};

    savo_speech::audio::MicrophoneGateConfig gate_config;
    gate_config.post_playback_hold =
      std::chrono::milliseconds{250};

    savo_speech::audio::MicrophoneGate microphone_gate{
      gate_config};

    savo_speech::audio::PlaybackWorkerConfig worker_config;
    worker_config.queue_capacity = 4U;
    worker_config.chunk_frames = options.chunk_frames;

    savo_speech::audio::PlaybackWorker worker{
      sink,
      microphone_gate,
      worker_config};

    const bool worker_started = worker.start();

    if (!worker_started) {
      throw std::runtime_error{
              "playback worker was already active"};
    }

    savo_speech::audio::PlaybackRequest request;
    request.request_id = 1U;
    request.audio = std::move(audio);

    const auto enqueue_result =
      worker.enqueue(std::move(request));

    if (
      enqueue_result !=
      savo_speech::audio::PlaybackEnqueueResult::Accepted)
    {
      worker.stop();

      throw std::runtime_error{
              "playback request was not accepted: " +
              std::string{
                savo_speech::audio::to_string(
                  enqueue_result)}};
    }

    std::cout
      << "Asynchronous playback started\n"
      << "Device:          " << options.device << '\n'
      << "Input:           " << options.input.string() << '\n'
      << "Audio frames:    " << audio_frames << '\n'
      << "Duration:        "
      << duration_seconds << " seconds\n"
      << "Chunk frames:    "
      << options.chunk_frames << '\n';

    const auto completion =
      worker.wait_completion_for(options.timeout);

    if (!completion.has_value()) {
      worker.cancel_all();
      worker.stop();

      throw std::runtime_error{
              "timed out waiting for playback completion"};
    }

    const auto gate_state = microphone_gate.state();
    const auto worker_statistics = worker.statistics();
    const auto sink_statistics = sink.statistics();

    worker.stop();

    std::cout
      << "Asynchronous playback finished\n"
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
      << "Microphone gate: "
      << savo_speech::audio::to_string(
        gate_state.reason)
      << '\n'
      << "Worker completed:"
      << ' ' << worker_statistics.completed_requests
      << '\n'
      << "Worker cancelled:"
      << ' ' << worker_statistics.cancelled_requests
      << '\n'
      << "Worker failed:   "
      << worker_statistics.failed_requests << '\n'
      << "ALSA writes:     "
      << sink_statistics.alsa_write_calls << '\n'
      << "XRUN recoveries: "
      << sink_statistics.xrun_recoveries << '\n';

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

    return EXIT_SUCCESS;
  } catch (const std::exception & exception) {
    std::cerr
      << "savo_speech asynchronous playback test failed: "
      << exception.what()
      << '\n';

    return EXIT_FAILURE;
  }
}
