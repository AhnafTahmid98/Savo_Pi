#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include "savo_speech/drivers/alsa_device_resolver.hpp"
#include "savo_speech/drivers/audio_device_descriptor.hpp"

namespace
{

void print_usage(const char * executable_name)
{
  std::cout
    << "Usage:\n"
    << "  " << executable_name << "\n"
    << "  " << executable_name << " --match <text>\n"
    << "  " << executable_name << " --help\n\n"
    << "Examples:\n"
    << "  " << executable_name << " --match ReSpeaker\n"
    << "  " << executable_name << " --match USB\n";
}

[[nodiscard]] std::string parse_match_query(
  const int argc,
  char * argv[])
{
  if (argc == 1) {
    return {};
  }

  const std::string_view first_argument{argv[1]};

  if (
    first_argument == "--help" ||
    first_argument == "-h")
  {
    print_usage(argv[0]);
    std::exit(EXIT_SUCCESS);
  }

  if (first_argument == "--match") {
    if (argc != 3) {
      throw std::invalid_argument{
              "--match requires exactly one text argument"};
    }

    return std::string{argv[2]};
  }

  throw std::invalid_argument{
          "unknown command-line argument: " +
          std::string{first_argument}};
}

void print_formats(
  const std::vector<std::string> & formats)
{
  if (formats.empty()) {
    std::cout << "none";
    return;
  }

  for (std::size_t index = 0U;
    index < formats.size();
    ++index)
  {
    if (index > 0U) {
      std::cout << ", ";
    }

    std::cout << formats[index];
  }
}

void print_device(
  const savo_speech::drivers::AudioDeviceDescriptor & device)
{
  using savo_speech::drivers::to_string;

  std::cout
    << "------------------------------------------------------------\n"
    << "Direction:       " << to_string(device.direction) << '\n'
    << "Card index:      " << device.card_index << '\n'
    << "Device index:    " << device.device_index << '\n'
    << "Subdevices:      " << device.subdevice_count << '\n'
    << "Card ID:         " << device.card_id << '\n'
    << "Card name:       " << device.card_name << '\n'
    << "Card long name:  " << device.card_long_name << '\n'
    << "PCM ID:          " << device.pcm_id << '\n'
    << "PCM name:        " << device.pcm_name << '\n'
    << "Numeric name:    " << device.hardware_name << '\n'
    << "Stable hw name:  " << device.stable_hardware_name << '\n'
    << "Stable plughw:   " << device.stable_plugin_name << '\n';

  if (!device.capabilities.query_succeeded) {
    std::cout
      << "Capabilities:    unavailable\n"
      << "Capability error:"
      << ' ' << device.capabilities.error << '\n';

    return;
  }

  std::cout
    << "Channels:        "
    << device.capabilities.minimum_channels
    << " - "
    << device.capabilities.maximum_channels
    << '\n'
    << "Sample rates:    "
    << device.capabilities.minimum_rate_hz
    << " - "
    << device.capabilities.maximum_rate_hz
    << " Hz\n"
    << "Formats:         ";

  print_formats(device.capabilities.supported_formats);
  std::cout << '\n';
}

}  // namespace

int main(int argc, char * argv[])
{
  try {
    const std::string query =
      parse_match_query(argc, argv);

    const auto inventory =
      savo_speech::drivers::AlsaDeviceResolver::enumerate();

    for (const auto & warning : inventory.warnings) {
      std::cerr << "WARNING: " << warning << '\n';
    }

    const auto devices =
      savo_speech::drivers::AlsaDeviceResolver::filter(
      inventory,
      query);

    std::cout
      << "ALSA device probe\n"
      << "Filter: "
      << (query.empty() ? "<none>" : query)
      << "\n"
      << "Detected stream endpoints: "
      << inventory.devices.size()
      << "\n"
      << "Matching endpoints: "
      << devices.size()
      << "\n";

    for (const auto & device : devices) {
      print_device(device);
    }

    if (devices.empty()) {
      std::cout
        << "No matching ALSA capture or playback endpoints found.\n";
    }

    return EXIT_SUCCESS;
  } catch (const std::exception & exception) {
    std::cerr
      << "savo_speech audio-device probe failed: "
      << exception.what()
      << '\n';

    return EXIT_FAILURE;
  }
}
