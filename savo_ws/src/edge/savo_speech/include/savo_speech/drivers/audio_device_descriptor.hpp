#ifndef SAVO_SPEECH__DRIVERS__AUDIO_DEVICE_DESCRIPTOR_HPP_
#define SAVO_SPEECH__DRIVERS__AUDIO_DEVICE_DESCRIPTOR_HPP_

#include <string>
#include <string_view>
#include <vector>

namespace savo_speech::drivers
{

enum class AudioStreamDirection
{
  Capture,
  Playback
};

[[nodiscard]] constexpr std::string_view to_string(
  const AudioStreamDirection direction) noexcept
{
  switch (direction) {
    case AudioStreamDirection::Capture:
      return "capture";

    case AudioStreamDirection::Playback:
      return "playback";
  }

  return "unknown";
}

struct AudioDeviceCapabilities
{
  bool query_succeeded{false};

  unsigned int minimum_channels{0U};
  unsigned int maximum_channels{0U};

  unsigned int minimum_rate_hz{0U};
  unsigned int maximum_rate_hz{0U};

  std::vector<std::string> supported_formats{};

  std::string error{};
};

struct AudioDeviceDescriptor
{
  int card_index{-1};
  int device_index{-1};

  unsigned int subdevice_count{0U};

  std::string card_id{};
  std::string card_name{};
  std::string card_long_name{};

  std::string pcm_id{};
  std::string pcm_name{};

  AudioStreamDirection direction{
    AudioStreamDirection::Capture};

  // Numeric ALSA address used for low-level probing.
  std::string hardware_name{};

  // Card-ID-based ALSA names are preferable for runtime configuration.
  std::string stable_hardware_name{};
  std::string stable_plugin_name{};

  AudioDeviceCapabilities capabilities{};
};

struct AudioDeviceInventory
{
  std::vector<AudioDeviceDescriptor> devices{};
  std::vector<std::string> warnings{};
};

}  // namespace savo_speech::drivers

#endif  // SAVO_SPEECH__DRIVERS__AUDIO_DEVICE_DESCRIPTOR_HPP_
