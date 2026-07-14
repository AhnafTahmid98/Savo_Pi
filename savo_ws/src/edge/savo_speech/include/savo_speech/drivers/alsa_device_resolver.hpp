#ifndef SAVO_SPEECH__DRIVERS__ALSA_DEVICE_RESOLVER_HPP_
#define SAVO_SPEECH__DRIVERS__ALSA_DEVICE_RESOLVER_HPP_

#include <string_view>
#include <vector>

#include "savo_speech/drivers/audio_device_descriptor.hpp"

namespace savo_speech::drivers
{

class AlsaDeviceResolver final
{
public:
  AlsaDeviceResolver() = delete;

  [[nodiscard]] static AudioDeviceInventory enumerate();

  [[nodiscard]] static bool matches(
    const AudioDeviceDescriptor & device,
    std::string_view query);

  [[nodiscard]] static std::vector<AudioDeviceDescriptor> filter(
    const AudioDeviceInventory & inventory,
    std::string_view query);
};

}  // namespace savo_speech::drivers

#endif  // SAVO_SPEECH__DRIVERS__ALSA_DEVICE_RESOLVER_HPP_
