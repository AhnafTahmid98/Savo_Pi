#include "savo_speech/drivers/alsa_device_resolver.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#include <alsa/asoundlib.h>

#include "savo_speech/drivers/alsa_error.hpp"

namespace savo_speech::drivers
{

namespace
{

[[nodiscard]] std::string safe_string(
  const char * value)
{
  return value == nullptr ? std::string{} : std::string{value};
}

[[nodiscard]] std::string alsa_error_text(
  const int error_code)
{
  return safe_string(snd_strerror(error_code));
}

[[nodiscard]] std::string lower_copy(
  const std::string_view value)
{
  std::string result;
  result.reserve(value.size());

  std::transform(
    value.begin(),
    value.end(),
    std::back_inserter(result),
    [](const char character) {
      return static_cast<char>(
        std::tolower(
          static_cast<unsigned char>(character)));
    });

  return result;
}

[[nodiscard]] bool contains_case_insensitive(
  const std::string_view value,
  const std::string_view query)
{
  if (query.empty()) {
    return true;
  }

  const std::string normalized_value = lower_copy(value);
  const std::string normalized_query = lower_copy(query);

  return normalized_value.find(normalized_query) !=
         std::string::npos;
}

[[nodiscard]] std::string numeric_pcm_name(
  const int card_index,
  const int device_index)
{
  return
    "hw:" + std::to_string(card_index) +
    "," + std::to_string(device_index);
}

[[nodiscard]] std::string stable_pcm_name(
  const std::string_view prefix,
  const std::string & card_id,
  const int card_index,
  const int device_index)
{
  std::ostringstream stream;

  stream << prefix << ":CARD=";

  if (!card_id.empty()) {
    stream << card_id;
  } else {
    stream << card_index;
  }

  stream << ",DEV=" << device_index;

  return stream.str();
}

[[nodiscard]] AudioDeviceCapabilities query_capabilities(
  const std::string & pcm_name,
  const snd_pcm_stream_t stream)
{
  AudioDeviceCapabilities result;

  snd_pcm_t * raw_pcm = nullptr;

  const int open_result = snd_pcm_open(
    &raw_pcm,
    pcm_name.c_str(),
    stream,
    SND_PCM_NONBLOCK);

  if (open_result < 0) {
    result.error =
      "snd_pcm_open failed: " +
      alsa_error_text(open_result);

    return result;
  }

  const auto pcm_deleter = [](snd_pcm_t * handle) {
      if (handle != nullptr) {
        static_cast<void>(snd_pcm_close(handle));
      }
    };

  const std::unique_ptr<snd_pcm_t, decltype(pcm_deleter)>
  pcm{raw_pcm, pcm_deleter};

  snd_pcm_hw_params_t * raw_params = nullptr;

  const int allocation_result =
    snd_pcm_hw_params_malloc(&raw_params);

  if (allocation_result < 0) {
    result.error =
      "snd_pcm_hw_params_malloc failed: " +
      alsa_error_text(allocation_result);

    return result;
  }

  const auto params_deleter =
    [](snd_pcm_hw_params_t * parameters) {
      if (parameters != nullptr) {
        snd_pcm_hw_params_free(parameters);
      }
    };

  const std::unique_ptr<
    snd_pcm_hw_params_t,
    decltype(params_deleter)>
  params{raw_params, params_deleter};

  const int params_result =
    snd_pcm_hw_params_any(pcm.get(), params.get());

  if (params_result < 0) {
    result.error =
      "snd_pcm_hw_params_any failed: " +
      alsa_error_text(params_result);

    return result;
  }

  int capability_result = snd_pcm_hw_params_get_channels_min(
    params.get(),
    &result.minimum_channels);

  if (capability_result < 0) {
    result.error =
      "failed to read minimum channels: " +
      alsa_error_text(capability_result);

    return result;
  }

  capability_result = snd_pcm_hw_params_get_channels_max(
    params.get(),
    &result.maximum_channels);

  if (capability_result < 0) {
    result.error =
      "failed to read maximum channels: " +
      alsa_error_text(capability_result);

    return result;
  }

  int rate_direction = 0;

  capability_result = snd_pcm_hw_params_get_rate_min(
    params.get(),
    &result.minimum_rate_hz,
    &rate_direction);

  if (capability_result < 0) {
    result.error =
      "failed to read minimum sample rate: " +
      alsa_error_text(capability_result);

    return result;
  }

  rate_direction = 0;

  capability_result = snd_pcm_hw_params_get_rate_max(
    params.get(),
    &result.maximum_rate_hz,
    &rate_direction);

  if (capability_result < 0) {
    result.error =
      "failed to read maximum sample rate: " +
      alsa_error_text(capability_result);

    return result;
  }

  struct FormatCandidate
  {
    snd_pcm_format_t format;
    const char * name;
  };

  constexpr std::array<FormatCandidate, 7U>
  format_candidates{{
    {SND_PCM_FORMAT_S16_LE, "S16_LE"},
    {SND_PCM_FORMAT_S24_LE, "S24_LE"},
    {SND_PCM_FORMAT_S24_3LE, "S24_3LE"},
    {SND_PCM_FORMAT_S32_LE, "S32_LE"},
    {SND_PCM_FORMAT_FLOAT_LE, "FLOAT_LE"},
    {SND_PCM_FORMAT_U8, "U8"},
    {SND_PCM_FORMAT_S8, "S8"}
  }};

  for (const auto & candidate : format_candidates) {
    const int test_result = snd_pcm_hw_params_test_format(
      pcm.get(),
      params.get(),
      candidate.format);

    if (test_result == 0) {
      result.supported_formats.emplace_back(candidate.name);
    }
  }

  result.query_succeeded = true;
  return result;
}

void append_stream_descriptor(
  AudioDeviceInventory & inventory,
  snd_ctl_t * control,
  snd_ctl_card_info_t * card_info,
  const int card_index,
  const int device_index,
  const snd_pcm_stream_t alsa_stream,
  const AudioStreamDirection direction)
{
  snd_pcm_info_t * pcm_info = nullptr;
  snd_pcm_info_alloca(&pcm_info);

  snd_pcm_info_set_device(
    pcm_info,
    static_cast<unsigned int>(device_index));

  snd_pcm_info_set_subdevice(pcm_info, 0U);
  snd_pcm_info_set_stream(pcm_info, alsa_stream);

  const int info_result =
    snd_ctl_pcm_info(control, pcm_info);

  if (info_result < 0) {
    return;
  }

  AudioDeviceDescriptor descriptor;

  descriptor.card_index = card_index;
  descriptor.device_index = device_index;

  descriptor.subdevice_count =
    snd_pcm_info_get_subdevices_count(pcm_info);

  descriptor.card_id =
    safe_string(snd_ctl_card_info_get_id(card_info));

  descriptor.card_name =
    safe_string(snd_ctl_card_info_get_name(card_info));

  descriptor.card_long_name =
    safe_string(snd_ctl_card_info_get_longname(card_info));

  descriptor.pcm_id =
    safe_string(snd_pcm_info_get_id(pcm_info));

  descriptor.pcm_name =
    safe_string(snd_pcm_info_get_name(pcm_info));

  descriptor.direction = direction;

  descriptor.hardware_name =
    numeric_pcm_name(card_index, device_index);

  descriptor.stable_hardware_name =
    stable_pcm_name(
    "hw",
    descriptor.card_id,
    card_index,
    device_index);

  descriptor.stable_plugin_name =
    stable_pcm_name(
    "plughw",
    descriptor.card_id,
    card_index,
    device_index);

  descriptor.capabilities = query_capabilities(
    descriptor.hardware_name,
    alsa_stream);

  inventory.devices.push_back(std::move(descriptor));
}

}  // namespace

AudioDeviceInventory AlsaDeviceResolver::enumerate()
{
  AudioDeviceInventory inventory;

  int card_index = -1;

  int result = snd_card_next(&card_index);

  if (result < 0) {
    throw AlsaError{
            "snd_card_next",
            result,
            alsa_error_text(result)};
  }

  while (card_index >= 0) {
    const std::string control_name =
      "hw:" + std::to_string(card_index);

    snd_ctl_t * raw_control = nullptr;

    const int open_result = snd_ctl_open(
      &raw_control,
      control_name.c_str(),
      0);

    if (open_result < 0) {
      inventory.warnings.push_back(
        "Unable to open ALSA card " +
        std::to_string(card_index) + ": " +
        alsa_error_text(open_result));
    } else {
      const auto control_deleter = [](snd_ctl_t * control) {
          if (control != nullptr) {
            static_cast<void>(snd_ctl_close(control));
          }
        };

      const std::unique_ptr<
        snd_ctl_t,
        decltype(control_deleter)>
      control{raw_control, control_deleter};

      snd_ctl_card_info_t * card_info = nullptr;
      snd_ctl_card_info_alloca(&card_info);

      const int card_info_result =
        snd_ctl_card_info(control.get(), card_info);

      if (card_info_result < 0) {
        inventory.warnings.push_back(
          "Unable to read ALSA card information for card " +
          std::to_string(card_index) + ": " +
          alsa_error_text(card_info_result));
      } else {
        int device_index = -1;

        while (true) {
          const int next_device_result =
            snd_ctl_pcm_next_device(
            control.get(),
            &device_index);

          if (next_device_result < 0) {
            inventory.warnings.push_back(
              "Unable to enumerate PCM devices for card " +
              std::to_string(card_index) + ": " +
              alsa_error_text(next_device_result));

            break;
          }

          if (device_index < 0) {
            break;
          }

          append_stream_descriptor(
            inventory,
            control.get(),
            card_info,
            card_index,
            device_index,
            SND_PCM_STREAM_CAPTURE,
            AudioStreamDirection::Capture);

          append_stream_descriptor(
            inventory,
            control.get(),
            card_info,
            card_index,
            device_index,
            SND_PCM_STREAM_PLAYBACK,
            AudioStreamDirection::Playback);
        }
      }
    }

    result = snd_card_next(&card_index);

    if (result < 0) {
      throw AlsaError{
              "snd_card_next",
              result,
              alsa_error_text(result)};
    }
  }

  return inventory;
}

bool AlsaDeviceResolver::matches(
  const AudioDeviceDescriptor & device,
  const std::string_view query)
{
  if (query.empty()) {
    return true;
  }

  return
    contains_case_insensitive(device.card_id, query) ||
    contains_case_insensitive(device.card_name, query) ||
    contains_case_insensitive(device.card_long_name, query) ||
    contains_case_insensitive(device.pcm_id, query) ||
    contains_case_insensitive(device.pcm_name, query) ||
    contains_case_insensitive(device.hardware_name, query) ||
    contains_case_insensitive(
    device.stable_hardware_name,
    query) ||
    contains_case_insensitive(
    device.stable_plugin_name,
    query);
}

std::vector<AudioDeviceDescriptor>
AlsaDeviceResolver::filter(
  const AudioDeviceInventory & inventory,
  const std::string_view query)
{
  std::vector<AudioDeviceDescriptor> results;

  for (const auto & device : inventory.devices) {
    if (matches(device, query)) {
      results.push_back(device);
    }
  }

  return results;
}

}  // namespace savo_speech::drivers
