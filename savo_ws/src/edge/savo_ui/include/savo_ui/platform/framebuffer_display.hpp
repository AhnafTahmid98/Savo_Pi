#pragma once

#include "savo_ui/render/canvas.hpp"

#include <cstdint>
#include <string>

namespace savo_ui
{

class FramebufferDisplay
{
public:
  FramebufferDisplay() = default;
  ~FramebufferDisplay();

  FramebufferDisplay(const FramebufferDisplay &) = delete;
  FramebufferDisplay & operator=(const FramebufferDisplay &) = delete;

  bool open_device(
    const std::string & device_path,
    int expected_width,
    int expected_height,
    std::string * error_message = nullptr);

  void close_device();

  bool is_open() const;
  int width() const;
  int height() const;
  int bits_per_pixel() const;
  std::string device_path() const;

  bool present_rgb_canvas(
    const Canvas & canvas,
    std::string * error_message = nullptr);

private:
  bool map_framebuffer(std::string * error_message);
  void unmap_framebuffer();

  int fb_fd_{-1};

  std::string device_path_;

  int width_{0};
  int height_{0};
  int bits_per_pixel_{0};
  int line_length_{0};

  std::size_t mapped_bytes_{0};
  std::uint8_t * mapped_memory_{nullptr};
};

}  // namespace savo_ui
