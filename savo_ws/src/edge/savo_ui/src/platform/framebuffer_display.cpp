#include "savo_ui/platform/framebuffer_display.hpp"

#include <fcntl.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>

namespace savo_ui
{
namespace
{

void set_error(std::string * error_message, const std::string & message)
{
  if (error_message != nullptr) {
    *error_message = message;
  }
}

std::string errno_text(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

}  // namespace

FramebufferDisplay::~FramebufferDisplay()
{
  close_device();
}

bool FramebufferDisplay::open_device(
  const std::string & device_path,
  const int expected_width,
  const int expected_height,
  std::string * error_message)
{
  close_device();

  if (device_path.empty()) {
    set_error(error_message, "framebuffer device path is empty");
    return false;
  }

  fb_fd_ = ::open(device_path.c_str(), O_RDWR);
  if (fb_fd_ < 0) {
    set_error(error_message, errno_text("failed to open framebuffer " + device_path));
    return false;
  }

  fb_var_screeninfo var_info {};
  fb_fix_screeninfo fix_info {};

  if (::ioctl(fb_fd_, FBIOGET_VSCREENINFO, &var_info) < 0) {
    set_error(error_message, errno_text("failed to read framebuffer variable info"));
    close_device();
    return false;
  }

  if (::ioctl(fb_fd_, FBIOGET_FSCREENINFO, &fix_info) < 0) {
    set_error(error_message, errno_text("failed to read framebuffer fixed info"));
    close_device();
    return false;
  }

  width_ = static_cast<int>(var_info.xres);
  height_ = static_cast<int>(var_info.yres);
  bits_per_pixel_ = static_cast<int>(var_info.bits_per_pixel);
  line_length_ = static_cast<int>(fix_info.line_length);
  device_path_ = device_path;

  if (width_ <= 0 || height_ <= 0 || line_length_ <= 0) {
    set_error(error_message, "invalid framebuffer geometry");
    close_device();
    return false;
  }

  if (expected_width > 0 && width_ != expected_width) {
    set_error(
      error_message,
      "framebuffer width mismatch: expected " + std::to_string(expected_width) +
      " got " + std::to_string(width_));
    close_device();
    return false;
  }

  if (expected_height > 0 && height_ != expected_height) {
    set_error(
      error_message,
      "framebuffer height mismatch: expected " + std::to_string(expected_height) +
      " got " + std::to_string(height_));
    close_device();
    return false;
  }

  if (bits_per_pixel_ != 32) {
    set_error(
      error_message,
      "unsupported framebuffer bpp: expected 32 got " + std::to_string(bits_per_pixel_));
    close_device();
    return false;
  }

  if (!map_framebuffer(error_message)) {
    close_device();
    return false;
  }

  return true;
}

void FramebufferDisplay::close_device()
{
  unmap_framebuffer();

  if (fb_fd_ >= 0) {
    ::close(fb_fd_);
    fb_fd_ = -1;
  }

  device_path_.clear();
  width_ = 0;
  height_ = 0;
  bits_per_pixel_ = 0;
  line_length_ = 0;
}

bool FramebufferDisplay::is_open() const
{
  return fb_fd_ >= 0 && mapped_memory_ != nullptr && mapped_bytes_ > 0U;
}

int FramebufferDisplay::width() const
{
  return width_;
}

int FramebufferDisplay::height() const
{
  return height_;
}

int FramebufferDisplay::bits_per_pixel() const
{
  return bits_per_pixel_;
}

std::string FramebufferDisplay::device_path() const
{
  return device_path_;
}

bool FramebufferDisplay::present_rgb_canvas(
  const Canvas & canvas,
  std::string * error_message)
{
  if (!is_open()) {
    set_error(error_message, "framebuffer is not open");
    return false;
  }

  if (!canvas.valid()) {
    set_error(error_message, "canvas is invalid");
    return false;
  }

  if (canvas.width() != width_ || canvas.height() != height_) {
    set_error(
      error_message,
      "canvas size mismatch: canvas=" + std::to_string(canvas.width()) + "x" +
      std::to_string(canvas.height()) + " framebuffer=" +
      std::to_string(width_) + "x" + std::to_string(height_));
    return false;
  }

  const auto & src = canvas.pixels_rgb();

  for (int y = 0; y < height_; ++y) {
    auto * dst_row = mapped_memory_ + static_cast<std::size_t>(y) *
      static_cast<std::size_t>(line_length_);

    for (int x = 0; x < width_; ++x) {
      const auto src_index =
        (static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) +
        static_cast<std::size_t>(x)) *
        3U;

      const auto dst_index = static_cast<std::size_t>(x) * 4U;

      const std::uint8_t r = src[src_index];
      const std::uint8_t g = src[src_index + 1U];
      const std::uint8_t b = src[src_index + 2U];

      // Linux framebuffer on Raspberry Pi commonly uses BGRA/XRGB byte order.
      dst_row[dst_index] = b;
      dst_row[dst_index + 1U] = g;
      dst_row[dst_index + 2U] = r;
      dst_row[dst_index + 3U] = 0U;
    }
  }

  return true;
}

bool FramebufferDisplay::map_framebuffer(std::string * error_message)
{
  mapped_bytes_ = static_cast<std::size_t>(line_length_) *
    static_cast<std::size_t>(height_);

  void * mapped = ::mmap(
    nullptr,
    mapped_bytes_,
    PROT_READ | PROT_WRITE,
    MAP_SHARED,
    fb_fd_,
    0);

  if (mapped == MAP_FAILED) {
    mapped_memory_ = nullptr;
    mapped_bytes_ = 0U;
    set_error(error_message, errno_text("failed to mmap framebuffer"));
    return false;
  }

  mapped_memory_ = static_cast<std::uint8_t *>(mapped);
  return true;
}

void FramebufferDisplay::unmap_framebuffer()
{
  if (mapped_memory_ != nullptr && mapped_bytes_ > 0U) {
    ::munmap(mapped_memory_, mapped_bytes_);
  }

  mapped_memory_ = nullptr;
  mapped_bytes_ = 0U;
}

}  // namespace savo_ui
