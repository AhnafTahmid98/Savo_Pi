#include "savo_ui/render/canvas.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>

namespace savo_ui
{

Canvas::Canvas(const int width, const int height)
{
  resize(width, height);
}

void Canvas::resize(const int width, const int height)
{
  width_ = std::max(0, width);
  height_ = std::max(0, height);

  const auto byte_count =
    static_cast<std::size_t>(width_) *
    static_cast<std::size_t>(height_) *
    3U;

  pixels_rgb_.assign(byte_count, 0U);
}

bool Canvas::valid() const
{
  return width_ > 0 && height_ > 0 && !pixels_rgb_.empty();
}

int Canvas::width() const
{
  return width_;
}

int Canvas::height() const
{
  return height_;
}

void Canvas::clear(const ColorRgb color)
{
  if (!valid()) {
    return;
  }

  for (std::size_t i = 0; i + 2U < pixels_rgb_.size(); i += 3U) {
    pixels_rgb_[i] = color.r;
    pixels_rgb_[i + 1U] = color.g;
    pixels_rgb_[i + 2U] = color.b;
  }
}

void Canvas::set_pixel(const int x, const int y, const ColorRgb color)
{
  if (!valid()) {
    return;
  }

  if (x < 0 || y < 0 || x >= width_ || y >= height_) {
    return;
  }

  const auto index =
    (static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) +
    static_cast<std::size_t>(x)) *
    3U;

  pixels_rgb_[index] = color.r;
  pixels_rgb_[index + 1U] = color.g;
  pixels_rgb_[index + 2U] = color.b;
}

void Canvas::blend_pixel(const int x, const int y, const ColorRgb color, const float alpha)
{
  if (!valid()) {
    return;
  }

  if (x < 0 || y < 0 || x >= width_ || y >= height_) {
    return;
  }

  const float a = std::clamp(alpha, 0.0F, 1.0F);
  if (a <= 0.0F) {
    return;
  }

  const auto index =
    (static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) +
    static_cast<std::size_t>(x)) *
    3U;

  const auto blend_channel = [a](const std::uint8_t dst, const std::uint8_t src) {
    const float value =
      static_cast<float>(dst) * (1.0F - a) +
      static_cast<float>(src) * a;

    return static_cast<std::uint8_t>(std::clamp(value, 0.0F, 255.0F));
  };

  pixels_rgb_[index] = blend_channel(pixels_rgb_[index], color.r);
  pixels_rgb_[index + 1U] = blend_channel(pixels_rgb_[index + 1U], color.g);
  pixels_rgb_[index + 2U] = blend_channel(pixels_rgb_[index + 2U], color.b);
}

void Canvas::fill_rect(
  const int x,
  const int y,
  const int rect_width,
  const int rect_height,
  const ColorRgb color)
{
  if (!valid() || rect_width <= 0 || rect_height <= 0) {
    return;
  }

  const int x0 = std::max(0, x);
  const int y0 = std::max(0, y);
  const int x1 = std::min(width_, x + rect_width);
  const int y1 = std::min(height_, y + rect_height);

  for (int yy = y0; yy < y1; ++yy) {
    for (int xx = x0; xx < x1; ++xx) {
      set_pixel(xx, yy, color);
    }
  }
}

void Canvas::blend_rect(
  const int x,
  const int y,
  const int rect_width,
  const int rect_height,
  const ColorRgb color,
  const float alpha)
{
  if (!valid() || rect_width <= 0 || rect_height <= 0) {
    return;
  }

  const int x0 = std::max(0, x);
  const int y0 = std::max(0, y);
  const int x1 = std::min(width_, x + rect_width);
  const int y1 = std::min(height_, y + rect_height);

  for (int yy = y0; yy < y1; ++yy) {
    for (int xx = x0; xx < x1; ++xx) {
      blend_pixel(xx, yy, color, alpha);
    }
  }
}

void Canvas::draw_progress_bar(
  const int x,
  const int y,
  const int bar_width,
  const int bar_height,
  const float progress,
  const ColorRgb back_color,
  const ColorRgb fill_color,
  const ColorRgb glow_color)
{
  if (!valid() || bar_width <= 0 || bar_height <= 0) {
    return;
  }

  const float p = std::clamp(progress, 0.0F, 1.0F);
  const int fill_width = static_cast<int>(static_cast<float>(bar_width) * p);

  fill_rect(x, y, bar_width, bar_height, back_color);

  if (fill_width > 0) {
    fill_rect(x, y, fill_width, bar_height, fill_color);

    const int glow_x = x + fill_width - 8;
    blend_rect(glow_x, y - 2, 16, bar_height + 4, glow_color, 0.35F);
  }
}

void Canvas::draw_circle_ring(
  const int center_x,
  const int center_y,
  const int radius,
  const int thickness,
  const ColorRgb color,
  const float alpha)
{
  if (!valid() || radius <= 0 || thickness <= 0) {
    return;
  }

  const int outer_radius = radius + thickness;
  const int inner_radius = std::max(0, radius - thickness);

  const int outer_sq = outer_radius * outer_radius;
  const int inner_sq = inner_radius * inner_radius;

  const int x0 = std::max(0, center_x - outer_radius);
  const int y0 = std::max(0, center_y - outer_radius);
  const int x1 = std::min(width_ - 1, center_x + outer_radius);
  const int y1 = std::min(height_ - 1, center_y + outer_radius);

  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      const int dx = x - center_x;
      const int dy = y - center_y;
      const int dist_sq = dx * dx + dy * dy;

      if (dist_sq >= inner_sq && dist_sq <= outer_sq) {
        const float dist = std::sqrt(static_cast<float>(dist_sq));
        const float edge_distance =
          std::min(
            std::abs(dist - static_cast<float>(inner_radius)),
            std::abs(static_cast<float>(outer_radius) - dist));

        const float softness = std::clamp(edge_distance / 4.0F, 0.15F, 1.0F);
        blend_pixel(x, y, color, alpha * softness);
      }
    }
  }
}

void Canvas::draw_spinner(
  const int cx,
  const int cy,
  const int radius,
  const float phase,
  const ColorRgb color)
{
  if (!valid()) {
    return;
  }

  constexpr int kDots = 12;
  constexpr float kPi = 3.1415926535F;

  const float head = std::fmod(std::max(phase, 0.0F), 1.0F) * static_cast<float>(kDots);

  for (int i = 0; i < kDots; ++i) {
    const float angle = (2.0F * kPi * static_cast<float>(i)) / static_cast<float>(kDots);
    const int px = cx + static_cast<int>(std::cos(angle) * static_cast<float>(radius));
    const int py = cy + static_cast<int>(std::sin(angle) * static_cast<float>(radius));

    float dist = head - static_cast<float>(i);
    if (dist < 0.0F) {
      dist += static_cast<float>(kDots);
    }

    const float alpha = 0.08F + (1.0F - dist / static_cast<float>(kDots)) * 0.85F;
    const int dot_r = (dist < 1.5F) ? 3 : 2;

    for (int dy = -dot_r; dy <= dot_r; ++dy) {
      for (int dx = -dot_r; dx <= dot_r; ++dx) {
        if ((dx * dx + dy * dy) <= (dot_r * dot_r)) {
          blend_pixel(px + dx, py + dy, color, alpha);
        }
      }
    }
  }
}

void Canvas::draw_glow_wave(
  const int x0,
  const int x1,
  const int base_y,
  const int amplitude,
  const float phase,
  const ColorRgb color)
{
  if (!valid() || x1 <= x0) {
    return;
  }

  constexpr float kPi = 3.1415926535F;
  const float phase_rad = phase * 2.0F * kPi;
  const int span = std::max(1, x1 - x0);

  for (int x = x0; x <= x1; ++x) {
    const float t = static_cast<float>(x - x0) / static_cast<float>(span);
    const float y =
      static_cast<float>(base_y) +
      std::sin(t * 2.8F * kPi - phase_rad) * static_cast<float>(amplitude);

    const int yi = static_cast<int>(y);

    for (int dy = -8; dy <= 8; ++dy) {
      const float falloff = std::exp(-0.08F * static_cast<float>(dy * dy));
      blend_pixel(x, yi + dy, color, 0.16F * falloff);
    }

    for (int dy = -2; dy <= 2; ++dy) {
      blend_pixel(x, yi + dy, color, 0.55F);
    }
  }
}

bool Canvas::draw_image(const ImageAsset & image, const int dst_x, const int dst_y)
{
  if (!valid() || !image.valid()) {
    return false;
  }

  const auto & src = image.pixels_rgb();

  for (int sy = 0; sy < image.height(); ++sy) {
    const int dy = dst_y + sy;
    if (dy < 0 || dy >= height_) {
      continue;
    }

    for (int sx = 0; sx < image.width(); ++sx) {
      const int dx = dst_x + sx;
      if (dx < 0 || dx >= width_) {
        continue;
      }

      const auto src_index =
        (static_cast<std::size_t>(sy) * static_cast<std::size_t>(image.width()) +
        static_cast<std::size_t>(sx)) *
        3U;

      const auto dst_index =
        (static_cast<std::size_t>(dy) * static_cast<std::size_t>(width_) +
        static_cast<std::size_t>(dx)) *
        3U;

      pixels_rgb_[dst_index] = src[src_index];
      pixels_rgb_[dst_index + 1U] = src[src_index + 1U];
      pixels_rgb_[dst_index + 2U] = src[src_index + 2U];
    }
  }

  return true;
}

bool Canvas::draw_image_fit(const ImageAsset & image)
{
  if (!valid() || !image.valid()) {
    return false;
  }

  const int dst_x = (width_ - image.width()) / 2;
  const int dst_y = (height_ - image.height()) / 2;

  return draw_image(image, dst_x, dst_y);
}

const std::vector<std::uint8_t> & Canvas::pixels_rgb() const
{
  return pixels_rgb_;
}


bool Canvas::write_ppm(const std::string & path) const
{
  if (!valid()) {
    return false;
  }

  std::ofstream out(path, std::ios::binary);
  if (!out) {
    return false;
  }

  out << "P6\n" << width_ << " " << height_ << "\n255\n";
  out.write(
    reinterpret_cast<const char *>(pixels_rgb_.data()),
    static_cast<std::streamsize>(pixels_rgb_.size()));

  return static_cast<bool>(out);
}


}  // namespace savo_ui
