#pragma once

#include "savo_ui/render/color.hpp"
#include "savo_ui/render/image_asset.hpp"

#include <cstdint>
#include <vector>

namespace savo_ui
{

class Canvas
{
public:
  Canvas() = default;
  Canvas(int width, int height);

  void resize(int width, int height);

  bool valid() const;
  int width() const;
  int height() const;

  void clear(ColorRgb color);
  void set_pixel(int x, int y, ColorRgb color);
  void blend_pixel(int x, int y, ColorRgb color, float alpha);

  void fill_rect(int x, int y, int width, int height, ColorRgb color);
  void blend_rect(int x, int y, int width, int height, ColorRgb color, float alpha);
  void draw_progress_bar(
    int x,
    int y,
    int width,
    int height,
    float progress,
    ColorRgb back_color,
    ColorRgb fill_color,
    ColorRgb glow_color);
  void draw_circle_ring(
    int center_x,
    int center_y,
    int radius,
    int thickness,
    ColorRgb color,
    float alpha);

  bool draw_image(const ImageAsset & image, int dst_x, int dst_y);
  bool draw_image_fit(const ImageAsset & image);

  const std::vector<std::uint8_t> & pixels_rgb() const;

private:
  int width_{0};
  int height_{0};
  std::vector<std::uint8_t> pixels_rgb_;
};

}  // namespace savo_ui
