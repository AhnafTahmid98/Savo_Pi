#pragma once

#include "savo_ui/render/canvas.hpp"
#include "savo_ui/render/color.hpp"
#include "savo_ui/render/image_asset.hpp"

#include <string>

namespace savo_ui
{

class Font
{
public:
  static void draw_text(
    Canvas & canvas,
    int x,
    int y,
    const std::string & text,
    ColorRgb color,
    int scale = 2,
    float alpha = 1.0F);

  static int text_width(const std::string & text, int scale = 2);

  static void draw_atlas_text(
    Canvas & canvas,
    const ImageAsset & atlas,
    int cell_width,
    int cell_height,
    int columns,
    int padding,
    int x,
    int y,
    const std::string & text,
    ColorRgb color,
    float alpha = 1.0F);

  static int atlas_text_width(
    const ImageAsset & atlas,
    int cell_width,
    int cell_height,
    int columns,
    const std::string & text);

private:
  static const char * glyph(char c);
};

}  // namespace savo_ui
