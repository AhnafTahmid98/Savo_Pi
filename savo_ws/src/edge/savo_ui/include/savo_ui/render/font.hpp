#pragma once

#include "savo_ui/render/canvas.hpp"
#include "savo_ui/render/color.hpp"

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

private:
  static const char * glyph(char c);
};

}  // namespace savo_ui
