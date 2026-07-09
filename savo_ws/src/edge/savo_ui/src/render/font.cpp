#include "savo_ui/render/font.hpp"

#include <algorithm>
#include <cctype>
#include <string>

namespace savo_ui
{
namespace
{

constexpr int kGlyphWidth = 5;
constexpr int kGlyphHeight = 7;

}  // namespace

int Font::text_width(const std::string & text, const int scale)
{
  return static_cast<int>(text.size()) * (kGlyphWidth + 1) * scale;
}

void Font::draw_text(
  Canvas & canvas,
  const int x,
  const int y,
  const std::string & text,
  const ColorRgb color,
  const int scale,
  const float alpha)
{
  if (!canvas.valid() || scale <= 0) {
    return;
  }

  int cursor_x = x;

  for (const char raw_char : text) {
    const char c = static_cast<char>(std::toupper(static_cast<unsigned char>(raw_char)));

    if (c == ' ') {
      cursor_x += (kGlyphWidth + 1) * scale;
      continue;
    }

    const char * pattern = glyph(c);

    for (int row = 0; row < kGlyphHeight; ++row) {
      for (int col = 0; col < kGlyphWidth; ++col) {
        if (pattern[row * kGlyphWidth + col] != '1') {
          continue;
        }

        for (int yy = 0; yy < scale; ++yy) {
          for (int xx = 0; xx < scale; ++xx) {
            canvas.blend_pixel(
              cursor_x + col * scale + xx,
              y + row * scale + yy,
              color,
              alpha);
          }
        }
      }
    }

    cursor_x += (kGlyphWidth + 1) * scale;
  }
}

const char * Font::glyph(const char c)
{
  switch (c) {
    case 'A': return "01110"
                     "10001"
                     "10001"
                     "11111"
                     "10001"
                     "10001"
                     "10001";
    case 'B': return "11110"
                     "10001"
                     "10001"
                     "11110"
                     "10001"
                     "10001"
                     "11110";
    case 'C': return "01111"
                     "10000"
                     "10000"
                     "10000"
                     "10000"
                     "10000"
                     "01111";
    case 'D': return "11110"
                     "10001"
                     "10001"
                     "10001"
                     "10001"
                     "10001"
                     "11110";
    case 'E': return "11111"
                     "10000"
                     "10000"
                     "11110"
                     "10000"
                     "10000"
                     "11111";
    case 'F': return "11111"
                     "10000"
                     "10000"
                     "11110"
                     "10000"
                     "10000"
                     "10000";
    case 'G': return "01111"
                     "10000"
                     "10000"
                     "10011"
                     "10001"
                     "10001"
                     "01111";
    case 'H': return "10001"
                     "10001"
                     "10001"
                     "11111"
                     "10001"
                     "10001"
                     "10001";
    case 'I': return "11111"
                     "00100"
                     "00100"
                     "00100"
                     "00100"
                     "00100"
                     "11111";
    case 'J': return "00111"
                     "00010"
                     "00010"
                     "00010"
                     "10010"
                     "10010"
                     "01100";
    case 'K': return "10001"
                     "10010"
                     "10100"
                     "11000"
                     "10100"
                     "10010"
                     "10001";
    case 'L': return "10000"
                     "10000"
                     "10000"
                     "10000"
                     "10000"
                     "10000"
                     "11111";
    case 'M': return "10001"
                     "11011"
                     "10101"
                     "10101"
                     "10001"
                     "10001"
                     "10001";
    case 'N': return "10001"
                     "11001"
                     "10101"
                     "10011"
                     "10001"
                     "10001"
                     "10001";
    case 'O': return "01110"
                     "10001"
                     "10001"
                     "10001"
                     "10001"
                     "10001"
                     "01110";
    case 'P': return "11110"
                     "10001"
                     "10001"
                     "11110"
                     "10000"
                     "10000"
                     "10000";
    case 'Q': return "01110"
                     "10001"
                     "10001"
                     "10001"
                     "10101"
                     "10010"
                     "01101";
    case 'R': return "11110"
                     "10001"
                     "10001"
                     "11110"
                     "10100"
                     "10010"
                     "10001";
    case 'S': return "01111"
                     "10000"
                     "10000"
                     "01110"
                     "00001"
                     "00001"
                     "11110";
    case 'T': return "11111"
                     "00100"
                     "00100"
                     "00100"
                     "00100"
                     "00100"
                     "00100";
    case 'U': return "10001"
                     "10001"
                     "10001"
                     "10001"
                     "10001"
                     "10001"
                     "01110";
    case 'V': return "10001"
                     "10001"
                     "10001"
                     "10001"
                     "10001"
                     "01010"
                     "00100";
    case 'W': return "10001"
                     "10001"
                     "10001"
                     "10101"
                     "10101"
                     "10101"
                     "01010";
    case 'X': return "10001"
                     "10001"
                     "01010"
                     "00100"
                     "01010"
                     "10001"
                     "10001";
    case 'Y': return "10001"
                     "10001"
                     "01010"
                     "00100"
                     "00100"
                     "00100"
                     "00100";
    case 'Z': return "11111"
                     "00001"
                     "00010"
                     "00100"
                     "01000"
                     "10000"
                     "11111";
    case '0': return "01110"
                     "10001"
                     "10011"
                     "10101"
                     "11001"
                     "10001"
                     "01110";
    case '1': return "00100"
                     "01100"
                     "00100"
                     "00100"
                     "00100"
                     "00100"
                     "01110";
    case '2': return "01110"
                     "10001"
                     "00001"
                     "00010"
                     "00100"
                     "01000"
                     "11111";
    case '3': return "11110"
                     "00001"
                     "00001"
                     "01110"
                     "00001"
                     "00001"
                     "11110";
    case '4': return "00010"
                     "00110"
                     "01010"
                     "10010"
                     "11111"
                     "00010"
                     "00010";
    case '5': return "11111"
                     "10000"
                     "10000"
                     "11110"
                     "00001"
                     "00001"
                     "11110";
    case '6': return "01110"
                     "10000"
                     "10000"
                     "11110"
                     "10001"
                     "10001"
                     "01110";
    case '7': return "11111"
                     "00001"
                     "00010"
                     "00100"
                     "01000"
                     "01000"
                     "01000";
    case '8': return "01110"
                     "10001"
                     "10001"
                     "01110"
                     "10001"
                     "10001"
                     "01110";
    case '9': return "01110"
                     "10001"
                     "10001"
                     "01111"
                     "00001"
                     "00001"
                     "01110";
    case ':': return "00000"
                     "00100"
                     "00100"
                     "00000"
                     "00100"
                     "00100"
                     "00000";
    case '.': return "00000"
                     "00000"
                     "00000"
                     "00000"
                     "00000"
                     "01100"
                     "01100";
    case '-': return "00000"
                     "00000"
                     "00000"
                     "11111"
                     "00000"
                     "00000"
                     "00000";
    default: return "00000"
                    "00000"
                    "00000"
                    "00000"
                    "00000"
                    "00000"
                    "00000";
  }
}

}  // namespace savo_ui
