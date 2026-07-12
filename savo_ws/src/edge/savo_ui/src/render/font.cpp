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


int Font::atlas_text_width(
  const ImageAsset & atlas,
  const int cell_width,
  const int cell_height,
  const int columns,
  const std::string & text)
{
  if (
    !atlas.valid() ||
    cell_width <= 0 ||
    cell_height <= 0 ||
    columns <= 0)
  {
    return 0;
  }

  constexpr int first_character = 32;
  constexpr int last_character = 126;

  const auto & pixels = atlas.pixels_rgb();
  int width = 0;

  for (const unsigned char raw_character : text) {
    int character = static_cast<int>(raw_character);

    if (character < first_character || character > last_character) {
      character = static_cast<int>('?');
    }

    const int glyph_index = character - first_character;
    const int source_x =
      (glyph_index % columns) * cell_width;
    const int source_y =
      (glyph_index / columns) * cell_height;

    const std::size_t pixel_index =
      (
        static_cast<std::size_t>(source_y) *
        static_cast<std::size_t>(atlas.width()) +
        static_cast<std::size_t>(source_x)
      ) * 3U;

    if (pixel_index >= pixels.size()) {
      continue;
    }

    width += std::max(1, static_cast<int>(pixels[pixel_index]));
  }

  return width;
}

void Font::draw_atlas_text(
  Canvas & canvas,
  const ImageAsset & atlas,
  const int cell_width,
  const int cell_height,
  const int columns,
  const int padding,
  const int x,
  const int y,
  const std::string & text,
  const ColorRgb color,
  const float alpha)
{
  if (
    !canvas.valid() ||
    !atlas.valid() ||
    cell_width <= 0 ||
    cell_height <= 0 ||
    columns <= 0)
  {
    return;
  }

  constexpr int first_character = 32;
  constexpr int last_character = 126;

  const auto & pixels = atlas.pixels_rgb();
  int cursor_x = x;

  for (const unsigned char raw_character : text) {
    int character = static_cast<int>(raw_character);

    if (character < first_character || character > last_character) {
      character = static_cast<int>('?');
    }

    const int glyph_index = character - first_character;
    const int source_x =
      (glyph_index % columns) * cell_width;
    const int source_y =
      (glyph_index / columns) * cell_height;

    const std::size_t metadata_index =
      (
        static_cast<std::size_t>(source_y) *
        static_cast<std::size_t>(atlas.width()) +
        static_cast<std::size_t>(source_x)
      ) * 3U;

    if (metadata_index >= pixels.size()) {
      continue;
    }

    const int advance =
      std::max(1, static_cast<int>(pixels[metadata_index]));

    for (int local_y = 0; local_y < cell_height; ++local_y) {
      for (int local_x = 0; local_x < cell_width; ++local_x) {
        // The first pixel contains glyph metadata, not artwork.
        if (local_x == 0 && local_y == 0) {
          continue;
        }

        const int atlas_x = source_x + local_x;
        const int atlas_y = source_y + local_y;

        if (
          atlas_x < 0 ||
          atlas_y < 0 ||
          atlas_x >= atlas.width() ||
          atlas_y >= atlas.height())
        {
          continue;
        }

        const std::size_t source_index =
          (
            static_cast<std::size_t>(atlas_y) *
            static_cast<std::size_t>(atlas.width()) +
            static_cast<std::size_t>(atlas_x)
          ) * 3U;

        if (source_index + 2U >= pixels.size()) {
          continue;
        }

        const std::uint8_t mask_value =
          std::max({
            pixels[source_index],
            pixels[source_index + 1U],
            pixels[source_index + 2U]
          });

        if (mask_value == 0U) {
          continue;
        }

        const float mask_alpha =
          static_cast<float>(mask_value) / 255.0F;

        canvas.blend_pixel(
          cursor_x + local_x - padding,
          y + local_y,
          color,
          alpha * mask_alpha);
      }
    }

    cursor_x += advance;
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
