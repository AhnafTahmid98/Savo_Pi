#include "savo_ui/render/image_asset.hpp"

#include <cctype>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace savo_ui
{
namespace
{

bool read_token(std::istream & input, std::string & token)
{
  token.clear();

  char ch = '\0';

  while (input.get(ch)) {
    if (std::isspace(static_cast<unsigned char>(ch))) {
      continue;
    }

    if (ch == '#') {
      std::string ignored;
      std::getline(input, ignored);
      continue;
    }

    token.push_back(ch);
    break;
  }

  if (token.empty()) {
    return false;
  }

  while (input.get(ch)) {
    if (std::isspace(static_cast<unsigned char>(ch))) {
      break;
    }

    if (ch == '#') {
      std::string ignored;
      std::getline(input, ignored);
      break;
    }

    token.push_back(ch);
  }

  return true;
}

bool parse_positive_int(const std::string & token, int & value)
{
  try {
    std::size_t parsed_chars = 0;
    const long parsed = std::stol(token, &parsed_chars, 10);

    if (parsed_chars != token.size()) {
      return false;
    }

    if (parsed <= 0 || parsed > std::numeric_limits<int>::max()) {
      return false;
    }

    value = static_cast<int>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

void set_error(std::string * error_message, const std::string & message)
{
  if (error_message != nullptr) {
    *error_message = message;
  }
}

}  // namespace

bool ImageAsset::load_ppm(const std::string & path, std::string * error_message)
{
  width_ = 0;
  height_ = 0;
  pixels_rgb_.clear();

  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    set_error(error_message, "failed to open image: " + path);
    return false;
  }

  std::string token;

  if (!read_token(input, token)) {
    set_error(error_message, "missing PPM magic header: " + path);
    return false;
  }

  if (token != "P6") {
    set_error(error_message, "unsupported PPM format, expected P6: " + path);
    return false;
  }

  if (!read_token(input, token) || !parse_positive_int(token, width_)) {
    set_error(error_message, "invalid PPM width: " + path);
    return false;
  }

  if (!read_token(input, token) || !parse_positive_int(token, height_)) {
    set_error(error_message, "invalid PPM height: " + path);
    return false;
  }

  int max_value = 0;
  if (!read_token(input, token) || !parse_positive_int(token, max_value)) {
    set_error(error_message, "invalid PPM max value: " + path);
    return false;
  }

  if (max_value != 255) {
    set_error(error_message, "unsupported PPM max value, expected 255: " + path);
    return false;
  }

  const auto pixel_count = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  const auto byte_count = pixel_count * 3U;

  pixels_rgb_.resize(byte_count);

  input.read(
    reinterpret_cast<char *>(pixels_rgb_.data()),
    static_cast<std::streamsize>(pixels_rgb_.size()));

  if (input.gcount() != static_cast<std::streamsize>(pixels_rgb_.size())) {
    set_error(error_message, "PPM pixel data is incomplete: " + path);
    pixels_rgb_.clear();
    width_ = 0;
    height_ = 0;
    return false;
  }

  return true;
}

bool ImageAsset::valid() const
{
  return width_ > 0 && height_ > 0 && !pixels_rgb_.empty();
}

int ImageAsset::width() const
{
  return width_;
}

int ImageAsset::height() const
{
  return height_;
}

const std::vector<std::uint8_t> & ImageAsset::pixels_rgb() const
{
  return pixels_rgb_;
}

}  // namespace savo_ui
