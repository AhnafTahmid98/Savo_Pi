#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace savo_ui
{

class ImageAsset
{
public:
  ImageAsset() = default;

  bool load_ppm(const std::string & path, std::string * error_message = nullptr);

  bool valid() const;
  int width() const;
  int height() const;
  const std::vector<std::uint8_t> & pixels_rgb() const;

private:
  int width_{0};
  int height_{0};
  std::vector<std::uint8_t> pixels_rgb_;
};

}  // namespace savo_ui
