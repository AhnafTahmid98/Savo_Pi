#pragma once

#include "savo_ui/render/canvas.hpp"

#include <string>

namespace savo_ui
{

class PreviewWriter
{
public:
  static bool write_canvas_ppm(
    const Canvas & canvas,
    const std::string & path,
    std::string * error_message = nullptr);
};

}  // namespace savo_ui
