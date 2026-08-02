// Copyright 2026 Ahnaf Tahmid
#pragma once

#include <string>

#include "savo_ui/render/canvas.hpp"

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
