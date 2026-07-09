#include "savo_ui/render/preview_writer.hpp"

#include <filesystem>
#include <fstream>
#include <string>

namespace savo_ui
{
namespace
{

void set_error(std::string * error_message, const std::string & message)
{
  if (error_message != nullptr) {
    *error_message = message;
  }
}

}  // namespace

bool PreviewWriter::write_canvas_ppm(
  const Canvas & canvas,
  const std::string & path,
  std::string * error_message)
{
  if (!canvas.valid()) {
    set_error(error_message, "cannot write preview: canvas is invalid");
    return false;
  }

  const std::filesystem::path output_path(path);
  const auto parent = output_path.parent_path();

  if (!parent.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(parent, ec);

    if (ec) {
      set_error(error_message, "failed to create preview directory: " + parent.string());
      return false;
    }
  }

  std::ofstream output(path, std::ios::binary);
  if (!output.is_open()) {
    set_error(error_message, "failed to open preview file: " + path);
    return false;
  }

  output << "P6\n";
  output << canvas.width() << " " << canvas.height() << "\n";
  output << "255\n";

  const auto & pixels = canvas.pixels_rgb();
  output.write(
    reinterpret_cast<const char *>(pixels.data()),
    static_cast<std::streamsize>(pixels.size()));

  if (!output.good()) {
    set_error(error_message, "failed to write preview file: " + path);
    return false;
  }

  return true;
}

}  // namespace savo_ui
