// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include "savo_bridge/version.hpp"

#include <string>

namespace savo_bridge
{

std::string version_string()
{
  return std::to_string(kVersionMajor) + "." +
         std::to_string(kVersionMinor) + "." +
         std::to_string(kVersionPatch);
}

}  // namespace savo_bridge
