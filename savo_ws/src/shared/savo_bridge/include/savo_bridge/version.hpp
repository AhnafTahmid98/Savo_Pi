// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#ifndef SAVO_BRIDGE__VERSION_HPP_
#define SAVO_BRIDGE__VERSION_HPP_

#include <string>

namespace savo_bridge
{

inline constexpr int kVersionMajor = 0;
inline constexpr int kVersionMinor = 1;
inline constexpr int kVersionPatch = 0;

[[nodiscard]] std::string version_string();

}  // namespace savo_bridge

#endif  // SAVO_BRIDGE__VERSION_HPP_
