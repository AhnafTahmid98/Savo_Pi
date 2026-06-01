// =============================================================================
// Robot SAVO — savo_control / src/utils/control_math.cpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Translation unit for `include/savo_control/control_math.hpp`.
//
// Why this file is intentionally minimal
// --------------------------------------
// The current `control_math.hpp` implements all functionality inline
// (templates + inline static helpers). That makes it a header-only utility,
// so there is no out-of-line implementation required in this .cpp right now.
//
// Keeping this file is still a professional choice because it:
//   - matches the organized `src/utils/` layout,
//   - keeps CMake source lists stable,
//   - provides a future home if implementations are moved out of the header.
//
// Future refactor note
// --------------------
// If you later move non-template method implementations into this file,
// remove the corresponding inline definitions from the header (or leave only
// declarations) to avoid duplicate symbol / ODR issues.
// =============================================================================

#include "savo_control/control_math.hpp"

namespace savo_control
{
// Intentionally empty.
//
// Current implementation lives in:
//   include/savo_control/control_math.hpp
//
// This file exists to preserve clean structure and allow future migration from
// header-only utilities to out-of-line implementation without changing paths.
}  // namespace savo_control