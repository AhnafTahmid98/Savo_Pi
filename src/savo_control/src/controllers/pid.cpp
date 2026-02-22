// =============================================================================
// Robot SAVO — savo_control / src/controllers/pid.cpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Translation unit for `include/savo_control/pid.hpp`.
//
// Why this file is intentionally minimal
// --------------------------------------
// The current `pid.hpp` is expected to be implemented as a header-only utility
// (methods defined inline in the header), consistent with other reusable
// controllers/utilities in `savo_control` at this stage.
//
// Therefore, there is no remaining out-of-line implementation required in this
// .cpp right now.
//
// Keeping this file is still professional because it:
//   - matches the organized `src/controllers/` structure,
//   - keeps CMake source lists stable,
//   - provides a future home if PID method implementations are moved out of the
//     header later.
//
// Future refactor note
// --------------------
// If you later move implementations into this .cpp, remove the corresponding
// inline method bodies from `pid.hpp` (or leave declarations only) to avoid
// duplicate symbol / ODR issues.
// =============================================================================

#include "savo_control/pid.hpp"

namespace savo_control
{
// Intentionally empty.
//
// Current implementation lives in:
//   include/savo_control/pid.hpp
//
// This file exists to preserve clean project structure and to support a future
// migration from header-only implementation to out-of-line implementation
// without changing file paths or CMake source lists.
}  // namespace savo_control