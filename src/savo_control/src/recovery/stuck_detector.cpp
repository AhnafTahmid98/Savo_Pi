// =============================================================================
// Robot SAVO — savo_control / src/controllers/stuck_detector.cpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Translation unit for `include/savo_control/stuck_detector.hpp`.
//
// Why this file is intentionally minimal
// --------------------------------------
// The current `stuck_detector.hpp` is implemented as a header-only utility:
// all methods are defined inline in the header (config normalization, state
// updates, timing helpers, and detector logic).
//
// Therefore, there is no remaining out-of-line implementation required in this
// .cpp at the moment.
//
// Keeping this file is still professional because it:
//   - matches the organized `src/controllers/` structure,
//   - keeps CMake source lists stable,
//   - provides a future home if StuckDetector implementations are moved out of
//     the header later.
//
// Future refactor note
// --------------------
// If you later move implementations into this .cpp, remove the corresponding
// inline method bodies from `stuck_detector.hpp` (or leave declarations only)
// to avoid duplicate symbol / ODR issues.
// =============================================================================

#include "savo_control/stuck_detector.hpp"

namespace savo_control
{
// Intentionally empty.
//
// Current implementation lives in:
//   include/savo_control/stuck_detector.hpp
//
// This file exists to preserve clean project structure and support a future
// migration from header-only implementation to out-of-line implementation
// without changing file paths or CMake source lists.
}  // namespace savo_control