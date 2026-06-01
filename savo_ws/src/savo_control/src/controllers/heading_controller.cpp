// =============================================================================
// Robot SAVO — savo_control / src/controllers/heading_controller.cpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Translation unit for `include/savo_control/heading_controller.hpp`.
//
// Why this file is intentionally minimal
// --------------------------------------
// The current `heading_controller.hpp` is implemented as a header-only utility:
// all methods are defined inline in the header (including config normalization,
// target management, update(), and update_to_target()).
//
// Therefore, there is no remaining out-of-line implementation required in this
// .cpp at the moment.
//
// Keeping this file is still professional because it:
//   - matches the organized `src/controllers/` structure,
//   - keeps CMake source lists stable,
//   - provides a future home if methods are moved out of the header.
//
// Future refactor note
// --------------------
// If you later move implementations into this .cpp, remove the corresponding
// inline method bodies from the header (or leave declarations only) to avoid
// duplicate symbol / ODR issues.
// =============================================================================

#include "savo_control/heading_controller.hpp"

namespace savo_control
{
// Intentionally empty.
//
// Current implementation lives in:
//   include/savo_control/heading_controller.hpp
//
// This file exists to preserve a clean project structure and support a future
// migration from header-only implementation to out-of-line implementation
// without changing file paths or CMake target source lists.
}  // namespace savo_control