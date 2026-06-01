// =============================================================================
// Robot SAVO — savo_control / src/filters/command_limiter.cpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Translation unit for `include/savo_control/command_limiter.hpp`.
//
// Why this file is intentionally minimal
// --------------------------------------
// The current `command_limiter.hpp` is implemented as a header-only utility:
// all methods are defined inline in the header (AxisLimitConfig helpers,
// CommandLimiterMath helpers, and CommandLimiter methods).
//
// In that design, there is no remaining out-of-line implementation required
// here. Keeping this file is still professional because it:
//   - matches the package's organized `src/filters/` structure,
//   - keeps build layout stable,
//   - provides a future home if methods are moved out of the header.
//
// Important note for future refactor
// ----------------------------------
// If you later move implementations into this .cpp, remove the inline method
// bodies from the header (or leave only declarations) to avoid duplicate
// symbol / ODR issues.
// =============================================================================

#include "savo_control/command_limiter.hpp"

namespace savo_control
{
// Intentionally empty.
//
// Current implementation lives in:
//   include/savo_control/command_limiter.hpp
//
// This file exists to preserve a clean project structure and allow a future
// migration from header-only to out-of-line implementation without changing
// paths or CMake target source lists.
}  // namespace savo_control