#include "savo_head/core/head_status.hpp"

namespace savo_head
{

static_assert(kHeadComponentController != nullptr);
static_assert(kHeadComponentScan != nullptr);
static_assert(kHeadComponentTf != nullptr);
static_assert(kHeadComponentStatus != nullptr);
static_assert(kHeadComponentCamera != nullptr);
static_assert(kHeadComponentAprilTag != nullptr);

static_assert(kPanCenterDeg == 72);
static_assert(kTiltCenterDeg == 55);
static_assert(kTiltMaxDeg == 130);

}  // namespace savo_head
