#include "savo_head/core/servo_calibration.hpp"

namespace savo_head
{

static_assert(kI2cBusDefault == 1);
static_assert(kPca9685AddressDefault == 0x40);
static_assert(kPca9685TicksPerCycle == 4096);

static_assert(kPanPca9685Channel == 15);
static_assert(kTiltPca9685Channel == 14);

static_assert(kPanCenterDeg == 72);
static_assert(kTiltCenterDeg == 55);
static_assert(kTiltMaxDeg == 130);

}  // namespace savo_head
