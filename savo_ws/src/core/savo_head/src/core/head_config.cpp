#include "savo_head/core/head_config.hpp"

namespace savo_head
{

static_assert(kHardwareProfileRobotSavoCoreV1 != nullptr);

static_assert(kCameraWidthDefault == 640);
static_assert(kCameraHeightDefault == 480);
static_assert(kCameraFpsDefault == 30);
static_assert(kCameraUdpPortDefault == 5000);
static_assert(kCameraBitrateKbpsDefault == 2000);

static_assert(kPanCenterDeg == 72);
static_assert(kTiltCenterDeg == 55);
static_assert(kTiltMaxDeg == 130);

static_assert(kPanPca9685Channel == 15);
static_assert(kTiltPca9685Channel == 14);

static_assert(kI2cBusDefault == 1);
static_assert(kPca9685AddressDefault == 0x40);

}  // namespace savo_head
