#ifndef SAVO_POWER__CONSTANTS_HPP_
#define SAVO_POWER__CONSTANTS_HPP_

#include <cstdint>

namespace savo_power
{
namespace constants
{

inline constexpr const char * kPackageName = "savo_power";
inline constexpr const char * kRobotName = "Robot Savo";

inline constexpr const char * kCoreUpsNodeName = "core_ups_node";
inline constexpr const char * kEdgeUpsNodeName = "edge_ups_node";
inline constexpr const char * kBaseBatteryNodeName = "base_battery_node";
inline constexpr const char * kPowerAggregatorNodeName = "power_aggregator_node";
inline constexpr const char * kPowerHealthNodeName = "power_health_node";
inline constexpr const char * kPowerDashboardNodeName = "power_dashboard_node";

inline constexpr int kI2cBusDefault = 1;

inline constexpr std::uint8_t kUpsHatAddrDefault = 0x36;
inline constexpr std::uint8_t kUpsHatVoltageRegister = 0x02;
inline constexpr std::uint8_t kUpsHatCapacityRegister = 0x04;

inline constexpr std::uint8_t kAds7830AddrDefault = 0x48;
inline constexpr std::uint8_t kAds7830CmdBase = 0x84;
inline constexpr int kBaseBatteryAdcChannelDefault = 2;

inline constexpr double kUpsLowVoltageDefault = 3.40;
inline constexpr double kUpsCriticalVoltageDefault = 3.20;

inline constexpr double kBaseBatteryEmptyVoltageDefault = 6.40;
inline constexpr double kBaseBatteryLowVoltageDefault = 7.20;
inline constexpr double kBaseBatteryFullVoltageDefault = 8.40;

inline constexpr double kBaseBatteryLowSocDefault = 20.0;
inline constexpr double kBaseBatteryFullSocDefault = 95.0;

inline constexpr double kSampleRateHzDefault = 1.0;
inline constexpr double kHealthPublishHzDefault = 1.0;
inline constexpr double kDashboardPublishHzDefault = 1.0;
inline constexpr double kStaleTimeoutSDefault = 5.0;

inline constexpr bool kAutomaticShutdownEnabledDefault = false;

}  // namespace constants
}  // namespace savo_power

#endif  // SAVO_POWER__CONSTANTS_HPP_
