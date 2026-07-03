#ifndef SAVO_PERCEPTION__CONSTANTS_HPP_
#define SAVO_PERCEPTION__CONSTANTS_HPP_

#include <cstdint>

namespace savo_perception
{
namespace constants
{

inline constexpr const char * kPackageName = "savo_perception";

inline constexpr const char * kVl53MuxNodeName = "vl53_mux_node";
inline constexpr const char * kUltrasonicNodeName = "ultrasonic_node";
inline constexpr const char * kSafetyStopNodeName = "safety_stop_node";
inline constexpr const char * kCmdVelSafetyGateNodeName = "cmd_vel_safety_gate";
inline constexpr const char * kRangeHealthNodeName = "range_health_node";
inline constexpr const char * kSensorDashboardNodeName = "sensor_dashboard_node";

inline constexpr int kI2cBusDefault = 1;
inline constexpr std::uint8_t kTca9548aAddrDefault = 0x70;
inline constexpr std::uint8_t kVl53l1xAddrDefault = 0x29;

inline constexpr int kVl53RightChannelDefault = 2;
inline constexpr int kVl53LeftChannelDefault = 3;
inline constexpr double kVl53RateHzDefault = 10.0;
inline constexpr int kVl53MedianWindowDefault = 5;
inline constexpr double kVl53SettleSDefault = 0.002;
inline constexpr double kVl53InitSettleSDefault = 0.05;
inline constexpr double kVl53ValidMinMDefault = 0.02;
inline constexpr double kVl53ValidMaxMDefault = 4.00;

inline constexpr int kUltrasonicTrigPinDefault = 27;
inline constexpr int kUltrasonicEchoPinDefault = 22;
inline constexpr double kUltrasonicMaxDistanceMDefault = 3.00;
inline constexpr double kUltrasonicValidMinMDefault = 0.02;
inline constexpr double kUltrasonicValidMaxMDefault = 3.00;
inline constexpr double kUltrasonicRateHzDefault = 10.0;

inline constexpr double kSafetyLoopHzDefault = 20.0;
inline constexpr double kCmdVelGateLoopHzDefault = 50.0;
inline constexpr double kRangeHealthPublishHzDefault = 2.0;

inline constexpr double kSensorStaleTimeoutSDefault = 0.30;
inline constexpr double kCmdTimeoutSDefault = 0.50;

inline constexpr double kFrontStopMDefault = 0.25;
inline constexpr double kFrontSlowMDefault = 0.80;
inline constexpr double kSideStopMDefault = 0.08;
inline constexpr double kSideSlowMDefault = 0.25;

inline constexpr double kFrontClearHysteresisMDefault = 0.010;
inline constexpr double kSideClearHysteresisMDefault = 0.010;

inline constexpr int kStopDebounceCountDefault = 2;
inline constexpr int kClearDebounceCountDefault = 4;

inline constexpr double kSlowdownMinDefault = 0.20;
inline constexpr double kSlowdownMaxDefault = 1.00;
inline constexpr double kSlowdownEmaAlphaDefault = 0.35;

inline constexpr bool kFailSafeOnStaleDefault = true;
inline constexpr bool kPublishNanOnErrorDefault = true;
inline constexpr bool kStartupFailIsFatalDefault = false;

inline constexpr double kMaxLinearXMpsDefault = 0.35;
inline constexpr double kMaxLinearYMpsDefault = 0.35;
inline constexpr double kMaxAngularZRadpsDefault = 0.80;

}  // namespace constants
}  // namespace savo_perception

#endif  // SAVO_PERCEPTION__CONSTANTS_HPP_
