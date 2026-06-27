#ifndef SAVO_PERCEPTION__TOPIC_NAMES_HPP_
#define SAVO_PERCEPTION__TOPIC_NAMES_HPP_

namespace savo_perception
{
namespace topics
{

inline constexpr const char * kCmdVel = "/cmd_vel";
inline constexpr const char * kCmdVelSafe = "/cmd_vel_safe";

inline constexpr const char * kDepthFrontM = "/depth/min_front_m";

inline constexpr const char * kTofLeftM = "/savo_perception/range/left_m";
inline constexpr const char * kTofRightM = "/savo_perception/range/right_m";
inline constexpr const char * kUltrasonicFrontM = "/savo_perception/range/front_ultrasonic_m";

inline constexpr const char * kSafetyStop = "/safety/stop";
inline constexpr const char * kSafetySlowdownFactor = "/safety/slowdown_factor";
inline constexpr const char * kSafetyState = "/savo_perception/safety_state";

inline constexpr const char * kCmdVelGateState = "/savo_perception/cmd_vel_gate_state";

inline constexpr const char * kRangeHealth = "/savo_perception/range_health";
inline constexpr const char * kSensorStatus = "/savo_perception/sensor_status";
inline constexpr const char * kHeartbeat = "/savo_perception/heartbeat";

inline constexpr const char * kDashboard = "/savo_perception/dashboard";
inline constexpr const char * kDashboardText = "/savo_perception/dashboard_text";

inline constexpr const char * kRecoveryCmdVel = "/savo_control/recovery_cmd_vel";

inline constexpr const char * kDiagRunRequest = "/savo_perception/diag/run_request";
inline constexpr const char * kDiagCancelRequest = "/savo_perception/diag/cancel_request";
inline constexpr const char * kDiagState = "/savo_perception/diag/state";
inline constexpr const char * kDiagEvent = "/savo_perception/diag/event";
inline constexpr const char * kDiagBusy = "/savo_perception/diag/busy";

}  // namespace topics
}  // namespace savo_perception

#endif  // SAVO_PERCEPTION__TOPIC_NAMES_HPP_