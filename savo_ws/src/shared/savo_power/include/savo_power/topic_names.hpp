#ifndef SAVO_POWER__TOPIC_NAMES_HPP_
#define SAVO_POWER__TOPIC_NAMES_HPP_

namespace savo_power
{
namespace topics
{

inline constexpr const char * kCoreUps = "/savo_power/core/ups";
inline constexpr const char * kEdgeUps = "/savo_power/edge/ups";
inline constexpr const char * kBaseBattery = "/savo_power/base/battery";

inline constexpr const char * kStatus = "/savo_power/status";
inline constexpr const char * kHealth = "/savo_power/health";

inline constexpr const char * kDashboard = "/savo_power/dashboard";
inline constexpr const char * kDashboardText = "/savo_power/dashboard_text";

inline constexpr const char * kShutdownRequest = "/savo_power/shutdown_request";

}  // namespace topics
}  // namespace savo_power

#endif  // SAVO_POWER__TOPIC_NAMES_HPP_
