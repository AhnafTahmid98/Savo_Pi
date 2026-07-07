#ifndef SAVO_POWER__POWER_NODE_PARAMS_HPP_
#define SAVO_POWER__POWER_NODE_PARAMS_HPP_

#include "savo_power/ads7830_driver.hpp"
#include "savo_power/battery_reading.hpp"
#include "savo_power/constants.hpp"
#include "savo_power/power_policy.hpp"

#include <cstdint>
#include <string_view>

namespace savo_power
{

namespace params
{

inline constexpr const char * kI2cBus = "i2c_bus";
inline constexpr const char * kDeviceAddress = "device_address";
inline constexpr const char * kSampleRateHz = "sample_rate_hz";
inline constexpr const char * kPublishRateHz = "publish_rate_hz";

inline constexpr const char * kBatterySource = "battery_source";
inline constexpr const char * kNodeRole = "node_role";

inline constexpr const char * kUpsAddress = "ups_address";

inline constexpr const char * kAds7830Address = "ads7830_address";
inline constexpr const char * kAds7830Channel = "ads7830_channel";
inline constexpr const char * kAds7830PcbVersion = "ads7830_pcb_version";

inline constexpr const char * kCoreUpsExpected = "core_ups_expected";
inline constexpr const char * kEdgeUpsExpected = "edge_ups_expected";
inline constexpr const char * kBaseBatteryExpected = "base_battery_expected";

inline constexpr const char * kUpsLowVoltage = "ups_low_voltage_v";
inline constexpr const char * kUpsCriticalVoltage = "ups_critical_voltage_v";

inline constexpr const char * kBaseEmptyVoltage = "base_empty_voltage_v";
inline constexpr const char * kBaseLowVoltage = "base_low_voltage_v";
inline constexpr const char * kBaseFullVoltage = "base_full_voltage_v";

inline constexpr const char * kBaseLowSoc = "base_low_soc_pct";
inline constexpr const char * kBaseFullSoc = "base_full_soc_pct";

inline constexpr const char * kFullCapacity = "full_capacity_pct";
inline constexpr const char * kStaleTimeoutS = "stale_timeout_s";

inline constexpr const char * kAutomaticShutdownEnabled =
  "automatic_shutdown_enabled";

}  // namespace params

enum class PowerNodeRole
{
  CORE_UPS,
  EDGE_UPS,
  BASE_BATTERY,
  AGGREGATOR,
  HEALTH,
  DASHBOARD,
  UNKNOWN
};

inline constexpr std::string_view to_string(PowerNodeRole role)
{
  switch (role) {
    case PowerNodeRole::CORE_UPS:
      return "core_ups";
    case PowerNodeRole::EDGE_UPS:
      return "edge_ups";
    case PowerNodeRole::BASE_BATTERY:
      return "base_battery";
    case PowerNodeRole::AGGREGATOR:
      return "aggregator";
    case PowerNodeRole::HEALTH:
      return "health";
    case PowerNodeRole::DASHBOARD:
      return "dashboard";
    case PowerNodeRole::UNKNOWN:
      return "unknown";
  }

  return "unknown";
}

inline constexpr BatterySource battery_source_for_role(PowerNodeRole role)
{
  switch (role) {
    case PowerNodeRole::CORE_UPS:
      return BatterySource::CORE_UPS;
    case PowerNodeRole::EDGE_UPS:
      return BatterySource::EDGE_UPS;
    case PowerNodeRole::BASE_BATTERY:
      return BatterySource::BASE_BATTERY;
    case PowerNodeRole::AGGREGATOR:
    case PowerNodeRole::HEALTH:
    case PowerNodeRole::DASHBOARD:
    case PowerNodeRole::UNKNOWN:
      break;
  }

  return BatterySource::UNKNOWN;
}

inline constexpr std::string_view to_string(Ads7830PcbVersion pcb_version)
{
  switch (pcb_version) {
    case Ads7830PcbVersion::V1:
      return "v1";
    case Ads7830PcbVersion::V2:
      return "v2";
  }

  return "v2";
}

inline constexpr Ads7830PcbVersion ads7830_pcb_version_from_string(
  std::string_view value)
{
  if (value == "v1" || value == "V1" || value == "1") {
    return Ads7830PcbVersion::V1;
  }

  return Ads7830PcbVersion::V2;
}

struct UpsNodeParams
{
  int i2c_bus{constants::kI2cBusDefault};
  std::uint8_t device_address{constants::kUpsHatAddrDefault};
  BatterySource source{BatterySource::CORE_UPS};

  double sample_rate_hz{constants::kSampleRateHzDefault};
  double publish_rate_hz{constants::kHealthPublishHzDefault};

  PowerPolicyThresholds thresholds{};
};

struct BaseBatteryNodeParams
{
  int i2c_bus{constants::kI2cBusDefault};
  std::uint8_t device_address{constants::kAds7830AddrDefault};
  std::uint8_t channel{constants::kBaseBatteryAdcChannelDefault};
  Ads7830PcbVersion pcb_version{Ads7830PcbVersion::V2};

  double sample_rate_hz{constants::kSampleRateHzDefault};
  double publish_rate_hz{constants::kHealthPublishHzDefault};

  PowerPolicyThresholds thresholds{};
};

struct PowerAggregatorNodeParams
{
  bool core_ups_expected{true};
  bool edge_ups_expected{true};
  bool base_battery_expected{true};

  double publish_rate_hz{constants::kHealthPublishHzDefault};
  double stale_timeout_s{constants::kStaleTimeoutSDefault};

  PowerPolicyThresholds thresholds{};
};

struct PowerHealthNodeParams
{
  double publish_rate_hz{constants::kHealthPublishHzDefault};
  double stale_timeout_s{constants::kStaleTimeoutSDefault};
};

struct PowerDashboardNodeParams
{
  double publish_rate_hz{constants::kDashboardPublishHzDefault};
  double stale_timeout_s{constants::kStaleTimeoutSDefault};
};

inline UpsNodeParams make_core_ups_node_params()
{
  UpsNodeParams params;
  params.source = BatterySource::CORE_UPS;
  return params;
}

inline UpsNodeParams make_edge_ups_node_params()
{
  UpsNodeParams params;
  params.source = BatterySource::EDGE_UPS;
  return params;
}

inline BaseBatteryNodeParams make_base_battery_node_params()
{
  return BaseBatteryNodeParams{};
}

}  // namespace savo_power

#endif  // SAVO_POWER__POWER_NODE_PARAMS_HPP_
