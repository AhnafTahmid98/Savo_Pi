#pragma once

#include <cstdint>
#include <string_view>

namespace savo_mapping::workflow
{

namespace lifecycle_state_id
{

inline constexpr std::uint8_t UNKNOWN{0};
inline constexpr std::uint8_t UNCONFIGURED{1};
inline constexpr std::uint8_t INACTIVE{2};
inline constexpr std::uint8_t ACTIVE{3};
inline constexpr std::uint8_t FINALIZED{4};

inline constexpr std::uint8_t CONFIGURING{10};
inline constexpr std::uint8_t CLEANING_UP{11};
inline constexpr std::uint8_t SHUTTING_DOWN{12};
inline constexpr std::uint8_t ACTIVATING{13};
inline constexpr std::uint8_t DEACTIVATING{14};
inline constexpr std::uint8_t ERROR_PROCESSING{15};

}  // namespace lifecycle_state_id

enum class ManualWorkflowState
{
  STARTING,
  WAITING_FOR_INPUTS,
  MAPPING,
  PAUSED,
  ERROR,
  COMPLETED
};

struct SlamLifecycleObservation
{
  bool service_available{false};
  bool response_received{false};
  bool response_fresh{false};
  bool startup_grace_expired{false};
  bool ever_active{false};

  std::uint8_t state_id{
    lifecycle_state_id::UNKNOWN};
};

std::string_view to_string(
  ManualWorkflowState state);

ManualWorkflowState evaluate_manual_workflow_state(
  const SlamLifecycleObservation & lifecycle,
  bool mapping_ready,
  std::string_view session_state);

bool lifecycle_state_is_active(
  std::uint8_t state_id);

bool lifecycle_state_is_terminal_error(
  std::uint8_t state_id);

}  // namespace savo_mapping::workflow
