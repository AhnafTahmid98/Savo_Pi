#ifndef SAVO_LOCATIONS__TOPIC_NAMES_HPP_
#define SAVO_LOCATIONS__TOPIC_NAMES_HPP_

#include <string_view>

namespace savo_locations::topic_names
{

inline constexpr std::string_view kStatus{
  "/savo_locations/status"};

inline constexpr std::string_view kEvents{
  "/savo_locations/events"};

inline constexpr std::string_view kHeartbeat{
  "/savo_locations/heartbeat"};

inline constexpr std::string_view kSnapshot{
  "/savo_locations/snapshot"};

}  // namespace savo_locations::topic_names

#endif  // SAVO_LOCATIONS__TOPIC_NAMES_HPP_
