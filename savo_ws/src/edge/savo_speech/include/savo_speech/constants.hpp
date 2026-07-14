#ifndef SAVO_SPEECH__CONSTANTS_HPP_
#define SAVO_SPEECH__CONSTANTS_HPP_

#include <string_view>

namespace savo_speech::constants
{

inline constexpr std::string_view kNodeName{
  "savo_speech_node"};

inline constexpr std::string_view kDefaultProfile{
  "edge_real_robot_v1"};

inline constexpr std::string_view kDefaultRobotId{
  "robot-savo"};

inline constexpr std::string_view kDefaultHostRole{
  "edge"};

inline constexpr std::string_view kDefaultDeviceId{
  "savo-edge"};

inline constexpr std::string_view kDefaultCaptureDevice{
  "savo_respeaker"};

inline constexpr std::string_view kDefaultPlaybackDevice{
  "savo_respeaker"};

inline constexpr double kDefaultStatusPublishRateHz{2.0};
inline constexpr double kDefaultHeartbeatRateHz{1.0};
inline constexpr double kMaximumTimerRateHz{100.0};

}  // namespace savo_speech::constants

#endif  // SAVO_SPEECH__CONSTANTS_HPP_
