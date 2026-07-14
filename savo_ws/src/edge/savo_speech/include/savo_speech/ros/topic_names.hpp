#ifndef SAVO_SPEECH__ROS__TOPIC_NAMES_HPP_
#define SAVO_SPEECH__ROS__TOPIC_NAMES_HPP_

#include <string_view>

namespace savo_speech::ros::topics
{

// Phase 1 runtime and diagnostic topics.
inline constexpr std::string_view kReadiness{
  "/savo_speech/readiness"};

inline constexpr std::string_view kDashboard{
  "/savo_speech/dashboard"};

inline constexpr std::string_view kHeartbeat{
  "/savo_speech/heartbeat"};

inline constexpr std::string_view kDiagnostics{
  "/diagnostics"};

// Reserved production speech topics.
// These constants are locked now, but publishers will be introduced only
// after the corresponding savo_msgs interfaces are finalized.
inline constexpr std::string_view kState{
  "/savo_speech/state"};

inline constexpr std::string_view kHealth{
  "/savo_speech/health"};

inline constexpr std::string_view kResult{
  "/savo_speech/result"};

inline constexpr std::string_view kTranscript{
  "/savo_speech/transcript"};

inline constexpr std::string_view kResponse{
  "/savo_speech/response"};

inline constexpr std::string_view kPlaybackState{
  "/savo_speech/playback/state"};

inline constexpr std::string_view kPlaybackFinished{
  "/savo_speech/playback/finished"};

}  // namespace savo_speech::ros::topics

#endif  // SAVO_SPEECH__ROS__TOPIC_NAMES_HPP_
