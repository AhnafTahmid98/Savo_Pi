// Copyright 2026 Ahnaf Tahmid
#ifndef SAVO_SPEECH__ROS__TOPIC_NAMES_HPP_
#define SAVO_SPEECH__ROS__TOPIC_NAMES_HPP_

#include <string_view>

namespace savo_speech::ros::topics
{

// Runtime and diagnostic topics.
inline constexpr std::string_view kReadiness{
  "/savo_speech/readiness"};

inline constexpr std::string_view kDashboard{
  "/savo_speech/dashboard"};

inline constexpr std::string_view kHeartbeat{
  "/savo_speech/heartbeat"};

inline constexpr std::string_view kDiagnostics{
  "/diagnostics"};

// Production speech lifecycle topics. State, transcript, response and
// playback events are privacy-bounded std_msgs strings; raw audio is never
// published continuously.
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
