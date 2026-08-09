// Copyright 2026 Ahnaf Tahmid
#pragma once

#include <string>

namespace savo_ui::v2
{

enum class Page
{
  Voice,
  Navigation
};

enum class VoicePhase
{
  Idle,
  Listening,
  Thinking,
  Speaking,
  Error
};

enum class NavigationPhase
{
  Idle,
  Preparing,
  Navigating,
  Paused,
  Arrived,
  Error
};

struct VoiceState
{
  VoicePhase phase{VoicePhase::Idle};
  double input_level{0.0};
  std::string speech_state{};
  std::string playback_state{};
};

struct NavigationState
{
  NavigationPhase phase{NavigationPhase::Idle};
  std::string goal_id{};
  std::string status{};
  std::string result{};
  double distance_remaining_m{-1.0};
  unsigned int recoveries{0U};
};

}  // namespace savo_ui::v2
