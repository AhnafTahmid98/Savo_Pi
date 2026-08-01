// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <string>

#include "savo_supervisor/edge_supervision.hpp"

namespace savo_supervisor
{

class EdgePayloadParser
{
public:
  [[nodiscard]] BridgeObservation ParseBridgeState(
    const std::string & payload) const;

  [[nodiscard]] RealSenseObservation ParseRealSenseStatus(
    const std::string & payload) const;

  [[nodiscard]] VoiceObservation ParseSpeechReadiness(
    const std::string & payload) const;

  [[nodiscard]] VisualOdometryObservation ParseVoHealth(
    const std::string & payload) const;
};

}  // namespace savo_supervisor
