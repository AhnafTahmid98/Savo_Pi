// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "savo_supervisor/mission_authority.hpp"

namespace savo_supervisor
{

class MissionPayloadParser
{
public:
  [[nodiscard]] MappingObservation ParseMappingStatus(
    const std::string & payload) const;

  [[nodiscard]] NavigationObservation ParseNavigationStatus(
    const std::string & payload) const;

  [[nodiscard]] LocationsObservation ParseLocationsStatus(
    const std::string & payload) const;

  [[nodiscard]] HeadObservation ParseHeadStatus(
    const diagnostic_msgs::msg::DiagnosticArray & message) const;
};

}  // namespace savo_supervisor
