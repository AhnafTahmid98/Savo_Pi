// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <optional>
#include <string>

#include "builtin_interfaces/msg/time.hpp"

namespace savo_supervisor
{

struct ParsedCorePayload
{
  bool valid{false};
  std::string state{"UNKNOWN"};
  bool ready{false};
  bool degraded{false};
  bool alive{true};
  std::string reason_code{"payload_not_parsed"};
  std::optional<builtin_interfaces::msg::Time> stamp{};
  std::string detail{};
};

class CorePayloadParser
{
public:
  ParsedCorePayload ParseBaseState(const std::string & payload) const;
  ParsedCorePayload ParseControlStatus(const std::string & payload) const;
  ParsedCorePayload ParsePerceptionHealth(const std::string & payload) const;
  ParsedCorePayload ParsePerceptionSafetyState(const std::string & payload) const;
  ParsedCorePayload ParsePerceptionHeartbeat(const std::string & payload) const;
  ParsedCorePayload ParseLidarState(const std::string & payload) const;
  ParsedCorePayload ParseLidarHeartbeat(const std::string & payload) const;
  ParsedCorePayload ParsePowerStatus(const std::string & payload) const;
  ParsedCorePayload ParsePowerHealth(const std::string & payload) const;
};

}  // namespace savo_supervisor
