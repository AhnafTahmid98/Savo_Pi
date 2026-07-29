// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <optional>
#include <string>

#include "builtin_interfaces/msg/time.hpp"

namespace savo_supervisor
{

enum class LocalizationPayloadKind
{
  HEALTH,
  SUMMARY,
  HEARTBEAT
};

struct ParsedLocalizationPayload
{
  bool valid = false;
  bool schema_supported = true;

  std::string state;
  bool ready = false;
  bool degraded = false;
  bool alive = false;

  std::string reason_code;

  std::optional<builtin_interfaces::msg::Time> stamp;

  std::string detail;
};

class LocalizationPayloadParser
{
public:
  explicit LocalizationPayloadParser(
    int expected_schema_version = 1);

  ParsedLocalizationPayload ParseHealth(
    const std::string & payload) const;

  ParsedLocalizationPayload ParseSummary(
    const std::string & payload) const;

  ParsedLocalizationPayload ParseHeartbeat(
    const std::string & payload) const;

private:
  ParsedLocalizationPayload Parse(
    const std::string & payload,
    LocalizationPayloadKind kind) const;

  int expected_schema_version_;
};

}  // namespace savo_supervisor
