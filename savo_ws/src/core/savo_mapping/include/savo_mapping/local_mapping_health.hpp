// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary
#pragma once

#include <chrono>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace savo_mapping::local_health
{
using Clock = std::chrono::steady_clock;
struct Source
{
  std::string name;
  std::string topic;
  double timeout_s;
  bool required{true};
};
struct Decision
{
  bool admission_ready{false};
  bool continuation_ready{false};
  bool semantic_ready{false};
  std::string reason{"mapping_local_startup"};
  std::string json() const;
};
class Monitor
{
public:
  explicit Monitor(bool edge_required = false);
  const std::vector<Source> & sources() const {return sources_;}
  void observe(const std::string & source, const std::string & payload, Clock::time_point now);
  Decision evaluate(bool mapping_ready, Clock::time_point now) const;

private:
  struct Observation
  {
    std::optional<Clock::time_point> receipt;
    std::optional<double> stamp;
    bool valid{false};
    bool admission{false};
    bool continuation{false};
    bool degraded{false};
    std::string state;
    std::string reason;
  };
  std::vector<Source> sources_;
  std::map<std::string, Observation> observations_;
};
}  // namespace savo_mapping::local_health
