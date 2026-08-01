// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <string>

namespace savo_supervisor
{

struct PersistentSystemState
{
  bool valid{false};
  bool fault_latched{false};
  std::uint64_t generation{0U};
  std::string reason;
  std::string error;
};

class SystemStateStore
{
public:
  explicit SystemStateStore(std::string path = {});

  [[nodiscard]] bool enabled() const noexcept;
  [[nodiscard]] const std::string & path() const noexcept;
  [[nodiscard]] PersistentSystemState Load() const;
  [[nodiscard]] bool Save(
    bool fault_latched,
    std::uint64_t generation,
    const std::string & reason,
    std::string & error) const;

private:
  std::string path_;
};

}  // namespace savo_supervisor
