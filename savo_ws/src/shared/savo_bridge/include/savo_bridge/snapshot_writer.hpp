// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#ifndef SAVO_BRIDGE__SNAPSHOT_WRITER_HPP_
#define SAVO_BRIDGE__SNAPSHOT_WRITER_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "savo_bridge/topic_observation.hpp"

namespace savo_bridge
{

struct SnapshotDocument
{
  std::uint64_t sequence{0U};
  std::vector<TopicObservation::Snapshot> topics;
};

[[nodiscard]] std::string serialize_snapshot(
  const SnapshotDocument & document);

struct SnapshotWriteResult
{
  std::size_t bytes_written{0U};
  bool replaced_existing{false};
};

class AtomicSnapshotWriter final
{
public:
  explicit AtomicSnapshotWriter(std::filesystem::path target_path);

  [[nodiscard]] SnapshotWriteResult write(
    const SnapshotDocument & document) const;

  [[nodiscard]] const std::filesystem::path &
  target_path() const noexcept;

private:
  [[nodiscard]] static std::filesystem::path validate_target_path(
    std::filesystem::path target_path);

  const std::filesystem::path target_path_;
};

}  // namespace savo_bridge

#endif  // SAVO_BRIDGE__SNAPSHOT_WRITER_HPP_
