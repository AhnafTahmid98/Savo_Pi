// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef SAVO_LOCALIZATION__PIPELINE_TIMING_HPP_
#define SAVO_LOCALIZATION__PIPELINE_TIMING_HPP_

#include <algorithm>
#include <cstdint>
#include <ostream>

namespace savo_localization
{

// Observational only: fixed storage, no logging, no readiness decisions.
struct DurationTiming
{
  double last_wall_s{0.0};
  double max_wall_s{0.0};
  double last_cpu_s{-1.0};
  double max_cpu_s{-1.0};

  void Record(const std::int64_t wall_ns, const std::int64_t cpu_ns) noexcept
  {
    last_wall_s = static_cast<double>(wall_ns) / 1.0e9;
    max_wall_s = std::max(max_wall_s, last_wall_s);
    last_cpu_s = cpu_ns >= 0 ? static_cast<double>(cpu_ns) / 1.0e9 : -1.0;
    max_cpu_s = std::max(max_cpu_s, last_cpu_s);
  }

  void WriteJson(std::ostream & output) const
  {
    output << "{\"wall_s\":" << last_wall_s << ",\"max_wall_s\":" << max_wall_s
           << ",\"cpu_s\":";
    if (last_cpu_s >= 0.0) {
      output << last_cpu_s;
    } else {
      output << "null";
    }
    output << ",\"max_cpu_s\":";
    if (max_cpu_s >= 0.0) {
      output << max_cpu_s;
    } else {
      output << "null";
    }
    output << '}';
  }
};

struct CallbackTiming
{
  std::uint64_t count{0};
  std::int64_t last_start_ns{-1};
  double gap_s{-1.0};
  double max_gap_s{0.0};
  std::int64_t source_timestamp_ns{0};
  std::int64_t received_timestamp_ns{0};
  double source_to_callback_s{0.0};
  double receive_to_callback_s{0.0};
  bool source_timing_available{false};
  bool receive_timing_available{false};
  double max_source_to_callback_s{0.0};
  double max_receive_to_callback_s{0.0};
  std::int64_t header_stamp_ns{0};
  double header_age_at_callback_s{0.0};
  DurationTiming work;

  void RecordHeader(const std::int64_t ros_now_ns, const std::int64_t stamp_ns) noexcept
  {
    header_stamp_ns = stamp_ns;
    header_age_at_callback_s = static_cast<double>(ros_now_ns - stamp_ns) / 1.0e9;
  }

  // DDS timestamps are system-clock values, NOT ROS or monotonic time. Zero
  // means unavailable. Preserve negative deltas so clock skew is visible.
  void Start(
    const std::int64_t steady_ns, const std::int64_t system_ns,
    const std::int64_t source_ns, const std::int64_t received_ns) noexcept
  {
    if (last_start_ns >= 0) {
      gap_s = static_cast<double>(steady_ns - last_start_ns) / 1.0e9;
      max_gap_s = std::max(max_gap_s, gap_s);
    }
    ++count;
    last_start_ns = steady_ns;
    source_timestamp_ns = source_ns;
    received_timestamp_ns = received_ns;
    source_to_callback_s = static_cast<double>(system_ns - source_ns) / 1.0e9;
    receive_to_callback_s = static_cast<double>(system_ns - received_ns) / 1.0e9;
    if (source_ns > 0) {
      max_source_to_callback_s = source_timing_available ?
        std::max(max_source_to_callback_s, source_to_callback_s) : source_to_callback_s;
      source_timing_available = true;
    }
    if (received_ns > 0) {
      max_receive_to_callback_s = receive_timing_available ?
        std::max(max_receive_to_callback_s, receive_to_callback_s) : receive_to_callback_s;
      receive_timing_available = true;
    }
  }

  void WriteJson(std::ostream & output) const
  {
    output << "{\"count\":" << count << ",\"last_start_monotonic_ns\":" << last_start_ns
           << ",\"gap_s\":" << gap_s << ",\"max_gap_s\":" << max_gap_s
           << ",\"dds_source_timestamp_ns\":" << source_timestamp_ns
           << ",\"dds_received_timestamp_ns\":" << received_timestamp_ns
           << ",\"source_to_callback_s\":";
    if (source_timestamp_ns > 0) {
      output << source_to_callback_s;
    } else {
      output << "null";
    }
    output << ",\"receive_to_callback_s\":";
    if (received_timestamp_ns > 0) {
      output << receive_to_callback_s;
    } else {
      output << "null";
    }
    output << ",\"max_source_to_callback_s\":";
    if (source_timing_available) {
      output << max_source_to_callback_s;
    } else {
      output << "null";
    }
    output << ",\"max_receive_to_callback_s\":";
    if (receive_timing_available) {
      output << max_receive_to_callback_s;
    } else {
      output << "null";
    }
    output << ",\"header_stamp_ns\":" << header_stamp_ns
           << ",\"header_age_at_callback_s\":";
    if (header_stamp_ns > 0) {
      output << header_age_at_callback_s;
    } else {
      output << "null";
    }
    output << ",\"work\":";
    work.WriteJson(output);
    output << '}';
  }
};

}  // namespace savo_localization

#endif  // SAVO_LOCALIZATION__PIPELINE_TIMING_HPP_
