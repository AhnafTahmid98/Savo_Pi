// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include "savo_bridge/snapshot_writer.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "savo_bridge/health_evaluator.hpp"

namespace savo_bridge
{
namespace
{

class FileDescriptor final
{
public:
  explicit FileDescriptor(const int descriptor) noexcept
  : descriptor_(descriptor)
  {
  }

  ~FileDescriptor()
  {
    if (descriptor_ >= 0) {
      (void)::close(descriptor_);
    }
  }

  FileDescriptor(const FileDescriptor &) = delete;
  FileDescriptor & operator=(const FileDescriptor &) = delete;

  FileDescriptor(FileDescriptor &&) = delete;
  FileDescriptor & operator=(FileDescriptor &&) = delete;

  [[nodiscard]] int get() const noexcept
  {
    return descriptor_;
  }

  void close_checked()
  {
    if (descriptor_ < 0) {
      return;
    }

    const int descriptor = descriptor_;
    descriptor_ = -1;

    if (::close(descriptor) != 0 && errno != EINTR) {
      throw std::system_error(
              errno,
              std::generic_category(),
              "Failed to close snapshot temporary file");
    }
  }

private:
  int descriptor_{-1};
};

class TemporaryFileGuard final
{
public:
  TemporaryFileGuard(
    const int directory_descriptor,
    std::string temporary_name)
  : directory_descriptor_(directory_descriptor),
    temporary_name_(std::move(temporary_name))
  {
  }

  ~TemporaryFileGuard()
  {
    if (active_) {
      (void)::unlinkat(
        directory_descriptor_,
        temporary_name_.c_str(),
        0);
    }
  }

  TemporaryFileGuard(const TemporaryFileGuard &) = delete;
  TemporaryFileGuard & operator=(const TemporaryFileGuard &) = delete;

  TemporaryFileGuard(TemporaryFileGuard &&) = delete;
  TemporaryFileGuard & operator=(TemporaryFileGuard &&) = delete;

  void release() noexcept
  {
    active_ = false;
  }

private:
  int directory_descriptor_{-1};
  std::string temporary_name_;
  bool active_{true};
};

struct TemporaryFile
{
  int descriptor{-1};
  std::string name;
};

[[noreturn]] void throw_system_error(
  const int error_number,
  const std::string & message)
{
  throw std::system_error(
          error_number,
          std::generic_category(),
          message);
}

void append_json_string(
  std::string & output,
  const std::string_view value)
{
  constexpr char hexadecimal[] = "0123456789abcdef";

  output.push_back('"');

  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        output += "\\\"";
        break;
      case '\\':
        output += "\\\\";
        break;
      case '\b':
        output += "\\b";
        break;
      case '\f':
        output += "\\f";
        break;
      case '\n':
        output += "\\n";
        break;
      case '\r':
        output += "\\r";
        break;
      case '\t':
        output += "\\t";
        break;
      default:
        if (character < 0x20U) {
          output += "\\u00";
          output.push_back(hexadecimal[(character >> 4U) & 0x0FU]);
          output.push_back(hexadecimal[character & 0x0FU]);
        } else {
          output.push_back(static_cast<char>(character));
        }
        break;
    }
  }

  output.push_back('"');
}

void append_boolean(
  std::string & output,
  const bool value)
{
  output += value ? "true" : "false";
}

template<typename Integer>
void append_integer(
  std::string & output,
  const Integer value)
{
  output += std::to_string(value);
}

void append_string_array(
  std::string & output,
  const std::vector<std::string> & values)
{
  output.push_back('[');

  for (std::size_t index = 0U; index < values.size(); ++index) {
    if (index != 0U) {
      output.push_back(',');
    }

    append_json_string(output, values.at(index));
  }

  output.push_back(']');
}

[[nodiscard]] std::int64_t duration_nanoseconds(
  const TopicObservation::Duration duration)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    duration).count();
}

void append_health(
  std::string & output,
  const HealthEvaluation & health)
{
  output += "\"health\":{";

  output += "\"status\":";
  append_json_string(output, to_string(health.status));

  output += ",\"reason\":";
  append_json_string(output, to_string(health.reason));

  output += ",\"required_topics_ready\":";
  append_boolean(output, health.required_topics_ready);

  output += ",\"all_topics_fresh\":";
  append_boolean(output, health.all_topics_fresh);

  output += ",\"total_topics\":";
  append_integer(output, health.total_topics);

  output += ",\"required_topics\":";
  append_integer(output, health.required_topics);

  output += ",\"optional_topics\":";
  append_integer(output, health.optional_topics);

  output += ",\"fresh_topics\":";
  append_integer(output, health.fresh_topics);

  output += ",\"stale_topics\":";
  append_integer(output, health.stale_topics);

  output += ",\"never_observed_topics\":";
  append_integer(output, health.never_observed_topics);

  output += ",\"clock_regression_topics\":";
  append_integer(output, health.clock_regression_topics);

  output += ",\"required_unavailable_topics\":";
  append_string_array(
    output,
    health.required_unavailable_topics);

  output += ",\"optional_unavailable_topics\":";
  append_string_array(
    output,
    health.optional_unavailable_topics);

  output += ",\"clock_regression_topic_names\":";
  append_string_array(
    output,
    health.clock_regression_topic_names);

  output.push_back('}');
}

void append_topic(
  std::string & output,
  const TopicObservation::Snapshot & topic)
{
  output.push_back('{');

  output += "\"topic_name\":";
  append_json_string(output, topic.topic_name);

  output += ",\"requirement\":";
  append_json_string(output, to_string(topic.requirement));

  output += ",\"classification\":";
  append_json_string(output, to_string(topic.classification));

  output += ",\"observed\":";
  append_boolean(output, topic.observed);

  output += ",\"accepted_observations\":";
  append_integer(output, topic.accepted_observations);

  output += ",\"rejected_regressions\":";
  append_integer(output, topic.rejected_regressions);

  output += ",\"age_ns\":";
  append_integer(output, duration_nanoseconds(topic.age));

  output += ",\"stale_after_ns\":";
  append_integer(
    output,
    duration_nanoseconds(topic.stale_after));

  output.push_back('}');
}

[[nodiscard]] TemporaryFile create_temporary_file(
  const int directory_descriptor,
  const std::string & target_filename)
{
  static std::atomic<std::uint64_t> counter{0U};

  constexpr std::size_t maximum_attempts = 256U;

  for (std::size_t attempt = 0U;
    attempt < maximum_attempts;
    ++attempt)
  {
    const std::uint64_t identifier =
      counter.fetch_add(1U, std::memory_order_relaxed);

    std::string temporary_name;
    temporary_name.reserve(target_filename.size() + 48U);

    temporary_name += ".";
    temporary_name += target_filename;
    temporary_name += ".tmp.";
    temporary_name += std::to_string(
      static_cast<std::uint64_t>(::getpid()));
    temporary_name += ".";
    temporary_name += std::to_string(identifier);

    const int descriptor = ::openat(
      directory_descriptor,
      temporary_name.c_str(),
      O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC,
      S_IRUSR | S_IWUSR | S_IRGRP);

    if (descriptor >= 0) {
      return TemporaryFile{
        descriptor,
        std::move(temporary_name)};
    }

    if (errno != EEXIST) {
      throw_system_error(
        errno,
        "Failed to create snapshot temporary file");
    }
  }

  throw std::runtime_error(
          "Unable to allocate a unique snapshot temporary file");
}

void write_all(
  const int descriptor,
  const std::string & payload)
{
  std::size_t offset = 0U;

  while (offset < payload.size()) {
    const std::size_t remaining = payload.size() - offset;

    const ssize_t written = ::write(
      descriptor,
      payload.data() + offset,
      remaining);

    if (written > 0) {
      offset += static_cast<std::size_t>(written);
      continue;
    }

    if (written < 0 && errno == EINTR) {
      continue;
    }

    if (written == 0) {
      throw std::runtime_error(
              "Snapshot temporary-file write made no progress");
    }

    throw_system_error(
      errno,
      "Failed to write snapshot temporary file");
  }
}

[[nodiscard]] bool target_exists_as_regular_file(
  const int directory_descriptor,
  const std::string & target_filename)
{
  struct stat target_status {};

  if (::fstatat(
      directory_descriptor,
      target_filename.c_str(),
      &target_status,
      AT_SYMLINK_NOFOLLOW) == 0)
  {
    if (!S_ISREG(target_status.st_mode)) {
      throw std::invalid_argument(
              "Snapshot target must be a regular file or absent");
    }

    return true;
  }

  if (errno == ENOENT) {
    return false;
  }

  throw_system_error(
    errno,
    "Failed to inspect snapshot target");

  return false;
}

}  // namespace

std::string serialize_snapshot(
  const SnapshotDocument & document)
{
  std::vector<TopicObservation::Snapshot> topics = document.topics;

  std::sort(
    topics.begin(),
    topics.end(),
    [](const auto & left, const auto & right) {
      return left.topic_name < right.topic_name;
    });

  const HealthEvaluator evaluator;
  const HealthEvaluation health = evaluator.evaluate(topics);

  std::string output;
  output.reserve(512U + (topics.size() * 256U));

  output += "{\"schema_name\":\"savo_bridge_snapshot\"";
  output += ",\"schema_version\":1";
  output += ",\"snapshot_sequence\":";
  append_integer(output, document.sequence);
  output.push_back(',');
  append_health(output, health);
  output += ",\"topics\":[";

  for (std::size_t index = 0U; index < topics.size(); ++index) {
    if (index != 0U) {
      output.push_back(',');
    }

    append_topic(output, topics.at(index));
  }

  output += "]}\n";

  return output;
}

AtomicSnapshotWriter::AtomicSnapshotWriter(
  std::filesystem::path target_path)
: target_path_(validate_target_path(std::move(target_path)))
{
}

SnapshotWriteResult AtomicSnapshotWriter::write(
  const SnapshotDocument & document) const
{
  const std::string payload = serialize_snapshot(document);

  std::filesystem::path parent_path = target_path_.parent_path();

  if (parent_path.empty()) {
    parent_path = ".";
  }

  const std::string target_filename =
    target_path_.filename().string();

  const int raw_directory_descriptor = ::open(
    parent_path.c_str(),
    O_RDONLY | O_DIRECTORY | O_CLOEXEC);

  if (raw_directory_descriptor < 0) {
    throw_system_error(
      errno,
      "Failed to open snapshot parent directory");
  }

  FileDescriptor directory_descriptor(
    raw_directory_descriptor);

  const bool replaced_existing =
    target_exists_as_regular_file(
    directory_descriptor.get(),
    target_filename);

  TemporaryFile temporary = create_temporary_file(
    directory_descriptor.get(),
    target_filename);

  FileDescriptor temporary_descriptor(temporary.descriptor);

  TemporaryFileGuard temporary_guard(
    directory_descriptor.get(),
    temporary.name);

  write_all(temporary_descriptor.get(), payload);

  if (::fsync(temporary_descriptor.get()) != 0) {
    throw_system_error(
      errno,
      "Failed to synchronize snapshot temporary file");
  }

  temporary_descriptor.close_checked();

  if (::renameat(
      directory_descriptor.get(),
      temporary.name.c_str(),
      directory_descriptor.get(),
      target_filename.c_str()) != 0)
  {
    throw_system_error(
      errno,
      "Failed to atomically publish snapshot");
  }

  temporary_guard.release();

  if (::fsync(directory_descriptor.get()) != 0) {
    throw_system_error(
      errno,
      "Snapshot committed but directory synchronization failed");
  }

  SnapshotWriteResult result;
  result.bytes_written = payload.size();
  result.replaced_existing = replaced_existing;
  return result;
}

const std::filesystem::path &
AtomicSnapshotWriter::target_path() const noexcept
{
  return target_path_;
}

std::filesystem::path AtomicSnapshotWriter::validate_target_path(
  std::filesystem::path target_path)
{
  target_path = target_path.lexically_normal();

  if (target_path.empty() || !target_path.has_filename()) {
    throw std::invalid_argument(
            "Snapshot target path must name a file");
  }

  const std::filesystem::path filename =
    target_path.filename();

  if (filename == "." || filename == "..") {
    throw std::invalid_argument(
            "Snapshot target path must name a file");
  }

  return target_path;
}

}  // namespace savo_bridge
