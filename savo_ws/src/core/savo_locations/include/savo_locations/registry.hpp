#ifndef SAVO_LOCATIONS__REGISTRY_HPP_
#define SAVO_LOCATIONS__REGISTRY_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <vector>

#include "savo_locations/model.hpp"
#include "savo_locations/validation.hpp"

namespace savo_locations
{

enum class MutationCode : std::uint8_t
{
  kInserted = 0U,
  kUpdated,
  kNotFound,
  kInvalidRecord,
  kStaleRevision,
  kRevisionSequenceError,
  kLocationIdConflict,
  kIdentityConflict,
  kTagConflict,
  kRetired,
};


enum class ResolveCode : std::uint8_t
{
  kResolved = 0U,
  kInvalidQuery,
  kNotFound,
  kAmbiguous,
  kDisabled,
  kRetired,
  kMapMismatch,
};


enum class MatchType : std::uint8_t
{
  kNone = 0U,
  kLocationId,
  kDisplayName,
  kAlias,
};


struct RegistryMutationResult
{
  bool success{false};
  MutationCode code{MutationCode::kInvalidRecord};
  std::string reason;

  std::vector<ValidationIssue> validation_issues;
  std::optional<LocationRecordData> record;
};


struct ResolveOptions
{
  bool enforce_map_context{false};
  MapContext map;
};


struct ResolveResult
{
  bool resolved{false};
  ResolveCode code{ResolveCode::kNotFound};
  std::string reason;
  std::string normalized_query;
  MatchType match_type{MatchType::kNone};

  std::optional<LocationRecordData> record;
  std::vector<std::string> ambiguous_location_ids;
};


[[nodiscard]]
std::string_view to_string(
  MutationCode code) noexcept;

[[nodiscard]]
std::string_view to_string(
  ResolveCode code) noexcept;

[[nodiscard]]
std::string_view to_string(
  MatchType type) noexcept;


class InMemoryRegistry
{
public:
  [[nodiscard]]
  RegistryMutationResult insert(
    LocationRecordData record);

  [[nodiscard]]
  RegistryMutationResult replace(
    LocationRecordData record,
    std::uint64_t expected_current_revision);

  [[nodiscard]]
  RegistryMutationResult set_enabled(
    std::string_view location_id,
    std::uint64_t expected_current_revision,
    bool enabled);

  [[nodiscard]]
  ResolveResult resolve(
    std::string_view query,
    const ResolveOptions & options =
      ResolveOptions{}) const;

  [[nodiscard]]
  std::optional<LocationRecordData> get(
    std::string_view location_id) const;

  [[nodiscard]]
  std::vector<LocationRecordData> list() const;

  [[nodiscard]]
  std::size_t size() const noexcept;

  void clear();

private:
  [[nodiscard]]
  bool has_identity_collision_locked(
    const LocationRecordData & candidate,
    std::string_view excluded_location_id,
    std::string * conflicting_location_id) const;

  [[nodiscard]]
  bool has_tag_collision_locked(
    const LocationRecordData & candidate,
    std::string_view excluded_location_id,
    std::string * conflicting_location_id) const;

  mutable std::shared_mutex mutex_;

  std::map<
    std::string,
    LocationRecordData,
    std::less<>>
  records_;
};

}  // namespace savo_locations

#endif  // SAVO_LOCATIONS__REGISTRY_HPP_
