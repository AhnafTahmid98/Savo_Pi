// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef SAVO_LOCATIONS__READ_ONLY_CATALOG_VIEW_HPP_
#define SAVO_LOCATIONS__READ_ONLY_CATALOG_VIEW_HPP_

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "savo_locations/sqlite_repository.hpp"
#include "savo_locations/types.hpp"


namespace savo_locations
{

enum class ReadResolveCode : std::uint8_t
{
  kResolved = 0U,
  kInvalidQuery,
  kNotFound,
  kAmbiguous,
  kDisabled,
  kRetired,
  kMapMismatch,
};


struct ReadResolveRequest
{
  std::string query;

  bool enforce_map_context{false};
  std::string map_id;
  std::uint32_t map_revision{0U};
};


struct ReadResolveResult
{
  ReadResolveCode code{
    ReadResolveCode::kNotFound};

  std::string reason;
  std::string normalized_query;

  ResolveMatchType match_type{
    ResolveMatchType::kNone};

  LocationRecordData location;

  std::vector<std::string>
    ambiguous_location_ids;
};


enum class ReadGetCode : std::uint8_t
{
  kFound = 0U,
  kInvalidId,
  kNotFound,
  kDisabled,
  kRetired,
};


struct ReadGetResult
{
  ReadGetCode code{
    ReadGetCode::kNotFound};

  std::string reason;
  LocationRecordData location;
};


struct ReadListRequest
{
  std::string map_id;
  std::uint32_t map_revision{0U};
  bool enforce_map_context{false};

  std::string semantic_type;
  std::uint8_t state_filter{0U};

  bool enabled_only{false};
};


struct ReadListResult
{
  bool valid{true};
  std::string reason;

  std::vector<LocationRecordData>
    locations;
};


class ReadOnlyCatalogView
{
public:
  ReadOnlyCatalogView() = default;

  explicit ReadOnlyCatalogView(
    CatalogSnapshot snapshot);

  void replace(
    CatalogSnapshot snapshot);

  [[nodiscard]]
  ReadResolveResult resolve(
    const ReadResolveRequest & request) const;

  [[nodiscard]]
  ReadGetResult get(
    std::string_view location_id,
    bool include_disabled,
    bool include_retired) const;

  [[nodiscard]]
  ReadListResult list(
    const ReadListRequest & request) const;

  [[nodiscard]]
  const std::vector<LocationRecordData> &
  locations() const noexcept;

private:
  std::vector<LocationRecordData>
    locations_;
};

}  // namespace savo_locations

#endif  // SAVO_LOCATIONS__READ_ONLY_CATALOG_VIEW_HPP_
