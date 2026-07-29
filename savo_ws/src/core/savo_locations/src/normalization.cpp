#include "savo_locations/normalization.hpp"

#include "savo_locations/constants.hpp"

namespace savo_locations
{
namespace
{

bool is_ascii_lower(
  const unsigned char value) noexcept
{
  return value >= static_cast<unsigned char>('a') &&
         value <= static_cast<unsigned char>('z');
}


bool is_ascii_upper(
  const unsigned char value) noexcept
{
  return value >= static_cast<unsigned char>('A') &&
         value <= static_cast<unsigned char>('Z');
}


bool is_ascii_digit(
  const unsigned char value) noexcept
{
  return value >= static_cast<unsigned char>('0') &&
         value <= static_cast<unsigned char>('9');
}


bool is_ascii_alphanumeric(
  const unsigned char value) noexcept
{
  return
    is_ascii_lower(value) ||
    is_ascii_upper(value) ||
    is_ascii_digit(value);
}


char ascii_to_lower(
  const unsigned char value) noexcept
{
  if (!is_ascii_upper(value)) {
    return static_cast<char>(value);
  }

  const auto offset =
    static_cast<unsigned char>(
      value -
      static_cast<unsigned char>('A'));

  return static_cast<char>(
    static_cast<unsigned char>('a') +
    offset);
}


char ascii_to_upper(
  const unsigned char value) noexcept
{
  if (!is_ascii_lower(value)) {
    return static_cast<char>(value);
  }

  const auto offset =
    static_cast<unsigned char>(
      value -
      static_cast<unsigned char>('a'));

  return static_cast<char>(
    static_cast<unsigned char>('A') +
    offset);
}

}  // namespace


bool is_ascii_whitespace(
  const char value) noexcept
{
  switch (value) {
    case ' ':
    case '\t':
    case '\n':
    case '\r':
    case '\f':
    case '\v':
      return true;

    default:
      return false;
  }
}


std::string trim_ascii(
  const std::string_view value)
{
  std::size_t begin = 0U;
  std::size_t end = value.size();

  while (
    begin < end &&
    is_ascii_whitespace(value[begin]))
  {
    ++begin;
  }

  while (
    end > begin &&
    is_ascii_whitespace(value[end - 1U]))
  {
    --end;
  }

  return std::string{
    value.substr(begin, end - begin)};
}


std::string collapse_ascii_whitespace(
  const std::string_view value)
{
  const auto trimmed = trim_ascii(value);

  std::string output;
  output.reserve(trimmed.size());

  bool pending_space = false;

  for (const char character : trimmed) {
    if (is_ascii_whitespace(character)) {
      pending_space = !output.empty();
      continue;
    }

    if (pending_space) {
      output.push_back(' ');
      pending_space = false;
    }

    output.push_back(character);
  }

  return output;
}


std::string normalize_lookup_key(
  const std::string_view value)
{
  const auto trimmed = trim_ascii(value);

  std::string output;
  output.reserve(trimmed.size());

  bool pending_separator = false;

  for (const char character : trimmed) {
    const auto byte =
      static_cast<unsigned char>(character);

    const bool is_non_ascii = byte >= 128U;

    if (
      is_ascii_alphanumeric(byte) ||
      is_non_ascii)
    {
      if (
        pending_separator &&
        !output.empty())
      {
        output.push_back(' ');
      }

      output.push_back(
        ascii_to_lower(byte));

      pending_separator = false;
      continue;
    }

    pending_separator = !output.empty();
  }

  return output;
}


std::string canonicalize_location_id(
  const std::string_view value)
{
  const auto trimmed = trim_ascii(value);

  std::string output;
  output.reserve(trimmed.size());

  bool previous_separator = false;

  for (const char character : trimmed) {
    const auto byte =
      static_cast<unsigned char>(character);

    if (is_ascii_whitespace(character)) {
      if (
        !output.empty() &&
        !previous_separator)
      {
        output.push_back('_');
        previous_separator = true;
      }

      continue;
    }

    output.push_back(
      ascii_to_upper(byte));

    previous_separator =
      character == '_' ||
      character == '-';
  }

  while (
    !output.empty() &&
    output.back() == '_')
  {
    output.pop_back();
  }

  return output;
}


bool is_canonical_location_id(
  const std::string_view value) noexcept
{
  if (
    value.empty() ||
    value.size() >
      kMaximumLocationIdLength)
  {
    return false;
  }

  const auto first =
    static_cast<unsigned char>(
      value.front());

  if (!is_ascii_alphanumeric(first)) {
    return false;
  }

  for (const char character : value) {
    const auto byte =
      static_cast<unsigned char>(character);

    const bool allowed =
      is_ascii_upper(byte) ||
      is_ascii_digit(byte) ||
      character == '_' ||
      character == '-';

    if (!allowed) {
      return false;
    }
  }

  return true;
}

}  // namespace savo_locations
