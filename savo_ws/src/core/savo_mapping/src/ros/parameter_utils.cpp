#include "savo_mapping/parameter_utils.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace savo_mapping::params
{

namespace
{

void require_valid_name(std::string_view name)
{
  if (!is_valid_parameter_name(name)) {
    throw std::invalid_argument("parameter name must not be empty or blank");
  }
}

std::string make_range_message(
  std::string_view name,
  double value,
  double minimum,
  double maximum)
{
  std::ostringstream out;
  out << "parameter '" << name << "' value " << value
      << " is outside [" << minimum << ", " << maximum << "]";
  return out.str();
}

std::string make_integer_range_message(
  std::string_view name,
  std::int64_t value,
  std::int64_t minimum,
  std::int64_t maximum)
{
  std::ostringstream out;
  out << "parameter '" << name << "' value " << value
      << " is outside [" << minimum << ", " << maximum << "]";
  return out.str();
}

}  // namespace

bool is_valid_parameter_name(std::string_view name)
{
  if (name.empty()) {
    return false;
  }

  return std::all_of(
    name.begin(),
    name.end(),
    [](const char ch) {
      return std::isspace(static_cast<unsigned char>(ch)) == 0;
    });
}

bool is_finite_number(double value)
{
  return std::isfinite(value);
}

bool is_positive_number(double value)
{
  return is_finite_number(value) && value > 0.0;
}

bool is_non_negative_number(double value)
{
  return is_finite_number(value) && value >= 0.0;
}

bool is_number_in_closed_range(double value, double minimum, double maximum)
{
  return is_finite_number(value) &&
         is_finite_number(minimum) &&
         is_finite_number(maximum) &&
         minimum <= maximum &&
         value >= minimum &&
         value <= maximum;
}

bool is_positive_integer(std::int64_t value)
{
  return value > 0;
}

bool is_non_negative_integer(std::int64_t value)
{
  return value >= 0;
}

bool is_integer_in_closed_range(
  std::int64_t value,
  std::int64_t minimum,
  std::int64_t maximum)
{
  return minimum <= maximum &&
         value >= minimum &&
         value <= maximum;
}

double require_finite_parameter(
  std::string_view name,
  double value)
{
  require_valid_name(name);

  if (!is_finite_number(value)) {
    throw std::invalid_argument(
            "parameter '" + std::string{name} + "' must be finite");
  }

  return value;
}

double require_positive_parameter(
  std::string_view name,
  double value)
{
  require_finite_parameter(name, value);

  if (value <= 0.0) {
    throw std::out_of_range(
            "parameter '" + std::string{name} + "' must be greater than zero");
  }

  return value;
}

double require_non_negative_parameter(
  std::string_view name,
  double value)
{
  require_finite_parameter(name, value);

  if (value < 0.0) {
    throw std::out_of_range(
            "parameter '" + std::string{name} + "' must not be negative");
  }

  return value;
}

double require_parameter_in_closed_range(
  std::string_view name,
  double value,
  double minimum,
  double maximum)
{
  require_valid_name(name);

  if (!is_finite_number(value) ||
      !is_finite_number(minimum) ||
      !is_finite_number(maximum))
  {
    throw std::invalid_argument(
            "parameter value and range bounds must be finite");
  }

  if (minimum > maximum) {
    throw std::invalid_argument(
            "parameter minimum must not be greater than maximum");
  }

  if (value < minimum || value > maximum) {
    throw std::out_of_range(
            make_range_message(name, value, minimum, maximum));
  }

  return value;
}

std::int64_t require_positive_integer_parameter(
  std::string_view name,
  std::int64_t value)
{
  require_valid_name(name);

  if (!is_positive_integer(value)) {
    throw std::out_of_range(
            "parameter '" + std::string{name} + "' must be greater than zero");
  }

  return value;
}

std::int64_t require_non_negative_integer_parameter(
  std::string_view name,
  std::int64_t value)
{
  require_valid_name(name);

  if (!is_non_negative_integer(value)) {
    throw std::out_of_range(
            "parameter '" + std::string{name} + "' must not be negative");
  }

  return value;
}

std::int64_t require_integer_parameter_in_closed_range(
  std::string_view name,
  std::int64_t value,
  std::int64_t minimum,
  std::int64_t maximum)
{
  require_valid_name(name);

  if (minimum > maximum) {
    throw std::invalid_argument(
            "parameter minimum must not be greater than maximum");
  }

  if (value < minimum || value > maximum) {
    throw std::out_of_range(
            make_integer_range_message(name, value, minimum, maximum));
  }

  return value;
}

}  // namespace savo_mapping::params
