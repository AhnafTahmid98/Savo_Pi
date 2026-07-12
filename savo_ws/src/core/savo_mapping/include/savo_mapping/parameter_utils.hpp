#pragma once

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

namespace savo_mapping::params
{

bool is_valid_parameter_name(std::string_view name);

template<typename T>
T declare_or_get(
  rclcpp::Node & node,
  const std::string & name,
  const T & default_value)
{
  if (!is_valid_parameter_name(name)) {
    throw std::invalid_argument("parameter name must not be empty or blank");
  }

  if (!node.has_parameter(name)) {
    return node.declare_parameter<T>(name, default_value);
  }

  return node.get_parameter(name).get_value<T>();
}

bool is_finite_number(double value);
bool is_positive_number(double value);
bool is_non_negative_number(double value);
bool is_number_in_closed_range(double value, double minimum, double maximum);

bool is_positive_integer(std::int64_t value);
bool is_non_negative_integer(std::int64_t value);
bool is_integer_in_closed_range(
  std::int64_t value,
  std::int64_t minimum,
  std::int64_t maximum);

double require_finite_parameter(
  std::string_view name,
  double value);

double require_positive_parameter(
  std::string_view name,
  double value);

double require_non_negative_parameter(
  std::string_view name,
  double value);

double require_parameter_in_closed_range(
  std::string_view name,
  double value,
  double minimum,
  double maximum);

std::int64_t require_positive_integer_parameter(
  std::string_view name,
  std::int64_t value);

std::int64_t require_non_negative_integer_parameter(
  std::string_view name,
  std::int64_t value);

std::int64_t require_integer_parameter_in_closed_range(
  std::string_view name,
  std::int64_t value,
  std::int64_t minimum,
  std::int64_t maximum);

}  // namespace savo_mapping::params
