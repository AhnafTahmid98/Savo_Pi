#ifndef SAVO_SPEECH__DRIVERS__ALSA_ERROR_HPP_
#define SAVO_SPEECH__DRIVERS__ALSA_ERROR_HPP_

#include <stdexcept>
#include <string>

namespace savo_speech::drivers
{

class AlsaError final : public std::runtime_error
{
public:
  AlsaError(
    std::string operation,
    const int error_code,
    std::string message)
  : std::runtime_error{
      operation + ": " + message +
      " (ALSA error " + std::to_string(error_code) + ")"},
    operation_{std::move(operation)},
    error_code_{error_code}
  {
  }

  [[nodiscard]] const std::string & operation() const noexcept
  {
    return operation_;
  }

  [[nodiscard]] int error_code() const noexcept
  {
    return error_code_;
  }

private:
  std::string operation_;
  int error_code_;
};

}  // namespace savo_speech::drivers

#endif  // SAVO_SPEECH__DRIVERS__ALSA_ERROR_HPP_
