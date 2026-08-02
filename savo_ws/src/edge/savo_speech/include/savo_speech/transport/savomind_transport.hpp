// Copyright 2026 Ahnaf Tahmid
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace savo_speech::transport
{

struct Request
{
  std::string request_id;
  std::string session_id;
  std::vector<std::uint8_t> wav;
};

struct Response
{
  std::string request_id;
  std::string transcript;
  std::string reply;
  std::vector<std::uint8_t> tts_wav;
};

struct Limits
{
  std::size_t maximum_request_bytes{25U * 1024U * 1024U};
  std::size_t maximum_response_bytes{25U * 1024U * 1024U};
  std::uint32_t required_sample_rate_hz{16000U};
  std::uint16_t required_channels{1U};
  std::uint16_t required_bits_per_sample{16U};
};

struct Validation
{
  bool accepted{false};
  std::string reason;
};

enum class ExchangeStatus
{
  Success,
  InvalidRequest,
  ConnectionFailure,
  Timeout,
  Canceled,
  MalformedResponse,
  IoFailure,
};

struct ExchangeOptions
{
  std::chrono::milliseconds timeout{5000};
  std::uint32_t maximum_attempts{1U};
};

struct ExchangeResult
{
  ExchangeStatus status{ExchangeStatus::IoFailure};
  Response response;
  std::string detail;
};

class SavoMindTransport
{
public:
  virtual ~SavoMindTransport() = default;
  virtual ExchangeResult exchange(
    const Request & request,
    const ExchangeOptions & options) = 0;
  virtual void cancel(const std::string & request_id) noexcept = 0;
  [[nodiscard]] virtual bool healthy() const noexcept = 0;
};

[[nodiscard]] Validation validate_request(const Request & request, const Limits & limits);
[[nodiscard]] Validation validate_response(
  const Request & request,
  const Response & response,
  const Limits & limits);
[[nodiscard]] ExchangeResult exchange_validated(
  SavoMindTransport & transport,
  const Request & request,
  const Limits & limits = {},
  const ExchangeOptions & options = {});

}  // namespace savo_speech::transport
