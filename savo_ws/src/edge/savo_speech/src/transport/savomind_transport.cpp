// Copyright 2026 Ahnaf Tahmid
#include <algorithm>

#include "savo_speech/transport/savomind_transport.hpp"

namespace savo_speech::transport
{
namespace
{

std::uint16_t read_u16(const std::vector<std::uint8_t> & bytes, const std::size_t offset)
{
  return static_cast<std::uint16_t>(bytes[offset]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[offset + 1U]) << 8U);
}

std::uint32_t read_u32(const std::vector<std::uint8_t> & bytes, const std::size_t offset)
{
  return static_cast<std::uint32_t>(bytes[offset]) |
         (static_cast<std::uint32_t>(bytes[offset + 1U]) << 8U) |
         (static_cast<std::uint32_t>(bytes[offset + 2U]) << 16U) |
         (static_cast<std::uint32_t>(bytes[offset + 3U]) << 24U);
}

bool has_tag(const std::vector<std::uint8_t> & bytes, const std::size_t offset, const char * tag)
{
  return std::equal(tag, tag + 4, bytes.begin() + static_cast<std::ptrdiff_t>(offset));
}

Validation validate_wav(const std::vector<std::uint8_t> & wav, const Limits & limits)
{
  if (wav.size() < 44U || !has_tag(wav, 0U, "RIFF") || !has_tag(wav, 8U, "WAVE") ||
    !has_tag(wav, 12U, "fmt "))
  {
    return {false, "invalid_wav_header"};
  }
  if (read_u16(wav, 20U) != 1U) {return {false, "tts_wav_not_pcm"};}
  if (read_u16(wav, 22U) != limits.required_channels) {return {false, "tts_wav_channel_mismatch"};}
  if (read_u32(wav, 24U) != limits.required_sample_rate_hz) {
    return {false, "tts_wav_sample_rate_mismatch"};
  }
  if (read_u16(wav, 34U) != limits.required_bits_per_sample) {
    return {false, "tts_wav_sample_width_mismatch"};
  }
  return {true, "accepted"};
}

}  // namespace

Validation validate_request(const Request & request, const Limits & limits)
{
  if (request.request_id.empty() || request.session_id.empty()) {
    return {false, "request_identity_missing"};
  }
  if (request.wav.empty()) {return {false, "request_audio_empty"};}
  if (request.wav.size() > limits.maximum_request_bytes) {
    return {false, "request_audio_oversized"};
  }
  return validate_wav(request.wav, limits);
}

Validation validate_response(
  const Request & request,
  const Response & response,
  const Limits & limits)
{
  if (response.request_id != request.request_id) {return {false, "wrong_request_id"};}
  if (response.transcript.size() + response.reply.size() + response.tts_wav.size() >
    limits.maximum_response_bytes)
  {
    return {false, "response_oversized"};
  }
  if (response.transcript.empty()) {return {false, "transcript_missing"};}
  if (response.reply.empty()) {return {false, "reply_missing"};}
  if (response.tts_wav.empty()) {return {false, "tts_audio_missing"};}
  return validate_wav(response.tts_wav, limits);
}

ExchangeResult exchange_validated(
  SavoMindTransport & transport,
  const Request & request,
  const Limits & limits,
  const ExchangeOptions & options)
{
  const auto request_validation = validate_request(request, limits);
  if (!request_validation.accepted || options.timeout.count() <= 0 ||
    options.maximum_attempts == 0U)
  {
    return {
      ExchangeStatus::InvalidRequest,
      {},
      request_validation.accepted ? "invalid_exchange_options" : request_validation.reason};
  }

  ExchangeResult result;
  for (std::uint32_t attempt = 0U; attempt < options.maximum_attempts; ++attempt) {
    result = transport.exchange(request, options);
    if (result.status == ExchangeStatus::Success) {
      const auto response_validation = validate_response(request, result.response, limits);
      if (!response_validation.accepted) {
        result.status = ExchangeStatus::MalformedResponse;
        result.detail = response_validation.reason;
      }
      return result;
    }

    const bool retryable = result.status == ExchangeStatus::ConnectionFailure ||
      result.status == ExchangeStatus::Timeout || result.status == ExchangeStatus::IoFailure;
    if (!retryable) {
      return result;
    }
  }
  return result;
}

}  // namespace savo_speech::transport
