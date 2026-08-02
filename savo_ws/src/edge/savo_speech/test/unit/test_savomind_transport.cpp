// Copyright 2026 Ahnaf Tahmid
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <deque>
#include <string>
#include <utility>
#include <vector>

#include "savo_speech/transport/savomind_transport.hpp"

namespace
{

std::vector<std::uint8_t> wav()
{
  std::vector<std::uint8_t> value(44U, 0U);
  value[0] = 'R'; value[1] = 'I'; value[2] = 'F'; value[3] = 'F';
  value[8] = 'W'; value[9] = 'A'; value[10] = 'V'; value[11] = 'E';
  value[12] = 'f'; value[13] = 'm'; value[14] = 't'; value[15] = ' ';
  value[20] = 1U; value[22] = 1U;
  value[24] = 0x80U; value[25] = 0x3eU;
  value[34] = 16U;
  return value;
}

savo_speech::transport::Response valid_response()
{
  return {"request-1", "hello", "hi", wav()};
}

class FakeSavoMindTransport final : public savo_speech::transport::SavoMindTransport
{
public:
  savo_speech::transport::ExchangeResult exchange(
    const savo_speech::transport::Request & request,
    const savo_speech::transport::ExchangeOptions & options) override
  {
    ++exchange_calls;
    last_request_id = request.request_id;
    last_timeout = options.timeout;
    if (script.empty()) {
      return {savo_speech::transport::ExchangeStatus::ConnectionFailure, {}, "unavailable"};
    }
    auto result = std::move(script.front());
    script.pop_front();
    connected = result.status == savo_speech::transport::ExchangeStatus::Success;
    return result;
  }

  void cancel(const std::string & request_id) noexcept override
  {
    canceled_request_id = request_id;
  }

  [[nodiscard]] bool healthy() const noexcept override {return connected;}

  std::deque<savo_speech::transport::ExchangeResult> script;
  std::uint32_t exchange_calls{0U};
  std::string last_request_id;
  std::string canceled_request_id;
  std::chrono::milliseconds last_timeout{0};
  bool connected{false};
};

}  // namespace

TEST(SavoMindTransport, AcceptsCorrelatedBoundedPcmWav)
{
  const savo_speech::transport::Request request{"request-1", "session-1", wav()};
  const savo_speech::transport::Response response{"request-1", "hello", "hi", wav()};
  EXPECT_TRUE(savo_speech::transport::validate_request(request, {}).accepted);
  EXPECT_TRUE(savo_speech::transport::validate_response(request, response, {}).accepted);
}

TEST(SavoMindTransport, RejectsWrongCorrelationAndInvalidAudio)
{
  const savo_speech::transport::Request request{"request-1", "session-1", wav()};
  auto response = savo_speech::transport::Response{"request-2", "hello", "hi", wav()};
  EXPECT_EQ(savo_speech::transport::validate_response(request, response, {}).reason,
    "wrong_request_id");
  response.request_id = request.request_id;
  response.tts_wav = {1U, 2U, 3U};
  EXPECT_EQ(savo_speech::transport::validate_response(request, response, {}).reason,
    "invalid_wav_header");
}

TEST(SavoMindTransport, RejectsMissingTtsAndOversizedResponse)
{
  const savo_speech::transport::Request request{"request-1", "session-1", wav()};
  savo_speech::transport::Response response{"request-1", "hello", "hi", {}};
  EXPECT_EQ(savo_speech::transport::validate_response(request, response, {}).reason,
    "tts_audio_missing");
  savo_speech::transport::Limits limits;
  limits.maximum_response_bytes = 4U;
  response.tts_wav = wav();
  EXPECT_EQ(savo_speech::transport::validate_response(request, response, limits).reason,
    "response_oversized");
}

TEST(SavoMindTransport, FakeServerCompletesCorrelatedExchange)
{
  FakeSavoMindTransport transport;
  transport.script.push_back(
    {savo_speech::transport::ExchangeStatus::Success, valid_response(), "ok"});
  const savo_speech::transport::Request request{"request-1", "session-1", wav()};
  savo_speech::transport::ExchangeOptions options;
  options.timeout = std::chrono::milliseconds{250};

  const auto result = savo_speech::transport::exchange_validated(
    transport, request, {}, options);

  EXPECT_EQ(result.status, savo_speech::transport::ExchangeStatus::Success);
  EXPECT_TRUE(transport.healthy());
  EXPECT_EQ(transport.last_request_id, request.request_id);
  EXPECT_EQ(transport.last_timeout, options.timeout);
}

TEST(SavoMindTransport, FakeServerReportsConnectionFailureAndTimeout)
{
  FakeSavoMindTransport transport;
  const savo_speech::transport::Request request{"request-1", "session-1", wav()};
  transport.script.push_back(
    {savo_speech::transport::ExchangeStatus::ConnectionFailure, {}, "refused"});
  EXPECT_EQ(
    savo_speech::transport::exchange_validated(transport, request).status,
    savo_speech::transport::ExchangeStatus::ConnectionFailure);

  transport.script.push_back(
    {savo_speech::transport::ExchangeStatus::Timeout, {}, "deadline"});
  EXPECT_EQ(
    savo_speech::transport::exchange_validated(transport, request).status,
    savo_speech::transport::ExchangeStatus::Timeout);
}

TEST(SavoMindTransport, FakeServerRejectsMalformedOversizedInvalidAndWrongIdResponses)
{
  const savo_speech::transport::Request request{"request-1", "session-1", wav()};
  const auto expect_reason = [&](savo_speech::transport::Response response,
    const std::string & reason, const savo_speech::transport::Limits & limits = {}) {
      FakeSavoMindTransport transport;
      transport.script.push_back(
        {savo_speech::transport::ExchangeStatus::Success, std::move(response), "ok"});
      const auto result = savo_speech::transport::exchange_validated(
        transport, request, limits);
      EXPECT_EQ(result.status, savo_speech::transport::ExchangeStatus::MalformedResponse);
      EXPECT_EQ(result.detail, reason);
    };

  auto malformed = valid_response();
  malformed.transcript.clear();
  expect_reason(std::move(malformed), "transcript_missing");

  auto invalid_audio = valid_response();
  invalid_audio.tts_wav = {1U, 2U, 3U};
  expect_reason(std::move(invalid_audio), "invalid_wav_header");

  auto wrong_id = valid_response();
  wrong_id.request_id = "request-2";
  expect_reason(std::move(wrong_id), "wrong_request_id");

  auto oversized = valid_response();
  savo_speech::transport::Limits limits;
  limits.maximum_response_bytes = 4U;
  expect_reason(std::move(oversized), "response_oversized", limits);
}

TEST(SavoMindTransport, FakeServerCancellationIsCorrelatedAndStopsRetry)
{
  FakeSavoMindTransport transport;
  const savo_speech::transport::Request request{"request-1", "session-1", wav()};
  transport.cancel(request.request_id);
  transport.script.push_back(
    {savo_speech::transport::ExchangeStatus::Canceled, {}, "operator_cancel"});
  savo_speech::transport::ExchangeOptions options;
  options.maximum_attempts = 3U;

  const auto result = savo_speech::transport::exchange_validated(
    transport, request, {}, options);

  EXPECT_EQ(result.status, savo_speech::transport::ExchangeStatus::Canceled);
  EXPECT_EQ(transport.canceled_request_id, request.request_id);
  EXPECT_EQ(transport.exchange_calls, 1U);
}

TEST(SavoMindTransport, FakeServerRetriesAndRecoversAfterRestart)
{
  FakeSavoMindTransport transport;
  transport.script.push_back(
    {savo_speech::transport::ExchangeStatus::ConnectionFailure, {}, "restarting"});
  transport.script.push_back(
    {savo_speech::transport::ExchangeStatus::Success, valid_response(), "reconnected"});
  const savo_speech::transport::Request request{"request-1", "session-1", wav()};
  savo_speech::transport::ExchangeOptions options;
  options.maximum_attempts = 2U;

  const auto result = savo_speech::transport::exchange_validated(
    transport, request, {}, options);

  EXPECT_EQ(result.status, savo_speech::transport::ExchangeStatus::Success);
  EXPECT_TRUE(transport.healthy());
  EXPECT_EQ(transport.exchange_calls, 2U);
}
