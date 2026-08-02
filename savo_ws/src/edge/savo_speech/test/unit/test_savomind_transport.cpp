// Copyright 2026 Ahnaf Tahmid
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

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

}  // namespace

TEST(SavoMindTransport, AcceptsCorrelatedBoundedPcmWav)
{
  const savo_speech::transport::Request request{"request-1", "session-1", wav()};
  savo_speech::transport::Response response;
  response.request_id = "request-1";
  response.transcript = "hello";
  response.reply = "hi";
  response.playback_token = "token-1";
  response.playback_ack_required = true;
  response.tts_wav = wav();
  EXPECT_TRUE(savo_speech::transport::validate_request(request, {}).accepted);
  EXPECT_TRUE(savo_speech::transport::validate_response(request, response, {}).accepted);
}

TEST(SavoMindTransport, RejectsWrongCorrelationAndInvalidAudio)
{
  const savo_speech::transport::Request request{"request-1", "session-1", wav()};
  savo_speech::transport::Response response;
  response.request_id = "request-2";
  response.transcript = "hello";
  response.reply = "hi";
  response.tts_wav = wav();
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
  savo_speech::transport::Response response;
  response.request_id = "request-1";
  response.transcript = "hello";
  response.reply = "hi";
  EXPECT_EQ(savo_speech::transport::validate_response(request, response, {}).reason,
    "tts_audio_missing");
  savo_speech::transport::Limits limits;
  limits.maximum_response_bytes = 4U;
  response.tts_wav = wav();
  EXPECT_EQ(savo_speech::transport::validate_response(request, response, limits).reason,
    "response_oversized");
}

TEST(SavoMindTransport, AcceptsPlaybackAcknowledgementContract)
{
  savo_speech::transport::PlaybackAckRequest request;
  request.request_id = "request-1";
  request.session_id = "session-1";
  request.playback_token = "token-1";
  request.playback_ok = true;
  savo_speech::transport::PlaybackAckResponse response;
  response.success = true;
  response.request_id = request.request_id;
  response.reason = "navigation_dispatched";
  EXPECT_TRUE(
    savo_speech::transport::validate_playback_ack_request(request, {}).accepted);
  EXPECT_TRUE(
    savo_speech::transport::validate_playback_ack_response(
      request, response, {}).accepted);
}
