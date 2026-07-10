#include "savo_mapping/session_state.hpp"

#include <gtest/gtest.h>

#include <string>

TEST(SessionStateContract, ConvertsStateToStableString)
{
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::SessionState::Idle)}, "idle");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::SessionState::Starting)}, "starting");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::SessionState::Active)}, "active");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::SessionState::Paused)}, "paused");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::SessionState::Saving)}, "saving");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::SessionState::Saved)}, "saved");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::SessionState::Cancelled)}, "cancelled");
  EXPECT_EQ(std::string{savo_mapping::to_string(savo_mapping::SessionState::Failed)}, "failed");
}

TEST(SessionStateContract, ParsesStableStringsToStates)
{
  EXPECT_EQ(savo_mapping::session_state_from_string("idle"), savo_mapping::SessionState::Idle);
  EXPECT_EQ(savo_mapping::session_state_from_string("starting"), savo_mapping::SessionState::Starting);
  EXPECT_EQ(savo_mapping::session_state_from_string("active"), savo_mapping::SessionState::Active);
  EXPECT_EQ(savo_mapping::session_state_from_string("paused"), savo_mapping::SessionState::Paused);
  EXPECT_EQ(savo_mapping::session_state_from_string("saving"), savo_mapping::SessionState::Saving);
  EXPECT_EQ(savo_mapping::session_state_from_string("saved"), savo_mapping::SessionState::Saved);
  EXPECT_EQ(savo_mapping::session_state_from_string("cancelled"), savo_mapping::SessionState::Cancelled);
  EXPECT_EQ(savo_mapping::session_state_from_string("failed"), savo_mapping::SessionState::Failed);
}

TEST(SessionStateContract, RejectsUnknownStateText)
{
  EXPECT_FALSE(savo_mapping::session_state_from_string("").has_value());
  EXPECT_FALSE(savo_mapping::session_state_from_string("mapping").has_value());
  EXPECT_FALSE(savo_mapping::session_state_from_string("complete").has_value());
  EXPECT_FALSE(savo_mapping::session_state_from_string("error").has_value());
  EXPECT_FALSE(savo_mapping::session_state_from_string("navigation").has_value());
}

TEST(SessionStateContract, ClassifiesRunningSessions)
{
  EXPECT_FALSE(savo_mapping::is_session_running(savo_mapping::SessionState::Idle));
  EXPECT_TRUE(savo_mapping::is_session_running(savo_mapping::SessionState::Starting));
  EXPECT_TRUE(savo_mapping::is_session_running(savo_mapping::SessionState::Active));
  EXPECT_TRUE(savo_mapping::is_session_running(savo_mapping::SessionState::Paused));
  EXPECT_TRUE(savo_mapping::is_session_running(savo_mapping::SessionState::Saving));
  EXPECT_FALSE(savo_mapping::is_session_running(savo_mapping::SessionState::Saved));
  EXPECT_FALSE(savo_mapping::is_session_running(savo_mapping::SessionState::Cancelled));
  EXPECT_FALSE(savo_mapping::is_session_running(savo_mapping::SessionState::Failed));
}

TEST(SessionStateContract, ClassifiesFinishedSessions)
{
  EXPECT_FALSE(savo_mapping::is_session_finished(savo_mapping::SessionState::Idle));
  EXPECT_FALSE(savo_mapping::is_session_finished(savo_mapping::SessionState::Starting));
  EXPECT_FALSE(savo_mapping::is_session_finished(savo_mapping::SessionState::Active));
  EXPECT_FALSE(savo_mapping::is_session_finished(savo_mapping::SessionState::Paused));
  EXPECT_FALSE(savo_mapping::is_session_finished(savo_mapping::SessionState::Saving));
  EXPECT_TRUE(savo_mapping::is_session_finished(savo_mapping::SessionState::Saved));
  EXPECT_TRUE(savo_mapping::is_session_finished(savo_mapping::SessionState::Cancelled));
  EXPECT_TRUE(savo_mapping::is_session_finished(savo_mapping::SessionState::Failed));
}

TEST(SessionStateContract, ClassifiesSuccessfulSession)
{
  EXPECT_FALSE(savo_mapping::is_successful_session(savo_mapping::SessionState::Idle));
  EXPECT_FALSE(savo_mapping::is_successful_session(savo_mapping::SessionState::Active));
  EXPECT_TRUE(savo_mapping::is_successful_session(savo_mapping::SessionState::Saved));
  EXPECT_FALSE(savo_mapping::is_successful_session(savo_mapping::SessionState::Cancelled));
  EXPECT_FALSE(savo_mapping::is_successful_session(savo_mapping::SessionState::Failed));
}

TEST(SessionStateContract, ClassifiesMappingDataAcceptance)
{
  EXPECT_FALSE(savo_mapping::can_accept_mapping_data(savo_mapping::SessionState::Idle));
  EXPECT_FALSE(savo_mapping::can_accept_mapping_data(savo_mapping::SessionState::Starting));
  EXPECT_TRUE(savo_mapping::can_accept_mapping_data(savo_mapping::SessionState::Active));
  EXPECT_TRUE(savo_mapping::can_accept_mapping_data(savo_mapping::SessionState::Paused));
  EXPECT_TRUE(savo_mapping::can_accept_mapping_data(savo_mapping::SessionState::Saving));
  EXPECT_FALSE(savo_mapping::can_accept_mapping_data(savo_mapping::SessionState::Saved));
  EXPECT_FALSE(savo_mapping::can_accept_mapping_data(savo_mapping::SessionState::Cancelled));
  EXPECT_FALSE(savo_mapping::can_accept_mapping_data(savo_mapping::SessionState::Failed));
}

TEST(SessionStateContract, ClassifiesSavePermission)
{
  EXPECT_FALSE(savo_mapping::can_request_save(savo_mapping::SessionState::Idle));
  EXPECT_FALSE(savo_mapping::can_request_save(savo_mapping::SessionState::Starting));
  EXPECT_TRUE(savo_mapping::can_request_save(savo_mapping::SessionState::Active));
  EXPECT_TRUE(savo_mapping::can_request_save(savo_mapping::SessionState::Paused));
  EXPECT_FALSE(savo_mapping::can_request_save(savo_mapping::SessionState::Saving));
  EXPECT_FALSE(savo_mapping::can_request_save(savo_mapping::SessionState::Saved));
  EXPECT_FALSE(savo_mapping::can_request_save(savo_mapping::SessionState::Cancelled));
  EXPECT_FALSE(savo_mapping::can_request_save(savo_mapping::SessionState::Failed));
}

TEST(SessionStateContract, ClassifiesCancelPermission)
{
  EXPECT_FALSE(savo_mapping::can_cancel_session(savo_mapping::SessionState::Idle));
  EXPECT_TRUE(savo_mapping::can_cancel_session(savo_mapping::SessionState::Starting));
  EXPECT_TRUE(savo_mapping::can_cancel_session(savo_mapping::SessionState::Active));
  EXPECT_TRUE(savo_mapping::can_cancel_session(savo_mapping::SessionState::Paused));
  EXPECT_FALSE(savo_mapping::can_cancel_session(savo_mapping::SessionState::Saving));
  EXPECT_FALSE(savo_mapping::can_cancel_session(savo_mapping::SessionState::Saved));
  EXPECT_FALSE(savo_mapping::can_cancel_session(savo_mapping::SessionState::Cancelled));
  EXPECT_FALSE(savo_mapping::can_cancel_session(savo_mapping::SessionState::Failed));
}
