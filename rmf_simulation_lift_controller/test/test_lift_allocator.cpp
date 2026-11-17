/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>

#include <rmf_simulation_lift_controller/LiftAllocator.hpp>

using rmf_simulation_lift_controller::LiftAllocator;
using rmf_simulation_lift_controller::make_lift_request;
using LiftSelectRequest = rmf_lift_msgs::msg::LiftSelectRequest;
using LiftState = rmf_lift_msgs::msg::LiftState;
using LiftRequest = rmf_lift_msgs::msg::LiftRequest;
using Outcome = LiftAllocator::Outcome;

namespace {

LiftSelectRequest make_select(
  const std::string& session,
  const std::vector<std::string>& lifts,
  uint8_t type = LiftSelectRequest::REQUEST_AGV_MODE,
  bool priority = false)
{
  LiftSelectRequest r;
  r.session_id = session;
  r.request_type = type;
  r.origin_floor = "L1";
  r.next_floor = "L3";
  r.priority = priority;
  r.chosen_lifts = lifts;
  return r;
}

LiftState make_state(const std::string& lift, const std::string& session = "")
{
  LiftState s;
  s.lift_name = lift;
  s.available_floors = {"L1", "L2", "L3"};
  s.current_floor = "L1";
  s.destination_floor = "L1";
  s.session_id = session;
  return s;
}

class LiftAllocatorTest : public ::testing::Test
{
protected:
  LiftAllocator allocator{5.0};
  double t = 0.0;

  void state(const std::string& lift, const std::string& session = "")
  {
    allocator.handle_state(make_state(lift, session), t);
  }

  std::string assigned_lift(const LiftAllocator::RequestResult& r)
  {
    EXPECT_EQ(r.outcome, Outcome::Assigned);
    return r.assignment.has_value() ? r.assignment->lift_name : "";
  }
};

} // anonymous namespace

//==============================================================================
TEST_F(LiftAllocatorTest, SingleAvailableLiftIsSelected)
{
  state("A");
  const auto r = allocator.handle_request(make_select("s1", {"A"}), t);
  EXPECT_EQ(assigned_lift(r), "A");
  EXPECT_TRUE(allocator.is_active("s1"));
  EXPECT_FALSE(allocator.is_available("A"));
}

TEST_F(LiftAllocatorTest, SingleOccupiedLiftQueuesRequest)
{
  state("A", "other");
  const auto r = allocator.handle_request(make_select("s1", {"A"}), t);
  EXPECT_EQ(r.outcome, Outcome::Queued);
  EXPECT_TRUE(allocator.is_pending("s1"));
  EXPECT_EQ(allocator.pending_count(), 1u);
}

TEST_F(LiftAllocatorTest, MultipleAvailableLiftsRoundRobin)
{
  state("A");
  state("B");
  state("C");
  const std::vector<std::string> group = {"A", "B", "C"};

  EXPECT_EQ(assigned_lift(allocator.handle_request(make_select("s1",
    group), t)), "A");
  // Release A so that every lift is free again for the next pick
  state("A", "s1");
  state("A");
  EXPECT_EQ(assigned_lift(allocator.handle_request(make_select("s2",
    group), t)), "B");
  state("B", "s2");
  state("B");
  EXPECT_EQ(assigned_lift(allocator.handle_request(make_select("s3",
    group), t)), "C");
  state("C", "s3");
  state("C");
  EXPECT_EQ(assigned_lift(allocator.handle_request(make_select("s4",
    group), t)), "A");
}

TEST_F(LiftAllocatorTest, BusyCandidateIsSkipped)
{
  // Spec example: previously selected A, B busy, C free -> select C,
  // next search starts from A.
  state("A");
  state("B");
  state("C");
  const std::vector<std::string> group = {"A", "B", "C"};

  EXPECT_EQ(assigned_lift(allocator.handle_request(make_select("s1",
    group), t)), "A");
  state("A", "s1");
  state("A");
  state("B", "busy");

  const auto r = allocator.handle_request(make_select("s2", group), t);
  EXPECT_EQ(assigned_lift(r), "C");
  ASSERT_EQ(r.skipped_lifts.size(), 1u);
  EXPECT_EQ(r.skipped_lifts[0], "B");

  EXPECT_EQ(assigned_lift(allocator.handle_request(make_select("s3",
    group), t)), "A");
}

TEST_F(LiftAllocatorTest, RoundRobinStateIsPerGroup)
{
  state("A");
  state("B");
  EXPECT_EQ(assigned_lift(allocator.handle_request(make_select("s1",
    {"A", "B"}), t)), "A");
  state("A", "s1");
  state("A");
  // A different group starts from its own first candidate
  EXPECT_EQ(assigned_lift(allocator.handle_request(make_select("s2",
    {"B", "A"}), t)), "B");
}

TEST_F(LiftAllocatorTest, AllCandidatesBusyQueuesRequest)
{
  state("A", "x");
  state("B", "y");
  const auto r = allocator.handle_request(make_select("s1", {"A", "B"}), t);
  EXPECT_EQ(r.outcome, Outcome::Queued);
  EXPECT_EQ(r.skipped_lifts.size(), 2u);
  EXPECT_TRUE(allocator.is_pending("s1"));
}

TEST_F(LiftAllocatorTest, LiftBecomingFreeProcessesPendingRequest)
{
  state("A", "x");
  state("B", "y");
  allocator.handle_request(make_select("s1", {"A", "B"}), t);

  // Unrelated update does not assign anything
  auto u = allocator.handle_state(make_state("A", "x"), t);
  EXPECT_TRUE(u.assignments.empty());

  u = allocator.handle_state(make_state("B"), t);
  ASSERT_EQ(u.assignments.size(), 1u);
  EXPECT_EQ(u.assignments[0].lift_name, "B");
  EXPECT_EQ(u.assignments[0].request.session_id, "s1");
  EXPECT_EQ(allocator.pending_count(), 0u);
  EXPECT_TRUE(allocator.is_active("s1"));
}

TEST_F(LiftAllocatorTest, PendingRequestsServedInFifoOrder)
{
  state("A", "x");
  allocator.handle_request(make_select("s1", {"A"}), t);
  allocator.handle_request(make_select("s2", {"A"}), t);

  auto u = allocator.handle_state(make_state("A"), t);
  ASSERT_EQ(u.assignments.size(), 1u);
  EXPECT_EQ(u.assignments[0].request.session_id, "s1");

  // s1 confirmed then released -> s2 gets the lift
  state("A", "s1");
  u = allocator.handle_state(make_state("A"), t);
  ASSERT_EQ(u.assignments.size(), 1u);
  EXPECT_EQ(u.assignments[0].request.session_id, "s2");
}

TEST_F(LiftAllocatorTest, BlockedPendingRequestDoesNotBlockOtherGroups)
{
  state("A", "x");
  state("B", "y");
  allocator.handle_request(make_select("s1", {"A"}), t);
  allocator.handle_request(make_select("s2", {"B"}), t);

  const auto u = allocator.handle_state(make_state("B"), t);
  ASSERT_EQ(u.assignments.size(), 1u);
  EXPECT_EQ(u.assignments[0].request.session_id, "s2");
  EXPECT_TRUE(allocator.is_pending("s1"));
}

TEST_F(LiftAllocatorTest, DuplicateActiveSessionIgnored)
{
  state("A");
  state("B");
  allocator.handle_request(make_select("s1", {"A", "B"}), t);
  // Reserved but not yet confirmed
  EXPECT_EQ(allocator.handle_request(make_select("s1", {"A", "B"}), t).outcome,
    Outcome::DuplicateActive);
  // Confirmed by LiftState
  state("A", "s1");
  EXPECT_EQ(allocator.handle_request(make_select("s1", {"B"}), t).outcome,
    Outcome::DuplicateActive);
  EXPECT_TRUE(allocator.is_available("B"));
}

TEST_F(LiftAllocatorTest, DuplicatePendingSessionIgnored)
{
  state("A", "x");
  EXPECT_EQ(allocator.handle_request(make_select("s1", {"A"}), t).outcome,
    Outcome::Queued);
  EXPECT_EQ(allocator.handle_request(make_select("s1", {"A"}), t).outcome,
    Outcome::DuplicatePending);
  EXPECT_EQ(allocator.pending_count(), 1u);
}

TEST_F(LiftAllocatorTest, RapidConsecutiveRequestsDoNotShareALift)
{
  state("A");
  state("B");
  const auto r1 = allocator.handle_request(make_select("s1", {"A", "B"}), t);
  const auto r2 = allocator.handle_request(make_select("s2", {"A", "B"}), t);
  const auto r3 = allocator.handle_request(make_select("s3", {"A", "B"}), t);
  EXPECT_EQ(assigned_lift(r1), "A");
  EXPECT_EQ(assigned_lift(r2), "B");
  EXPECT_EQ(r3.outcome, Outcome::Queued);

  // A stale state (lift has not processed the request yet) must not release
  // the reservation.
  const auto u = allocator.handle_state(make_state("A"), t + 0.1);
  EXPECT_TRUE(u.assignments.empty());
  EXPECT_FALSE(allocator.is_available("A"));
}

TEST_F(LiftAllocatorTest, ReservationExpiresWithoutConfirmation)
{
  state("A");
  allocator.handle_request(make_select("s1", {"A"}), t);
  allocator.handle_request(make_select("s2", {"A"}), t);

  auto u = allocator.handle_state(make_state("A"), 4.9);
  EXPECT_TRUE(u.expired.empty());
  EXPECT_TRUE(u.assignments.empty());

  u = allocator.handle_state(make_state("A"), 5.0);
  ASSERT_EQ(u.expired.size(), 1u);
  EXPECT_EQ(u.expired[0].session_id, "s1");
  ASSERT_EQ(u.assignments.size(), 1u);
  EXPECT_EQ(u.assignments[0].request.session_id, "s2");
}

TEST_F(LiftAllocatorTest, ConfirmationReported)
{
  state("A");
  allocator.handle_request(make_select("s1", {"A"}), t);
  const auto u = allocator.handle_state(make_state("A", "s1"), t);
  ASSERT_EQ(u.confirmed.size(), 1u);
  EXPECT_EQ(u.confirmed[0].lift_name, "A");
  EXPECT_TRUE(allocator.is_active("s1"));
}

TEST_F(LiftAllocatorTest, EmptyChosenLiftsIgnored)
{
  state("A");
  const auto r = allocator.handle_request(make_select("s1", {}), t);
  EXPECT_EQ(r.outcome, Outcome::Invalid);
  EXPECT_EQ(r.reason, "empty chosen_lifts");
  EXPECT_EQ(allocator.pending_count(), 0u);
  EXPECT_TRUE(allocator.is_available("A"));
}

TEST_F(LiftAllocatorTest, UnknownLiftStateNotAllocated)
{
  const auto r = allocator.handle_request(make_select("s1", {"A"}), t);
  EXPECT_EQ(r.outcome, Outcome::Queued);

  // Becomes known (and free) -> assigned
  const auto u = allocator.handle_state(make_state("A"), t);
  ASSERT_EQ(u.assignments.size(), 1u);
  EXPECT_EQ(u.assignments[0].lift_name, "A");
}

TEST_F(LiftAllocatorTest, UnknownCandidateSkippedForKnownFreeOne)
{
  state("B");
  EXPECT_EQ(assigned_lift(allocator.handle_request(make_select("s1",
    {"A", "B"}), t)), "B");
}

TEST_F(LiftAllocatorTest, AgvModeProducesAgvLiftRequest)
{
  state("A");
  const auto r = allocator.handle_request(
    make_select("s1", {"A"}, LiftSelectRequest::REQUEST_AGV_MODE), t);
  ASSERT_TRUE(r.assignment.has_value());
  const auto msg = make_lift_request(*r.assignment);
  EXPECT_EQ(msg.lift_name, "A");
  EXPECT_EQ(msg.session_id, "s1");
  EXPECT_EQ(msg.destination_floor, "L1");  // origin_floor, not next_floor
  EXPECT_EQ(msg.request_type, LiftRequest::REQUEST_AGV_MODE);
  EXPECT_EQ(msg.door_state, LiftRequest::DOOR_OPEN);
}

TEST_F(LiftAllocatorTest, HumanModeProducesHumanLiftRequest)
{
  state("A");
  const auto r = allocator.handle_request(
    make_select("s1", {"A"}, LiftSelectRequest::REQUEST_HUMAN_MODE), t);
  ASSERT_TRUE(r.assignment.has_value());
  const auto msg = make_lift_request(*r.assignment);
  EXPECT_EQ(msg.request_type, LiftRequest::REQUEST_HUMAN_MODE);
  EXPECT_EQ(msg.destination_floor, "L1");
  EXPECT_EQ(msg.door_state, LiftRequest::DOOR_OPEN);
}

TEST_F(LiftAllocatorTest, EndSessionIgnored)
{
  state("A", "s1");
  const auto r = allocator.handle_request(
    make_select("s1", {"A"}, LiftSelectRequest::REQUEST_END_SESSION), t);
  EXPECT_EQ(r.outcome, Outcome::EndSessionIgnored);
  EXPECT_FALSE(r.assignment.has_value());
  EXPECT_EQ(allocator.pending_count(), 0u);
}

TEST_F(LiftAllocatorTest, UnsupportedRequestTypeIgnored)
{
  state("A");
  const auto r = allocator.handle_request(make_select("s1", {"A"}, 42), t);
  EXPECT_EQ(r.outcome, Outcome::Invalid);
  EXPECT_TRUE(allocator.is_available("A"));
}

TEST_F(LiftAllocatorTest, EmptySessionIdIgnored)
{
  state("A");
  const auto r = allocator.handle_request(make_select("", {"A"}), t);
  EXPECT_EQ(r.outcome, Outcome::Invalid);
  EXPECT_TRUE(allocator.is_available("A"));
}

TEST_F(LiftAllocatorTest, PriorityDoesNotAffectAllocation)
{
  state("A", "x");
  allocator.handle_request(make_select("normal", {"A"},
    LiftSelectRequest::REQUEST_AGV_MODE, false), t);
  allocator.handle_request(make_select("urgent", {"A"},
    LiftSelectRequest::REQUEST_AGV_MODE, true), t);

  const auto u = allocator.handle_state(make_state("A"), t);
  ASSERT_EQ(u.assignments.size(), 1u);
  EXPECT_EQ(u.assignments[0].request.session_id, "normal");
}

TEST_F(LiftAllocatorTest, PendingSessionActivatedElsewhereIsDropped)
{
  state("A", "x");
  state("B", "y");
  allocator.handle_request(make_select("s1", {"A"}), t);
  // s1 obtains lift B through the conventional lift_requests path
  const auto u = allocator.handle_state(make_state("B", "s1"), t);
  ASSERT_EQ(u.dropped_pending_sessions.size(), 1u);
  EXPECT_EQ(allocator.pending_count(), 0u);
}
