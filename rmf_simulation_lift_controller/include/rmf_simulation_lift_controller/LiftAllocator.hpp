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

#ifndef RMF_SIMULATION_LIFT_CONTROLLER__LIFTALLOCATOR_HPP
#define RMF_SIMULATION_LIFT_CONTROLLER__LIFTALLOCATOR_HPP

#include <cstddef>
#include <deque>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <rmf_lift_msgs/msg/lift_request.hpp>
#include <rmf_lift_msgs/msg/lift_select_request.hpp>
#include <rmf_lift_msgs/msg/lift_state.hpp>

namespace rmf_simulation_lift_controller {

//==============================================================================
/// ROS-independent lift selection logic.
///
/// Keeps track of the latest known occupancy of each lift, selects lifts with
/// availability-aware round-robin, queues requests that cannot be served yet
/// and protects against duplicate sessions. It never publishes anything; the
/// caller is responsible for turning returned assignments into LiftRequests.
///
/// This class is not thread-safe. Calls must be serialized by the caller.
class LiftAllocator
{
public:
  using LiftSelectRequest = rmf_lift_msgs::msg::LiftSelectRequest;
  using LiftState = rmf_lift_msgs::msg::LiftState;

  struct Assignment
  {
    std::string lift_name;
    LiftSelectRequest request;
  };

  enum class Outcome
  {
    Assigned,
    Queued,
    DuplicateActive,
    DuplicatePending,
    EndSessionIgnored,
    Invalid
  };

  struct RequestResult
  {
    Outcome outcome;
    /// Set when outcome == Assigned
    std::optional<Assignment> assignment;
    /// Candidates that were skipped because they were busy, reserved or their
    /// state is not yet known.
    std::vector<std::string> skipped_lifts;
    /// Human readable explanation when outcome == Invalid
    std::string reason;
  };

  struct Reservation
  {
    std::string lift_name;
    std::string session_id;
  };

  struct StateUpdateResult
  {
    /// Pending requests that were assigned because of this update
    std::vector<Assignment> assignments;
    /// Reservations confirmed by this LiftState
    std::vector<Reservation> confirmed;
    /// Reservations that were never confirmed within the timeout
    std::vector<Reservation> expired;
    /// Pending requests dropped because their session became active through
    /// another path
    std::vector<std::string> dropped_pending_sessions;
  };

  /// \param[in] confirmation_timeout
  ///   Seconds to wait for a LiftState that confirms an assignment before the
  ///   internal reservation is released.
  explicit LiftAllocator(double confirmation_timeout = 5.0);

  /// Handle a new LiftSelectRequest.
  /// \param[in] now Current time in seconds.
  RequestResult handle_request(const LiftSelectRequest& request, double now);

  /// Handle a LiftState update, reconcile reservations and try to serve the
  /// pending queue.
  /// \param[in] now Current time in seconds.
  StateUpdateResult handle_state(const LiftState& state, double now);

  /// True if the lift state is known, it has no session and no assignment is
  /// awaiting confirmation.
  bool is_available(const std::string& lift_name) const;

  /// True if the session holds a lift, or is reserved on one.
  bool is_active(const std::string& session_id) const;

  /// True if the session is waiting in the pending queue.
  bool is_pending(const std::string& session_id) const;

  std::size_t pending_count() const;

private:
  struct LiftInfo
  {
    bool known = false;
    std::string session_id;
    std::string reserved_session;
    double reserved_since = 0.0;
  };

  std::optional<std::string> _select(
    const std::vector<std::string>& candidates,
    std::vector<std::string>* skipped);

  void _reserve(const std::string& lift_name, const std::string& session_id,
    double now);

  void _process_pending(double now, StateUpdateResult& result);

  double _confirmation_timeout;
  std::unordered_map<std::string, LiftInfo> _lifts;
  /// Round-robin state: candidate group key -> index of last selected lift
  std::unordered_map<std::string, std::size_t> _last_index;
  std::deque<LiftSelectRequest> _pending;
};

//==============================================================================
/// Convert an assignment into the LiftRequest sent to the simulated lift.
/// The lift is called to the boarding (origin) floor with doors open.
rmf_lift_msgs::msg::LiftRequest make_lift_request(
  const LiftAllocator::Assignment& assignment);

//==============================================================================
const char* to_string(LiftAllocator::Outcome outcome);

} // namespace rmf_simulation_lift_controller

#endif // RMF_SIMULATION_LIFT_CONTROLLER__LIFTALLOCATOR_HPP
