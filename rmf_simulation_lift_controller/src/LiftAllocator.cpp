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

#include <rmf_simulation_lift_controller/LiftAllocator.hpp>

#include <algorithm>

namespace rmf_simulation_lift_controller {

namespace {
//==============================================================================
std::string group_key(const std::vector<std::string>& candidates)
{
  // Lift names cannot contain newlines, so this is an unambiguous key that
  // also preserves the order given by the Lift Manager.
  std::string key;
  for (const auto& name : candidates)
  {
    key += name;
    key += '\n';
  }
  return key;
}
} // anonymous namespace

//==============================================================================
LiftAllocator::LiftAllocator(double confirmation_timeout)
: _confirmation_timeout(confirmation_timeout)
{
  // Do nothing
}

//==============================================================================
auto LiftAllocator::handle_request(
  const LiftSelectRequest& request, double now) -> RequestResult
{
  RequestResult result;

  if (request.request_type == LiftSelectRequest::REQUEST_END_SESSION)
  {
    result.outcome = Outcome::EndSessionIgnored;
    return result;
  }

  if (request.request_type != LiftSelectRequest::REQUEST_AGV_MODE &&
    request.request_type != LiftSelectRequest::REQUEST_HUMAN_MODE)
  {
    result.outcome = Outcome::Invalid;
    result.reason = "unsupported request_type "
      + std::to_string(request.request_type);
    return result;
  }

  if (request.session_id.empty())
  {
    result.outcome = Outcome::Invalid;
    result.reason = "empty session_id";
    return result;
  }

  if (request.chosen_lifts.empty())
  {
    result.outcome = Outcome::Invalid;
    result.reason = "empty chosen_lifts";
    return result;
  }

  if (request.origin_floor.empty())
  {
    result.outcome = Outcome::Invalid;
    result.reason = "empty origin_floor";
    return result;
  }

  if (is_active(request.session_id))
  {
    result.outcome = Outcome::DuplicateActive;
    return result;
  }

  if (is_pending(request.session_id))
  {
    result.outcome = Outcome::DuplicatePending;
    return result;
  }

  const auto selected = _select(request.chosen_lifts, &result.skipped_lifts);
  if (!selected.has_value())
  {
    _pending.push_back(request);
    result.outcome = Outcome::Queued;
    return result;
  }

  _reserve(*selected, request.session_id, now);
  result.outcome = Outcome::Assigned;
  result.assignment = Assignment{*selected, request};
  return result;
}

//==============================================================================
auto LiftAllocator::handle_state(
  const LiftState& state, double now) -> StateUpdateResult
{
  StateUpdateResult result;

  auto& info = _lifts[state.lift_name];
  info.known = true;
  info.session_id = state.session_id;

  // Reconcile the reservation of this lift. An empty session_id does not
  // release the reservation: the lift may simply not have processed our
  // LiftRequest yet.
  if (!info.reserved_session.empty() &&
    state.session_id == info.reserved_session)
  {
    result.confirmed.push_back({state.lift_name, info.reserved_session});
    info.reserved_session.clear();
  }

  // Release reservations that were never confirmed, e.g. because the
  // simulated lift rejected the LiftRequest.
  for (auto& [name, lift] : _lifts)
  {
    if (!lift.reserved_session.empty() &&
      now - lift.reserved_since >= _confirmation_timeout)
    {
      result.expired.push_back({name, lift.reserved_session});
      lift.reserved_session.clear();
    }
  }

  _process_pending(now, result);
  return result;
}

//==============================================================================
bool LiftAllocator::is_available(const std::string& lift_name) const
{
  const auto it = _lifts.find(lift_name);
  if (it == _lifts.end())
    return false;

  const auto& info = it->second;
  return info.known && info.session_id.empty() &&
    info.reserved_session.empty();
}

//==============================================================================
bool LiftAllocator::is_active(const std::string& session_id) const
{
  if (session_id.empty())
    return false;

  for (const auto& [name, info] : _lifts)
  {
    if (info.session_id == session_id || info.reserved_session == session_id)
      return true;
  }
  return false;
}

//==============================================================================
bool LiftAllocator::is_pending(const std::string& session_id) const
{
  return std::any_of(_pending.begin(), _pending.end(),
      [&](const LiftSelectRequest& r) { return r.session_id == session_id; });
}

//==============================================================================
std::size_t LiftAllocator::pending_count() const
{
  return _pending.size();
}

//==============================================================================
std::optional<std::string> LiftAllocator::_select(
  const std::vector<std::string>& candidates,
  std::vector<std::string>* skipped)
{
  const std::size_t n = candidates.size();
  if (n == 0)
    return std::nullopt;

  // Start "one before" index 0 so the first pick of a new group is its first
  // available candidate.
  auto& last = _last_index.try_emplace(group_key(candidates), n - 1)
    .first->second;

  for (std::size_t i = 0; i < n; ++i)
  {
    const std::size_t idx = (last + 1 + i) % n;
    const auto& name = candidates[idx];
    if (is_available(name))
    {
      last = idx;
      return name;
    }

    if (skipped)
      skipped->push_back(name);
  }

  return std::nullopt;
}

//==============================================================================
void LiftAllocator::_reserve(
  const std::string& lift_name, const std::string& session_id, double now)
{
  auto& info = _lifts[lift_name];
  info.reserved_session = session_id;
  info.reserved_since = now;
}

//==============================================================================
void LiftAllocator::_process_pending(double now, StateUpdateResult& result)
{
  // FIFO among the requests that can currently be served. A request whose
  // candidates are all busy does not block a later request for a different
  // set of lifts.
  for (auto it = _pending.begin(); it != _pending.end(); )
  {
    if (is_active(it->session_id))
    {
      result.dropped_pending_sessions.push_back(it->session_id);
      it = _pending.erase(it);
      continue;
    }

    const auto selected = _select(it->chosen_lifts, nullptr);
    if (!selected.has_value())
    {
      ++it;
      continue;
    }

    _reserve(*selected, it->session_id, now);
    result.assignments.push_back(Assignment{*selected, *it});
    it = _pending.erase(it);
  }
}

//==============================================================================
rmf_lift_msgs::msg::LiftRequest make_lift_request(
  const LiftAllocator::Assignment& assignment)
{
  using LiftRequest = rmf_lift_msgs::msg::LiftRequest;
  using LiftSelectRequest = rmf_lift_msgs::msg::LiftSelectRequest;

  LiftRequest msg;
  msg.lift_name = assignment.lift_name;
  msg.session_id = assignment.request.session_id;
  msg.destination_floor = assignment.request.origin_floor;
  msg.door_state = LiftRequest::DOOR_OPEN;

  switch (assignment.request.request_type)
  {
    case LiftSelectRequest::REQUEST_HUMAN_MODE:
      msg.request_type = LiftRequest::REQUEST_HUMAN_MODE;
      break;
    case LiftSelectRequest::REQUEST_AGV_MODE:
    default:
      msg.request_type = LiftRequest::REQUEST_AGV_MODE;
      break;
  }

  return msg;
}

//==============================================================================
const char* to_string(LiftAllocator::Outcome outcome)
{
  using Outcome = LiftAllocator::Outcome;
  switch (outcome)
  {
    case Outcome::Assigned: return "Assigned";
    case Outcome::Queued: return "Queued";
    case Outcome::DuplicateActive: return "DuplicateActive";
    case Outcome::DuplicatePending: return "DuplicatePending";
    case Outcome::EndSessionIgnored: return "EndSessionIgnored";
    case Outcome::Invalid: return "Invalid";
  }
  return "Unknown";
}

} // namespace rmf_simulation_lift_controller
