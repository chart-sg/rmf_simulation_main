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

#include <rmf_simulation_lift_controller/SimulatedLiftController.hpp>

namespace rmf_simulation_lift_controller {

namespace {
//==============================================================================
std::string join(const std::vector<std::string>& names)
{
  std::string out;
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    if (i > 0)
      out += ", ";
    out += names[i];
  }
  return out;
}
} // anonymous namespace

//==============================================================================
SimulatedLiftController::SimulatedLiftController(
  const rclcpp::NodeOptions& options)
: rclcpp::Node("rmf_simulation_lift_controller", options),
  _allocator(declare_parameter<double>("confirmation_timeout", 5.0))
{
  // Matches the QoS used by the Gazebo lift plugin (lift.cpp)
  const auto reliable_qos = rclcpp::QoS(100).reliable();

  _lift_request_pub = create_publisher<LiftRequest>(
    LiftRequestTopicName, reliable_qos);

  _lift_state_sub = create_subscription<LiftState>(
    LiftStateTopicName, reliable_qos,
    [this](const LiftState& msg) { _on_lift_state(msg); });

  _select_request_sub = create_subscription<LiftSelectRequest>(
    LiftSelectRequestTopicName, reliable_qos,
    [this](const LiftSelectRequest& msg) { _on_select_request(msg); });

  RCLCPP_INFO(get_logger(),
    "Simulated lift controller started. Subscribed to [%s] and [%s], "
    "publishing to [%s]",
    _select_request_sub->get_topic_name(),
    _lift_state_sub->get_topic_name(),
    _lift_request_pub->get_topic_name());
}

//==============================================================================
void SimulatedLiftController::_on_select_request(const LiftSelectRequest& msg)
{
  RCLCPP_INFO(get_logger(),
    "Received LiftSelectRequest: session [%s] type [%u] origin [%s] "
    "next [%s] priority [%s] chosen_lifts [%s]",
    msg.session_id.c_str(), msg.request_type, msg.origin_floor.c_str(),
    msg.next_floor.c_str(), msg.priority ? "true" : "false",
    join(msg.chosen_lifts).c_str());

  const auto result = _allocator.handle_request(msg, _now_sec());

  for (const auto& lift : result.skipped_lifts)
  {
    RCLCPP_INFO(get_logger(),
      "Skipping lift [%s] for session [%s]: busy or state unknown",
      lift.c_str(), msg.session_id.c_str());
  }

  using Outcome = LiftAllocator::Outcome;
  switch (result.outcome)
  {
    case Outcome::Assigned:
      RCLCPP_INFO(get_logger(), "Selected lift [%s] for session [%s]",
        result.assignment->lift_name.c_str(), msg.session_id.c_str());
      _publish(*result.assignment);
      break;
    case Outcome::Queued:
      RCLCPP_INFO(get_logger(),
        "No available lift among [%s] for session [%s]; request queued "
        "(%zu pending)",
        join(msg.chosen_lifts).c_str(), msg.session_id.c_str(),
        _allocator.pending_count());
      break;
    case Outcome::DuplicateActive:
      RCLCPP_WARN(get_logger(),
        "Session [%s] already holds or is assigned a lift; ignoring "
        "duplicate request", msg.session_id.c_str());
      break;
    case Outcome::DuplicatePending:
      RCLCPP_WARN(get_logger(),
        "Session [%s] is already pending; ignoring duplicate request",
        msg.session_id.c_str());
      break;
    case Outcome::EndSessionIgnored:
      RCLCPP_WARN(get_logger(),
        "REQUEST_END_SESSION for session [%s] is not handled by this "
        "controller; ignoring", msg.session_id.c_str());
      break;
    case Outcome::Invalid:
      RCLCPP_WARN(get_logger(),
        "Ignoring invalid LiftSelectRequest for session [%s]: %s",
        msg.session_id.c_str(), result.reason.c_str());
      break;
  }
}

//==============================================================================
void SimulatedLiftController::_on_lift_state(const LiftState& msg)
{
  const auto result = _allocator.handle_state(msg, _now_sec());

  for (const auto& r : result.confirmed)
  {
    RCLCPP_INFO(get_logger(), "Lift [%s] confirmed session [%s]",
      r.lift_name.c_str(), r.session_id.c_str());
  }

  for (const auto& r : result.expired)
  {
    RCLCPP_WARN(get_logger(),
      "Lift [%s] did not confirm session [%s] in time; releasing the "
      "reservation. The lift may have rejected the request.",
      r.lift_name.c_str(), r.session_id.c_str());
  }

  for (const auto& session : result.dropped_pending_sessions)
  {
    RCLCPP_WARN(get_logger(),
      "Pending session [%s] became active through another request; removing "
      "it from the queue", session.c_str());
  }

  for (const auto& assignment : result.assignments)
  {
    RCLCPP_INFO(get_logger(),
      "Pending request for session [%s] assigned to lift [%s] (%zu pending)",
      assignment.request.session_id.c_str(), assignment.lift_name.c_str(),
      _allocator.pending_count());
    _publish(assignment);
  }
}

//==============================================================================
void SimulatedLiftController::_publish(
  const LiftAllocator::Assignment& assignment)
{
  auto msg = make_lift_request(assignment);
  msg.request_time = now();
  _lift_request_pub->publish(msg);

  RCLCPP_INFO(get_logger(),
    "Published LiftRequest: lift [%s] session [%s] destination [%s] "
    "type [%u] door [%u]",
    msg.lift_name.c_str(), msg.session_id.c_str(),
    msg.destination_floor.c_str(), msg.request_type, msg.door_state);
}

//==============================================================================
double SimulatedLiftController::_now_sec()
{
  return now().seconds();
}

} // namespace rmf_simulation_lift_controller
