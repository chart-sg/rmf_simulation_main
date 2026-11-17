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

#ifndef RMF_SIMULATION_LIFT_CONTROLLER__SIMULATEDLIFTCONTROLLER_HPP
#define RMF_SIMULATION_LIFT_CONTROLLER__SIMULATEDLIFTCONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmf_lift_msgs/msg/lift_request.hpp>
#include <rmf_lift_msgs/msg/lift_select_request.hpp>
#include <rmf_lift_msgs/msg/lift_state.hpp>

#include <rmf_simulation_lift_controller/LiftAllocator.hpp>

namespace rmf_simulation_lift_controller {

//==============================================================================
/// ROS 2 node that translates LiftSelectRequest messages into LiftRequest
/// messages for the simulated (Gazebo) lifts.
///
///   lift_select_requests (sub) -> LiftAllocator -> lift_requests (pub)
///   lift_states (sub)          -> LiftAllocator -> lift_requests (pub)
class SimulatedLiftController : public rclcpp::Node
{
public:
  using LiftRequest = rmf_lift_msgs::msg::LiftRequest;
  using LiftSelectRequest = rmf_lift_msgs::msg::LiftSelectRequest;
  using LiftState = rmf_lift_msgs::msg::LiftState;

  static constexpr const char* LiftSelectRequestTopicName =
    "lift_select_requests";
  static constexpr const char* LiftStateTopicName = "lift_states";
  static constexpr const char* LiftRequestTopicName = "lift_requests";

  explicit SimulatedLiftController(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void _on_select_request(const LiftSelectRequest& msg);
  void _on_lift_state(const LiftState& msg);
  void _publish(const LiftAllocator::Assignment& assignment);
  double _now_sec();

  LiftAllocator _allocator;

  rclcpp::Subscription<LiftSelectRequest>::SharedPtr _select_request_sub;
  rclcpp::Subscription<LiftState>::SharedPtr _lift_state_sub;
  rclcpp::Publisher<LiftRequest>::SharedPtr _lift_request_pub;
};

} // namespace rmf_simulation_lift_controller

#endif // RMF_SIMULATION_LIFT_CONTROLLER__SIMULATEDLIFTCONTROLLER_HPP
